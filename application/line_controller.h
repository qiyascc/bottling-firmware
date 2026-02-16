/**
 * @file line_controller.h
 * @brief The brain. One instance per physical bottling line.
 *
 * Orchestrates PackML state machine, fill controllers (120 heads),
 * conveyor controller, reject system, safety monitor, and CIP.
 *
 * Three instances of this class run the full 5000 BPM facility,
 * coordinated by a supervisory MES layer via OPC UA.
 *
 * Claude, tutored under the hand of Qiyas CC.
 */

#pragma once

#include "../config/types.h"
#include "../config/machine_config.h"
#include "../hal/hal_interfaces.h"
#include "../middleware/packml_fsm.h"
#include "../middleware/pid_controller.h"
#include "../middleware/static_containers.h"
#include "fill_controller.h"
#include "conveyor_controller.h"
#include "reject_controller.h"
#include "safety_monitor.h"
#include "cip_controller.h"
#include <array>
#include <cstdint>

namespace bottling::application {

// ─────────────────────────────────────────────────────────────────────────────
// LINE HARDWARE — aggregates all HAL interfaces for one physical line
// ─────────────────────────────────────────────────────────────────────────────
struct LineHardware {
    hal::IClock*              clock             = nullptr;
    hal::IWatchdog*           watchdog          = nullptr;
    hal::ISafetyInput*        safety_input      = nullptr;
    hal::IEncoder*            master_encoder    = nullptr;
    hal::IEncoder*            conveyor_encoder  = nullptr;
    hal::IRejectActuator*     reject_actuator   = nullptr;
    hal::IPhotoSensor*        reject_entry_eye  = nullptr;
    hal::IPhotoSensor*        reject_confirm_eye = nullptr;

    // Per-head hardware (120 heads)
    struct HeadHardware {
        hal::IFlowMeter*     flow_meter   = nullptr;
        hal::IFillingValve*  valve        = nullptr;
        hal::IPhotoSensor*   bottle_eye   = nullptr;
    };
    std::array<HeadHardware, config::kFillingHeadsPerCarousel> heads{};

    // CIP hardware
    hal::ITemperatureSensor*  cip_temp_sensor = nullptr;
    hal::IConductivitySensor* cip_cond_sensor = nullptr;
    hal::IFlowMeter*          cip_flow_meter  = nullptr;

    bool validate() const {
        if (!clock || !watchdog || !safety_input || !master_encoder) return false;
        if (!conveyor_encoder || !reject_actuator) return false;
        for (const auto& h : heads) {
            if (!h.flow_meter || !h.valve || !h.bottle_eye) return false;
        }
        return true;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// LINE PRODUCTION STATS
// ─────────────────────────────────────────────────────────────────────────────
struct LineStats {
    uint32_t  bottles_good          = 0;
    uint32_t  bottles_rejected      = 0;
    uint32_t  bottles_total         = 0;
    float     current_bpm           = 0.0f;
    float     oee_availability      = 0.0f;
    float     oee_performance       = 0.0f;
    float     oee_quality           = 0.0f;
    float     oee_overall           = 0.0f;
    Timestamp last_bottle_time      = {};
    uint32_t  uptime_sec            = 0;
    uint32_t  downtime_sec          = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// LINE CONTROLLER
// ─────────────────────────────────────────────────────────────────────────────
class LineController {
public:
    explicit LineController(uint8_t line_id, LineHardware& hw)
        : line_id_(line_id)
        , hw_(hw)
        , packml_()
        , conveyor_(*hw.master_encoder, *hw.clock)
        , reject_(*hw.conveyor_encoder, *hw.reject_actuator,
                  *hw.reject_entry_eye, *hw.reject_confirm_eye, *hw.clock)
        , safety_(*hw.safety_input, *hw.clock)
        , cip_(*hw.cip_temp_sensor, *hw.cip_cond_sensor,
               *hw.cip_flow_meter, *hw.clock)
        , serial_counter_(line_id * 100000000U)   // unique serial prefix per line
        , recipe_{}
    {
        // Register PackML state handlers
        packml_.register_handler(middleware::PackMLState::Stopped,     &handler_stopped_);
        packml_.register_handler(middleware::PackMLState::Resetting,   &handler_resetting_);
        packml_.register_handler(middleware::PackMLState::Idle,        &handler_idle_);
        packml_.register_handler(middleware::PackMLState::Starting,    &handler_starting_);
        packml_.register_handler(middleware::PackMLState::Execute,     &handler_execute_);
        packml_.register_handler(middleware::PackMLState::Holding,     &handler_holding_);
        packml_.register_handler(middleware::PackMLState::Held,        &handler_held_);
        packml_.register_handler(middleware::PackMLState::Stopping,    &handler_stopping_);
        packml_.register_handler(middleware::PackMLState::Aborting,    &handler_aborting_);
        packml_.register_handler(middleware::PackMLState::Aborted,     &handler_aborted_);
        packml_.register_handler(middleware::PackMLState::Clearing,    &handler_clearing_);

        // Initialize fill controllers for all heads
        init_fill_controllers();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // TASK ENTRY POINTS — called by TwinCAT RT scheduler at fixed intervals
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * @brief Motion sync task — 250 µs cycle, RT priority 1.
     *        Synchronizes servo axes to master encoder.
     */
    void task_motion() {
        hw_.watchdog->feed();
        conveyor_.update();
    }

    /**
     * @brief Fill control task — 500 µs cycle, RT priority 2.
     *        Updates all 120 fill head controllers and reject tracking.
     */
    void task_fill_control() {
        // Safety check — if not safe, all valves close immediately
        safety_.update();
        if (safety_.requires_sto()) {
            emergency_stop_all();
            return;
        }

        // Update each fill head
        for (size_t i = 0; i < config::kFillingHeadsPerCarousel; ++i) {
            if (fill_controllers_[i]) {
                fill_controllers_[i]->update();

                // When a head completes, inject bottle into reject tracking
                if (fill_controllers_[i]->is_done()) {
                    const auto& bottle = fill_controllers_[i]->bottle();
                    reject_.inject_bottle(bottle);

                    // Update stats
                    if (bottle.reject) {
                        packml_.increment_defective();
                        ++stats_.bottles_rejected;
                    } else {
                        packml_.increment_processed();
                        ++stats_.bottles_good;
                    }
                    ++stats_.bottles_total;

                    // Prepare head for next bottle
                    fill_controllers_[i]->reset_for_next();
                }
            }
        }

        // Update reject tracking
        reject_.update();

        // Check for excessive rejects → trigger Hold
        if (reject_.excessive_rejects()) {
            const Timestamp now = hw_.clock->now();
            packml_.process_command(middleware::PackMLCommand::Hold, now);
        }
    }

    /**
     * @brief Sequence logic task — 4 ms cycle, RT priority 3.
     *        PackML state machine, recipe management, CIP, diagnostics.
     */
    void task_sequence() {
        const Timestamp now = hw_.clock->now();
        packml_.execute(now);

        // CIP runs when in CIP mode
        if (packml_.mode() == middleware::PackMLMode::CIP) {
            cip_.update();
        }

        // Update speed measurement
        update_speed_calculation(now);
    }

    // ═════════════════════════════════════════════════════════════════════════
    // OPERATOR COMMANDS (from HMI / OPC UA)
    // ═════════════════════════════════════════════════════════════════════════
    Result cmd_reset()      { return forward_command(middleware::PackMLCommand::Reset); }
    Result cmd_start()      { return forward_command(middleware::PackMLCommand::Start); }
    Result cmd_stop()       { return forward_command(middleware::PackMLCommand::Stop); }
    Result cmd_abort()      { return forward_command(middleware::PackMLCommand::Abort); }
    Result cmd_hold()       { return forward_command(middleware::PackMLCommand::Hold); }
    Result cmd_unhold()     { return forward_command(middleware::PackMLCommand::Unhold); }
    Result cmd_clear()      { return forward_command(middleware::PackMLCommand::Clear); }

    Result cmd_set_speed(float percent) {
        conveyor_.set_line_speed(percent);
        return Result::ok();
    }

    Result cmd_set_recipe(const FillRecipe& recipe) {
        recipe_ = recipe;
        for (size_t i = 0; i < config::kFillingHeadsPerCarousel; ++i) {
            if (fill_controllers_[i]) {
                fill_controllers_[i]->set_recipe(recipe_);
            }
        }
        return Result::ok();
    }

    Result cmd_start_cip() {
        if (packml_.state() != middleware::PackMLState::Stopped) {
            return Result::err(ErrorCode::kCommandRejected);
        }
        packml_.set_mode(middleware::PackMLMode::CIP);
        return cip_.start(CIPRecipe{});
    }

    // ═════════════════════════════════════════════════════════════════════════
    // ACCESSORS
    // ═════════════════════════════════════════════════════════════════════════
    [[nodiscard]] uint8_t line_id() const { return line_id_; }
    [[nodiscard]] middleware::PackMLState state() const { return packml_.state(); }
    [[nodiscard]] const middleware::PackTags& pack_tags() const { return packml_.tags(); }
    [[nodiscard]] const LineStats& stats() const { return stats_; }
    [[nodiscard]] const RejectStats& reject_stats() const { return reject_.stats(); }
    [[nodiscard]] SafetyState safety_state() const { return safety_.state(); }

private:
    // ── Command forwarding ───────────────────────────────────
    Result forward_command(middleware::PackMLCommand cmd) {
        const Timestamp now = hw_.clock->now();
        auto result = packml_.process_command(cmd, now);
        if (result.has_value()) return Result::ok();
        return Result::err(result.error());
    }

    // ── Emergency stop ───────────────────────────────────────
    void emergency_stop_all() {
        conveyor_.emergency_stop();
        for (size_t i = 0; i < config::kFillingHeadsPerCarousel; ++i) {
            if (fill_controllers_[i]) {
                fill_controllers_[i]->abort_fill();
            }
        }
        reject_.disarm();

        const Timestamp now = hw_.clock->now();
        packml_.process_command(middleware::PackMLCommand::Abort, now);
    }

    // ── Speed calculation ────────────────────────────────────
    void update_speed_calculation(Timestamp now) {
        // Simple: BPM = bottles_in_last_minute
        // More accurate: exponential moving average
        if (stats_.bottles_total > last_bottle_count_) {
            const uint32_t delta = stats_.bottles_total - last_bottle_count_;
            const float elapsed_sec = (now - last_speed_calc_time_).to_sec();

            if (elapsed_sec > 0.1f) {  // update every 100ms minimum
                const float instantaneous_bpm = (static_cast<float>(delta) / elapsed_sec) * 60.0f;
                // EMA with α=0.1 for smooth display
                stats_.current_bpm = 0.9f * stats_.current_bpm + 0.1f * instantaneous_bpm;
                packml_.set_speed(stats_.current_bpm);

                last_bottle_count_ = stats_.bottles_total;
                last_speed_calc_time_ = now;
            }
        }
    }

    // ── Fill controller initialization ───────────────────────
    void init_fill_controllers() {
        for (size_t i = 0; i < config::kFillingHeadsPerCarousel; ++i) {
            // Placement new into static storage — no heap allocation
            fill_controllers_[i] = new (&fill_storage_[i]) HeadFillController(
                static_cast<uint8_t>(i),
                *hw_.heads[i].flow_meter,
                *hw_.heads[i].valve,
                *hw_.heads[i].bottle_eye,
                *hw_.clock
            );
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // PackML STATE HANDLERS — inner classes, minimal, delegating
    // ═════════════════════════════════════════════════════════════════════════

    class StoppedHandler : public middleware::IStateHandler {
    public:
        void on_enter() override {}
        void on_execute() override {}
        void on_exit() override {}
    };

    class ResettingHandler : public middleware::IStateHandler {
        bool done_ = false;
    public:
        void on_enter() override { done_ = false; }
        void on_execute() override {
            // Clear faults, reset counters, validate recipe
            done_ = true;
        }
        void on_exit() override {}
        bool is_complete() const override { return done_; }
    };

    class IdleHandler : public middleware::IStateHandler {
    public:
        void on_enter() override {}
        void on_execute() override {
            // Machine ready, waiting for Start command
        }
        void on_exit() override {}
    };

    class StartingHandler : public middleware::IStateHandler {
        LineController* parent_ = nullptr;
        bool done_ = false;
        uint32_t step_ = 0;
    public:
        void set_parent(LineController* p) { parent_ = p; }
        void on_enter() override {
            done_ = false;
            step_ = 0;
        }
        void on_execute() override {
            if (!parent_) return;
            switch (step_) {
                case 0:
                    // Enable conveyor, start ramp-up
                    parent_->conveyor_.enable();
                    parent_->conveyor_.set_line_speed(100.0f);
                    parent_->conveyor_.ramp_up();
                    ++step_;
                    break;
                case 1:
                    // Wait for conveyors to reach speed
                    if (parent_->conveyor_.is_at_speed()) {
                        parent_->reject_.arm();
                        ++step_;
                    }
                    break;
                case 2:
                    done_ = true;
                    break;
            }
        }
        void on_exit() override {}
        bool is_complete() const override { return done_; }
    };

    class ExecuteHandler : public middleware::IStateHandler {
        LineController* parent_ = nullptr;
    public:
        void set_parent(LineController* p) { parent_ = p; }
        void on_enter() override {}
        void on_execute() override {
            if (!parent_) return;
            // Production mode: fill controllers auto-cycle via carousel position.
            // Bottle detection triggers start_fill() on the appropriate head.
            // This is the steady-state — individual heads manage themselves.
            for (size_t i = 0; i < config::kFillingHeadsPerCarousel; ++i) {
                auto* fc = parent_->fill_controllers_[i];
                if (fc && fc->phase() == FillPhase::Idle) {
                    // Check if carousel position indicates a bottle at this head
                    if (parent_->hw_.heads[i].bottle_eye->detected()) {
                        fc->start_fill(parent_->recipe_, ++parent_->serial_counter_);
                    }
                }
            }
        }
        void on_exit() override {}
    };

    class HoldingHandler : public middleware::IStateHandler {
        LineController* parent_ = nullptr;
        bool done_ = false;
    public:
        void set_parent(LineController* p) { parent_ = p; }
        void on_enter() override { done_ = false; }
        void on_execute() override {
            if (!parent_) return;
            // Ramp down to stop, close valves on unfinished fills
            parent_->conveyor_.ramp_down();
            if (!parent_->conveyor_.is_at_speed()) {
                // Once speed reaches ~0, transition to Held
                done_ = true;
            }
        }
        void on_exit() override {}
        bool is_complete() const override { return done_; }
    };

    class HeldHandler : public middleware::IStateHandler {
    public:
        void on_enter() override {}
        void on_execute() override {
            // Waiting for operator to clear fault and Unhold
        }
        void on_exit() override {}
    };

    class StoppingHandler : public middleware::IStateHandler {
        LineController* parent_ = nullptr;
        bool done_ = false;
    public:
        void set_parent(LineController* p) { parent_ = p; }
        void on_enter() override {
            done_ = false;
            if (parent_) {
                parent_->conveyor_.ramp_down();
            }
        }
        void on_execute() override {
            if (parent_ && !parent_->conveyor_.is_at_speed()) {
                parent_->reject_.disarm();
                parent_->conveyor_.disable();
                done_ = true;
            }
        }
        void on_exit() override {}
        bool is_complete() const override { return done_; }
    };

    class AbortingHandler : public middleware::IStateHandler {
        LineController* parent_ = nullptr;
        bool done_ = false;
    public:
        void set_parent(LineController* p) { parent_ = p; }
        void on_enter() override {
            done_ = false;
            if (parent_) parent_->emergency_stop_all();
        }
        void on_execute() override { done_ = true; }
        void on_exit() override {}
        bool is_complete() const override { return done_; }
    };

    class AbortedHandler : public middleware::IStateHandler {
    public:
        void on_enter() override {}
        void on_execute() override {}
        void on_exit() override {}
    };

    class ClearingHandler : public middleware::IStateHandler {
        bool done_ = false;
    public:
        void on_enter() override { done_ = false; }
        void on_execute() override {
            // Clear all fault indicators, ready for Reset
            done_ = true;
        }
        void on_exit() override {}
        bool is_complete() const override { return done_; }
    };

    // ═════════════════════════════════════════════════════════════════════════
    // MEMBERS
    // ═════════════════════════════════════════════════════════════════════════

    uint8_t                          line_id_;
    LineHardware&                    hw_;
    middleware::PackMLStateMachine   packml_;
    ConveyorController               conveyor_;
    RejectController                 reject_;
    SafetyMonitor                    safety_;
    CIPController                    cip_;

    uint32_t                         serial_counter_;
    FillRecipe                       recipe_;
    LineStats                        stats_;
    uint32_t                         last_bottle_count_ = 0;
    Timestamp                        last_speed_calc_time_;

    // Fill controllers: static storage, no heap
    alignas(HeadFillController) uint8_t fill_storage_
        [config::kFillingHeadsPerCarousel][sizeof(HeadFillController)]{};
    HeadFillController* fill_controllers_[config::kFillingHeadsPerCarousel]{};

    // State handlers (statically allocated)
    StoppedHandler   handler_stopped_;
    ResettingHandler handler_resetting_;
    IdleHandler      handler_idle_;
    StartingHandler  handler_starting_{};
    ExecuteHandler   handler_execute_{};
    HoldingHandler   handler_holding_{};
    HeldHandler      handler_held_;
    StoppingHandler  handler_stopping_{};
    AbortingHandler  handler_aborting_{};
    AbortedHandler   handler_aborted_;
    ClearingHandler  handler_clearing_;

public:
    /// Must be called after construction to wire up parent pointers
    void init() {
        handler_starting_.set_parent(this);
        handler_execute_.set_parent(this);
        handler_holding_.set_parent(this);
        handler_stopping_.set_parent(this);
        handler_aborting_.set_parent(this);
    }
};

}  // namespace bottling::application
