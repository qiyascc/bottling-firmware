/**
 * @file fill_controller.h
 * @brief Per-head fill sequence controller with multi-phase profiling.
 *
 * Each of 120 filling heads runs its own instance of this controller.
 * The fill phases: IDLE → CENTERING → PRE_FILL → MAIN_FILL → TOP_OFF →
 *                  SETTLING → DRIP_WAIT → COMPLETE (or REJECT)
 *
 * At 1667 BPM per carousel with 120 heads, each head processes
 * ~13.9 bottles per minute. Fill time budget: ~3240 ms per head per rotation.
 * Plenty of time. The hard part is ±0.25% accuracy at that rate.
 *
 * Claude, tutored under the hand of Qiyas CC.
 */

#pragma once

#include "../config/types.h"
#include "../config/machine_config.h"
#include "../hal/hal_interfaces.h"
#include "../middleware/pid_controller.h"
#include <cstdint>

namespace bottling::application {

// ─────────────────────────────────────────────────────────────────────────────
// FILL PHASE — each head steps through this sequence per bottle
// ─────────────────────────────────────────────────────────────────────────────
enum class FillPhase : uint8_t {
    Idle,           // no bottle present, valve closed
    Centering,      // bottle gripped, aligning under nozzle
    PreFill,        // slow initial fill to prevent splash
    MainFill,       // full-speed fill (PFR valve wide open)
    TopOff,         // slow fill approaching target — PID active
    Settling,       // valve closed, liquid settling
    DripWait,       // drip compensation delay
    Complete,       // fill done, good bottle → release to discharge
    Reject,         // fill failed → flag for rejection
};

constexpr const char* to_string(FillPhase p) {
    switch (p) {
        case FillPhase::Idle:       return "IDLE";
        case FillPhase::Centering:  return "CENTERING";
        case FillPhase::PreFill:    return "PRE_FILL";
        case FillPhase::MainFill:   return "MAIN_FILL";
        case FillPhase::TopOff:     return "TOP_OFF";
        case FillPhase::Settling:   return "SETTLING";
        case FillPhase::DripWait:   return "DRIP_WAIT";
        case FillPhase::Complete:   return "COMPLETE";
        case FillPhase::Reject:     return "REJECT";
        default:                    return "UNKNOWN";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FILL RECIPE — per-product configuration
// ─────────────────────────────────────────────────────────────────────────────
struct FillRecipe {
    float    target_volume_ml        = config::kBottleVolumeMl;
    float    drip_compensation_ml    = config::kDripCompensationMl;
    float    pre_fill_rate_ml_sec    = 80.0f;       // slow start
    float    main_fill_rate_ml_sec   = config::kMaxFlowRateMlPerSec;
    float    topoff_rate_ml_sec      = config::kSlowFillRateMlPerSec;
    float    slow_fill_threshold     = config::kSlowFillThresholdPercent;
    float    overfill_reject_ml      = config::kOverfillRejectThresholdMl;
    float    underfill_reject_ml     = config::kUnderfillRejectThresholdMl;
    uint16_t centering_time_ms       = 50;
    uint16_t settling_time_ms        = 80;
    uint16_t drip_wait_ms            = 30;
    uint16_t fill_timeout_ms         = 5000;

    constexpr float effective_target() const {
        return target_volume_ml - drip_compensation_ml;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// HEAD FILL CONTROLLER — one instance per filling head
// ─────────────────────────────────────────────────────────────────────────────
class HeadFillController {
public:
    HeadFillController(uint8_t head_id,
                       hal::IFlowMeter& flow_meter,
                       hal::IFillingValve& valve,
                       hal::IPhotoSensor& bottle_sensor,
                       hal::IClock& clock)
        : head_id_(head_id)
        , flow_meter_(flow_meter)
        , valve_(valve)
        , bottle_sensor_(bottle_sensor)
        , clock_(clock)
        , phase_(FillPhase::Idle)
        , cascade_(build_outer_params(), build_inner_params())
        , bottle_{} {}

    /**
     * @brief Called every fill control cycle (500 µs).
     *        Executes the current fill phase and transitions as needed.
     */
    void update() {
        const Timestamp now = clock_.now();

        switch (phase_) {
            case FillPhase::Idle:
                handle_idle();
                break;

            case FillPhase::Centering:
                handle_centering(now);
                break;

            case FillPhase::PreFill:
                handle_pre_fill(now);
                break;

            case FillPhase::MainFill:
                handle_main_fill(now);
                break;

            case FillPhase::TopOff:
                handle_top_off(now);
                break;

            case FillPhase::Settling:
                handle_settling(now);
                break;

            case FillPhase::DripWait:
                handle_drip_wait(now);
                break;

            case FillPhase::Complete:
            case FillPhase::Reject:
                // Held until carousel logic resets us for next bottle
                break;
        }
    }

    // ── Lifecycle ────────────────────────────────────────────
    void start_fill(const FillRecipe& recipe, uint32_t bottle_serial) {
        recipe_ = recipe;
        bottle_ = BottleRecord{};
        bottle_.serial = bottle_serial;
        bottle_.head_id = head_id_;
        bottle_.target_volume_ml = recipe_.target_volume_ml;

        flow_meter_.reset_totalizer();
        cascade_.reset();
        cascade_.enable();

        phase_ = FillPhase::Centering;
        phase_start_ = clock_.now();
    }

    void abort_fill() {
        valve_.emergency_close();
        cascade_.disable();
        phase_ = FillPhase::Reject;
        bottle_.reject = true;
        bottle_.reject_reason = ErrorCode::kAbortRequired;
    }

    void reset_for_next() {
        valve_.emergency_close();
        cascade_.disable();
        phase_ = FillPhase::Idle;
    }

    // ── Accessors ────────────────────────────────────────────
    [[nodiscard]] FillPhase         phase() const    { return phase_; }
    [[nodiscard]] uint8_t           head_id() const  { return head_id_; }
    [[nodiscard]] const BottleRecord& bottle() const { return bottle_; }
    [[nodiscard]] bool is_done() const {
        return phase_ == FillPhase::Complete || phase_ == FillPhase::Reject;
    }

    void set_recipe(const FillRecipe& r) { recipe_ = r; }

private:
    // ── Phase handlers ───────────────────────────────────────
    void handle_idle() {
        // Wait — carousel logic will call start_fill() when a bottle arrives
    }

    void handle_centering(Timestamp now) {
        // Wait for centering time to elapse (grippers engage bottle)
        if (elapsed_ms(now) >= recipe_.centering_time_ms) {
            bottle_.fill_start = now;
            flow_meter_.reset_totalizer();
            transition_to(FillPhase::PreFill, now);

            // Pre-fill: gentle start to prevent splash
            valve_.set_opening(opening_for_rate(recipe_.pre_fill_rate_ml_sec));
        }
    }

    void handle_pre_fill(Timestamp now) {
        const auto volume = flow_meter_.totalizer();
        if (!volume.has_value()) { fault_reject(volume.error()); return; }

        const float filled_ml = volume.value();
        const float threshold = recipe_.effective_target() * 0.15f;  // 15% → switch to main

        if (filled_ml >= threshold) {
            transition_to(FillPhase::MainFill, now);
            valve_.set_opening(opening_for_rate(recipe_.main_fill_rate_ml_sec));
        }

        check_timeout(now);
    }

    void handle_main_fill(Timestamp now) {
        const auto volume = flow_meter_.totalizer();
        if (!volume.has_value()) { fault_reject(volume.error()); return; }

        const float filled_ml = volume.value();
        const float slow_threshold = recipe_.effective_target() * recipe_.slow_fill_threshold;

        if (filled_ml >= slow_threshold) {
            // Switch to PID-controlled top-off phase
            transition_to(FillPhase::TopOff, now);
            cascade_.initialize(
                opening_for_rate(recipe_.topoff_rate_ml_sec),
                recipe_.topoff_rate_ml_sec,
                filled_ml
            );
        }

        check_timeout(now);
    }

    void handle_top_off(Timestamp now) {
        const auto volume = flow_meter_.totalizer();
        const auto flow   = flow_meter_.flow_rate();
        if (!volume.has_value()) { fault_reject(volume.error()); return; }
        if (!flow.has_value())   { fault_reject(flow.error()); return; }

        const float filled_ml = volume.value();
        const float flow_rate = flow.value();

        // Cascade PID: outer(volume) → inner(flow rate) → valve
        const float valve_cmd = cascade_.compute(
            recipe_.effective_target(),
            filled_ml,
            flow_rate,
            opening_for_rate(recipe_.topoff_rate_ml_sec)  // feedforward
        );

        valve_.set_opening(valve_cmd);

        // Check if target reached
        if (filled_ml >= recipe_.effective_target()) {
            valve_.set_opening(0.0f);  // close valve
            cascade_.disable();
            transition_to(FillPhase::Settling, now);
        }

        check_timeout(now);
    }

    void handle_settling(Timestamp now) {
        if (elapsed_ms(now) >= recipe_.settling_time_ms) {
            transition_to(FillPhase::DripWait, now);
        }
    }

    void handle_drip_wait(Timestamp now) {
        if (elapsed_ms(now) >= recipe_.drip_wait_ms) {
            // Final volume reading
            const auto final_vol = flow_meter_.totalizer();
            if (final_vol.has_value()) {
                bottle_.actual_volume_ml = final_vol.value() + recipe_.drip_compensation_ml;
            }

            bottle_.fill_end = now;

            // Quality check
            if (bottle_.actual_volume_ml > recipe_.overfill_reject_ml) {
                fault_reject(ErrorCode::kOverfill);
            } else if (bottle_.actual_volume_ml < recipe_.underfill_reject_ml) {
                fault_reject(ErrorCode::kUnderfill);
            } else {
                phase_ = FillPhase::Complete;
            }
        }
    }

    // ── Helpers ──────────────────────────────────────────────
    void transition_to(FillPhase next, Timestamp now) {
        phase_ = next;
        phase_start_ = now;
    }

    void fault_reject(ErrorCode reason) {
        valve_.emergency_close();
        cascade_.disable();
        bottle_.reject = true;
        bottle_.reject_reason = reason;
        bottle_.fill_end = clock_.now();

        // Read whatever volume we managed
        const auto vol = flow_meter_.totalizer();
        if (vol.has_value()) {
            bottle_.actual_volume_ml = vol.value();
        }

        phase_ = FillPhase::Reject;
    }

    void check_timeout(Timestamp now) {
        const auto total_elapsed = (now - bottle_.fill_start).to_ms();
        if (total_elapsed > static_cast<float>(recipe_.fill_timeout_ms)) {
            fault_reject(ErrorCode::kFillTimeout);
        }
    }

    uint32_t elapsed_ms(Timestamp now) const {
        return static_cast<uint32_t>((now - phase_start_).to_ms());
    }

    /// Convert desired flow rate to valve opening percentage.
    /// Linear approximation — real systems use a characterized valve curve.
    static float opening_for_rate(float target_ml_sec) {
        return (target_ml_sec / config::kMaxFlowRateMlPerSec) * 100.0f;
    }

    /// Build PID params at compile-time-ish
    static middleware::PIDParams build_outer_params() {
        middleware::PIDParams p;
        p.kp             = config::kPID_Volume_Kp;
        p.ki             = config::kPID_Volume_Ki;
        p.kd             = config::kPID_Volume_Kd;
        p.output_min     = 0.0f;
        p.output_max     = config::kMaxFlowRateMlPerSec;
        p.dt_sec         = static_cast<float>(config::kFillTaskCycleUs) / 1000000.0f;
        p.derivative_lpf = 0.15f;
        p.tracking_tc    = config::kPID_AntiWindupTrackingTc;
        p.feedforward    = config::kSlowFillRateMlPerSec;
        return p;
    }

    static middleware::PIDParams build_inner_params() {
        middleware::PIDParams p;
        p.kp             = config::kPID_Flow_Kp;
        p.ki             = config::kPID_Flow_Ki;
        p.kd             = config::kPID_Flow_Kd;
        p.output_min     = config::kPID_OutputMin;
        p.output_max     = config::kPID_OutputMax;
        p.dt_sec         = static_cast<float>(config::kFillTaskCycleUs) / 1000000.0f;
        p.derivative_lpf = 0.1f;
        p.tracking_tc    = config::kPID_AntiWindupTrackingTc;
        p.feedforward    = 0.0f;
        return p;
    }

    // ── Members ──────────────────────────────────────────────
    uint8_t                      head_id_;
    hal::IFlowMeter&             flow_meter_;
    hal::IFillingValve&          valve_;
    hal::IPhotoSensor&           bottle_sensor_;
    hal::IClock&                 clock_;

    FillPhase                    phase_;
    Timestamp                    phase_start_;
    FillRecipe                   recipe_;
    middleware::CascadePID       cascade_;
    BottleRecord                 bottle_;
};

}  // namespace bottling::application
