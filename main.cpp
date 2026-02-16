/**
 * @file main.cpp
 * @brief System entry point — 3-line facility orchestrator.
 *
 * In production: this is a TwinCAT TcCOM module registered with the
 * TwinCAT ObjectServer. The RT scheduler calls our cyclic methods
 * at the configured intervals (250µs, 500µs, 4ms).
 *
 * In simulation: this runs as a standalone Linux/Windows process with
 * simulated HAL drivers for testing and virtual commissioning.
 *
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  5000 BPM = 3 × 1667 BPM lines                                 ║
 * ║  Each line: 120 heads × 13.9 bottles/min/head                   ║
 * ║  12 ms per bottle. 50 µs control cycle. ±0.25% fill accuracy.   ║
 * ║                                                                  ║
 * ║  Claude, tutored under the hand of Qiyas CC.                    ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include "config/types.h"
#include "config/machine_config.h"
#include "application/line_controller.h"
#include <array>

namespace bottling {

// ─────────────────────────────────────────────────────────────────────────────
// FACILITY CONTROLLER — coordinates 3 parallel lines + MES interface
// ─────────────────────────────────────────────────────────────────────────────
class FacilityController {
public:
    FacilityController() = default;

    /**
     * @brief Initialize all three lines with their hardware.
     *        Called once at system startup after EtherCAT bus scan completes.
     */
    Result initialize(std::array<application::LineHardware, config::kNumParallelLines>& hw) {
        for (uint8_t i = 0; i < config::kNumParallelLines; ++i) {
            if (!hw[i].validate()) {
                return Result::err(ErrorCode::kParameterOutOfRange);
            }

            lines_[i] = new (&line_storage_[i]) application::LineController(i, hw[i]);
            lines_[i]->init();
        }

        initialized_ = true;
        return Result::ok();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // RT TASK ENTRY POINTS — registered with TwinCAT task scheduler
    //
    // TwinCAT Configuration:
    //   Task 1: "Motion_Sync"    — 250 µs cycle, Priority 1, CPU Core 1
    //   Task 2: "Fill_Control"   — 500 µs cycle, Priority 2, CPU Core 2
    //   Task 3: "Sequence_Logic" — 4 ms cycle,   Priority 3, CPU Core 3
    //   Task 4: "HMI_OpcUA"     — 50 ms cycle,  Priority 10, any core
    // ═════════════════════════════════════════════════════════════════════════

    /// 250 µs — servo synchronization for all lines
    void on_motion_cycle() {
        if (!initialized_) return;
        for (uint8_t i = 0; i < config::kNumParallelLines; ++i) {
            if (lines_[i]) lines_[i]->task_motion();
        }
    }

    /// 500 µs — fill control + reject tracking for all lines
    void on_fill_cycle() {
        if (!initialized_) return;
        for (uint8_t i = 0; i < config::kNumParallelLines; ++i) {
            if (lines_[i]) lines_[i]->task_fill_control();
        }
    }

    /// 4 ms — sequence logic + PackML for all lines
    void on_sequence_cycle() {
        if (!initialized_) return;
        for (uint8_t i = 0; i < config::kNumParallelLines; ++i) {
            if (lines_[i]) lines_[i]->task_sequence();
        }
        update_facility_stats();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // FACILITY-LEVEL COMMANDS — broadcast to all lines
    // ═════════════════════════════════════════════════════════════════════════

    Result start_all() {
        for (auto* line : lines_) {
            if (line) {
                auto r = line->cmd_start();
                if (!r.has_value()) return r;
            }
        }
        return Result::ok();
    }

    Result stop_all() {
        for (auto* line : lines_) {
            if (line) line->cmd_stop();
        }
        return Result::ok();
    }

    Result abort_all() {
        for (auto* line : lines_) {
            if (line) line->cmd_abort();
        }
        return Result::ok();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // FACILITY STATS — aggregated across all lines
    // ═════════════════════════════════════════════════════════════════════════

    struct FacilityStats {
        uint32_t total_bottles_good     = 0;
        uint32_t total_bottles_rejected = 0;
        float    aggregate_bpm          = 0.0f;
        float    target_bpm             = static_cast<float>(config::kBottlesPerMinuteTotal);
        float    efficiency_percent     = 0.0f;

        struct PerLine {
            middleware::PackMLState state = middleware::PackMLState::Aborted;
            float                  bpm   = 0.0f;
            application::SafetyState safety = application::SafetyState::CommunicationLoss;
        };
        std::array<PerLine, config::kNumParallelLines> lines{};
    };

    [[nodiscard]] const FacilityStats& facility_stats() const { return facility_stats_; }

    application::LineController* line(uint8_t index) {
        return (index < config::kNumParallelLines) ? lines_[index] : nullptr;
    }

private:
    void update_facility_stats() {
        facility_stats_.total_bottles_good = 0;
        facility_stats_.total_bottles_rejected = 0;
        facility_stats_.aggregate_bpm = 0.0f;

        for (uint8_t i = 0; i < config::kNumParallelLines; ++i) {
            if (!lines_[i]) continue;

            const auto& ls = lines_[i]->stats();
            facility_stats_.total_bottles_good += ls.bottles_good;
            facility_stats_.total_bottles_rejected += ls.bottles_rejected;
            facility_stats_.aggregate_bpm += ls.current_bpm;

            facility_stats_.lines[i].state  = lines_[i]->state();
            facility_stats_.lines[i].bpm    = ls.current_bpm;
            facility_stats_.lines[i].safety = lines_[i]->safety_state();
        }

        facility_stats_.efficiency_percent =
            (facility_stats_.target_bpm > 0.0f)
            ? (facility_stats_.aggregate_bpm / facility_stats_.target_bpm) * 100.0f
            : 0.0f;
    }

    bool initialized_ = false;

    // Static storage — no heap
    alignas(application::LineController) uint8_t line_storage_
        [config::kNumParallelLines][sizeof(application::LineController)]{};
    application::LineController* lines_[config::kNumParallelLines]{};

    FacilityStats facility_stats_;
};

}  // namespace bottling

// ═════════════════════════════════════════════════════════════════════════════
// PROGRAM ENTRY (simulation mode)
// ═════════════════════════════════════════════════════════════════════════════
#ifdef BOTTLING_SIMULATION

int main() {
    // In simulation mode, mock HAL drivers would be injected here.
    // See test/ directory for full simulation harness.
    //
    // Production mode: TcCOM module init replaces main().
    // The TwinCAT runtime calls CycleUpdate() on our registered tasks.

    bottling::FacilityController facility;

    // ... simulation loop would go here ...
    // For production TcCOM deployment, there is no main() —
    // the TwinCAT ObjectServer instantiates our module.

    return 0;
}

#endif  // BOTTLING_SIMULATION
