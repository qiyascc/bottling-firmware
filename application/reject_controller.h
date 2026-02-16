/**
 * @file reject_controller.h
 * @brief Encoder-driven bottle rejection with dual tracking:
 *        bit-shift register for position accuracy + FIFO for data.
 *
 * The bottle is flagged at the filler. It travels 1500mm down the conveyor.
 * At 6.67 m/s, that's 225 ms of travel time. Miss the reject window by
 * 1 ms and the bad bottle is in someone's case pack. Unacceptable.
 *
 * Claude, tutored under the hand of Qiyas CC.
 */

#pragma once

#include "../config/types.h"
#include "../config/machine_config.h"
#include "../hal/hal_interfaces.h"
#include "../middleware/static_containers.h"
#include <cstdint>

namespace bottling::application {

// ─────────────────────────────────────────────────────────────────────────────
// REJECT STATISTICS
// ─────────────────────────────────────────────────────────────────────────────
struct RejectStats {
    uint32_t total_rejects       = 0;
    uint32_t overfill_rejects    = 0;
    uint32_t underfill_rejects   = 0;
    uint32_t timeout_rejects     = 0;
    uint32_t sensor_fault_rejects = 0;
    uint32_t missed_rejects      = 0;   // fired but confirmator didn't see ejection
    uint32_t consecutive_rejects = 0;   // reset on good bottle

    void record(ErrorCode reason) {
        ++total_rejects;
        ++consecutive_rejects;

        switch (reason) {
            case ErrorCode::kOverfill:       ++overfill_rejects; break;
            case ErrorCode::kUnderfill:      ++underfill_rejects; break;
            case ErrorCode::kFillTimeout:    ++timeout_rejects; break;
            case ErrorCode::kSensorTimeout:
            case ErrorCode::kFlowMeterDrift: ++sensor_fault_rejects; break;
            default: break;
        }
    }

    void record_good() { consecutive_rejects = 0; }

    bool excessive() const {
        return consecutive_rejects >= config::kMaxConsecutiveFaults;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// REJECT CONTROLLER
// ─────────────────────────────────────────────────────────────────────────────
class RejectController {
public:
    RejectController(hal::IEncoder& conveyor_encoder,
                     hal::IRejectActuator& reject_actuator,
                     hal::IPhotoSensor& entry_sensor,
                     hal::IPhotoSensor& reject_confirm_sensor,
                     hal::IClock& clock)
        : encoder_(conveyor_encoder)
        , actuator_(reject_actuator)
        , entry_sensor_(entry_sensor)
        , confirm_sensor_(reject_confirm_sensor)
        , clock_(clock)
        , last_encoder_pos_(0)
        , armed_(false) {}

    /**
     * @brief Called every fill control cycle.
     *        Shifts the tracking register by encoder delta and fires rejects.
     */
    void update() {
        if (!armed_) return;

        const uint32_t current_pos = encoder_.position();
        const uint32_t delta = current_pos - last_encoder_pos_;
        last_encoder_pos_ = current_pos;

        // Shift the bit register by the number of encoder pulses since last call.
        // Each shift = one encoder pulse = 0.1mm of conveyor travel.
        for (uint32_t i = 0; i < delta; ++i) {
            const bool should_fire = shift_register_.shift();
            if (should_fire) {
                fire_reject();
            }
        }

        // Also shift out bottle records from the data FIFO based on
        // the confirmator sensor seeing bottles pass
        check_confirmation();
    }

    /**
     * @brief Inject a bottle into the tracking system.
     *
     * @param record   Complete bottle data from the fill controller
     *
     * Called when a bottle exits the filler onto the rejection conveyor.
     * The shift register tracks POSITION, the FIFO tracks DATA.
     */
    void inject_bottle(const BottleRecord& record) {
        // Inject reject flag into bit position 0
        shift_register_.inject(record.reject);

        // Store full record in FIFO for data retrieval at reject station
        (void)bottle_fifo_.push(record);
    }

    void arm()    { armed_ = true; last_encoder_pos_ = encoder_.position(); }
    void disarm() { armed_ = false; shift_register_.clear(); bottle_fifo_.clear(); }

    [[nodiscard]] const RejectStats& stats() const { return stats_; }
    [[nodiscard]] bool excessive_rejects() const { return stats_.excessive(); }

    void reset_stats() { stats_ = RejectStats{}; }

private:
    void fire_reject() {
        if (actuator_.is_ready()) {
            const auto result = actuator_.fire();
            if (result.has_value()) {
                // Pop the bottle record from FIFO
                BottleRecord record{};
                if (bottle_fifo_.pop(record)) {
                    stats_.record(record.reject_reason);
                }
            }
        }
    }

    void check_confirmation() {
        // The confirmator sensor sits just after the reject station.
        // If we fired a reject and the sensor STILL sees a bottle,
        // the reject failed → log a missed reject alarm.
        if (confirm_sensor_.rising_edge() && pending_reject_confirm_) {
            // Bottle is still there after reject → missed
            ++stats_.missed_rejects;
            pending_reject_confirm_ = false;
        }
    }

    // ── Members ──────────────────────────────────────────────
    hal::IEncoder&          encoder_;
    hal::IRejectActuator&   actuator_;
    hal::IPhotoSensor&      entry_sensor_;
    hal::IPhotoSensor&      confirm_sensor_;
    hal::IClock&            clock_;

    // Position tracking: bit-shift register sized to reject station distance.
    // Each bit = one encoder pulse. Register length = distance / resolution.
    BitShiftRegister<config::kRejectDelayPulses + 64>  shift_register_;

    // Data tracking: FIFO of bottle records between filler and reject station
    StaticRingBuffer<BottleRecord, config::kBottleTrackingFIFOSize>  bottle_fifo_;

    uint32_t     last_encoder_pos_;
    bool         armed_;
    bool         pending_reject_confirm_ = false;
    RejectStats  stats_;
};

}  // namespace bottling::application
