/**
 * @file conveyor_controller.h
 * @brief Conveyor speed control + star wheel electronic cam synchronization.
 *
 * The carousel is the master axis. Everything else gears to it.
 * Infeed star wheel, discharge star wheel, capper, labeler —
 * all phase-locked via electronic cam profiles to the main encoder.
 *
 * At 6.67 m/s belt speed, 0.1mm encoder resolution generates
 * 66,700 pulses/sec. This is why we use hardware counters.
 *
 * Claude, tutored under the hand of Qiyas CC.
 */

#pragma once

#include "../config/types.h"
#include "../config/machine_config.h"
#include "../hal/hal_interfaces.h"
#include <array>
#include <cstdint>
#include <cmath>

namespace bottling::application {

// ─────────────────────────────────────────────────────────────────────────────
// ELECTRONIC CAM PROFILE — master position → slave position
// ─────────────────────────────────────────────────────────────────────────────
struct CamPoint {
    float master_deg;    // master position in degrees (0..360)
    float slave_deg;     // slave position in degrees
};

template<size_t N>
struct CamProfile {
    std::array<CamPoint, N> points{};
    size_t count = 0;

    /**
     * @brief Interpolate slave position for given master position.
     *        Linear interpolation between table points.
     *        This is called every 250 µs — must be fast.
     */
    float interpolate(float master_deg) const {
        if (count < 2) return master_deg;  // 1:1 fallback

        // Normalize to 0..360
        while (master_deg >= 360.0f) master_deg -= 360.0f;
        while (master_deg < 0.0f)    master_deg += 360.0f;

        // Binary search for bracket (table is sorted by master_deg)
        size_t lo = 0, hi = count - 1;
        while (lo + 1 < hi) {
            const size_t mid = (lo + hi) / 2;
            if (points[mid].master_deg <= master_deg) lo = mid;
            else hi = mid;
        }

        // Linear interpolation
        const float m0 = points[lo].master_deg;
        const float m1 = points[hi].master_deg;
        const float s0 = points[lo].slave_deg;
        const float s1 = points[hi].slave_deg;

        const float range = (m1 > m0) ? (m1 - m0) : (m1 + 360.0f - m0);
        const float frac  = (master_deg - m0) / range;

        return s0 + frac * (s1 - s0);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// SLAVE AXIS — one per synchronized element (star wheel, capper, etc.)
// ─────────────────────────────────────────────────────────────────────────────
struct SlaveAxis {
    hal::IServoDrive*  drive    = nullptr;
    CamProfile<64>     cam;
    float              gear_ratio = 1.0f;     // electronic gear ratio to master
    float              phase_offset_deg = 0.0f; // fine-tune alignment
    bool               enabled = false;

    /// Compute target position from master encoder
    int32_t compute_target(float master_deg, int32_t counts_per_rev) const {
        const float slave_deg = cam.interpolate(master_deg + phase_offset_deg);
        return static_cast<int32_t>(
            (slave_deg / 360.0f) * static_cast<float>(counts_per_rev) * gear_ratio
        );
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// CONVEYOR ZONE — independent speed-controlled belt section
// ─────────────────────────────────────────────────────────────────────────────
enum class ZoneState : uint8_t {
    Stopped,
    Accelerating,
    Running,
    Decelerating,
};

struct ConveyorZone {
    hal::IServoDrive*   drive       = nullptr;
    hal::IEncoder*      encoder     = nullptr;
    hal::IPhotoSensor*  entry_eye   = nullptr;
    hal::IPhotoSensor*  exit_eye    = nullptr;

    float  target_speed_mm_sec = 0.0f;
    float  actual_speed_mm_sec = 0.0f;
    float  accel_mm_sec2       = 5000.0f;    // 5 m/s² ramp
    float  decel_mm_sec2       = 8000.0f;    // 8 m/s² ramp (faster stop)
    ZoneState state             = ZoneState::Stopped;

    /// S-curve velocity profile per cycle
    float compute_velocity_step(float dt_sec) const {
        switch (state) {
            case ZoneState::Accelerating: {
                const float step = accel_mm_sec2 * dt_sec;
                return std::min(actual_speed_mm_sec + step, target_speed_mm_sec);
            }
            case ZoneState::Decelerating: {
                const float step = decel_mm_sec2 * dt_sec;
                return std::max(actual_speed_mm_sec - step, 0.0f);
            }
            case ZoneState::Running:
                return target_speed_mm_sec;
            case ZoneState::Stopped:
            default:
                return 0.0f;
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// CONVEYOR CONTROLLER — master orchestrator for all motion
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr size_t kMaxSlaveAxes     = 8;   // star wheels, capper, labeler, etc.
inline constexpr size_t kMaxConveyorZones = 6;   // infeed, fill, outfeed, reject, etc.

class ConveyorController {
public:
    ConveyorController(hal::IEncoder& master_encoder,
                       hal::IClock& clock)
        : master_encoder_(master_encoder)
        , clock_(clock)
        , master_position_deg_(0.0f)
        , line_speed_percent_(0.0f)
        , enabled_(false) {}

    // ── Configuration ────────────────────────────────────────
    void add_slave_axis(size_t index, SlaveAxis axis) {
        if (index < kMaxSlaveAxes) {
            slaves_[index] = axis;
        }
    }

    void add_zone(size_t index, ConveyorZone zone) {
        if (index < kMaxConveyorZones) {
            zones_[index] = zone;
        }
    }

    // ── Motion control cycle (called every 250 µs) ──────────
    void update() {
        if (!enabled_) return;

        const float dt = static_cast<float>(config::kMotionTaskCycleUs) / 1000000.0f;

        // Read master encoder and convert to degrees
        update_master_position();

        // Synchronize all slave axes to master
        for (size_t i = 0; i < kMaxSlaveAxes; ++i) {
            if (slaves_[i].enabled && slaves_[i].drive) {
                const int32_t target = slaves_[i].compute_target(
                    master_position_deg_,
                    config::kEncoderResolutionPPR
                );
                slaves_[i].drive->set_target_position(target);
            }
        }

        // Update conveyor zone speeds
        for (size_t i = 0; i < kMaxConveyorZones; ++i) {
            auto& zone = zones_[i];
            if (!zone.drive) continue;

            zone.actual_speed_mm_sec = zone.compute_velocity_step(dt);

            // Convert mm/s to encoder counts/s for the drive
            const int32_t vel_counts = static_cast<int32_t>(
                zone.actual_speed_mm_sec / config::kEncoderMmPerPulse
            );
            zone.drive->set_target_velocity(vel_counts);
        }
    }

    // ── Line speed control ───────────────────────────────────
    void set_line_speed(float percent) {
        line_speed_percent_ = std::clamp(percent, 0.0f, 100.0f);
        const float speed_factor = line_speed_percent_ / 100.0f;

        for (size_t i = 0; i < kMaxConveyorZones; ++i) {
            zones_[i].target_speed_mm_sec = config::kConveyorSpeedMmPerSec * speed_factor;
        }
    }

    void ramp_up() {
        for (auto& z : zones_) z.state = ZoneState::Accelerating;
    }

    void ramp_down() {
        for (auto& z : zones_) z.state = ZoneState::Decelerating;
    }

    void emergency_stop() {
        for (auto& z : zones_) {
            z.state = ZoneState::Stopped;
            z.target_speed_mm_sec = 0.0f;
            z.actual_speed_mm_sec = 0.0f;
            if (z.drive) z.drive->safe_torque_off();
        }
        for (auto& s : slaves_) {
            if (s.drive) s.drive->safe_torque_off();
        }
        enabled_ = false;
    }

    void enable()  { enabled_ = true; }
    void disable() { enabled_ = false; ramp_down(); }

    // ── Accessors ────────────────────────────────────────────
    [[nodiscard]] float master_position_deg() const { return master_position_deg_; }
    [[nodiscard]] float line_speed_percent() const  { return line_speed_percent_; }
    [[nodiscard]] bool  is_at_speed() const {
        for (const auto& z : zones_) {
            if (z.drive && std::abs(z.actual_speed_mm_sec - z.target_speed_mm_sec) > 10.0f) {
                return false;
            }
        }
        return true;
    }

private:
    void update_master_position() {
        const uint32_t raw = master_encoder_.position();
        master_position_deg_ = (static_cast<float>(raw % config::kEncoderResolutionPPR)
                                / static_cast<float>(config::kEncoderResolutionPPR))
                               * 360.0f;
    }

    hal::IEncoder&  master_encoder_;
    hal::IClock&    clock_;

    float master_position_deg_;
    float line_speed_percent_;
    bool  enabled_;

    std::array<SlaveAxis, kMaxSlaveAxes>       slaves_{};
    std::array<ConveyorZone, kMaxConveyorZones> zones_{};
};

}  // namespace bottling::application
