/**
 * @file safety_monitor.h
 * @brief SIL 2 / PLd safety monitoring layer.
 *
 * This module does NOT implement the safety PLC logic itself —
 * that runs on a certified safety controller (TwinSAFE, PROFIsafe, etc.)
 * with its own certified firmware.
 *
 * This module MONITORS safety states from the safety PLC via FSoE (Fail Safe
 * over EtherCAT) and triggers appropriate application-level responses.
 *
 * RULE: Safety logic shall never be "optimized." Clarity over cleverness.
 *       A safety auditor reads this at 2 AM during a commissioning night.
 *       Make it obvious.
 *
 * Claude, tutored under the hand of Qiyas CC.
 */

#pragma once

#include "../config/types.h"
#include "../config/machine_config.h"
#include "../hal/hal_interfaces.h"
#include <cstdint>

namespace bottling::application {

// ─────────────────────────────────────────────────────────────────────────────
// SAFETY STATE — mirrors the safety PLC's assessment
// ─────────────────────────────────────────────────────────────────────────────
enum class SafetyState : uint8_t {
    AllClear,           // all safety circuits OK, machine may run
    GuardOpen,          // access guard open → SLS or STO depending on zone
    EStopActive,        // E-stop pressed → STO on all drives
    LightCurtainTrip,   // light curtain interrupted → SS1 then STO
    SafetyRelayFault,   // safety relay diagnostic failure → immediate STO
    CommunicationLoss,  // FSoE/PROFIsafe timeout → STO (fail-safe)
};

// ─────────────────────────────────────────────────────────────────────────────
// SAFE MOTION FUNCTION — IEC 61800-5-2
// ─────────────────────────────────────────────────────────────────────────────
enum class SafeMotionFunction : uint8_t {
    None,
    STO,     // Safe Torque Off — immediate power removal
    SS1,     // Safe Stop 1 — controlled decel then STO
    SS2,     // Safe Stop 2 — controlled decel, monitoring continues
    SLS,     // Safely Limited Speed — max speed enforced
    SOS,     // Safe Operating Stop — standstill monitoring
};

// ─────────────────────────────────────────────────────────────────────────────
// SAFETY MONITOR
// ─────────────────────────────────────────────────────────────────────────────
class SafetyMonitor {
public:
    using EmergencyCallback = void(*)(SafetyState reason);

    explicit SafetyMonitor(hal::ISafetyInput& safety_input,
                           hal::IClock& clock)
        : safety_input_(safety_input)
        , clock_(clock)
        , state_(SafetyState::CommunicationLoss)   // assume unsafe until proven otherwise
        , callback_(nullptr)
        , last_heartbeat_()
        , heartbeat_timeout_us_(config::kWatchdogTimeoutMs * 1000ULL) {}

    /**
     * @brief Called every scan cycle (highest priority after hardware ISR).
     *        Reads all safety inputs and determines required response.
     */
    void update() {
        const Timestamp now = clock_.now();
        const SafetyState previous = state_;

        // ── Priority-ordered evaluation (highest danger first) ──

        // 1. Communication health (FSoE heartbeat)
        if ((now - last_heartbeat_).us > heartbeat_timeout_us_) {
            state_ = SafetyState::CommunicationLoss;
            required_function_ = SafeMotionFunction::STO;
        }
        // 2. Safety relay diagnostic
        else if (!safety_input_.safety_relay_ok()) {
            state_ = SafetyState::SafetyRelayFault;
            required_function_ = SafeMotionFunction::STO;
        }
        // 3. E-stop
        else if (safety_input_.e_stop_active()) {
            state_ = SafetyState::EStopActive;
            required_function_ = SafeMotionFunction::STO;
        }
        // 4. Light curtain
        else if (!safety_input_.light_curtain_clear()) {
            state_ = SafetyState::LightCurtainTrip;
            required_function_ = SafeMotionFunction::SS1;
        }
        // 5. Guard door
        else if (!safety_input_.guard_door_closed()) {
            state_ = SafetyState::GuardOpen;
            required_function_ = SafeMotionFunction::SLS;  // allow slow maintenance
        }
        // 6. All clear
        else {
            state_ = SafetyState::AllClear;
            required_function_ = SafeMotionFunction::None;
        }

        // Fire callback on state change
        if (state_ != previous && callback_) {
            callback_(state_);
        }
    }

    /// Called when a valid FSoE frame is received — resets heartbeat
    void heartbeat_received() {
        last_heartbeat_ = clock_.now();
    }

    void set_emergency_callback(EmergencyCallback cb) { callback_ = cb; }

    // ── Accessors ────────────────────────────────────────────
    [[nodiscard]] SafetyState       state() const          { return state_; }
    [[nodiscard]] SafeMotionFunction required_function() const { return required_function_; }
    [[nodiscard]] bool              is_safe_to_run() const { return state_ == SafetyState::AllClear; }
    [[nodiscard]] bool              requires_sto() const {
        return required_function_ == SafeMotionFunction::STO;
    }
    [[nodiscard]] float safe_speed_limit() const {
        if (required_function_ == SafeMotionFunction::SLS) {
            return config::kSafeSpeedMmPerSec;
        }
        return 0.0f;  // STO or SS1 → no movement allowed
    }

private:
    hal::ISafetyInput&  safety_input_;
    hal::IClock&        clock_;

    SafetyState          state_;
    SafeMotionFunction   required_function_ = SafeMotionFunction::STO;
    EmergencyCallback    callback_;
    Timestamp            last_heartbeat_;
    uint64_t             heartbeat_timeout_us_;
};

}  // namespace bottling::application
