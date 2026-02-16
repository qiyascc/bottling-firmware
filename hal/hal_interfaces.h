/**
 * @file hal_interfaces.h
 * @brief Hardware Abstraction Layer — pure interfaces, zero implementation.
 *
 * Every hardware interaction in this codebase goes through these interfaces.
 * Production code provides real drivers. Tests inject mocks.
 * You touch a register directly in application code? Fired.
 *
 * Claude, tutored under the hand of Qiyas CC.
 */

#pragma once

#include "../config/types.h"
#include <cstdint>

namespace bottling::hal {

// ─────────────────────────────────────────────────────────────────────────────
// SYSTEM CLOCK
// ─────────────────────────────────────────────────────────────────────────────
class IClock {
public:
    virtual ~IClock() = default;
    virtual Timestamp now() const = 0;
    virtual uint64_t  ticks() const = 0;      // raw hardware ticks
    virtual uint32_t  tick_freq_hz() const = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// GPIO — individual pin control
// ─────────────────────────────────────────────────────────────────────────────
class IGpio {
public:
    virtual ~IGpio() = default;
    virtual bool read(uint16_t pin) const = 0;
    virtual void write(uint16_t pin, bool state) = 0;
    virtual void toggle(uint16_t pin) = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// ANALOG INPUT — flow meters, pressure transducers, temperature
// ─────────────────────────────────────────────────────────────────────────────
class IAnalogInput {
public:
    virtual ~IAnalogInput() = default;

    /// Read raw ADC value [0..65535 for 16-bit]
    virtual Expected<uint16_t> read_raw(uint8_t channel) const = 0;

    /// Read scaled engineering units (mL/s, bar, °C, etc.)
    virtual Expected<float>    read_scaled(uint8_t channel) const = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// ANALOG OUTPUT — valve position commands
// ─────────────────────────────────────────────────────────────────────────────
class IAnalogOutput {
public:
    virtual ~IAnalogOutput() = default;

    /// Write raw DAC value [0..65535]
    virtual Result write_raw(uint8_t channel, uint16_t value) = 0;

    /// Write scaled engineering units (0..100%)
    virtual Result write_scaled(uint8_t channel, float value) = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// ENCODER — high-speed position counting (hardware counter)
// ─────────────────────────────────────────────────────────────────────────────
class IEncoder {
public:
    virtual ~IEncoder() = default;

    /// Current position in encoder counts (32-bit rollover-safe)
    virtual uint32_t position() const = 0;

    /// Current velocity in counts/second
    virtual int32_t  velocity() const = 0;

    /// Reset position counter to zero
    virtual void     reset() = 0;

    /// Check for hardware fault (wire break, signal loss)
    virtual bool     is_healthy() const = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// FLOW METER — electromagnetic (E+H Promag or equivalent)
// ─────────────────────────────────────────────────────────────────────────────
class IFlowMeter {
public:
    virtual ~IFlowMeter() = default;

    /// Instantaneous flow rate in mL/sec
    virtual Expected<float> flow_rate() const = 0;

    /// Totalizer: accumulated volume since last reset, in mL
    virtual Expected<float> totalizer() const = 0;

    /// Reset totalizer to zero (at start of fill)
    virtual Result          reset_totalizer() = 0;

    /// Diagnostic: coil excitation healthy
    virtual bool            is_healthy() const = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// FILLING VALVE — PFR (Proportional Flow Regulator) or pneumatic
// ─────────────────────────────────────────────────────────────────────────────
class IFillingValve {
public:
    virtual ~IFillingValve() = default;

    /// Set valve opening (0.0 = closed, 100.0 = fully open)
    virtual Result set_opening(float percent) = 0;

    /// Emergency close — bypasses ramp, immediate shutoff
    virtual void   emergency_close() = 0;

    /// Current valve position feedback (0..100%)
    virtual Expected<float> current_opening() const = 0;

    /// Is the valve reporting healthy diagnostics?
    virtual bool   is_healthy() const = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// SERVO DRIVE — Beckhoff AX8000 via EtherCAT CSP mode
// ─────────────────────────────────────────────────────────────────────────────
class IServoDrive {
public:
    virtual ~IServoDrive() = default;

    enum class Mode : uint8_t {
        kDisabled,
        kCSP,       // Cyclic Synchronous Position
        kCSV,       // Cyclic Synchronous Velocity
        kCST,       // Cyclic Synchronous Torque
        kHoming,
    };

    struct Status {
        bool    enabled;
        bool    fault;
        bool    target_reached;
        bool    following_error;
        int32_t actual_position;   // encoder counts
        int32_t actual_velocity;   // counts/sec
        int16_t actual_torque;     // 0.1% rated
    };

    virtual Result  enable() = 0;
    virtual Result  disable() = 0;
    virtual Result  set_mode(Mode mode) = 0;

    /// CSP mode: set target position for next cycle
    virtual Result  set_target_position(int32_t counts) = 0;

    /// CSV mode: set target velocity
    virtual Result  set_target_velocity(int32_t counts_per_sec) = 0;

    /// Read current drive status
    virtual Status  status() const = 0;

    /// Acknowledge fault and reset drive
    virtual Result  fault_reset() = 0;

    /// STO — Safe Torque Off (safety function)
    virtual void    safe_torque_off() = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// PHOTOELECTRIC SENSOR — bottle presence detection
// ─────────────────────────────────────────────────────────────────────────────
class IPhotoSensor {
public:
    virtual ~IPhotoSensor() = default;

    /// True if beam is broken (bottle present)
    virtual bool detected() const = 0;

    /// Rising edge: bottle just arrived
    virtual bool rising_edge() = 0;

    /// Falling edge: bottle just left
    virtual bool falling_edge() = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// REJECT MECHANISM — air blast ejector
// ─────────────────────────────────────────────────────────────────────────────
class IRejectActuator {
public:
    virtual ~IRejectActuator() = default;

    /// Fire the reject mechanism for configured duration
    virtual Result fire() = 0;

    /// Is the mechanism currently firing?
    virtual bool   is_active() const = 0;

    /// Ready for next reject (cooldown expired)
    virtual bool   is_ready() const = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// SAFETY INPUT — E-stop, guard doors, light curtains
// ─────────────────────────────────────────────────────────────────────────────
class ISafetyInput {
public:
    virtual ~ISafetyInput() = default;

    virtual bool e_stop_active() const = 0;
    virtual bool guard_door_closed() const = 0;
    virtual bool light_curtain_clear() const = 0;
    virtual bool safety_relay_ok() const = 0;

    /// Combined: all safety conditions met for run
    virtual bool all_safe() const = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// TEMPERATURE SENSOR — CIP monitoring
// ─────────────────────────────────────────────────────────────────────────────
class ITemperatureSensor {
public:
    virtual ~ITemperatureSensor() = default;
    virtual Expected<float> read_celsius() const = 0;
    virtual bool            is_healthy() const = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// CONDUCTIVITY SENSOR — CIP final rinse verification
// ─────────────────────────────────────────────────────────────────────────────
class IConductivitySensor {
public:
    virtual ~IConductivitySensor() = default;
    virtual Expected<float> read_uS_cm() const = 0;  // µS/cm
};

// ─────────────────────────────────────────────────────────────────────────────
// WATCHDOG — hardware watchdog timer
// ─────────────────────────────────────────────────────────────────────────────
class IWatchdog {
public:
    virtual ~IWatchdog() = default;
    virtual void feed() = 0;             // reset countdown
    virtual void enable(uint16_t timeout_ms) = 0;
    virtual void disable() = 0;
};

}  // namespace bottling::hal
