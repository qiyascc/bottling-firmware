/**
 * @file types.h
 * @brief Foundation types for a no-exceptions, no-RTTI embedded system.
 *
 * Expected<T,E> replaces exceptions. ErrorCode replaces errno.
 * Every function that can fail returns Expected. Period.
 *
 * Claude, tutored under the hand of Qiyas CC.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

namespace bottling {

// ─────────────────────────────────────────────────────────────────────────────
// ERROR CODES — exhaustive, categorized, no overlap
// ─────────────────────────────────────────────────────────────────────────────
enum class ErrorCode : uint16_t {
    kOk                         = 0x0000,

    // Hardware faults [0x01xx]
    kSensorTimeout              = 0x0100,
    kSensorOutOfRange           = 0x0101,
    kEncoderFault               = 0x0102,
    kServoDriveFault            = 0x0103,
    kServoFollowingError        = 0x0104,
    kValveStuck                 = 0x0105,
    kEtherCATLinkDown           = 0x0106,
    kWatchdogExpired            = 0x0107,

    // Fill control faults [0x02xx]
    kOverfill                   = 0x0200,
    kUnderfill                  = 0x0201,
    kNoBottleDetected           = 0x0202,
    kFlowMeterDrift             = 0x0203,
    kFillTimeout                = 0x0204,
    kPIDSaturation              = 0x0205,

    // Safety faults [0x03xx]
    kEmergencyStop              = 0x0300,
    kGuardDoorOpen              = 0x0301,
    kSafetyRelayFault           = 0x0302,
    kSILDiagnosticFail          = 0x0303,
    kSafeTorqueOffActive        = 0x0304,

    // State machine errors [0x04xx]
    kInvalidTransition          = 0x0400,
    kCommandRejected            = 0x0401,
    kAbortRequired              = 0x0402,

    // Configuration errors [0x05xx]
    kRecipeInvalid              = 0x0500,
    kParameterOutOfRange        = 0x0501,
    kBufferFull                 = 0x0502,
    kBufferEmpty                = 0x0503,

    // CIP errors [0x06xx]
    kCIP_TempOutOfRange         = 0x0600,
    kCIP_ConductivityHigh       = 0x0601,
    kCIP_FlowBelowMinimum       = 0x0602,
};

constexpr bool is_critical(ErrorCode e) {
    const auto v = static_cast<uint16_t>(e);
    return (v >= 0x0300 && v <= 0x03FF);  // safety faults are always critical
}

constexpr const char* to_string(ErrorCode e) {
    switch (e) {
        case ErrorCode::kOk:                    return "OK";
        case ErrorCode::kSensorTimeout:         return "SENSOR_TIMEOUT";
        case ErrorCode::kOverfill:              return "OVERFILL";
        case ErrorCode::kUnderfill:             return "UNDERFILL";
        case ErrorCode::kEmergencyStop:         return "E_STOP";
        case ErrorCode::kGuardDoorOpen:         return "GUARD_DOOR_OPEN";
        case ErrorCode::kInvalidTransition:     return "INVALID_TRANSITION";
        case ErrorCode::kBufferFull:            return "BUFFER_FULL";
        case ErrorCode::kBufferEmpty:           return "BUFFER_EMPTY";
        default:                                return "UNKNOWN_ERROR";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Expected<T, E> — Rust-style Result for C++ without exceptions
// ─────────────────────────────────────────────────────────────────────────────
template<typename T, typename E = ErrorCode>
class Expected {
    union {
        T   value_;
        E   error_;
    };
    bool has_value_;

public:
    // Success construction
    Expected(const T& val) : value_(val), has_value_(true) {}                       // NOLINT
    Expected(T&& val) : value_(std::move(val)), has_value_(true) {}                 // NOLINT

    // Error construction — explicit tag to avoid ambiguity
    struct ErrorTag {};
    Expected(ErrorTag, E err) : error_(err), has_value_(false) {}

    // Named constructors for clarity
    static Expected ok(const T& val) { return Expected(val); }
    static Expected ok(T&& val)      { return Expected(std::move(val)); }
    static Expected err(E error)     { return Expected(ErrorTag{}, error); }

    [[nodiscard]] bool     has_value() const { return has_value_; }
    [[nodiscard]] explicit operator bool() const { return has_value_; }

    [[nodiscard]] const T& value() const  { return value_; }   // UB if !has_value — caller must check
    [[nodiscard]] T&       value()        { return value_; }
    [[nodiscard]] E        error() const  { return error_; }

    // Monadic: map, and_then, or_else
    template<typename F>
    auto map(F&& f) const -> Expected<decltype(f(value_)), E> {
        if (has_value_) return Expected<decltype(f(value_)), E>::ok(f(value_));
        return Expected<decltype(f(value_)), E>::err(error_);
    }

    template<typename F>
    T value_or(F&& fallback) const {
        return has_value_ ? value_ : static_cast<T>(fallback);
    }

    ~Expected() {
        if (has_value_)  value_.~T();
        else             error_.~E();
    }

    Expected(const Expected& o) : has_value_(o.has_value_) {
        if (has_value_) new (&value_) T(o.value_);
        else            new (&error_) E(o.error_);
    }

    Expected& operator=(const Expected& o) {
        if (this != &o) {
            this->~Expected();
            new (this) Expected(o);
        }
        return *this;
    }
};

// Specialization for void — operations that succeed or fail, no value
template<typename E>
class Expected<void, E> {
    E    error_;
    bool has_value_;
public:
    Expected() : error_(), has_value_(true) {}                                      // NOLINT
    Expected(E err) : error_(err), has_value_(false) {}                             // NOLINT

    static Expected ok()        { return Expected(); }
    static Expected err(E e)    { return Expected(e); }

    [[nodiscard]] bool has_value() const { return has_value_; }
    [[nodiscard]] explicit operator bool() const { return has_value_; }
    [[nodiscard]] E error() const { return error_; }
};

// Convenience alias
using Result = Expected<void, ErrorCode>;

// ─────────────────────────────────────────────────────────────────────────────
// TIMESTAMP — 64-bit microsecond monotonic clock
// ─────────────────────────────────────────────────────────────────────────────
struct Timestamp {
    uint64_t us;    // microseconds since system boot

    constexpr Timestamp() : us(0) {}
    constexpr explicit Timestamp(uint64_t microseconds) : us(microseconds) {}

    constexpr float     to_ms()  const { return static_cast<float>(us) / 1000.0f; }
    constexpr float     to_sec() const { return static_cast<float>(us) / 1000000.0f; }
    constexpr Timestamp operator-(const Timestamp& o) const { return Timestamp{us - o.us}; }
    constexpr bool      operator>(const Timestamp& o)  const { return us > o.us; }
    constexpr bool      operator<(const Timestamp& o)  const { return us < o.us; }
};

// ─────────────────────────────────────────────────────────────────────────────
// ALARM SEVERITY (IEC 62682)
// ─────────────────────────────────────────────────────────────────────────────
enum class AlarmSeverity : uint8_t {
    kInfo       = 0,
    kWarning    = 1,
    kHigh       = 2,
    kCritical   = 3,   // requires operator acknowledgment
    kEmergency  = 4,   // auto-triggers Abort
};

struct AlarmRecord {
    uint32_t       id;
    ErrorCode      code;
    AlarmSeverity  severity;
    Timestamp      timestamp;
    uint8_t        line_id;
    uint8_t        head_id;
    bool           acknowledged;
    char           message[64];
};

// ─────────────────────────────────────────────────────────────────────────────
// BOTTLE RECORD — travels with the bottle through the entire line
// ─────────────────────────────────────────────────────────────────────────────
struct BottleRecord {
    uint32_t  serial;               // unique production serial
    uint8_t   line_id;              // which parallel line
    uint8_t   head_id;              // which filling head
    float     target_volume_ml;     // from recipe
    float     actual_volume_ml;     // measured by flow meter
    float     actual_weight_g;      // checkweigher (0 if not yet weighed)
    uint32_t  encoder_position;     // position when detected
    Timestamp fill_start;
    Timestamp fill_end;
    bool      reject;               // flagged for rejection
    ErrorCode reject_reason;        // why it was rejected

    constexpr BottleRecord()
        : serial(0), line_id(0), head_id(0),
          target_volume_ml(0), actual_volume_ml(0), actual_weight_g(0),
          encoder_position(0), fill_start(), fill_end(),
          reject(false), reject_reason(ErrorCode::kOk) {}
};

}  // namespace bottling
