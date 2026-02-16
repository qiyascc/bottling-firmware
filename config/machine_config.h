/**
 * @file machine_config.h
 * @brief Compile-time machine parameters — every magic number lives here.
 *
 * Claude, tutored under the hand of Qiyas CC.
 *
 * RULE: If you hardcode a number anywhere else, you failed the code review.
 *       Every physical dimension, timing constraint, and threshold is constexpr.
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace bottling::config {

// ─────────────────────────────────────────────────────────────────────────────
// LINE TOPOLOGY
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr uint8_t  kNumParallelLines          = 3;
inline constexpr uint16_t kBottlesPerMinuteTotal     = 5000;
inline constexpr uint16_t kBottlesPerMinutePerLine   = 1667;  // ceil(5000/3)
inline constexpr float    kCycleTimePerBottleMs      = 60000.0f / kBottlesPerMinuteTotal;  // 12.0 ms

// ─────────────────────────────────────────────────────────────────────────────
// ROTARY CAROUSEL GEOMETRY
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr uint8_t  kFillingHeadsPerCarousel    = 120;
inline constexpr float    kCarouselPitchDiameterMm    = 4320.0f;
inline constexpr float    kCarouselCircumferenceMm    = kCarouselPitchDiameterMm * 3.14159265f;
inline constexpr float    kCarouselRPM                = static_cast<float>(kBottlesPerMinutePerLine)
                                                        / kFillingHeadsPerCarousel;  // ~13.9 RPM
inline constexpr float    kFillingArcDegrees          = 270.0f;
inline constexpr float    kFillingTimePerHeadMs       = (kFillingArcDegrees / 360.0f)
                                                        * (60000.0f / kCarouselRPM);  // ~3240 ms

// ─────────────────────────────────────────────────────────────────────────────
// CONVEYOR PHYSICAL PARAMETERS
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr float    kBottlePitchMm              = 80.0f;
inline constexpr float    kConveyorSpeedMmPerSec      = (kBottlesPerMinuteTotal / 60.0f) * kBottlePitchMm;
                                                        // 6666.7 mm/s ≈ 6.67 m/s
inline constexpr float    kBottleVolumeMl             = 500.0f;
inline constexpr float    kBottleWeightEmptyG          = 12.5f;   // typical 0.5L PET
inline constexpr float    kBottleWeightFullG           = 512.5f;  // 500 mL water = 500 g

// ─────────────────────────────────────────────────────────────────────────────
// ENCODER CONFIGURATION
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr uint32_t kEncoderResolutionPPR       = 8192;    // 13-bit absolute
inline constexpr float    kEncoderMmPerPulse          = 0.1f;    // after gear ratio
inline constexpr uint32_t kPulsesPerBottlePitch       = static_cast<uint32_t>(
                                                            kBottlePitchMm / kEncoderMmPerPulse);  // 800

// ─────────────────────────────────────────────────────────────────────────────
// FILL CONTROL PARAMETERS
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr float    kMaxFlowRateMlPerSec        = 250.0f;
inline constexpr float    kSlowFillThresholdPercent   = 0.90f;   // switch to slow fill at 90%
inline constexpr float    kSlowFillRateMlPerSec       = 50.0f;
inline constexpr float    kFillToleranceMl            = 2.5f;    // ±0.5% of 500 mL
inline constexpr float    kDripCompensationMl         = 0.8f;    // learned nozzle drip volume
inline constexpr float    kFillTargetMl               = kBottleVolumeMl - kDripCompensationMl;
inline constexpr float    kOverfillRejectThresholdMl  = kBottleVolumeMl + (2.0f * kFillToleranceMl);
inline constexpr float    kUnderfillRejectThresholdMl = kBottleVolumeMl - 15.0f;  // EU ℮-mark TNE

// ─────────────────────────────────────────────────────────────────────────────
// PID TUNING (cascade: outer=volume, inner=flow rate)
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr float    kPID_Volume_Kp              = 2.5f;
inline constexpr float    kPID_Volume_Ki              = 0.8f;
inline constexpr float    kPID_Volume_Kd              = 0.05f;
inline constexpr float    kPID_Flow_Kp                = 4.0f;
inline constexpr float    kPID_Flow_Ki                = 1.2f;
inline constexpr float    kPID_Flow_Kd                = 0.02f;
inline constexpr float    kPID_OutputMin               = 0.0f;
inline constexpr float    kPID_OutputMax               = 100.0f;  // valve opening %
inline constexpr float    kPID_AntiWindupTrackingTc    = 0.1f;    // back-calc time constant

// ─────────────────────────────────────────────────────────────────────────────
// REJECT SYSTEM
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr float    kRejectStationDistanceMm    = 1500.0f;
inline constexpr uint32_t kRejectDelayPulses          = static_cast<uint32_t>(
                                                            kRejectStationDistanceMm / kEncoderMmPerPulse);
inline constexpr uint16_t kAirBlastDurationMs          = 25;
inline constexpr uint16_t kMaxBottlesInTransit         = static_cast<uint16_t>(
                                                            kRejectStationDistanceMm / kBottlePitchMm) + 4;

// ─────────────────────────────────────────────────────────────────────────────
// TASK TIMING (TwinCAT real-time tasks)
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr uint32_t kMotionTaskCycleUs           = 250;     // 250 µs — servo sync
inline constexpr uint32_t kFillTaskCycleUs             = 500;     // 500 µs — valve + PID
inline constexpr uint32_t kSequenceTaskCycleMs         = 4;       // 4 ms — PackML logic
inline constexpr uint32_t kHmiTaskCycleMs              = 50;      // 50 ms — OPC UA + HMI
inline constexpr uint32_t kDiagnosticsTaskCycleMs      = 200;     // 200 ms — logging

// ─────────────────────────────────────────────────────────────────────────────
// SAFETY
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr uint16_t kWatchdogTimeoutMs           = 50;
inline constexpr uint16_t kEStopResponseTimeUs         = 10;      // hardware ISR
inline constexpr uint8_t  kMaxConsecutiveFaults        = 3;       // before line halt
inline constexpr float    kSafeSpeedMmPerSec           = 200.0f;  // SLS for maintenance

// ─────────────────────────────────────────────────────────────────────────────
// STATIC BUFFER SIZES (no heap allocation!)
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr size_t   kAlarmQueueCapacity          = 256;
inline constexpr size_t   kEventLogCapacity            = 1024;
inline constexpr size_t   kRecipeSlotCount             = 32;
inline constexpr size_t   kBottleTrackingFIFOSize      = 512;
inline constexpr size_t   kSPSCQueueCapacity           = 256;     // must be power-of-2

// ─────────────────────────────────────────────────────────────────────────────
// CIP (Clean-In-Place) PARAMETERS
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr float    kCIP_PreRinseTempC           = 45.0f;
inline constexpr uint16_t kCIP_PreRinseDurationSec     = 300;     // 5 min
inline constexpr float    kCIP_CausticConcentration    = 1.5f;    // % NaOH
inline constexpr float    kCIP_CausticTempC            = 80.0f;
inline constexpr uint16_t kCIP_CausticDurationSec      = 1200;    // 20 min
inline constexpr float    kCIP_AcidConcentration       = 0.8f;    // % HNO₃
inline constexpr float    kCIP_MinFlowVelocityMPerSec  = 1.5f;    // turbulent cleaning
inline constexpr float    kCIP_FinalRinseConductivity  = 20.0f;   // µS/cm — matches water

}  // namespace bottling::config
