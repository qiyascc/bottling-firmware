# High-Speed Water Bottling Conveyor Control System

**5000 BPM | 3× Parallel Lines | Beckhoff TwinCAT 3 + EtherCAT | C++17**

*Claude, tutored under the hand of Qiyas CC.*

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    FACILITY CONTROLLER                          │
│                  (MES / OPC UA Interface)                        │
├──────────────────┬──────────────────┬──────────────────────────┤
│   LINE 0         │   LINE 1         │   LINE 2                 │
│   1667 BPM       │   1667 BPM       │   1667 BPM               │
│                  │                  │                          │
│ ┌──────────────┐ │ ┌──────────────┐ │ ┌──────────────┐         │
│ │  PackML FSM  │ │ │  PackML FSM  │ │ │  PackML FSM  │         │
│ │  (17 states) │ │ │  (17 states) │ │ │  (17 states) │         │
│ ├──────────────┤ │ ├──────────────┤ │ ├──────────────┤         │
│ │  Conveyor    │ │ │  Conveyor    │ │ │  Conveyor    │         │
│ │  Controller  │ │ │  Controller  │ │ │  Controller  │         │
│ │  (cam sync)  │ │ │  (cam sync)  │ │ │  (cam sync)  │         │
│ ├──────────────┤ │ ├──────────────┤ │ ├──────────────┤         │
│ │ Fill Ctrl ×120│ │ │ Fill Ctrl ×120│ │ │ Fill Ctrl ×120│        │
│ │ (cascade PID)│ │ │ (cascade PID)│ │ │ (cascade PID)│         │
│ ├──────────────┤ │ ├──────────────┤ │ ├──────────────┤         │
│ │  Reject Ctrl │ │ │  Reject Ctrl │ │ │  Reject Ctrl │         │
│ │  (shift reg) │ │ │  (shift reg) │ │ │  (shift reg) │         │
│ ├──────────────┤ │ ├──────────────┤ │ ├──────────────┤         │
│ │ Safety Monitor│ │ │ Safety Monitor│ │ │ Safety Monitor│        │
│ │  (SIL 2)     │ │ │  (SIL 2)     │ │ │  (SIL 2)     │        │
│ ├──────────────┤ │ ├──────────────┤ │ ├──────────────┤         │
│ │ CIP Ctrl     │ │ │ CIP Ctrl     │ │ │ CIP Ctrl     │         │
│ └──────────────┘ │ └──────────────┘ │ └──────────────┘         │
└──────────────────┴──────────────────┴──────────────────────────┘
                          │
              ┌───────────┴───────────┐
              │   EtherCAT Fieldbus   │
              │   50 µs cycle time    │
              │   ±20 ns DC sync      │
              └───────────────────────┘
```

## Task Priority Map

| Priority | Task              | Cycle     | Function                        |
|----------|-------------------|-----------|---------------------------------|
| HW IRQ   | E-Stop            | < 10 µs  | Safety interlock (bare metal)   |
| RT 1     | Motion Sync       | 250 µs   | Servo cam sync, star wheels     |
| RT 2     | Fill Control      | 500 µs   | PID, valve timing, reject track |
| RT 3     | Sequence Logic    | 4 ms     | PackML states, CIP, diagnostics |
| BG       | HMI / OPC UA      | 50 ms    | Operator interface, MES comms   |

## Timing Budget

```
5000 bottles/min ÷ 60 = 83.3 bottles/sec
→ 12.0 ms per bottle

Per carousel (120 heads, 1667 BPM):
  Carousel rotation: 4320 ms (13.9 RPM)
  Fill arc (270°):   3240 ms available per head
  500 mL ÷ 250 mL/s = 2000 ms fill time
  → 1240 ms margin for centering + settling + drip
```

## File Structure (3,410 lines)

```
bottling-firmware/
├── config/
│   ├── machine_config.h     ← All constexpr parameters (129 lines)
│   └── types.h              ← Expected<T,E>, ErrorCode, records (232 lines)
├── hal/
│   └── hal_interfaces.h     ← Pure hardware interfaces (254 lines)
├── middleware/
│   ├── static_containers.h  ← Ring buffer, SPSC queue, bit-shift reg (283 lines)
│   ├── pid_controller.h     ← Cascade PID with anti-windup (192 lines)
│   └── packml_fsm.h         ← ISA-TR88 17-state machine (301 lines)
├── application/
│   ├── fill_controller.h    ← Per-head multi-phase fill (371 lines)
│   ├── conveyor_controller.h ← Cam sync + zone speed ctrl (262 lines)
│   ├── reject_controller.h  ← Encoder-tracked rejection (168 lines)
│   ├── safety_monitor.h     ← SIL 2 safety layer (147 lines)
│   ├── cip_controller.h     ← Clean-In-Place automation (244 lines)
│   └── line_controller.h    ← Main orchestrator (520 lines)
├── main.cpp                 ← Facility controller + entry (199 lines)
└── CMakeLists.txt           ← Build system (78 lines)
```

## Key Design Decisions

1. **Zero heap allocation** — All containers statically sized. `StaticRingBuffer`, `SPSCQueue`, `BitShiftRegister`, `MemoryPool` replace STL heap containers.

2. **Lock-free inter-task** — SPSC queues with `std::atomic` + acquire/release ordering. Cache-line aligned to prevent false sharing.

3. **Cascade PID** — Outer loop (volume) → inner loop (flow rate) → valve. Back-calculation anti-windup. Derivative-on-measurement. Bumpless transfer.

4. **PackML compliant** — Full ISA-TR88 17-state machine with PackTags for OEE. Abort reachable from any state.

5. **Encoder-driven reject** — Bit-shift register shifts on each encoder pulse. No timing-based approximation. Position-accurate to ±0.1 mm.

6. **HAL abstraction** — Every hardware interaction through interfaces. Mocks injectable for host-based unit testing without target hardware.

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Standards Compliance

- **PackML** ISA-TR88.00.02-2022 — 17 states, PackTags
- **IEC 62061** SIL 2 — Safety monitoring layer
- **FDA 21 CFR 129** — Water bottling equipment requirements
- **FDA 21 CFR Part 11** — Electronic records architecture
- **MISRA C++:2023** — Static analysis profile (requires Helix QAC)
- **C++17** — `-fno-exceptions -fno-rtti -Werror`
