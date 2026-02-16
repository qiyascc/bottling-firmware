/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║                                                                            ║
 * ║   HIGH-SPEED WATER BOTTLING CONVEYOR CONTROL SYSTEM                        ║
 * ║   Target: Beckhoff TwinCAT 3 / EtherCAT — 5000 BPM (3x parallel lines)    ║
 * ║                                                                            ║
 * ║   Platform  : Beckhoff C6030 IPC + AX8000 Servo + EK1100 I/O              ║
 * ║   Fieldbus  : EtherCAT (50 µs cycle, Distributed Clocks ±20 ns)           ║
 * ║   Standard  : C++17 (-fno-exceptions -fno-rtti)                            ║
 * ║   Compliance: PackML ISA-TR88, IEC 62061 SIL 2, FDA 21 CFR 129/11         ║
 * ║   MISRA     : MISRA C++:2023 — safety-critical subset enforced             ║
 * ║                                                                            ║
 * ║   Claude, tutored under the hand of Qiyas CC.                              ║
 * ║                                                                            ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 *
 * TIMING BUDGET (per bottle):
 *   5000 BPM → 12.0 ms per bottle
 *   Motion sync task  : 250 µs cycle (RT priority 1)
 *   Fill control task  : 500 µs cycle (RT priority 2)
 *   Sequence logic task: 4 ms cycle   (RT priority 3)
 *
 * MEMORY POLICY:
 *   Zero dynamic allocation after init. All containers statically sized.
 *   No exceptions. No RTTI. Error propagation via Expected<T,E>.
 *
 * BUILD:
 *   Compiler: MSVC (TwinCAT TcCOM) or GCC/Clang cross-compile
 *   Flags: -std=c++17 -fno-exceptions -fno-rtti -O2 -Wall -Wextra -Werror
 */
