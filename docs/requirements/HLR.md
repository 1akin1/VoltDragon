# High-Level Requirements (HLR)

Status: **Draft v0.1** · Phase 0

These requirements describe the externally observable behaviour of the VoltDragon
system. Each one is traced down to Low-Level Requirements (LLR), code and tests in
the traceability matrix produced in Phase 6, following a DO-178C-style process.

**Verification methods:** **T** = Test (Robot Framework / Renode or pytest),
**A** = Analysis, **I** = Inspection.

## Flight safety

| ID | Requirement | Rationale | Verif. |
|----|-------------|-----------|--------|
| HLR-001 | The system shall not let the UAV get closer than **10 m** to the power line. | Prevents collision and electrical arcing. | T |
| HLR-002 | When the estimated line distance drops below **12 m**, the system shall raise a `PROXIMITY_WARNING` within **100 ms** and command a lateral move away from the line. | A 2 m buffer gives the controller time to react before HLR-001 is violated. | T |
| HLR-003 | If the ground link is lost for more than **3 s**, the system shall enter `RETURN_TO_HOME` mode. | A link-loss failsafe is mandatory for BVLOS-style inspection. | T |
| HLR-004 | If battery state of charge falls below **20 %**, the system shall enter `RETURN_TO_HOME` mode, and below **10 %** it shall enter `LAND` mode. | Ensures enough energy to recover the aircraft. | T |
| HLR-005 | Mode transitions shall follow only the transitions defined in the flight-mode state machine; any undefined transition request shall be rejected and logged. | Deterministic, analysable behaviour. | T, A |
| HLR-006 | A ground-station command shall not override an active safety mode (`RETURN_TO_HOME`, `LAND`) unless the operator sends an explicit override command. | Prevents unsafe commands while a failsafe is active. | T |

## Sensing and fault detection

| ID | Requirement | Rationale | Verif. |
|----|-------------|-----------|--------|
| HLR-007 | Node A shall sample the IMU at **100 Hz** or faster. | Enough bandwidth for attitude estimation and vibration analysis. | T, A |
| HLR-008 | Node A shall detect magnetometer readings that are corrupted by the power line's magnetic field and shall stop using magnetometer heading while the corruption lasts. | Strong fields near high-voltage lines corrupt compass readings. | T |
| HLR-009 | Node A shall detect motor/propeller vibration faults by on-board inference and shall report a `VIBRATION_FAULT` alarm to the ground station within **2 s** of fault onset. | Early fault detection with Edge AI. | T |
| HLR-010 | Each sensor sample shall be range- and staleness-checked, and invalid data shall be flagged and excluded from control. | Robustness against sensor failures. | T |

## Communication

| ID | Requirement | Rationale | Verif. |
|----|-------------|-----------|--------|
| HLR-011 | Node A shall send sensor and status data to Node B over CAN at **50 Hz** or faster. | Inter-node data path. | T |
| HLR-012 | Node B shall forward telemetry to the ground station as UDP packets at **10 Hz** or faster. | Real-time monitoring. | T |
| HLR-013 | Every CAN frame and telemetry packet shall carry a sequence counter and a CRC, and the receiver shall discard corrupted frames and count lost ones. | Data integrity and link-quality monitoring. | T |
| HLR-014 | Node B shall accept operator commands over its UART command interface and over UDP, and shall acknowledge or reject each command. | Operator control and auditability. | T |
| HLR-015 | Node B may output selected parameters (altitude, heading, line distance) as ARINC 429 words. *(Optional)* | Shows avionics bus integration. | T |

## Robustness and recording

| ID | Requirement | Rationale | Verif. |
|----|-------------|-----------|--------|
| HLR-016 | Each node shall be supervised by a hardware watchdog and shall recover to a safe state after a watchdog reset. | Recovery from software hangs. | T |
| HLR-017 | Each node shall keep a reset counter and the last fault cause across resets, and shall report them in telemetry at start-up. | Post-incident diagnosis. | T |
| HLR-018 | Node A shall record flight data, mode changes and alarms to SPI flash at **10 Hz** or faster. | Flight data recorder for post-flight analysis. | T |
| HLR-019 | The ground station shall show the UAV position on a map, live telemetry plots and active alarms. | Operator situational awareness. | I, T |
| HLR-020 | All flight software shall comply with the project's MISRA C:2012 subset, and every deviation shall be documented. | Coding-standard compliance. | A |

## Open points

- The limits (10 m, 3 s, 20 %) are engineering assumptions for the simulation, not
  values taken from a regulation.
- Timing requirements are verified in Renode, which is not cycle-accurate. They must
  be re-verified on real hardware (see README, *To be verified in the hardware phase*).
- HLR-009 does not say how small a fault must be detected. Phase 5 set the limit at
  severity 0.3 of the plant's fault model (0.075 g of rotor imbalance at hover), after a
  soak test showed that weaker faults overlap with a rough but healthy airframe
  ([edge-ai.md](../edge-ai.md)). A future revision of HLR-009 should state this limit
  and a maximum false-alarm rate.
