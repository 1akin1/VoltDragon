# Robot Framework tests

Renode-driven system tests. Build the firmware first, then run from the
repository root:

```sh
renode-test -r build/robot tests/robot/*.robot
```

| Suite | Covers |
|-------|--------|
| `node_a_boot.robot` | Boot banner, heartbeat, fault capture across reset, watchdog recovery, reset counter (HLR-016, HLR-017) |
| `node_a_flashlog.robot` | Flight data recorder: 10 Hz recording, boot and IMU records, log kept across resets, erase boundaries, running without flash (HLR-018) |
| `node_b_commands.robot` | Operator command interface on USART3: ACK/NAK for every command, corrupted frames, resynchronisation, retransmission, command bursts (HLR-014) |
| `system_can.robot` | Both nodes on one CAN bus: 50 Hz delivery, corrupted and lost frames, hardware filtering, Node A restart, link statistics (HLR-011, HLR-013) |
| `system_telemetry.robot` | Phase 2 milestone: UDP telemetry checked byte by byte on the wire, Node A data reaching the ground station, rate change, commands over UDP, running without Ethernet (HLR-012, HLR-013, HLR-014) |
| `node_a_imu.robot` | LSM9DS1 detection, 100 Hz sampling, sensor values fed through the Renode model, running without an IMU (HLR-007, HLR-010) |
| `node_a_nav.robot` | GPS (NMEA on UART4): fix parsing, corrupted sentences, no fix; magnetometer heading, disturbance detection and recovery, GPS course fallback (HLR-008, HLR-010) |

The co-simulated mission, with the plant model driving both nodes, is a pytest
suite in `tests/integration` (see [simulation.md](../../docs/simulation.md)).

The safety scenarios (minimum line distance, return-to-home on link loss) are
added in Phase 4.
