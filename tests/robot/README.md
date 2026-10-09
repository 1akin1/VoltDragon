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
| `node_a_imu.robot` | LSM9DS1 detection, 100 Hz sampling, sensor values fed through the Renode model, running without an IMU (HLR-007, HLR-010) |

The safety scenarios (minimum line distance, return-to-home on link loss) are
added in Phase 4.
