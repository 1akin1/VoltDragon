# Robot Framework tests

Renode-driven system tests. Build the firmware first, then run from the
repository root:

```sh
renode-test -r build/robot tests/robot/*.robot
```

| Suite | Covers |
|-------|--------|
| `node_a_boot.robot` | Boot banner, heartbeat, fault capture across reset, watchdog recovery, reset counter (HLR-016, HLR-017) |

The safety scenarios (minimum line distance, return-to-home on link loss) are
added in Phase 4.
