# Node A on FreeRTOS

Node A runs FreeRTOS V11.1.0 (submodule `third_party/freertos`, ARM_CM4F port)
since Phase 4. Node B stays bare metal: a main loop that polls the network,
the CAN controller and the command UART is all it needs.

Implementation: [`tasks.c`](../firmware/node_a/src/tasks.c),
[`rtos/FreeRTOSConfig.h`](../firmware/node_a/src/rtos/FreeRTOSConfig.h),
[`rtos/rtos_port.c`](../firmware/node_a/src/rtos/rtos_port.c),
[`rtos/lock.h`](../firmware/node_a/src/rtos/lock.h).

## Tasks

| Task | Priority | Period | Work |
|------|----------|--------|------|
| CanTxTask | 5 (highest) | 2 ms | One CAN slot of the 20 ms schedule; receives Node B's frames ([can-messages.md](can-messages.md)) |
| ImuTask | 4 | 10 ms | IMU sample (100 Hz) and heading filter |
| ControlTask | 3 | 20 ms | GPS sentences, safety logic and autopilot orders ([autopilot-link.md](autopilot-link.md)), flight-data records at 10 Hz |
| LoadTask | 2 | - | Idle; only used by the priority-inversion demonstration |
| LogTask | 1 | 10 ms | Debug console, flash recorder writes, watchdog supervision, the 1 s report |
| Idle | 0 | - | `WFI` until the next interrupt |

Rate-monotonic order: the shorter the period, the higher the priority. The
periodic tasks use `xTaskDelayUntil`, so their periods do not drift with their
execution time. The tick is 1 kHz.

The task planned as AiTask (line detection) arrives with Phase 5.

## Design decisions

- **Static allocation only** (`configSUPPORT_DYNAMIC_ALLOCATION 0`): every task
  control block, stack and mutex is a static object, so memory use is fixed at
  link time and visible in the map file. There is no heap.
- **Interrupts stay out of the kernel.** The UART receive interrupts (debug
  console, GPS, autopilot) only fill ring buffers and run at priority 0, above
  `configMAX_SYSCALL_INTERRUPT_PRIORITY`, so the kernel never masks them; they
  call no FreeRTOS function. The tasks drain the buffers.
- **One SysTick.** The existing 1 ms SysTick keeps the system time
  (`systick_now_ms`) and calls the kernel's tick handler through a hook once the
  scheduler runs (`configOVERRIDE_DEFAULT_TICK_CONFIGURATION`), so the two
  clocks cannot disagree.
- **Locks** (`lock.h`) are statically allocated FreeRTOS mutexes; they are
  no-ops before the scheduler starts, so the same driver code runs during
  boot. Shared state is published as copies under a lock: the IMU sample,
  the GPS fix, the navigation state, the safety state, the flight recorder's
  queue, and the debug log (one lock, so lines from different tasks do not
  interleave; the log does not lock in interrupts or fault handlers).
- **Failure handling.** `configASSERT` and the stack-overflow hook
  (`configCHECK_FOR_STACK_OVERFLOW 2`) log the failure and reset the node,
  which the reset-cause log then shows (HLR-016).

## Watchdog supervision

The independent watchdog (500 ms) is reloaded by LogTask, the lowest-priority
task, and only if CanTxTask, ImuTask and ControlTask have each completed a
cycle since the previous reload (checked every 100 ms). A stalled, starved or
crashed task, or an overloaded CPU that never lets LogTask run, therefore
resets the node. The debug key `k` suspends ControlTask to show this
(`tests/robot/node_a_rtos.robot`).

## Stack margins

Each task has 512 words (2 KiB) of stack. The high-water marks are reported
every 10 s; in the mission co-simulation the free words were:

| CanTxTask | ImuTask | ControlTask | LogTask |
|-----------|---------|-------------|---------|
| 432 | 419 | 396 | 316 |

The test suite requires at least 100 free words in each.

## Priority inversion

The flight recorder's queue is shared by ControlTask (priority 3), which
queues a record every 100 ms and checks the queue every cycle, and LogTask
(priority 1), which writes the records to flash. If LogTask holds the lock and
a medium-priority task becomes ready, LogTask stops running, and ControlTask,
waiting for the lock, is blocked by a task that does not even use it: an
unbounded priority inversion (the Mars Pathfinder failure).

The demonstration (debug keys `i` and `m`) reproduces it: LogTask holds the
recorder lock for 5 ms of "flash work", and at that moment LoadTask
(priority 2) starts a 100 ms CPU burst.

| Recorder lock | ControlTask's longest wait | Control deadlines missed |
|---------------|----------------------------|--------------------------|
| Binary semaphore (`i`), no priority inheritance | 85 ms | 3 |
| Mutex (`m`), priority inheritance | 5 ms | 0 |

With the semaphore, ControlTask waits for the rest of LoadTask's burst. With
the mutex, LogTask inherits ControlTask's priority while it holds the lock,
finishes its 5 ms of work ahead of LoadTask, and ControlTask waits only for the
critical section itself, as it should. The recorder uses the mutex; the
semaphore exists only for the demonstration, which restores the mutex
afterwards. Both cases are checked in `tests/robot/node_a_rtos.robot`.

A deadline miss is a control cycle starting more than 5 ms late. The burst
also delays the watchdog reload (LogTask gets no CPU during it), which is why
it is limited to 100 ms: the worst-case gap between reloads, two 100 ms checks
plus the burst, stays well under the 500 ms timeout.
