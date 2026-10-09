/**
 * @file tasks.h
 * @brief Node A's FreeRTOS tasks (docs/rtos.md).
 *
 *   Task          Priority  Period   Work
 *   CanTxTask     5         2 ms     one CAN schedule slot
 *   ImuTask       4         10 ms    IMU sample, heading and magnetometer check
 *   ControlTask   3         20 ms    GPS input, safety logic and autopilot commands;
 *                                    flight-data records every 100 ms
 *   LoadTask      2         -        idle; a CPU burst for the priority-inversion demo
 *   LogTask       1         10 ms    flash recorder, console, reports, watchdog
 *
 * LogTask, the lowest priority, reloads the watchdog only while every periodic
 * task has completed a cycle since the last reload, so a stalled or starved
 * task resets the node (HLR-016).
 */
#ifndef TASKS_H
#define TASKS_H

/** Creates the tasks. The scheduler is started separately with vTaskStartScheduler(). */
void tasks_create(void);

#endif /* TASKS_H */
