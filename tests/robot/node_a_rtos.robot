*** Comments ***
Phase 4: Node A runs on FreeRTOS. The watchdog is reloaded only while every
periodic task keeps running (HLR-016), and the recorder lock's priority
inheritance prevents priority inversion (docs/rtos.md).

Priority-inversion demo: low-priority LogTask holds the recorder lock while
medium-priority LoadTask runs a 100 ms CPU burst, and high-priority
ControlTask needs the lock every 20 ms.

Run from the repository root (inside WSL):
    renode-test tests/robot/node_a_rtos.robot


*** Settings ***
Resource            ${RENODEKEYWORDS}

Suite Setup         Setup
Suite Teardown      Teardown
Test Setup          Reset Emulation
Test Teardown       Test Teardown


*** Variables ***
${ROOT}             ${CURDIR}/../..
${ELF}              ${ROOT}/build/debug/firmware/node_a/node_a.elf
${BOOT_BANNER}      VoltDragon Node A v


*** Keywords ***
Create Node A
    Execute Command    $bin_a=@${ELF}
    Execute Command    include @${ROOT}/renode/node_a.resc
    Create Terminal Tester    sysbus.usart2    defaultPauseEmulation=True
    Execute Command    sysbus.i2c1.imu AccelerationZ 1
    Wait For Line On Uart    rtos: FreeRTOS V11.1.0, starting the scheduler

Expect
    [Arguments]    ${text}    ${timeout}=2
    Wait For Line On Uart    ${text}    timeout=${timeout}    treatAsRegex=true


*** Test Cases ***
Should Run The Tasks Under FreeRTOS
    Create Node A
    # Each task's work shows in the 1 s report: IMU at 100 Hz, CAN slots, recorder at 10 Hz.
    Wait For Line On Uart    heartbeat 2    timeout=2.2
    Expect    imu: rate 100 Hz
    Expect    can: tx \\d+/s, dropped 0
    Expect    flashlog: 10 rec/s

Should Report Task Stack Margins
    Create Node A
    # Every tenth report; at least 100 of the 512 stack words left in every task.
    Expect    rtos: free stack words: can \\d{3}, imu \\d{3}, control \\d{3}, log \\d{3}    timeout=11

Should Show Priority Inversion With A Plain Semaphore
    Create Node A
    Wait For Line On Uart    heartbeat 1
    Write Char On Uart    i
    Expect    recorder lock is a binary semaphore
    # ControlTask is blocked for most of LoadTask's burst and misses deadlines.
    Expect    ControlTask waited up to ([5-9]\\d|\\d{3,}) ms for the recorder lock, [1-9]\\d* control deadlines missed

Should Bound The Wait With Priority Inheritance
    Create Node A
    Wait For Line On Uart    heartbeat 1
    Write Char On Uart    m
    Expect    recorder lock is a mutex
    # Only LogTask's own 5 ms under the lock remains.
    Expect    ControlTask waited up to [0-6] ms for the recorder lock, 0 control deadlines missed

Should Recover From A Stalled Task Through The Watchdog
    Create Node A
    Wait For Line On Uart    heartbeat 1
    Write Char On Uart    k
    Wait For Line On Uart    suspending ControlTask
    # The other tasks keep running, but the watchdog is no longer reloaded (500 ms timeout).
    Should Not Be On Uart    ${BOOT_BANNER}    timeout=0.4
    Wait For Line On Uart    ${BOOT_BANNER}    timeout=0.4
    Wait For Line On Uart    reset count: 1
