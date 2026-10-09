*** Comments ***
Phase 1 milestone: Node A boots in Renode, logs to UART, and recovers from
faults and hangs while keeping its reset history (HLR-016, HLR-017).

Run from the repository root (inside WSL):
    renode-test tests/robot/node_a_boot.robot


*** Settings ***
Resource            ${RENODEKEYWORDS}

Suite Setup         Setup
Suite Teardown      Teardown
Test Setup          Reset Emulation
Test Teardown       Test Teardown


*** Variables ***
${ROOT}             ${CURDIR}/../..
${ELF}              ${ROOT}/build/debug/firmware/node_a/node_a.elf
${UART}             sysbus.usart2
${BOOT_BANNER}      VoltDragon Node A v


*** Keywords ***
Create Node A
    Execute Command    $bin_a=@${ELF}
    Execute Command    include @${ROOT}/renode/node_a.resc
    Create Terminal Tester    ${UART}    defaultPauseEmulation=True
    Wait For Line On Uart    ${BOOT_BANNER}

Wait Until Ready
    Wait For Line On Uart    debug keys:

Trigger Fault And Expect Report
    [Arguments]    ${key}    ${cause}
    Wait Until Ready
    Write Char On Uart    ${key}
    Wait For Line On Uart    *** FAULT ***
    Wait For Line On Uart    cause: ${cause}
    Wait For Line On Uart    resetting
    # The next boot must report what ended the previous run.
    Wait For Line On Uart    ${BOOT_BANNER}
    Wait For Line On Uart    reset cause: SOFTWARE, reset count: 1
    Wait For Line On Uart    previous run ended with a fault
    Wait For Line On Uart    cause: ${cause}


*** Test Cases ***
Should Boot And Report A Cold Start
    Create Node A
    Wait For Line On Uart    reset cause: POWER_ON, reset count: 0
    Wait For Line On Uart    watchdog started, timeout 500 ms

Should Print A Heartbeat Every Second
    Create Node A
    Wait For Line On Uart    I heartbeat 1    timeout=1.1
    Wait For Line On Uart    I heartbeat 2    timeout=1.1

Should Record An Undefined Instruction Fault Across Reset
    Create Node A
    Trigger Fault And Expect Report    f    UNDEFINSTR FORCED

Should Record A Divide By Zero Fault Across Reset
    Create Node A
    Trigger Fault And Expect Report    z    DIVBYZERO FORCED

Should Record An Invalid State Fault Across Reset
    Create Node A
    Trigger Fault And Expect Report    s    INVSTATE FORCED

Should Reset Through The Watchdog When The Main Loop Hangs
    Create Node A
    Wait Until Ready
    Write Char On Uart    w
    Wait For Line On Uart    hanging main loop
    # Nominal timeout is 500 ms; allow margin, and require that it is not instant.
    Should Not Be On Uart    ${BOOT_BANNER}    timeout=0.4
    Wait For Line On Uart    ${BOOT_BANNER}    timeout=0.3
    # Renode does not model the RCC reset flags, so a watchdog reset is reported as UNKNOWN.
    Wait For Line On Uart    reset cause: UNKNOWN, reset count: 1

Should Count Consecutive Software Resets
    Create Node A
    Wait Until Ready
    Write Char On Uart    r
    Wait For Line On Uart    reset cause: SOFTWARE, reset count: 1
    Wait Until Ready
    Write Char On Uart    r
    Wait For Line On Uart    reset cause: SOFTWARE, reset count: 2
