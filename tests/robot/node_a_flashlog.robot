*** Comments ***
Phase 2: Node A records flight data to the MT25Q SPI flash at 10 Hz (HLR-018),
keeps it across resets, and keeps running when the flash is missing.

The flash model starts erased for every test, like a new part. IMU values
include the Renode model gain described in node_a_imu.robot.

Run from the repository root (inside WSL):
    renode-test tests/robot/node_a_flashlog.robot


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
${IMU}              sysbus.i2c1.imu
${MAG}              sysbus.i2c1.mag
${FLASH_INFO}       flashlog: MT25Q 16384 KiB,


*** Keywords ***
Create Node A
    [Arguments]    ${platform}=${ROOT}/renode/node_a.repl
    Execute Command    $bin_a=@${ELF}
    Execute Command    $platform_a=@${platform}
    Execute Command    include @${ROOT}/renode/node_a.resc
    Create Terminal Tester    ${UART}    defaultPauseEmulation=True

Run For
    [Arguments]    ${seconds}
    Execute Command    emulation RunFor "${seconds}"

Dump Log
    Write Char On Uart    d
    Wait For Line On Uart    flashlog: last


*** Test Cases ***
Should Find An Empty Flash On First Boot
    Create Node A
    Wait For Line On Uart    ${FLASH_INFO} 0 records found, 262144 free

Should Record At 10 Hz Without Losses
    Create Node A
    Wait For Line On Uart    heartbeat 1
    Wait For Line On Uart    flashlog: 10 rec/s, total 10, dropped 0, errors 0
    Wait For Line On Uart    flashlog: 10 rec/s, total 20, dropped 0, errors 0    timeout=1.1

Should Record The Boot And The Imu Data
    Create Node A
    Execute Command    ${IMU} AccelerationX -0.5
    Execute Command    ${IMU} AccelerationY 0.25
    Execute Command    ${IMU} AccelerationZ 1
    Execute Command    ${IMU} AngularRateX 10
    Execute Command    ${IMU} AngularRateY -100
    Execute Command    ${MAG} MagneticX 0.25
    Execute Command    ${MAG} MagneticY -0.1
    Execute Command    ${MAG} MagneticZ -0.4
    Wait For Line On Uart    node A keys
    Run For    0.25
    Dump Log
    Wait For Line On Uart    log #0 0.001 BOOT cause POWER_ON, count 0
    # IMU records follow every 100 ms; the period starts once initialisation has finished.
    Wait For Line On Uart    log #1 0\\.10\\d IMU acc -500 250 999 mg, gyro 10500 -105000 0 mdps, mag 287 -115 -459 mG    treatAsRegex=true
    Wait For Line On Uart    log #2 0\\.20\\d IMU acc -500 250 999 mg    treatAsRegex=true

Should Keep The Log Across A Reset
    Create Node A
    Wait For Line On Uart    flashlog: 10 rec/s, total 20
    Write Char On Uart    r
    Wait For Line On Uart    reset cause: SOFTWARE, reset count: 1
    # Records written before the reset are found again and new ones are appended after them.
    Wait For Line On Uart    ${FLASH_INFO} 2\\d records found    treatAsRegex=true
    Run For    0.05
    Dump Log
    Wait For Line On Uart    BOOT cause SOFTWARE, count 1

Should Cross Erase Boundaries Without Errors
    Create Node A
    # 140 records span three 4 KiB subsectors (64 records each), so two more erases happen on the way.
    Wait For Line On Uart    flashlog: 10 rec/s, total 140, dropped 0, errors 0    timeout=15
    Dump Log
    Wait For Line On Uart    log #136
    Should Not Be On Uart    corrupt    timeout=0.1

Should Keep Running Without A Flash
    Create Node A    platform=${ROOT}/renode/stm32f407.repl
    Wait For Line On Uart    flashlog: init failed
    Wait For Line On Uart    heartbeat 2
    Should Not Be On Uart    flashlog: 10 rec/s    timeout=1.1
