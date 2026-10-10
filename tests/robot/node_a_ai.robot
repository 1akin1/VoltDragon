*** Comments ***
Phase 5: Node A's vibration classifier, an INT8 model on TensorFlow Lite Micro
in AiTask (docs/edge-ai.md). A damaged propeller's signature is queued into the
LSM9DS1 model with vibration_feed (renode/plant_feed.py); the alarm must be
raised within 2 s (HLR-009), recorded in the flight-data recorder (HLR-018)
and cleared once the vibration stops.

The co-simulated flights (tests/integration) check the same with the plant
model's vibration, through to the ground station.

Run from the repository root (inside WSL):
    renode-test tests/robot/node_a_ai.robot


*** Settings ***
Resource            ${RENODEKEYWORDS}

Suite Setup         Setup
Suite Teardown      Teardown
Test Setup          Reset Emulation
Test Teardown       Test Teardown


*** Variables ***
${ROOT}             ${CURDIR}/../..
${ELF}              ${ROOT}/build/debug/firmware/node_a/node_a.elf


*** Keywords ***
Create Node A
    Execute Command    $bin_a=@${ELF}
    Execute Command    include @${ROOT}/renode/node_a.resc
    Execute Command    include @${ROOT}/renode/plant_feed.py
    Create Terminal Tester    sysbus.usart2    defaultPauseEmulation=True
    Execute Command    sysbus.i2c1.imu AccelerationZ 1
    Wait For Line On Uart    rtos: FreeRTOS V11.1.0, starting the scheduler

Expect
    [Arguments]    ${text}    ${timeout}=2
    Wait For Line On Uart    ${text}    timeout=${timeout}    treatAsRegex=true


*** Test Cases ***
Should Load The Model At Start-Up
    Execute Command    $bin_a=@${ELF}
    Execute Command    include @${ROOT}/renode/node_a.resc
    Create Terminal Tester    sysbus.usart2    defaultPauseEmulation=True
    Expect    ai: vibration classifier ready, tensor arena \\d+ bytes; a window every 320 ms

Should Classify A Healthy Airframe As Nominal
    Create Node A
    # Three windows per second, all nominal; features and inference well within a window.
    Wait For Line On Uart    heartbeat 3    timeout=3.2
    Expect    ai: active, alarm nominal, last nominal \\d+ %, windows [6-9] \\(fault 0\\), inference \\d+ us .*overruns 0, errors 0

Should Raise The Alarm For A Damaged Propeller Within 2 s
    Create Node A
    Wait For Line On Uart    heartbeat 1
    # 0.25 g and 4 deg/s at the 20 Hz alias of an 80 Hz rotor: a severity 1 imbalance.
    Execute Command    vibration_feed "0.25" "4.0" "20" "6"
    Expect    ai: VIBRATION FAULT: imbalance \\(\\d+ %\\)    timeout=2
    Expect    ai: active, alarm imbalance, last imbalance

Should Clear The Alarm When The Vibration Stops
    Create Node A
    Wait For Line On Uart    heartbeat 1
    Execute Command    vibration_feed "0.25" "4.0" "20" "3"
    Expect    ai: VIBRATION FAULT: imbalance
    # The queue runs dry after 3 s; eight nominal windows (2.56 s) later the alarm clears.
    Expect    ai: vibration alarm cleared    timeout=6

Should Record Alarm Changes In The Flight Recorder
    Create Node A
    Wait For Line On Uart    heartbeat 1
    Execute Command    vibration_feed "0.25" "4.0" "20" "3"
    # The dump shows the last five records (half a second of IMU records at 10 Hz),
    # so it is taken right after each change.
    Expect    ai: VIBRATION FAULT: imbalance
    Write Char On Uart    d
    Expect    log #\\d+ [\\d.]+ VIBRATION alarm imbalance \\(\\d+ %\\) after \\d+ windows
    Expect    ai: vibration alarm cleared    timeout=6
    Write Char On Uart    d
    Expect    log #\\d+ [\\d.]+ VIBRATION alarm nominal
