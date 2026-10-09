*** Comments ***
Phase 2: Node A reads the LSM9DS1 IMU over I2C at 100 Hz (HLR-007) and keeps
running, without IMU data, when the sensor does not respond (HLR-010).

Renode's LSM9DS1 model scales its outputs by fixed counts per unit (accel
16384 LSB/g, gyro 120 LSB/dps, mag 8192 LSB/gauss) instead of the datasheet
sensitivities the firmware uses (0.061 mg, 8.75 mdps, 0.14 mgauss per LSB).
Readings are therefore expected to be 5 % high for the gyroscope and 14.7 %
high for the magnetometer; the expected values below include that gain.

Run from the repository root (inside WSL):
    renode-test tests/robot/node_a_imu.robot


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


*** Keywords ***
Create Node A
    [Arguments]    ${platform}=${ROOT}/renode/node_a.repl
    Execute Command    $bin_a=@${ELF}
    Execute Command    $platform_a=@${platform}
    Execute Command    include @${ROOT}/renode/node_a.resc
    Create Terminal Tester    ${UART}    defaultPauseEmulation=True

Feed Imu
    [Arguments]    ${ax}    ${ay}    ${az}    ${gx}    ${gy}    ${gz}    ${mx}    ${my}    ${mz}
    Execute Command    ${IMU} AccelerationX ${ax}
    Execute Command    ${IMU} AccelerationY ${ay}
    Execute Command    ${IMU} AccelerationZ ${az}
    Execute Command    ${IMU} AngularRateX ${gx}
    Execute Command    ${IMU} AngularRateY ${gy}
    Execute Command    ${IMU} AngularRateZ ${gz}
    Execute Command    ${MAG} MagneticX ${mx}
    Execute Command    ${MAG} MagneticY ${my}
    Execute Command    ${MAG} MagneticZ ${mz}


*** Test Cases ***
Should Detect The Imu At Boot
    Create Node A
    Wait For Line On Uart    imu: LSM9DS1 ready, sampling every 10 ms

Should Sample The Imu At 100 Hz
    Create Node A
    # The first window includes the sample taken at start-up, so check the following ones.
    Wait For Line On Uart    heartbeat 1
    Wait For Line On Uart    imu: rate 100 Hz    timeout=1.1
    Wait For Line On Uart    imu: rate 100 Hz    timeout=1.1

Should Report The Sensor Values Fed To The Model
    Create Node A
    Wait For Line On Uart    imu: LSM9DS1 ready
    Feed Imu    -0.5    0.25    1    10    -100    0    0.25    -0.1    -0.4
    # Accel: 1 g reads 999 mg. Gyro: 10 dps * 1.05 = 10500 mdps. Mag: 0.25 gauss * 1.147 = 287 mG.
    Wait For Line On Uart    acc -500 250 999 mg, gyro 10500 -105000 0 mdps, mag 287 -115 -459 mG

Should Track Changing Sensor Values
    Create Node A
    Feed Imu    0    0    1    0    0    0    0    0    0
    Wait For Line On Uart    acc 0 0 999 mg, gyro 0 0 0 mdps, mag 0 0 0 mG
    Feed Imu    0    0    -1    20    0    0    0    0    1
    Wait For Line On Uart    acc 0 0 -999 mg, gyro 21000 0 0 mdps, mag 0 0 1147 mG

Should Keep Running Without An Imu
    Create Node A    platform=${ROOT}/renode/stm32f407.repl
    Wait For Line On Uart    imu: init failed (BUS, i2c NACK), running without IMU
    Wait For Line On Uart    heartbeat 2
    Should Not Be On Uart    imu: rate    timeout=1.1
