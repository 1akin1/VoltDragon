# Co-simulation helpers: feed the Python plant model's data into the nodes.
#
# Load with:  include @renode/plant_feed.py
#
#   plant_feed "<samples>" "<gps hex>" "<mag>" "<autopilot hex>"     (Node A selected)
#
# <samples>:       ';'-separated accelerometer and gyroscope samples, each
#                  'ax,ay,az,gx,gy,gz' in g and deg/s. They are queued in the LSM9DS1
#                  model, which hands out one sample per firmware read, so a whole
#                  co-simulation step is fed at once (ahead of time).
# <gps hex>:       bytes for UART4 (NMEA sentences) as hex, or '-' for none.
# <mag>:           'mx,my,mz' in gauss, set as the magnetometer's current value. The
#                  magnetometer model ignores queued samples, so it is updated once per step.
# <autopilot hex>: bytes from the autopilot for UART5 ($VDAPS) as hex, or '-' for none.
#
#   vibration_feed "<accel g>" "<gyro dps>" "<frequency Hz>" "<seconds>"     (Node A selected)
#
# Queues a damaged propeller's signature for the vibration classifier tests: a
# tone rotating in the body x/y plane (accelerometer and gyroscope) on top of
# 1 g, at 100 samples per second. The rotor's ~80 Hz arrives aliased at 100 Hz
# sampling, so the frequency given is the aliased one (e.g. 20 Hz).
#
#   uart_write "<uart>" "<hex>"        (any machine selected)
#
# Writes bytes into a UART of the selected machine, e.g. the ground station's
# heartbeat into Node B's command interface: uart_write "sysbus.usart3" "24312c...".
#
# Renode turns functions named mc_<name> into monitor commands; this runs in
# Renode's IronPython 2.7.

import math

import System
from System import Byte, Decimal

_INVARIANT = System.Globalization.CultureInfo.InvariantCulture


def _decimal(text):
    return Decimal.Parse(text, _INVARIANT)


def _write_hex(uart, text):
    text = str(text)
    if text == "-":
        return
    for i in range(0, len(text), 2):
        uart.WriteChar(Byte(int(text[i:i + 2], 16)))


def mc_plant_feed(samples, gps_hex, mag_values, autopilot_hex="-"):
    machine = monitor.Machine  # noqa: F821
    imu = machine["sysbus.i2c1.imu"]
    for sample in str(samples).split(";"):
        if not sample:
            continue
        v = [_decimal(x) for x in sample.split(",")]
        imu.FeedAccelerationSample(v[0], v[1], v[2], 1)
        imu.FeedAngularRateSample(v[3], v[4], v[5], 1)

    m = [_decimal(x) for x in str(mag_values).split(",")]
    mag = machine["sysbus.i2c1.mag"]
    mag.MagneticX = m[0]
    mag.MagneticY = m[1]
    mag.MagneticZ = m[2]

    _write_hex(machine["sysbus.uart4"], gps_hex)
    _write_hex(machine["sysbus.uart5"], autopilot_hex)


# Renode's LSM9DS1 model reports these readings per unit fed (sim/cosim.py: ACCEL_GAIN, GYRO_GAIN).
_ACCEL_GAIN = 16384 * 0.061 / 1000.0
_GYRO_GAIN = 120 * 8.75 / 1000.0
_SAMPLE_RATE_HZ = 100.0


def mc_vibration_feed(accel_g, gyro_dps, frequency_hz, seconds):
    imu = monitor.Machine["sysbus.i2c1.imu"]  # noqa: F821
    a, g, f = float(accel_g), float(gyro_dps), float(frequency_hz)
    for n in range(int(float(seconds) * _SAMPLE_RATE_HZ)):
        phase = 2.0 * math.pi * f * n / _SAMPLE_RATE_HZ
        c, s = math.cos(phase), math.sin(phase)
        imu.FeedAccelerationSample(Decimal(a * c / _ACCEL_GAIN), Decimal(a * s / _ACCEL_GAIN),
                                   Decimal(1.0 / _ACCEL_GAIN), 1)
        imu.FeedAngularRateSample(Decimal(g * s / _GYRO_GAIN), Decimal(g * c / _GYRO_GAIN),
                                  Decimal(0.0), 1)


def mc_uart_write(uart_name, data_hex):
    _write_hex(monitor.Machine[str(uart_name)], data_hex)  # noqa: F821
