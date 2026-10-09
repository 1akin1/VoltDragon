# Co-simulation helper: feeds the Python plant model's sensor data into Node A.
#
# Load with:  include @renode/plant_feed.py   (with Node A selected: mach set "node_a")
#
#   plant_feed "<samples>" "<gps hex>" "<mag>"
#
# <samples>: ';'-separated accelerometer and gyroscope samples, each
#            'ax,ay,az,gx,gy,gz' in g and deg/s. They are queued in the LSM9DS1
#            model, which hands out one sample per firmware read, so a whole
#            co-simulation step is fed at once (ahead of time).
# <gps hex>: bytes for UART4 (NMEA sentences) as hex, or '-' for none.
# <mag>:     'mx,my,mz' in gauss, set as the magnetometer's current value. The
#            magnetometer model ignores queued samples, so it is updated once per step.
#
# Renode turns functions named mc_<name> into monitor commands; this runs in
# Renode's IronPython 2.7.

import System
from System import Byte, Decimal

_INVARIANT = System.Globalization.CultureInfo.InvariantCulture


def _decimal(text):
    return Decimal.Parse(text, _INVARIANT)


def mc_plant_feed(samples, gps_hex, mag_values):
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

    gps_hex = str(gps_hex)
    if gps_hex != "-":
        uart = machine["sysbus.uart4"]
        for i in range(0, len(gps_hex), 2):
            uart.WriteChar(Byte(int(gps_hex[i:i + 2], 16)))
