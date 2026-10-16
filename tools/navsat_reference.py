#!/usr/bin/env python3
"""Reference fixes of an NMEA log, as nmea_navsat_driver publishes them.

Reads the log with the parser and checksum check of nmea_navsat_driver (ROS 2
branch, commit bb4f132, BSD) and applies its GGA branch of `add_sentence`:
status from the fix quality, latitude and longitude signed by hemisphere,
altitude plus geoid separation. Prints one CSV row per GGA:

    seconds,status,latitude,longitude,altitude

`seconds` is the time of day; NaN where the driver writes NaN. Covariances
are not compared: the driver's come from HDOP and fixed errors, KINAVIS's
from GST.

    git clone https://github.com/ros-drivers/nmea_navsat_driver.git
    git -C nmea_navsat_driver checkout bb4f132d9df6aca5ca704f9e38d5c9820c041752
    python3 -I tools/navsat_reference.py nmea_navsat_driver LOG > LOG.reference.csv
"""

import sys
import types

# The parser logs through rclpy; only its logger is needed.
rclpy = types.ModuleType("rclpy")
rclpy.logging = types.SimpleNamespace(
    get_logger=lambda _name: types.SimpleNamespace(debug=lambda *_: None, warn=lambda *_: None)
)
sys.modules["rclpy"] = rclpy

# sensor_msgs/NavSatStatus.
STATUS_NO_FIX, STATUS_FIX, STATUS_SBAS_FIX, STATUS_GBAS_FIX = -1, 0, 1, 2
# The driver's gps_qualities, status only.
QUALITY = {-1: STATUS_NO_FIX, 0: STATUS_NO_FIX, 1: STATUS_FIX, 2: STATUS_SBAS_FIX,
           4: STATUS_GBAS_FIX, 5: STATUS_GBAS_FIX, 9: STATUS_GBAS_FIX}


def main(driver, log):
    sys.path.insert(0, f"{driver}/src")
    from libnmea_navsat_driver import parser
    from libnmea_navsat_driver.checksum_utils import check_nmea_checksum

    print("seconds,status,latitude,longitude,altitude")
    with open(log, encoding="ascii", errors="replace") as lines:
        for line in lines:
            line = line.strip()
            if not line.startswith("$") or not check_nmea_checksum(line):
                continue
            parsed = parser.parse_nmea_sentence(line)
            if not parsed or "GGA" not in parsed:
                continue
            data = parsed["GGA"]
            time = line.split(",")[1]
            seconds = (int(time[0:2]) * 3600 + int(time[2:4]) * 60 + float(time[4:])
                       if len(time) >= 6 else float("nan"))
            status = QUALITY.get(data["fix_type"], STATUS_NO_FIX)
            latitude = data["latitude"]
            if data["latitude_direction"] == "S":
                latitude = -latitude
            longitude = data["longitude"]
            if data["longitude_direction"] == "W":
                longitude = -longitude
            altitude = data["altitude"] + data["mean_sea_level"]
            print(f"{seconds:.3f},{status},{latitude!r},{longitude!r},{altitude!r}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
