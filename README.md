# ros2-kinavis

ROS 2 nodes on the [KINAVIS](https://github.com/KINAVIS/kinavis) navigation
crates, in Rust on [`rclrs`](https://github.com/ros2-rust/ros2_rust).

## `kinavis_navsat`: an NMEA 0183 GNSS receiver

`nmea_navsat` reads a receiver's `RMC`, `GGA` and `GST` and publishes, per
epoch, under the topic names of
[`nmea_navsat_driver`](https://github.com/ros-drivers/nmea_navsat_driver):

| Topic | Type | |
|---|---|---|
| `fix` | `sensor_msgs/NavSatFix` | position, height above the WGS 84 ellipsoid, covariance |
| `vel` | `geometry_msgs/TwistStamped` | velocity over the ground, ENU (x east, y north) |
| `time_reference` | `sensor_msgs/TimeReference` | receiver's UTC time against the node's clock |

What differs from `nmea_navsat_driver`:

- **Covariance from the receiver.** The horizontal block is the error
  ellipse of `GST`, with its orientation, so the cross term is kept; the
  vertical, its altitude sigma. Without `GST` the covariance is `UNKNOWN`:
  no figure is made up from HDOP.
- **One fix per epoch.** `RMC`, `GGA` and `GST` of one time of day are taken
  together; the epoch is published once it has every sentence type the
  previous one had, or when the next epoch begins.
- **Height above the ellipsoid only.** `GGA` altitude plus geoid
  separation; `NaN` when the receiver does not give the separation, never the
  height above sea level passed off as ellipsoidal.
- **Date from `RMC`**, carried across midnight; not from the host's clock.
- **Every sentence checked** by `kinavis-nmea0183`: checksum, field ranges,
  receiver quirks of the gpsd regression logs.

Dead reckoning (`GGA` quality 6), manual and simulated positions are
published as `STATUS_NO_FIX`, with latitude, longitude and altitude `NaN`.

### Parameters

| Parameter | Default | |
|---|---|---|
| `input` | `/dev/ttyACM0` | device or file path (`/dev/stdin` too), or `tcp://host:port` |
| `frame_id` | `gps` | frame of the antenna |
| `time_ref_source` | `gps` | `source` of `time_reference` |
| `use_receiver_time` | `false` | stamp `fix` and `vel` with the receiver's time, not the node's clock |
| `replay_rate` | `1.0` | replay speed of a regular file; `0` for no pauses |

A regular file is replayed at the pace of its epochs, and the node stops at
its end. A device or stream that ends is an error, for a launch file to
respawn the node.

The node does not set a serial port's speed. A USB receiver (u-blox
`/dev/ttyACM0`) needs none; for a serial line, set it first:

```sh
stty -F /dev/ttyUSB0 38400 raw
```

### Build and run

Rust messages are generated in the ROS 2 workspace from the message
packages' sources, as [`ros2_rust`](https://github.com/ros2-rust/ros2_rust)
describes. On Jazzy:

```sh
sudo apt install -y git libclang-dev python3-pip python3-vcstool
pip install --break-system-packages \
  git+https://github.com/colcon/colcon-cargo.git \
  git+https://github.com/colcon/colcon-ros-cargo.git

mkdir -p ws/src && cd ws
git clone https://github.com/KINAVIS/ros2-kinavis.git src/ros2-kinavis
vcs import src < src/ros2-kinavis/tools/jazzy.repos
. /opt/ros/jazzy/setup.sh
colcon build --packages-up-to kinavis_navsat

. install/setup.sh
ros2 run kinavis_navsat nmea_navsat --ros-args -p input:=/dev/ttyACM0
```

A recording, with the receiver's time:

```sh
ros2 run kinavis_navsat nmea_navsat --ros-args \
  -p input:=src/ros2-kinavis/kinavis_navsat_core/tests/data/ublox-neo-m9n-nmea.nmea \
  -p use_receiver_time:=true
```

## Layout

- `kinavis_navsat/`: the ROS 2 package (`ament_cargo`), the node.
- `kinavis_navsat_core/`: epochs to messages, without ROS: tested with plain
  `cargo test` on any machine.
- `tests/e2e.sh`: the node on a recording in a ROS 2 workspace, every
  published fix checked against the reference.
- `tools/navsat_reference.py`: the reference, the fixes
  `nmea_navsat_driver` publishes for the same log.

## Checked

- Three receivers of the gpsd regression logs (u-blox ZED-F9P and NEO-M9N,
  Quectel L70): every epoch's status, latitude, longitude and height are
  those of `nmea_navsat_driver` for the same log; the covariance is the one
  of their `GST`, the velocity the one of their `RMC`.
- All 199 gpsd logs read without failure, every fix on the Earth
  (`GPSD_LOGS=<gpsd>/test/daemon cargo test`).
- In CI, on ROS 2 Jazzy: the node built with colcon, run on the three
  recordings, every fix it publishes matched against the reference.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.

The receiver logs in `kinavis_navsat_core/tests/data` are from the gpsd
project, BSD-2-Clause: see
[`LICENSE-gpsd`](kinavis_navsat_core/tests/data/LICENSE-gpsd).

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in this work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.
