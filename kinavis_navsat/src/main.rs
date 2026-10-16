//! `nmea_navsat`: a ROS 2 node for an NMEA 0183 GNSS receiver.
//!
//! Reads `RMC`, `GGA` and `GST` from a device, a file or a TCP stream, and
//! publishes per epoch, under the names of
//! `nmea_navsat_driver`:
//!
//! - `fix` (`sensor_msgs/NavSatFix`): height above the ellipsoid, covariance
//!   from the receiver's error ellipse (`GST`), `UNKNOWN` without one;
//! - `vel` (`geometry_msgs/TwistStamped`): velocity over the ground, ENU;
//! - `time_reference` (`sensor_msgs/TimeReference`): the receiver's UTC time
//!   against the node's clock.
//!
//! Parameters:
//!
//! - `input` (`/dev/ttyACM0`): a device or file path (`/dev/stdin` too), or
//!   `tcp://host:port`. A regular file is replayed at the pace of its epochs.
//! - `frame_id` (`gps`): the antenna's frame.
//! - `time_ref_source` (`gps`): `source` of `time_reference`.
//! - `use_receiver_time` (`false`): stamp `fix` and `vel` with the receiver's
//!   time instead of the node's clock.
//! - `replay_rate` (`1.0`): replay speed of a file; `0` for no pauses.
//!
//! The node stops at the end of the input: with success for a file, with an
//! error for a device or a stream.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::net::TcpStream;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use kinavis_navsat_core::{pace, Epochs, Lines, Solution};
use kinavis_nmea0183::{parse, TimeOfDay};
use rclrs::{
    log_error, log_info, log_warn, Clock, Context, CreateBasicExecutor, Node, Publisher,
    RclrsErrorFilter, SpinOptions, ToLogParams,
};
use ros_env::{builtin_interfaces, geometry_msgs, sensor_msgs, std_msgs};

/// Settings read once at start.
struct Settings {
    frame_id: String,
    time_ref_source: String,
    use_receiver_time: bool,
    replay_rate: f64,
}

struct Publishers {
    fix: Publisher<sensor_msgs::msg::NavSatFix>,
    vel: Publisher<geometry_msgs::msg::TwistStamped>,
    time_reference: Publisher<sensor_msgs::msg::TimeReference>,
}

/// The receiver's byte stream, and whether it is a recording.
struct Input {
    reader: Box<dyn BufRead + Send>,
    recording: bool,
}

fn open(input: &str) -> Result<Input> {
    if let Some(address) = input.strip_prefix("tcp://") {
        let stream = TcpStream::connect(address).with_context(|| format!("connect {address}"))?;
        return Ok(Input {
            reader: Box::new(BufReader::new(stream)),
            recording: false,
        });
    }
    let file = File::open(input).with_context(|| format!("open {input}"))?;
    let recording = file.metadata().is_ok_and(|m| m.is_file());
    Ok(Input {
        reader: Box::new(BufReader::new(file)),
        recording,
    })
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("nmea_navsat")?;

    let text = |name: &str, default: &str, description: &str| {
        node.declare_parameter::<Arc<str>>(name)
            .default(default.into())
            .description(description)
            .read_only()
            .map(|p| p.get().to_string())
    };
    let input = text(
        "input",
        "/dev/ttyACM0",
        "Device or file path, or tcp://host:port",
    )?;
    let settings = Settings {
        frame_id: text("frame_id", "gps", "Frame of the antenna")?,
        time_ref_source: text("time_ref_source", "gps", "Source of time_reference")?,
        use_receiver_time: node
            .declare_parameter("use_receiver_time")
            .default(false)
            .description("Stamp fix and vel with the receiver's time, not the node's clock")
            .read_only()?
            .get(),
        replay_rate: node
            .declare_parameter("replay_rate")
            .default(1.0)
            .description("Replay speed of a file; 0 for no pauses")
            .read_only()?
            .get(),
    };
    if !(settings.replay_rate >= 0.0 && settings.replay_rate.is_finite()) {
        bail!("replay_rate must be finite and not negative");
    }
    let publishers = Publishers {
        fix: node.create_publisher("fix")?,
        vel: node.create_publisher("vel")?,
        time_reference: node.create_publisher("time_reference")?,
    };
    let source = open(&input)?;
    log_info!(node.logger(), "reading {input}");

    let reader = {
        let node = Arc::clone(&node);
        let commands = Arc::clone(executor.commands());
        thread::spawn(move || {
            let outcome = read(source, &node, &settings, &publishers);
            commands.halt_spinning();
            outcome
        })
    };
    executor
        .spin(SpinOptions::default())
        .ignore_non_errors()
        .first_error()?;
    // Spinning ends when the reader halts it, at the end of the input: the
    // reader is joined before anything of ROS is dropped, as it still drops
    // its publishers after the halt. On shutdown the reader may be blocked in
    // a read, and ends with the process.
    if context.ok() {
        match reader.join() {
            Ok(outcome) => outcome?,
            Err(_) => bail!("reader thread failed"),
        }
    }
    Ok(())
}

/// Reads the input to its end, publishing every epoch.
fn read(input: Input, node: &Node, settings: &Settings, publishers: &Publishers) -> Result<()> {
    let clock = node.get_clock();
    let mut lines = Lines::new(input.reader);
    let mut epochs = Epochs::new();
    let pacing = input.recording && settings.replay_rate > 0.0;
    let mut before: Option<TimeOfDay> = None;
    let mut emit = |solution: Solution| -> Result<()> {
        if pacing {
            if let Some(before) = before {
                thread::sleep(pace(before, solution.time).div_f64(settings.replay_rate));
            }
            before = Some(solution.time);
        }
        publish(node, &clock, settings, publishers, &solution)
    };
    while let Some(line) = lines.next_line().context("read")? {
        if line.is_empty() {
            continue;
        }
        match parse(line) {
            Ok(sentence) => {
                for solution in epochs.push(&sentence) {
                    emit(solution)?;
                }
            }
            Err(error) => log_warn!(
                node.logger().throttle(Duration::from_secs(10)),
                "sentence refused: {error}: {}",
                String::from_utf8_lossy(line)
            ),
        }
    }
    if let Some(solution) = epochs.finish() {
        emit(solution)?;
    }
    if input.recording {
        log_info!(node.logger(), "end of the recording");
        Ok(())
    } else {
        log_error!(node.logger(), "the input ended");
        bail!("the input ended")
    }
}

fn publish(
    node: &Node,
    clock: &Clock,
    settings: &Settings,
    publishers: &Publishers,
    solution: &Solution,
) -> Result<()> {
    let now = clock.now().to_ros_msg().context("node clock")?;
    let receiver = solution
        .receiver_time
        .map(|_| builtin_interfaces::msg::Time {
            sec: solution.fix.stamp.sec,
            nanosec: solution.fix.stamp.nanosec,
        });
    let stamp = match (&receiver, settings.use_receiver_time) {
        (Some(receiver), true) => receiver.clone(),
        (None, true) => {
            log_warn!(
                node.logger().throttle(Duration::from_secs(10)),
                "no date from the receiver yet (RMC): stamped with the node's clock"
            );
            now.clone()
        }
        (_, false) => now.clone(),
    };
    let header = std_msgs::msg::Header {
        stamp,
        frame_id: settings.frame_id.clone(),
    };
    let fix = &solution.fix;
    publishers.fix.publish(sensor_msgs::msg::NavSatFix {
        header: header.clone(),
        status: sensor_msgs::msg::NavSatStatus {
            status: fix.status,
            service: fix.service,
        },
        latitude: fix.latitude,
        longitude: fix.longitude,
        altitude: fix.altitude,
        position_covariance: fix.position_covariance,
        position_covariance_type: fix.position_covariance_type,
    })?;
    if let Some(velocity) = solution.velocity {
        publishers.vel.publish(geometry_msgs::msg::TwistStamped {
            header,
            twist: geometry_msgs::msg::Twist {
                linear: geometry_msgs::msg::Vector3 {
                    x: velocity.x,
                    y: velocity.y,
                    z: velocity.z,
                },
                angular: geometry_msgs::msg::Vector3::default(),
            },
        })?;
    }
    if let Some(time_ref) = receiver {
        publishers
            .time_reference
            .publish(sensor_msgs::msg::TimeReference {
                header: std_msgs::msg::Header {
                    stamp: now,
                    frame_id: settings.frame_id.clone(),
                },
                time_ref,
                source: settings.time_ref_source.clone(),
            })?;
    }
    Ok(())
}
