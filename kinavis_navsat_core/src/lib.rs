//! NMEA 0183 receiver output to the messages of a ROS 2 navigation satellite
//! node, without ROS: the part of `kinavis_navsat` that is tested on any
//! machine.
//!
//! Sentences are parsed by `kinavis-nmea0183`, grouped into epochs by their
//! time of day, and each epoch becomes a [`Solution`]: a `NavSatFix` built by
//! `kinavis-ros2`, the velocity over the ground in ENU and the receiver's
//! time.
//!
//! - **Epochs**: `RMC`, `GGA` and `GST` of one time of day. An epoch is
//!   complete once it has every sentence type of the epoch before it, or when
//!   a sentence of another time arrives.
//! - **Position and quality** from `GGA`; from `RMC` when the receiver sends
//!   no `GGA`.
//! - **Height above the ellipsoid**: `GGA` altitude plus geoid separation;
//!   `NaN` without the separation, never the altitude above sea level.
//! - **Covariance** from the error ellipse and altitude sigma of `GST`;
//!   `UNKNOWN` without `GST`. Not from HDOP.
//! - **Date** from `RMC`, carried across midnight.
//!
//! ```rust
//! use kinavis_navsat_core::Epochs;
//! use kinavis_nmea0183::parse;
//!
//! let mut epochs = Epochs::new();
//! let log = [
//!     "$GNRMC,223745.00,A,3806.62964,N,12237.61382,W,0.040,,110720,,,D,V*0E",
//!     "$GNGGA,223745.00,3806.62964,N,12237.61382,W,2,12,0.54,83.1,M,-29.5,M,,0000*44",
//!     "$GNGST,223745.00,228650,2.5,2.4,126,0.98,1.0,3.1*41",
//! ];
//! for line in log {
//!     assert_eq!(epochs.push(&parse(line.as_bytes())?).count(), 0);
//! }
//! let solution = epochs.finish().ok_or("no epoch")?;
//! assert!((solution.fix.latitude - 38.110_494).abs() < 1e-6);
//! assert!((solution.fix.altitude - 53.6).abs() < 1e-9);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

mod epoch;
mod lines;

pub use epoch::{Epochs, Solution};
pub use lines::{pace, Lines, MAX_LINE_BYTES};
