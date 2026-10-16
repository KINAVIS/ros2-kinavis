//! Sentences of one time of day into one solution.

use core::time::Duration;

use kinavis_kernel::{Distance, GnssFix, Height, Instant, Ned, Speed, Utc, Vector3};
use kinavis_nmea0183::{Date, Gga, Gst, Rmc, Sentence, Status, TimeOfDay};
use kinavis_ros2::{covariance_type, enu, nav_sat_fix, nav_sat_status, stamp, NavSatFix, Vector};

const RMC: u8 = 1;
const GGA: u8 = 2;
const GST: u8 = 4;

const DAY: Duration = Duration::from_secs(86_400);
const HALF_DAY: Duration = Duration::from_secs(43_200);

/// What one epoch of the receiver gives a ROS node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Solution {
    /// Time of day of the epoch, as the receiver wrote it.
    pub time: TimeOfDay,
    /// UTC instant of the epoch; `None` until a date is known (`RMC`).
    pub receiver_time: Option<Instant<Utc>>,
    /// The fix. Its stamp is the receiver time, or zero without one.
    pub fix: NavSatFix,
    /// Velocity over the ground, ENU, m/s; `None` without a valid `RMC`
    /// carrying both speed and course. Vertical component zero.
    pub velocity: Option<Vector>,
}

/// Sentences of one time of day.
#[derive(Debug, Clone, Copy)]
struct Epoch {
    time: TimeOfDay,
    kinds: u8,
    rmc: Option<Rmc>,
    gga: Option<Gga>,
    gst: Option<Gst>,
}

impl Epoch {
    const fn new(time: TimeOfDay) -> Self {
        Self {
            time,
            kinds: 0,
            rmc: None,
            gga: None,
            gst: None,
        }
    }

    fn add(&mut self, sentence: &Sentence) {
        match sentence {
            Sentence::Rmc(rmc) => {
                self.rmc = Some(*rmc);
                self.kinds |= RMC;
            }
            Sentence::Gga(gga) => {
                self.gga = Some(*gga);
                self.kinds |= GGA;
            }
            Sentence::Gst(gst) => {
                self.gst = Some(*gst);
                self.kinds |= GST;
            }
            _ => {}
        }
    }
}

/// Groups sentences into epochs and solves each one once.
///
/// Feed every parsed sentence to [`Epochs::push`]; at the end of the input,
/// [`Epochs::finish`] gives the last epoch.
#[derive(Debug, Clone, Default)]
pub struct Epochs {
    current: Option<Epoch>,
    emitted: bool,
    /// Sentence types of the epoch before: the current one is complete with
    /// all of them.
    expected: u8,
    /// Last date of `RMC`, with its instant.
    date: Option<(Date, Instant<Utc>)>,
}

impl Epochs {
    /// No epoch yet, no date.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Takes one sentence; gives the epochs it completes: none, the one
    /// before (a sentence of another time), the current one, or both.
    ///
    /// Sentences other than `RMC`, `GGA` and `GST`, and those without a time,
    /// are ignored, as is a sentence for an epoch already solved.
    pub fn push(&mut self, sentence: &Sentence) -> impl Iterator<Item = Solution> {
        let mut done = [None, None];
        let time = match sentence {
            Sentence::Rmc(rmc) => rmc.time,
            Sentence::Gga(gga) => gga.time,
            Sentence::Gst(gst) => gst.time,
            _ => None,
        };
        let Some(time) = time else {
            return done.into_iter().flatten();
        };
        if let Sentence::Rmc(Rmc {
            date: Some(date), ..
        }) = sentence
        {
            if let Ok(at) = date.at(time) {
                self.date = Some((*date, at));
            }
        }
        match &mut self.current {
            Some(epoch) if epoch.time == time => {
                if self.emitted {
                    return done.into_iter().flatten();
                }
                epoch.add(sentence);
            }
            current => {
                if let Some(before) = current.take() {
                    self.expected = before.kinds;
                    if !self.emitted {
                        done[0] = Some(self.solve(&before));
                    }
                }
                let mut epoch = Epoch::new(time);
                epoch.add(sentence);
                self.current = Some(epoch);
                self.emitted = false;
            }
        }
        if let Some(epoch) = self.current {
            if self.expected != 0 && epoch.kinds & self.expected == self.expected {
                self.emitted = true;
                done[1] = Some(self.solve(&epoch));
            }
        }
        done.into_iter().flatten()
    }

    /// The epoch in progress, if not yet solved: at the end of the input.
    pub fn finish(&mut self) -> Option<Solution> {
        let epoch = self.current.take()?;
        let emitted = core::mem::replace(&mut self.emitted, false);
        (!emitted).then(|| self.solve(&epoch))
    }

    /// UTC instant of a time of day: the last date, a day either way where
    /// that brings it within twelve hours of the date's own instant.
    fn instant(&self, time: TimeOfDay) -> Option<Instant<Utc>> {
        let (date, anchor) = self.date?;
        let at = date.at(time).ok()?;
        if at
            .checked_duration_since(anchor)
            .is_some_and(|d| d > HALF_DAY)
        {
            at.checked_sub(DAY)
        } else if anchor
            .checked_duration_since(at)
            .is_some_and(|d| d > HALF_DAY)
        {
            at.checked_add(DAY)
        } else {
            Some(at)
        }
    }

    fn solve(&self, epoch: &Epoch) -> Solution {
        let receiver_time = self.instant(epoch.time);
        let rmc = epoch.rmc.filter(|rmc| rmc.status == Status::Valid);
        Solution {
            time: epoch.time,
            receiver_time,
            fix: fix(epoch, rmc, receiver_time),
            velocity: rmc.and_then(velocity),
        }
    }
}

fn fix(epoch: &Epoch, rmc: Option<Rmc>, receiver_time: Option<Instant<Utc>>) -> NavSatFix {
    let (fix_type, position) = match (epoch.gga, rmc) {
        (Some(gga), _) => (gga.fix_type, gga.position),
        (None, Some(rmc)) => (rmc.fix_type(), rmc.position),
        (None, None) => return no_fix(receiver_time),
    };
    let Some(position) = position.filter(|_| fix_type.is_position_fix()) else {
        return no_fix(receiver_time);
    };
    // Without a date the fix still converts; its stamp is then cleared.
    let taken_at = receiver_time.unwrap_or(Instant::from_unix_seconds(0));
    let mut builder = GnssFix::builder(taken_at, position).fix_type(fix_type);
    if let Some(gga) = epoch.gga {
        if let Some(satellites) = gga.satellites {
            builder = builder.satellites(satellites);
        }
        if let Some(hdop) = gga.hdop {
            builder = builder.hdop(hdop);
        }
    }
    if let Some(ellipse) = epoch.gst.and_then(|gst| gst.error_ellipse()) {
        builder = builder.horizontal_error(ellipse);
    }
    let gnss = builder.build();
    let height = epoch.gga.and_then(ellipsoidal);
    let sigma = height.and(epoch.gst).and_then(|gst| gst.altitude_sigma);
    let unknown = Height::above_ellipsoid(Distance::ZERO);
    let Ok(mut fix) = nav_sat_fix(&gnss, height.unwrap_or(unknown), sigma) else {
        return no_fix(receiver_time);
    };
    if height.is_none() {
        fix.altitude = f64::NAN;
    }
    if receiver_time.is_none() {
        fix.stamp = kinavis_ros2::Stamp::default();
    }
    fix
}

/// `GGA` altitude above sea level plus the geoid separation.
fn ellipsoidal(gga: Gga) -> Option<Height> {
    let metres = gga.altitude?.metres() + gga.geoid_separation?.metres();
    Distance::from_metres(metres)
        .ok()
        .map(Height::above_ellipsoid)
}

fn no_fix(receiver_time: Option<Instant<Utc>>) -> NavSatFix {
    NavSatFix {
        stamp: receiver_time
            .and_then(|at| stamp(at).ok())
            .unwrap_or_default(),
        status: nav_sat_status::STATUS_NO_FIX,
        service: nav_sat_status::SERVICE_GPS,
        latitude: f64::NAN,
        longitude: f64::NAN,
        altitude: f64::NAN,
        position_covariance: [0.0; 9],
        position_covariance_type: covariance_type::UNKNOWN,
    }
}

/// ENU velocity of a valid `RMC` with both speed and course.
fn velocity(rmc: Rmc) -> Option<Vector> {
    let speed = rmc.speed_over_ground?.metres_per_second();
    let (sin, cos) = rmc.course_over_ground?.radians().sin_cos();
    let north = Speed::from_metres_per_second(speed * cos).ok()?;
    let east = Speed::from_metres_per_second(speed * sin).ok()?;
    Some(enu(Vector3::<Ned, Speed>::new(north, east, Speed::ZERO)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use kinavis_nmea0183::parse;

    fn feed(epochs: &mut Epochs, line: &str) -> Vec<Solution> {
        let sentence = parse(line.as_bytes()).expect(line);
        epochs.push(&sentence).collect()
    }

    #[test]
    fn second_epoch_completes_on_its_last_type() {
        let mut epochs = Epochs::new();
        let first = [
            "$GNRMC,223745.00,A,3806.62964,N,12237.61382,W,0.040,,110720,,,D,V*0E",
            "$GNGGA,223745.00,3806.62964,N,12237.61382,W,2,12,0.54,83.1,M,-29.5,M,,0000*44",
        ];
        for line in first {
            assert_eq!(feed(&mut epochs, line).len(), 0);
        }
        let rmc = "$GNRMC,223746.00,A,3806.62964,N,12237.61382,W,0.040,,110720,,,D,V*0D";
        let done = feed(&mut epochs, rmc);
        assert_eq!(done.len(), 1, "the time change closes the first epoch");
        let gga = "$GNGGA,223746.00,3806.62964,N,12237.61382,W,2,12,0.54,83.1,M,-29.5,M,,0000*47";
        let done = feed(&mut epochs, gga);
        assert_eq!(done.len(), 1, "RMC and GGA: complete as the first was");
        assert_eq!(done.first().map(|s| s.time.second), Some(46));
        assert!(epochs.finish().is_none(), "already solved");
    }

    #[test]
    fn date_carries_across_midnight() {
        let mut epochs = Epochs::new();
        let before = "$GPRMC,235959.00,A,5104.0859,N,11405.7387,W,0.09,194.17,090322,,,D*4F";
        assert_eq!(feed(&mut epochs, before).len(), 0);
        // GGA of the next day before its RMC: the old date, plus one day.
        let after = "$GPGGA,000000.00,5104.0859,N,11405.7387,W,2,8,0.94,1102.4,M,-17.5,M,,*6E";
        let done = feed(&mut epochs, after);
        let before = done.first().and_then(|s| s.receiver_time);
        let after = epochs.finish().and_then(|s| s.receiver_time);
        assert_eq!(
            after.and_then(|a| before.and_then(|b| a.checked_duration_since(b))),
            Some(Duration::from_secs(1))
        );
    }

    #[test]
    fn no_position_is_no_fix() {
        let mut epochs = Epochs::new();
        let gga = "$GPGGA,000001.00,,,,,0,00,99.99,,,,,,*67";
        assert_eq!(feed(&mut epochs, gga).len(), 0);
        let solution = epochs.finish();
        assert_eq!(
            solution.map(|s| s.fix.status),
            Some(nav_sat_status::STATUS_NO_FIX)
        );
        assert!(solution.is_some_and(|s| s.fix.latitude.is_nan() && s.receiver_time.is_none()));
    }

    #[test]
    fn velocity_is_east_north_up() {
        let mut epochs = Epochs::new();
        // 10 knots on 090°: east.
        let rmc = "$GPRMC,120000.00,A,5104.0859,N,11405.7387,W,10.0,90.0,090322,,,A*43";
        assert_eq!(feed(&mut epochs, rmc).len(), 0);
        let velocity = epochs.finish().and_then(|s| s.velocity);
        assert!(velocity.is_some_and(|v| (v.x - 5.144_444).abs() < 1e-6 && v.y.abs() < 1e-9));
    }
}
