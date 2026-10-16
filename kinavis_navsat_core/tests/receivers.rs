//! Recorded receivers against the fixes `nmea_navsat_driver` publishes for the
//! same logs (`tools/navsat_reference.py`), and against what their `GST` and
//! `RMC` say.
//!
//! Logs from the gpsd regression tests (commit 43362cd, BSD-2-Clause, see
//! `data/LICENSE-gpsd`): u-blox ZED-F9P (autonomous, GST without the
//! ellipse), u-blox NEO-M9N (SBAS, full GST), Quectel L70 (SBAS, GST with the
//! ellipse). With `GPSD_LOGS` set to gpsd's `test/daemon`, every log there is
//! read as well.

#![allow(clippy::expect_used, clippy::indexing_slicing)]

use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use kinavis_navsat_core::{Epochs, Lines, Solution};
use kinavis_nmea0183::parse;
use kinavis_ros2::{covariance_type, nav_sat_status};

fn solve(path: &Path) -> Vec<Solution> {
    let file = File::open(path).expect("log");
    let mut lines = Lines::new(BufReader::new(file));
    let mut epochs = Epochs::new();
    let mut solutions = Vec::new();
    while let Some(line) = lines.next_line().expect("read") {
        if let Ok(sentence) = parse(line) {
            solutions.extend(epochs.push(&sentence));
        }
    }
    solutions.extend(epochs.finish());
    solutions
}

struct Reference {
    seconds: f64,
    status: i8,
    latitude: f64,
    longitude: f64,
    altitude: f64,
}

fn reference(path: &Path) -> Vec<Reference> {
    let text = std::fs::read_to_string(path).expect("reference");
    text.lines()
        .skip(1)
        .map(|row| {
            let fields: Vec<&str> = row.split(',').collect();
            let number = |i: usize| fields[i].parse::<f64>().expect(row);
            Reference {
                seconds: number(0),
                status: fields[1].parse().expect(row),
                latitude: number(2),
                longitude: number(3),
                altitude: number(4),
            }
        })
        .collect()
}

fn seconds(solution: &Solution) -> f64 {
    let t = solution.time;
    f64::from(t.hour) * 3600.0
        + f64::from(t.minute) * 60.0
        + f64::from(t.second)
        + f64::from(t.nanos) * 1e-9
}

fn same(a: f64, b: f64, tolerance: f64) -> bool {
    (a.is_nan() && b.is_nan()) || (a - b).abs() <= tolerance
}

/// Every epoch of the log, as the reference driver publishes it.
fn check(name: &str) -> Vec<Solution> {
    let data = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    let solutions = solve(&data.join(format!("{name}.nmea")));
    let expected = reference(&data.join(format!("{name}.reference.csv")));
    assert_eq!(solutions.len(), expected.len(), "{name}: epochs");
    for (ours, theirs) in solutions.iter().zip(&expected) {
        let at = theirs.seconds;
        assert!(same(seconds(ours), at, 5e-4), "{name} {at}: time");
        assert_eq!(ours.fix.status, theirs.status, "{name} {at}: status");
        // Parsed decimal minutes: the same digits, a different rounding.
        assert!(
            same(ours.fix.latitude, theirs.latitude, 1e-12),
            "{name} {at}: latitude"
        );
        assert!(
            same(ours.fix.longitude, theirs.longitude, 1e-12),
            "{name} {at}: longitude"
        );
        assert!(
            same(ours.fix.altitude, theirs.altitude, 1e-9),
            "{name} {at}: altitude"
        );
    }
    solutions
}

#[test]
fn zed_f9p_sigmas_without_an_ellipse() {
    let solutions = check("ublox-zed-f9p-nmea");
    let first = &solutions[0];
    // $GNGST,003956.00,235111,,,,2.3,3.5,4.0: latitude and longitude sigmas.
    let c = first.fix.position_covariance;
    assert_eq!(
        first.fix.position_covariance_type,
        covariance_type::APPROXIMATED
    );
    assert!(same(c[0], 3.5 * 3.5, 1e-9) && same(c[4], 2.3 * 2.3, 1e-9));
    assert!(same(c[8], 4.0 * 4.0, 1e-9) && c[1].abs() < 1e-12, "{c:?}");
    // 12 April 2019, from RMC.
    let at = first.receiver_time.expect("date");
    assert_eq!((at.seconds(), at.subsec_nanos()), (1_555_029_596, 0));
    assert_eq!(first.fix.stamp.sec, 1_555_029_596);
    // 0.025 kn with no course: no velocity.
    assert!(first.velocity.is_none());
}

#[test]
fn neo_m9n_ellipse_and_altitude_sigma() {
    let solutions = check("ublox-neo-m9n-nmea");
    // $GNGST,223745.00,228650,2.5,2.4,126,0.98,1.0,3.1: the ellipse.
    let first = &solutions[0];
    let c = first.fix.position_covariance;
    let (east, north, cross) = (c[0], c[4], c[1]);
    // Semi-axes 2.5 and 2.4 m, the major axis on 126°.
    assert!(same(east + north, 2.5 * 2.5 + 2.4 * 2.4, 1e-9));
    let theta = 126_f64.to_radians();
    let (major, minor) = (2.5_f64 * 2.5, 2.4_f64 * 2.4);
    assert!(same(
        east,
        major * theta.sin().powi(2) + minor * theta.cos().powi(2),
        1e-9
    ));
    assert!(same(
        cross,
        (major - minor) * theta.sin() * theta.cos(),
        1e-9
    ));
    assert!(same(c[8], 3.1 * 3.1, 1e-9));
    assert_eq!(first.fix.status, nav_sat_status::STATUS_SBAS_FIX);
}

#[test]
fn quectel_l70_velocity() {
    let solutions = check("quectel-L70");
    // $GPRMC,150119.000,A,...,0.09,194.17: 0.09 kn on 194.17°.
    let v = solutions[0].velocity.expect("velocity");
    let speed = 0.09 * 1852.0 / 3600.0;
    let course = 194.17_f64.to_radians();
    assert!(same(v.x, speed * course.sin(), 1e-12));
    assert!(same(v.y, speed * course.cos(), 1e-12));
    assert!(same(v.z, 0.0, 0.0));
}

/// Every gpsd log: no failure, and each fix either has none or a finite
/// position on the Earth.
#[test]
fn every_gpsd_log() {
    let Some(directory) = std::env::var_os("GPSD_LOGS") else {
        return;
    };
    let mut fixes = 0;
    for entry in std::fs::read_dir(directory).expect("GPSD_LOGS") {
        let path = entry.expect("entry").path();
        if path.extension().is_none_or(|e| e != "log") {
            continue;
        }
        for solution in solve(&path) {
            let fix = solution.fix;
            if fix.status == nav_sat_status::STATUS_NO_FIX {
                assert!(fix.latitude.is_nan(), "{}", path.display());
                continue;
            }
            fixes += 1;
            assert!(fix.latitude.abs() <= 90.0, "{}", path.display());
            assert!(fix.longitude.abs() <= 180.0, "{}", path.display());
        }
    }
    assert!(fixes > 0);
}
