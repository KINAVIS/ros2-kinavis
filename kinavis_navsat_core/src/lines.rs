//! Lines of a byte stream, and the pace of a replay.

use std::io::{self, BufRead};
use std::time::Duration;

use kinavis_nmea0183::TimeOfDay;

/// Longest line kept, bytes. NMEA allows 82; receivers write more (RTK,
/// proprietary sentences). A longer line is dropped whole.
pub const MAX_LINE_BYTES: usize = 1024;

/// Longest pause of a replay: a gap in a recording is not waited out.
const MAX_PAUSE: Duration = Duration::from_secs(5);

/// Lines of a reader, without their `CR LF`, at most [`MAX_LINE_BYTES`]:
/// a device sending binary without line ends cannot grow the buffer.
#[derive(Debug)]
pub struct Lines<R> {
    reader: R,
    line: Vec<u8>,
}

impl<R: BufRead> Lines<R> {
    /// Lines of `reader`.
    pub fn new(reader: R) -> Self {
        Self {
            reader,
            line: Vec::with_capacity(MAX_LINE_BYTES),
        }
    }

    /// The next line; `None` at the end of the stream.
    ///
    /// # Errors
    ///
    /// The reader's, other than [`io::ErrorKind::Interrupted`].
    pub fn next_line(&mut self) -> io::Result<Option<&[u8]>> {
        self.line.clear();
        let mut overlong = false;
        loop {
            let available = match self.reader.fill_buf() {
                Ok(available) => available,
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                Err(error) => return Err(error),
            };
            if available.is_empty() {
                // The end: a last line without its line end still counts.
                return Ok((!self.line.is_empty() && !overlong).then_some(trim(&self.line)));
            }
            let (chunk, found) = match available.iter().position(|&b| b == b'\n') {
                Some(end) => (available.get(..end).unwrap_or_default(), Some(end)),
                None => (available, None),
            };
            if self.line.len() + chunk.len() > MAX_LINE_BYTES {
                overlong = true;
                self.line.clear();
            } else if !overlong {
                self.line.extend_from_slice(chunk);
            }
            let used = found.map_or(available.len(), |end| end + 1);
            self.reader.consume(used);
            if found.is_some() {
                if overlong {
                    overlong = false;
                    continue;
                }
                return Ok(Some(trim(&self.line)));
            }
        }
    }
}

fn trim(line: &[u8]) -> &[u8] {
    line.strip_suffix(b"\r").unwrap_or(line)
}

/// Pause between two epochs of a recording: the difference of their times
/// of day across midnight, at most five seconds.
#[must_use]
pub fn pace(before: TimeOfDay, next: TimeOfDay) -> Duration {
    let nanos = |t: TimeOfDay| {
        (u64::from(t.hour) * 3600 + u64::from(t.minute) * 60 + u64::from(t.second)) * 1_000_000_000
            + u64::from(t.nanos)
    };
    let day = 86_400 * 1_000_000_000;
    let difference = (nanos(next) + day - nanos(before) % day) % day;
    Duration::from_nanos(difference).min(MAX_PAUSE)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all(input: &[u8]) -> Vec<Vec<u8>> {
        let mut lines = Lines::new(io::BufReader::with_capacity(7, input));
        let mut out = Vec::new();
        while let Some(line) = lines.next_line().unwrap() {
            out.push(line.to_vec());
        }
        out
    }

    #[test]
    fn lines_across_buffer_refills() {
        let lines = all(b"$GPGGA,1*00\r\n\n$GPRMC,2*00\nlast");
        assert_eq!(
            lines,
            [&b"$GPGGA,1*00"[..], b"", b"$GPRMC,2*00", b"last"].map(<[u8]>::to_vec)
        );
    }

    #[test]
    fn an_overlong_line_is_dropped_whole() {
        let mut input = vec![b'x'; MAX_LINE_BYTES + 1];
        input.extend_from_slice(b"\n$GPGGA*00\n");
        input.extend(vec![b'y'; MAX_LINE_BYTES + 1]);
        assert_eq!(all(&input), [b"$GPGGA*00".to_vec()]);
    }

    #[test]
    fn pace_wraps_midnight_and_caps() {
        let at = |hour, minute, second, nanos| TimeOfDay {
            hour,
            minute,
            second,
            nanos,
        };
        assert_eq!(
            pace(at(23, 59, 59, 900_000_000), at(0, 0, 0, 0)),
            Duration::from_millis(100)
        );
        assert_eq!(pace(at(12, 0, 0, 0), at(12, 1, 0, 0)), MAX_PAUSE);
        assert_eq!(pace(at(12, 0, 0, 0), at(12, 0, 0, 0)), Duration::ZERO);
    }
}
