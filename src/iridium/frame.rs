//! Parsing of the demodulator's frame lines, and recovery of absolute time.
//!
//! A line looks like:
//!
//! ```text
//! RWA: i-1786714486.024-t1 0006737.9200 1626087311 N:34.49-100.00 I:00000000  90% 0.500 513 00110000...
//!      ^name               ^ms into cap ^Hz        ^SNR           ^id  ^conf  ^lvl ^n  ^bits
//! ```
//!
//! The name is where absolute time lives. iridium-toolkit reconstructs each
//! frame's UTC from it, and if the name does not match one of its patterns it
//! silently assumes zero -- which is how this station spent a long time feeding
//! the toolkit frames it dated to 1970, producing satellite matches whose TLEs
//! were "20,679 days old" while still reporting 100% confidence.
//!
//! `RWA` and `RAW` differ in bit order: the Python sets `swapped = (kind !=
//! "RWA")`, so RAW frames arrive with the symbol pairs the other way round.

/// One demodulated burst.
#[derive(Debug, Clone)]
pub struct Frame {
    /// `RWA` or `RAW`
    pub kind: String,
    /// the name field verbatim; callers map frames back to a capture through it
    pub name: String,
    /// capture start, unix seconds, from the name field
    pub start_epoch: Option<f64>,
    /// milliseconds from the start of the capture
    pub offset_ms: f64,
    /// centre frequency of the burst, Hz
    pub freq_hz: f64,
    /// demodulator confidence, percent
    pub confidence: u32,
    /// signal level as reported by the demodulator
    pub level: f64,
    /// symbol count claimed by the demodulator
    pub symbols: usize,
    /// the raw bit string
    pub bits: String,
}

impl Frame {
    /// Absolute time of this burst, unix seconds, if the name carried a date.
    pub fn epoch(&self) -> Option<f64> {
        self.start_epoch.map(|s| s + self.offset_ms / 1000.0)
    }
}

/// Pull the capture start out of a name field.
///
/// Accepts the canonical `i-<seconds>-t1` form, with an optional fractional part
/// and an optional `-o+N` suffix, mirroring the patterns iridium-toolkit itself
/// accepts. Anything else yields None -- deliberately, because a name we do not
/// understand must not be silently dated to the epoch.
pub fn start_epoch_from_name(name: &str) -> Option<f64> {
    let rest = name.strip_prefix("i-")?;
    // strip an optional -o+123 / -o-123 offset suffix
    let rest = match rest.find("-o") {
        Some(i) => &rest[..i],
        None => rest,
    };
    let (num, tail) = rest.rsplit_once('-')?;
    // the toolkit accepts a single letter from this set followed by '1'
    let mut ch = tail.chars();
    let letter = ch.next()?;
    if !"vbsrtl".contains(letter) || ch.next() != Some('1') || ch.next().is_some() {
        return None;
    }
    num.parse::<f64>().ok()
}

/// Parse one frame line. Returns None for comments, blanks and anything that is
/// not a frame.
pub fn parse_line(line: &str) -> Option<Frame> {
    let line = line.trim_end();
    let (kind, rest) = line.split_once(": ")?;
    if kind != "RWA" && kind != "RAW" && kind != "NC1" {
        return None;
    }
    let mut it = rest.split_whitespace();
    let name = it.next()?;
    let offset_ms: f64 = it.next()?.parse().ok()?;
    let freq_hz: f64 = it.next()?.parse().ok()?;
    let _snr = it.next()?; // N:34.49-100.00 or A:...
    let _id = it.next()?; // I:00000000 or L:...
    let conf = it.next()?.trim_end_matches('%').parse::<u32>().ok()?;
    let level: f64 = it.next()?.parse().unwrap_or(f64::NAN);
    let symbols: usize = it.next()?.parse().ok()?;
    let bits: String = it.next()?.chars().filter(|c| *c == '0' || *c == '1').collect();
    if bits.is_empty() {
        return None;
    }
    Some(Frame {
        kind: kind.to_string(),
        name: name.to_string(),
        start_epoch: start_epoch_from_name(name),
        offset_ms,
        freq_hz,
        confidence: conf,
        level,
        symbols,
        bits,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const LINE: &str = "RWA: i-1786714486.024-t1 0006737.9200 1626087311 \
                        N:34.49-100.00 I:00000000  90% 0.500 513 001100000011000011110011";

    #[test]
    fn parses_a_frame_line() {
        let f = parse_line(LINE).expect("should parse");
        assert_eq!(f.kind, "RWA");
        assert_eq!(f.offset_ms, 6737.92);
        assert_eq!(f.freq_hz, 1626087311.0);
        assert_eq!(f.confidence, 90);
        assert_eq!(f.symbols, 513);
        assert!(f.bits.starts_with("001100000011"));
    }

    #[test]
    fn recovers_absolute_time() {
        let f = parse_line(LINE).unwrap();
        assert_eq!(f.start_epoch, Some(1786714486.024));
        let e = f.epoch().unwrap();
        assert!((e - (1786714486.024 + 6.73792)).abs() < 1e-6, "epoch {}", e);
    }

    #[test]
    fn a_name_without_a_date_yields_none_rather_than_zero() {
        // THE bug this module exists to prevent. The old exporter wrote
        // `i-c00001-000-t1`; dating that to the epoch produced satellite
        // matches against TLEs 56 years stale, reported at "100%" confidence.
        assert_eq!(start_epoch_from_name("i-c00001-000-t1"), None);
        assert_eq!(start_epoch_from_name("garbage"), None);
        assert_eq!(start_epoch_from_name("i-1234"), None);
    }

    #[test]
    fn accepts_the_name_forms_the_toolkit_accepts() {
        assert_eq!(start_epoch_from_name("i-1786714486-t1"), Some(1786714486.0));
        assert_eq!(start_epoch_from_name("i-1786714486.024-t1"), Some(1786714486.024));
        assert_eq!(start_epoch_from_name("i-1786714486-s1"), Some(1786714486.0));
        assert_eq!(start_epoch_from_name("i-1786714486-t1-o+42"), Some(1786714486.0));
    }

    #[test]
    fn ignores_comments_and_blanks() {
        assert!(parse_line("# t_start 1786714486.024 cycle 1").is_none());
        assert!(parse_line("").is_none());
        assert!(parse_line("nonsense").is_none());
    }

    #[test]
    fn a_name_with_an_unknown_suffix_is_rejected() {
        // the toolkit accepts one letter from "vbsrtl" followed by '1'
        assert_eq!(start_epoch_from_name("i-1786714486-z1"), None);
        assert_eq!(start_epoch_from_name("i-1786714486-t2"), None);
        assert_eq!(start_epoch_from_name("i-1786714486-t1x"), None);
        assert_eq!(start_epoch_from_name("i-notanumber-t1"), None);
    }

    #[test]
    fn an_unknown_line_kind_is_ignored() {
        assert!(parse_line("IRA: i-1786714486-t1 0 0 N:0-0 I:0 0% 0 0 0101").is_none());
        assert!(parse_line("something: else entirely").is_none());
    }

    #[test]
    fn a_line_whose_payload_has_no_bits_is_rejected() {
        // a frame with a non-binary payload field carries nothing to decode;
        // returning an empty Frame would push the problem downstream
        assert!(parse_line(
            "RWA: i-1786714486-t1 0006737.9200 1626087311 N:34.49-100.00 I:00000000  90% 0.500 513 xyzzy"
        ).is_none());
    }

    #[test]
    fn a_truncated_line_is_rejected_not_half_parsed() {
        assert!(parse_line("RWA: i-1786714486-t1 0006737.9200").is_none());
    }

    #[test]
    fn a_line_cut_short_at_any_field_is_rejected() {
        // The recorder appends while readers parse, so a line can be read
        // mid-write at ANY field boundary. Each one must reject rather than
        // return a Frame with defaults filled in, because a half-parsed frame
        // carries a plausible-looking frequency or timestamp that is simply
        // wrong -- and nothing downstream would know.
        let fields: Vec<&str> = LINE.split_whitespace().collect();
        for n in 1..fields.len() {
            let partial = fields[..n].join(" ");
            assert!(parse_line(&partial).is_none(),
                    "a line cut after {} fields parsed: {:?}", n, partial);
        }
        // the complete line still parses, so the loop above is not vacuous
        assert!(parse_line(LINE).is_some());
    }

    #[test]
    fn unparsable_numeric_fields_are_rejected() {
        // each numeric field in turn replaced by something that is not a number
        // indices count from the "RWA:" prefix, which parse_line strips:
        // 2 = offset_ms, 3 = frequency, 6 = confidence, 8 = symbol count
        for (idx, bad) in [(2usize, "notanumber"), (3, "xx"), (6, "NN%"), (8, "zz")] {
            let mut fields: Vec<String> =
                LINE.split_whitespace().map(|s| s.to_string()).collect();
            fields[idx] = bad.to_string();
            let line = fields.join(" ");
            assert!(parse_line(&line).is_none(),
                    "field {} = {:?} should have been rejected", idx, bad);
        }
    }

    #[test]
    fn an_unreadable_level_does_not_sink_the_frame() {
        // level is diagnostic rather than structural: a demodulator that writes
        // "inf" or "nan" there should not cost us the frame's bits
        let mut fields: Vec<String> =
            LINE.split_whitespace().map(|s| s.to_string()).collect();
        fields[7] = "inf".to_string();   // the level field
        let f = parse_line(&fields.join(" ")).expect("frame should still parse");
        assert!(f.level.is_infinite() || f.level.is_nan());
        assert!(!f.bits.is_empty());
    }
}
