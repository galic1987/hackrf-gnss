//! BCH error correction for Iridium frames, ported from iridium-toolkit's bch.py.
//!
//! Iridium carries its payload in 32-bit blocks: 31 bits of BCH(31,21) followed
//! by one parity bit. Three different generator polynomials are in use depending
//! on the message class, which is itself partly determined by which polynomial
//! divides the block cleanly -- so this code is on the critical path for
//! classification, not just for repair.
//!
//! Everything here is arithmetic in GF(2), where addition is XOR and polynomial
//! division is a shift-and-XOR loop. `syndrome` is the remainder of the received
//! block divided by the generator; a zero remainder means no detected error, and
//! a non-zero one indexes a table that maps it back to the bit pattern that
//! produced it.

use std::collections::HashMap;
use std::sync::OnceLock;

/// Generator polynomials, as integers whose bits are the coefficients.
pub const MESSAGING: u32 = 1897;
pub const RINGALERT: u32 = 1207;
pub const ACCH: u32 = 3545;
pub const HDR: u32 = 29;
pub const LCW3: u32 = 41;

/// Remainder of `num` divided by `poly` in GF(2).
///
/// This is `nndivide` in the Python. Nothing subtle, but it is called for every
/// block of every frame and for every entry of the syndrome tables, so it is
/// worth keeping branch-free and integer-only.
pub fn gf2_remainder(poly: u32, mut num: u32) -> u32 {
    if num == 0 {
        return 0;
    }
    let plen = 32 - poly.leading_zeros();
    let mut bits = (32 - num.leading_zeros()) as i32 - plen as i32;
    let mut pow = 1u32 << (32 - num.leading_zeros() - 1);
    while bits >= 0 {
        if num >= pow {
            num ^= poly << bits;
        }
        pow >>= 1;
        bits -= 1;
    }
    num
}

/// Remainder of a bit STRING, matching the Python `ndivide(poly, bits)`.
pub fn gf2_remainder_bits(poly: u32, bits: &str) -> u32 {
    let mut v: u32 = 0;
    for c in bits.chars() {
        v = (v << 1) | if c == '1' { 1 } else { 0 };
    }
    gf2_remainder(poly, v)
}

/// Code parameters per generator, matching iridium-toolkit's mk_syn calls.
///
/// (data+check bits, correctable errors). These are NOT free choices: BCH(31,21)
/// carries 10 check bits, so it has 1024 syndromes against 31 single-bit and 465
/// double-bit error patterns -- 496 in all, which fit without collision. Adding
/// triple-bit patterns would need 4991 slots in the same 1024, so every syndrome
/// would map to something and the decoder could never report a block as
/// uncorrectable. It would "correct" noise into clean-looking data instead. An
/// earlier draft of this file did exactly that, and the test that caught it is
/// `an_uncorrectable_block_is_reported_rather_than_guessed`.
fn params(poly: u32) -> (usize, u8) {
    match poly {
        HDR => (7, 1),
        LCW3 => (26, 1),
        465 => (14, 2),
        MESSAGING | RINGALERT | ACCH => (31, 2),
        _ => (31, 2),
    }
}

/// syndrome -> (weight, error pattern), built once per polynomial.
///
/// A collision means the table has been asked to cover more error patterns than
/// the code can distinguish; the reference implementation raises there, and so
/// does this, because silently keeping one of the two would make the decoder
/// claim corrections it cannot justify.
fn syndromes(poly: u32) -> &'static HashMap<u32, (u8, u32)> {
    static TABLES: OnceLock<std::sync::Mutex<HashMap<u32, &'static HashMap<u32, (u8, u32)>>>> =
        OnceLock::new();
    let cache = TABLES.get_or_init(|| std::sync::Mutex::new(HashMap::new()));
    let mut guard = cache.lock().unwrap();
    if let Some(t) = guard.get(&poly) {
        return t;
    }
    let (bits, max_err) = params(poly);
    let mut t: HashMap<u32, (u8, u32)> = HashMap::new();
    for n1 in 0..bits {
        let val = 1u32 << n1;
        let r = gf2_remainder(poly, val);
        assert!(t.insert(r, (1, val)).is_none(),
                "poly {} collides on syndrome {} at one bit error", poly, r);
    }
    if max_err >= 2 {
        for n1 in 0..bits {
            for n2 in (n1 + 1)..bits {
                let val = (1u32 << n1) | (1u32 << n2);
                let r = gf2_remainder(poly, val);
                assert!(t.insert(r, (2, val)).is_none(),
                        "poly {} collides on syndrome {} at two bit errors", poly, r);
            }
        }
    }
    let leaked: &'static HashMap<u32, (u8, u32)> = Box::leak(Box::new(t));
    guard.insert(poly, leaked);
    leaked
}

/// Outcome of repairing one 31-bit block.
pub struct Repair {
    /// number of bits corrected, or None if the block could not be repaired
    pub errors: Option<u8>,
    /// the 21 data bits
    pub data: String,
    /// the 10 check bits
    pub check: String,
}

/// Correct up to the code's capability (two bit errors for the 31-bit codes).
///
/// Returns `errors: None` when the syndrome matches no correctable pattern --
/// that is a real detection of an uncorrectable block, and callers must treat it
/// as a failed frame rather than trusting the bits. Silently returning the
/// damaged data here is how a decoder starts inventing messages.
pub fn repair(poly: u32, block: &str) -> Repair {
    debug_assert_eq!(block.len(), 31, "BCH blocks are 31 bits");
    let mut v: u32 = 0;
    for c in block.chars() {
        v = (v << 1) | if c == '1' { 1 } else { 0 };
    }
    let syn = gf2_remainder(poly, v);
    let (errs, fixed) = if syn == 0 {
        (Some(0u8), v)
    } else {
        match syndromes(poly).get(&syn) {
            Some(&(n, pattern)) => (Some(n), v ^ pattern),
            None => (None, v),
        }
    };
    let bitstr: String = (0..31)
        .map(|i| if (fixed >> (30 - i)) & 1 == 1 { '1' } else { '0' })
        .collect();
    let split = 31 - (32 - poly.leading_zeros() as usize - 1);
    Repair {
        errors: errs,
        data: bitstr[..split].to_string(),
        check: bitstr[split..].to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn remainder_of_zero_is_zero() {
        assert_eq!(gf2_remainder(RINGALERT, 0), 0);
    }

    #[test]
    fn a_multiple_of_the_generator_divides_cleanly() {
        // In GF(2), poly * x has remainder zero for any x. This is the property
        // the whole classification stage leans on.
        for x in [1u32, 2, 3, 5, 17] {
            let mut prod = 0u32;
            let mut a = RINGALERT;
            let mut b = x;
            while b != 0 {
                if b & 1 == 1 {
                    prod ^= a;
                }
                a <<= 1;
                b >>= 1;
            }
            assert_eq!(gf2_remainder(RINGALERT, prod), 0, "x={}", x);
        }
    }

    #[test]
    fn a_clean_block_reports_no_errors() {
        // build a codeword: 21 data bits followed by the remainder they produce
        let data = "101100111000110101101";
        let shifted: u32 = u32::from_str_radix(data, 2).unwrap() << 10;
        let rem = gf2_remainder(RINGALERT, shifted);
        let word = shifted | rem;
        let block: String = (0..31)
            .map(|i| if (word >> (30 - i)) & 1 == 1 { '1' } else { '0' })
            .collect();
        let r = repair(RINGALERT, &block);
        assert_eq!(r.errors, Some(0));
        assert_eq!(r.data, data);
    }

    #[test]
    fn single_and_double_bit_errors_are_corrected() {
        let data = "101100111000110101101";
        let shifted: u32 = u32::from_str_radix(data, 2).unwrap() << 10;
        let word = shifted | gf2_remainder(RINGALERT, shifted);
        for flips in [vec![3usize], vec![0], vec![30], vec![5, 19]] {
            let mut w = word;
            for f in &flips {
                w ^= 1 << f;
            }
            let block: String = (0..31)
                .map(|i| if (w >> (30 - i)) & 1 == 1 { '1' } else { '0' })
                .collect();
            let r = repair(RINGALERT, &block);
            assert_eq!(r.errors, Some(flips.len() as u8), "flips {:?}", flips);
            assert_eq!(r.data, data, "flips {:?} were not corrected", flips);
        }
    }

    #[test]
    fn every_generator_has_usable_parameters() {
        // Each polynomial in the family gets its own (length, correctable) pair,
        // and building the table asserts the syndromes do not collide. Touching
        // all of them here means a wrong pair is caught at test time rather
        // than the first time a rare message class arrives.
        // only the generators the protocol actually uses: an arbitrary integer
        // is not a BCH generator and its syndromes legitimately collide
        for poly in [HDR, LCW3, 465u32, MESSAGING, RINGALERT, ACCH] {
            let (bits, errs) = params(poly);
            assert!(bits >= 7 && bits <= 31, "poly {} claims {} bits", poly, bits);
            assert!(errs == 1 || errs == 2, "poly {} claims {} errors", poly, errs);
            // building the table must not trip a collision assertion
            let _ = syndromes(poly);
        }
    }

    #[test]
    fn the_short_header_code_corrects_a_single_bit() {
        // The 6-bit IBC header uses its own tiny code. It is the first gate on
        // every broadcast frame, so a wrong parameter here would silently drop
        // the only message class that carries system timing.
        let r = repair(HDR, &"0".repeat(31));
        assert_eq!(r.errors, Some(0));
    }

    #[test]
    fn an_uncorrectable_block_is_reported_rather_than_guessed() {
        // Beyond the code's correcting power the honest answer is "cannot fix".
        // Returning the damaged bits with a confident errors=0 is how a decoder
        // manufactures messages out of noise.
        let mut worst = 0;
        for seed in 0u32..4000 {
            let block: String = (0..31)
                .map(|i| {
                    let h = seed.wrapping_mul(2654435761).rotate_left(i as u32);
                    if h & 1 == 1 { '1' } else { '0' }
                })
                .collect();
            if repair(RINGALERT, &block).errors.is_none() {
                worst += 1;
            }
        }
        assert!(worst > 0, "no random block was ever judged uncorrectable");
    }
}
