//! Inmarsat-C / AERO channel codec: CRC-16/CCITT, the K=7 (171,133)
//! convolutional code with a soft-decision Viterbi decoder, the block
//! interleaver, and the x^15+x^14+1 additive scrambler. These are the standard
//! forward-error-correction primitives an L-band Inmarsat-C TDM decoder is built
//! from. Ported from `validation/inmarsat_lib.py`.
//!
//! SCOPE: this is the codec, validated by round-trip (encode -> decode). The
//! higher-level Inmarsat-C *frame/packet layout* in the Python reference is an
//! acknowledged reconstruction, and no real Inmarsat signal is available at this
//! station, so a real-signal demodulator (carrier/timing recovery + that
//! framing) is deliberately NOT ported here — only the verifiable FEC core.

const K_CC: u32 = 7;
const NSTATES: usize = 1 << (K_CC - 1); // 64
const G1: u32 = 0o171;
const G2: u32 = 0o133;

fn parity(x: u32) -> u8 {
    (x.count_ones() & 1) as u8
}

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
pub fn crc16(data: &[u8], init: u16) -> u16 {
    let mut c = init;
    for &b in data {
        let idx = (((c >> 8) ^ b as u16) & 0xFF) as usize;
        let mut t = (idx as u16) << 8;
        for _ in 0..8 {
            t = if t & 0x8000 != 0 { (t << 1) ^ 0x1021 } else { t << 1 };
        }
        c = ((c << 8) & 0xFFFF) ^ t;
    }
    c
}

/// Rate-1/2 convolutional encode; `bits` must already carry 6 tail zeros so the
/// trellis terminates in state 0. Returns 2*len output bits.
pub fn conv_encode(bits: &[u8]) -> Vec<u8> {
    let mut s: u32 = 0;
    let mut out = Vec::with_capacity(bits.len() * 2);
    for &b in bits {
        let v = ((b as u32) << 6) | s;
        out.push(parity(v & G1));
        out.push(parity(v & G2));
        s = v >> 1;
    }
    out
}

/// Output symbols (+1/-1) for the transition into `ns` from predecessor `pred`.
fn trans_out(ns: usize, which: usize) -> (f64, f64) {
    let bit = ((ns >> 5) & 1) as u32;
    let pred = ((ns & 31) * 2 + which) as u32;
    let v = (bit << 6) | pred;
    let c0 = 1.0 - 2.0 * parity(v & G1) as f64;
    let c1 = 1.0 - 2.0 * parity(v & G2) as f64;
    (c0, c1)
}

/// Soft-decision Viterbi for the terminated trellis. `r` holds 2N soft symbols
/// (nominally +/-1, +1 <-> bit 0). Returns N decoded bits (last 6 are the tail).
pub fn viterbi_soft(r: &[f64]) -> Vec<u8> {
    let n = r.len() / 2;
    const NEG: f64 = -1e18;
    let mut pm = [NEG; NSTATES];
    pm[0] = 0.0;
    let mut dec = vec![[0u8; NSTATES]; n];

    for t in 0..n {
        let (a, b) = (r[2 * t], r[2 * t + 1]);
        let mut npm = [NEG; NSTATES];
        for ns in 0..NSTATES {
            let p0 = (ns & 31) * 2;
            let p1 = p0 + 1;
            let (o00, o01) = trans_out(ns, 0);
            let (o10, o11) = trans_out(ns, 1);
            let m0 = pm[p0] + a * o00 + b * o01;
            let m1 = pm[p1] + a * o10 + b * o11;
            if m1 > m0 {
                npm[ns] = m1;
                dec[t][ns] = 1;
            } else {
                npm[ns] = m0;
                dec[t][ns] = 0;
            }
        }
        let mx = npm.iter().cloned().fold(NEG, f64::max);
        for v in npm.iter_mut() {
            *v -= mx;
        }
        pm = npm;
    }
    // traceback from state 0 (terminated trellis)
    let mut s = 0usize;
    let mut bits = vec![0u8; n];
    for t in (0..n).rev() {
        bits[t] = ((s >> 5) & 1) as u8;
        s = (s & 31) * 2 + dec[t][s] as usize;
    }
    bits
}

/// Block interleaver: read into `rows`x`cols`, read out column-major.
pub fn interleave<T: Copy>(x: &[T], rows: usize, cols: usize) -> Vec<T> {
    let mut out = Vec::with_capacity(x.len());
    for c in 0..cols {
        for r in 0..rows {
            out.push(x[r * cols + c]);
        }
    }
    out
}

pub fn deinterleave<T: Copy>(x: &[T], rows: usize, cols: usize) -> Vec<T> {
    let mut out = vec![x[0]; rows * cols];
    for r in 0..rows {
        for c in 0..cols {
            out[r * cols + c] = x[c * rows + r];
        }
    }
    out
}

/// Additive-scrambler LFSR sequence (x^15 + x^14 + 1).
pub fn scrambler_seq(n: usize, seed: u16) -> Vec<u8> {
    let mut out = Vec::with_capacity(n);
    let mut st = seed & 0x7FFF;
    for _ in 0..n {
        let b = (((st >> 14) ^ (st >> 13)) & 1) as u8;
        out.push(b);
        st = ((st << 1) | b as u16) & 0x7FFF;
    }
    out
}

/// XOR-scramble hard bits.
pub fn scramble(bits: &[u8], seed: u16) -> Vec<u8> {
    let s = scrambler_seq(bits.len(), seed);
    bits.iter().zip(&s).map(|(b, x)| b ^ x).collect()
}

/// Undo an additive scrambler on SOFT values by flipping their sign.
pub fn descramble_soft(soft: &[f64], seed: u16) -> Vec<f64> {
    let s = scrambler_seq(soft.len(), seed);
    soft.iter().zip(&s).map(|(v, x)| v * (1.0 - 2.0 * *x as f64)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    // deterministic pseudo-random payload (no RNG needed): 90 bits + 6 tail
    fn payload() -> Vec<u8> {
        let mut b: Vec<u8> = (0..90).map(|i| ((i * 37 + 11) % 7 % 2) as u8).collect();
        b.extend([0, 0, 0, 0, 0, 0]); // tail
        b
    }

    #[test]
    fn viterbi_round_trips_the_convolutional_code() {
        let bits = payload();
        let coded = conv_encode(&bits);
        let soft: Vec<f64> = coded.iter().map(|&c| 1.0 - 2.0 * c as f64).collect();
        let decoded = viterbi_soft(&soft);
        assert_eq!(decoded, bits, "noiseless Viterbi must be exact");
    }

    #[test]
    fn viterbi_corrects_soft_noise() {
        let bits = payload();
        let coded = conv_encode(&bits);
        // add a deterministic mild perturbation and a few sign flips it can fix
        let mut soft: Vec<f64> =
            coded.iter().enumerate().map(|(i, &c)| (1.0 - 2.0 * c as f64) * (0.6 + 0.1 * ((i % 3) as f64))).collect();
        for k in [5usize, 33, 71] {
            soft[k] = -soft[k]; // isolated errors within the code's power
        }
        assert_eq!(viterbi_soft(&soft), bits, "should correct sparse soft errors");
    }

    #[test]
    fn scrambler_is_involutive_and_matches_soft() {
        let bits = payload();
        let sc = scramble(&bits, 0x7FFF);
        assert_ne!(sc, bits);
        assert_eq!(scramble(&sc, 0x7FFF), bits, "double scramble = identity");
        // soft descrambler undoes a scramble applied to +/-1 soft values
        let soft: Vec<f64> = sc.iter().map(|&c| 1.0 - 2.0 * c as f64).collect();
        let un = descramble_soft(&soft, 0x7FFF);
        let hard: Vec<u8> = un.iter().map(|&v| (v < 0.0) as u8).collect();
        assert_eq!(hard, bits);
    }

    #[test]
    fn interleaver_round_trips() {
        let x: Vec<u16> = (0..24).collect();
        let il = interleave(&x, 4, 6);
        assert_eq!(deinterleave(&il, 4, 6), x);
        assert_ne!(il, x);
    }

    #[test]
    fn crc16_ccitt_false_known_vector() {
        // CRC-16/CCITT-FALSE("123456789") = 0x29B1
        assert_eq!(crc16(b"123456789", 0xFFFF), 0x29B1);
    }

    #[test]
    fn full_chain_encode_scramble_interleave_and_back() {
        let bits = payload(); // 96 bits
        let coded = conv_encode(&bits); // 192
        let scrambled = scramble(&coded, 0x7FFF);
        let interleaved = interleave(&scrambled, 8, 24);
        // channel: to +/-1 soft
        let soft: Vec<f64> = interleaved.iter().map(|&c| 1.0 - 2.0 * c as f64).collect();
        // receive: deinterleave, descramble (soft), viterbi
        let de = deinterleave(&soft, 8, 24);
        let un = descramble_soft(&de, 0x7FFF);
        let out = viterbi_soft(&un);
        assert_eq!(out, bits, "full codec chain must round-trip");
    }
}
