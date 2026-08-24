//! Iridium simplex-burst demodulator, a port of `validation/demod3.py`.
//!
//! Near-coherent DQPSK detection (a phase reference smoothed over ~33 symbols and
//! refined decision-directed, then the hard decisions differenced — recovering
//! most of the ~2.3 dB differential detection loses), Oerder-Meyr timing, and
//! preamble+unique-word correlation framing that accepts a unique word with up to
//! 3 bit errors.
//!
//! Bit-exact parity with scipy's `decimate`/`resample_poly` is not the goal (and
//! not achievable across FFT/filter implementations); the goal is that a REAL
//! captured burst decodes to the SAME frame Python recovers. Each stage is
//! validated against a Python oracle dump on real snippets — see
//! `scripts/demod3_oracle.py` and `tests/iridium_demod3.rs`.

use num_complex::Complex;
use std::f64::consts::PI;

pub const BAUD: f64 = 25000.0;
pub const SPS: usize = 8;
pub const NSYM_BURST: usize = 508;
pub const PREAMBLE: usize = 64;
pub const UW_SYMS: [u8; 12] = [0, 2, 0, 0, 0, 2, 0, 0, 2, 2, 0, 2];
/// symbol (0..3, Gray) -> two bits, MSB first
pub const SYM2BITS: [[u8; 2]; 4] = [[0, 0], [0, 1], [1, 1], [1, 0]];

type C32 = Complex<f32>;

// ---------------------------------------------------------------- FIR helpers

/// Low-pass FIR via the windowed-sinc method with a Hamming window, matching
/// scipy.signal.firwin(numtaps, cutoff, window="hamming") for a normalised
/// cutoff in (0,1) (1.0 == Nyquist). numtaps is odd so the filter is linear
/// phase with an integer group delay.
fn firwin_hamming(numtaps: usize, cutoff: f64) -> Vec<f64> {
    assert!(numtaps % 2 == 1, "use an odd tap count");
    let m = numtaps - 1;
    let alpha = m as f64 / 2.0;
    let mut h = vec![0.0f64; numtaps];
    for (i, hi) in h.iter_mut().enumerate() {
        let n = i as f64 - alpha;
        // ideal low-pass impulse response (cutoff in half-cycles/sample)
        let sinc = if n.abs() < 1e-9 {
            cutoff
        } else {
            (PI * cutoff * n).sin() / (PI * n)
        };
        // Hamming window
        let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / m as f64).cos();
        *hi = sinc * w;
    }
    // normalise to unity gain at DC (scipy scales so the passband gain is 1)
    let s: f64 = h.iter().sum();
    for hi in h.iter_mut() {
        *hi /= s;
    }
    h
}

/// Forward-backward (zero-phase) FIR filtering of complex data, with odd
/// reflection padding at the edges. Not byte-identical to scipy's filtfilt edge
/// handling, but identical in the interior — where a burst well inside the
/// record lives — which is all the demodulator reads.
fn filtfilt(b: &[f64], x: &[C32]) -> Vec<C32> {
    let fwd = fir_forward(b, x);
    let mut rev: Vec<C32> = fwd.iter().rev().copied().collect();
    rev = fir_forward(b, &rev);
    rev.reverse();
    rev
}

fn fir_forward(b: &[f64], x: &[C32]) -> Vec<C32> {
    let n = x.len();
    let ntaps = b.len();
    let pad = ntaps; // odd-reflection pad length
    // build padded signal: reflect around the endpoints (2*x[0]-x[pad-i])
    let mut xp: Vec<C32> = Vec::with_capacity(n + 2 * pad);
    for i in 0..pad {
        let idx = (pad - i).min(n - 1);
        xp.push(x[0] * 2.0 - x[idx]);
    }
    xp.extend_from_slice(x);
    for i in 0..pad {
        let idx = if n >= 2 + i { n - 2 - i } else { 0 };
        xp.push(x[n - 1] * 2.0 - x[idx]);
    }
    // FIR convolution, keep the samples aligned to the original (group delay
    // (ntaps-1)/2 removed by the forward-backward pass, so here just full conv
    // and slice back the padded region)
    let gd = (ntaps - 1) / 2;
    let mut y = vec![C32::new(0.0, 0.0); xp.len()];
    for i in 0..xp.len() {
        let mut acc = C32::new(0.0, 0.0);
        for (k, &bk) in b.iter().enumerate() {
            if i >= k {
                acc += xp[i - k] * (bk as f32);
            }
        }
        y[i] = acc;
    }
    // remove group delay and the padding
    y[pad + gd..pad + gd + n].to_vec()
}

/// Decimate by integer factor q: zero-phase low-pass (firwin/Hamming, 20q+1
/// taps, cutoff 1/q) then take every q-th sample. Mirrors
/// scipy.signal.decimate(x, q, ftype="fir", zero_phase=True).
pub fn decimate(x: &[C32], q: usize) -> Vec<C32> {
    let ntaps = 20 * q + 1;
    let b = firwin_hamming(ntaps, 1.0 / q as f64);
    let y = filtfilt(&b, x);
    y.iter().step_by(q).copied().collect()
}

// ---------------------------------------------------------------- burst detect

/// A detected burst: start time (s), duration (s), centre frequency (Hz).
#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct Burst {
    pub t: f64,
    pub dur: f64,
    pub fcen: f64,
}

/// Detect Iridium simplex bursts in a capture, a port of
/// Sample word accepted by the Iridium front end: i8 (standard gateware)
/// or i16 (extended-precision gateware; 12-bit data right-justified).
/// Everything downstream runs in f32, and all thresholds are relative
/// (percentile / dB), so the 256x amplitude difference between the two
/// input widths is immaterial.
pub trait IqSample: Copy {
    fn to_f32(self) -> f32;
}
impl IqSample for i8 {
    #[inline]
    fn to_f32(self) -> f32 { self as f32 }
}
impl IqSample for i16 {
    #[inline]
    fn to_f32(self) -> f32 {
        // ext-precision stream: 12-bit sample right-justified. The top
        // nibble carries the timestamp nibble (see crate::ts_nibble), not
        // sign extension — mask it and re-extend from bit 11.
        ((self << 4) >> 4) as f32
    }
}

/// `validation/demod3.py:find_bursts2`. Finds connected blobs in the
/// time-frequency plane (per-bin background normalised, then thresholded), which
/// keeps time-overlapping bursts on different channels apart. `raw` is the whole
/// interleaved int8 capture; times are seconds from its start.
pub fn find_bursts2<S: IqSample>(raw: &[S], fc: f64, fs: f64, dur: f64, lo: f64, hi: f64, thr_db: f64) -> Vec<Burst> {
    const NFFT: usize = 2048;
    let chan_khz = 25.0;
    let chunk = 4.0;
    let overlap = 0.06;
    let (dmin, dmax) = (0.007, 0.040);
    let chan_hz = 41.667e3; // Iridium channel spacing for multi-channel splitting
    let tres = NFFT as f64 / fs;
    let bin_hz = fs / NFFT as f64;

    // shifted frequency axis: shifted index p -> freq (p - N/2)*bin_hz, fft bin (p+N/2)%N
    let band: Vec<(usize, f64)> = (0..NFFT)
        .filter_map(|p| {
            let f = (p as f64 - (NFFT / 2) as f64) * bin_hz + fc;
            if f > lo && f < hi {
                Some(((p + NFFT / 2) % NFFT, f))
            } else {
                None
            }
        })
        .collect();
    let nband = band.len();
    if nband == 0 {
        return Vec::new();
    }
    let fb: Vec<f64> = band.iter().map(|&(_, f)| f).collect();
    let w = ((chan_khz * 1e3 / bin_hz).round() as usize).max(3);

    let mut planner = rustfft::FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(NFFT);
    let han: Vec<f32> = (0..NFFT)
        .map(|i| (0.5 - 0.5 * (2.0 * PI * i as f64 / (NFFT as f64 - 1.0)).cos()) as f32)
        .collect();

    let nchunks = (dur / chunk).ceil() as usize;
    let mut out: Vec<Burst> = Vec::new();

    for c in 0..nchunks {
        let t_off = c as f64 * chunk - if c > 0 { overlap } else { 0.0 };
        let off = ((t_off * fs) as isize * 2).max(0) as usize;
        let n = ((chunk + 2.0 * overlap) * fs) as usize * 2;
        let end = (off + n).min(raw.len());
        if end <= off {
            break;
        }
        let slice = &raw[off..end];
        let nsamp = slice.len() / 2;
        let nrow = nsamp / NFFT;
        if nrow < 8 {
            break;
        }
        // chunk-mean removal (Python subtracts x.mean() over the whole chunk)
        let (mut mr, mut mi) = (0.0f64, 0.0f64);
        for k in 0..nsamp {
            mr += slice[2 * k].to_f32() as f64;
            mi += slice[2 * k + 1].to_f32() as f64;
        }
        let (mr, mi) = ((mr / nsamp as f64) as f32, (mi / nsamp as f64) as f32);

        // Pb[row][band] = |fftshift(fft(row*hann))|^2 over the band columns
        let mut pb = vec![0f64; nrow * nband];
        let mut buf = vec![Complex::<f32>::new(0.0, 0.0); NFFT];
        for r in 0..nrow {
            for i in 0..NFFT {
                let s = 2 * (r * NFFT + i);
                buf[i] = Complex::new(
                    (slice[s].to_f32() - mr) * han[i],
                    (slice[s + 1].to_f32() - mi) * han[i],
                );
            }
            fft.process(&mut buf);
            for (bi, &(bin, _)) in band.iter().enumerate() {
                pb[r * nband + bi] = buf[bin].norm_sqr() as f64 + 1e-9;
            }
        }
        // R = Pb / median over rows (per band column)
        let mut r_mat = vec![0f64; nrow * nband];
        for bi in 0..nband {
            let mut col: Vec<f64> = (0..nrow).map(|r| pb[r * nband + bi]).collect();
            col.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let med = col[nrow / 2].max(1e-30);
            for r in 0..nrow {
                r_mat[r * nband + bi] = pb[r * nband + bi] / med;
            }
        }
        // smooth each row over frequency (box, width w) and threshold in dB
        let mut hot = vec![false; nrow * nband];
        for r in 0..nrow {
            let row = &r_mat[r * nband..(r + 1) * nband];
            let d = movavg_box_f(row, w);
            for bi in 0..nband {
                if 10.0 * d[bi].max(1e-9).log10() > thr_db {
                    hot[r * nband + bi] = true;
                }
            }
        }
        // 4-connectivity connected components over hot cells
        let mut seen = vec![false; nrow * nband];
        for start in 0..nrow * nband {
            if !hot[start] || seen[start] {
                continue;
            }
            let mut stack = vec![start];
            seen[start] = true;
            let mut cells: Vec<(usize, usize)> = Vec::new();
            while let Some(idx) = stack.pop() {
                let (r, bi) = (idx / nband, idx % nband);
                cells.push((r, bi));
                let mut nb = [(0isize, 0isize); 4];
                nb[0] = (1, 0);
                nb[1] = (-1, 0);
                nb[2] = (0, 1);
                nb[3] = (0, -1);
                for (dr, dc) in nb {
                    let (nr, nc) = (r as isize + dr, bi as isize + dc);
                    if nr >= 0 && nr < nrow as isize && nc >= 0 && nc < nband as isize {
                        let ni = nr as usize * nband + nc as usize;
                        if hot[ni] && !seen[ni] {
                            seen[ni] = true;
                            stack.push(ni);
                        }
                    }
                }
            }
            emit_blob(&cells, &r_mat, &fb, nband, tres, bin_hz, chan_hz, t_off, dur, dmin, dmax, &mut out);
        }
    }
    out.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));
    // collapse chunk overlaps
    let mut ded: Vec<Burst> = Vec::new();
    for b in out {
        if let Some(last) = ded.last() {
            if (b.t - last.t).abs() < 0.004 && (b.fcen - last.fcen).abs() < 15e3 {
                continue;
            }
        }
        ded.push(b);
    }
    ded
}

#[allow(clippy::too_many_arguments)]
fn emit_blob(
    cells: &[(usize, usize)],
    r_mat: &[f64],
    fb: &[f64],
    nband: usize,
    tres: f64,
    bin_hz: f64,
    chan_hz: f64,
    t_off: f64,
    dur: f64,
    dmin: f64,
    dmax: f64,
    out: &mut Vec<Burst>,
) {
    let t0 = cells.iter().map(|c| c.0).min().unwrap();
    let t1 = cells.iter().map(|c| c.0).max().unwrap();
    if !(dmin..=dmax * 1.6).contains(&((t1 - t0 + 1) as f64 * tres)) {
        return;
    }
    let qmin = cells.iter().map(|c| c.1).min().unwrap();
    let qmax = cells.iter().map(|c| c.1).max().unwrap();
    let nsub = (((qmax - qmin) as f64 * bin_hz / chan_hz).round() as usize).max(1);

    // (row_start, row_end, weighted freq) candidates
    let mut cands: Vec<(usize, usize, f64)> = Vec::new();
    if nsub == 1 {
        let (mut fs_num, mut wsum) = (0.0, 0.0);
        for &(r, bi) in cells {
            let wgt = r_mat[r * nband + bi];
            fs_num += fb[bi] * wgt;
            wsum += wgt;
        }
        cands.push((t0, t1, fs_num / wsum));
    } else {
        // split the blob into nsub frequency sub-blocks
        for s in 0..nsub {
            let a = qmin as f64 + (qmax + 1 - qmin) as f64 * s as f64 / nsub as f64;
            let b = qmin as f64 + (qmax + 1 - qmin) as f64 * (s + 1) as f64 / nsub as f64;
            let sub: Vec<&(usize, usize)> =
                cells.iter().filter(|(_, bi)| (*bi as f64) >= a && (*bi as f64) < b).collect();
            if sub.len() < 4 {
                continue;
            }
            let sr0 = sub.iter().map(|c| c.0).min().unwrap();
            let sr1 = sub.iter().map(|c| c.0).max().unwrap();
            let (mut fs_num, mut wsum) = (0.0, 0.0);
            for &&(r, bi) in &sub {
                let wgt = r_mat[r * nband + bi];
                fs_num += fb[bi] * wgt;
                wsum += wgt;
            }
            cands.push((sr0, sr1, fs_num / wsum));
        }
    }
    for (a, b, fcen) in cands {
        let dd = (b - a + 1) as f64 * tres;
        let t = t_off + a as f64 * tres;
        if (dmin..=dmax).contains(&dd) && (0.0..dur).contains(&t) {
            out.push(Burst { t, dur: dd, fcen });
        }
    }
}

// ---------------------------------------------------------------- front end

/// Read a burst from a capture, mix `fcen` to baseband, and decimate to fs/8.
/// Returns (baseband, fsy). None if the slice is too short.
pub fn load_bb<S: IqSample>(
    raw: &[S],
    t0: f64,
    dur: f64,
    fcen: f64,
    fc: f64,
    fs: f64,
    guard: f64,
) -> Option<(Vec<C32>, f64)> {
    let off = ((t0 - guard).max(0.0) * fs) as usize * 2;
    let n = ((dur + 2.0 * guard + 0.002) * fs) as usize * 2;
    if off >= raw.len() {
        return None;
    }
    let end = (off + n).min(raw.len());
    let slice = &raw[off..end];
    if slice.len() < 4000 {
        return None;
    }
    let ns = slice.len() / 2;
    let mut x: Vec<C32> = (0..ns)
        .map(|i| C32::new(slice[2 * i].to_f32(), slice[2 * i + 1].to_f32()))
        .collect();
    // remove DC
    let mean: C32 = x.iter().sum::<C32>() / ns as f32;
    for v in x.iter_mut() {
        *v -= mean;
    }
    // mix fcen down to baseband
    let w = -2.0 * PI * (fcen - fc) / fs;
    for (k, v) in x.iter_mut().enumerate() {
        let ph = w * k as f64;
        *v *= C32::new(ph.cos() as f32, ph.sin() as f32);
    }
    let y = decimate(&x, 4);
    let y = decimate(&y, 2);
    Some((y, fs / 8.0))
}

/// Root-raised-cosine filter, matching validation/demod3.py:rrc.
pub fn rrc(beta: f64, sps: usize, span: usize) -> Vec<f64> {
    let n0 = (span * sps) as i64;
    let mut h = Vec::new();
    for i in -n0..=n0 {
        let t = i as f64 / sps as f64;
        let v = if t.abs() < 1e-8 {
            1.0 - beta + 4.0 * beta / PI
        } else if beta > 0.0 && ((4.0 * beta * t).abs() - 1.0).abs() < 1e-8 {
            beta / 2.0f64.sqrt()
                * ((1.0 + 2.0 / PI) * (PI / (4.0 * beta)).sin()
                    + (1.0 - 2.0 / PI) * (PI / (4.0 * beta)).cos())
        } else {
            ((PI * t * (1.0 - beta)).sin() + 4.0 * beta * t * (PI * t * (1.0 + beta)).cos())
                / (PI * t * (1.0 - (4.0 * beta * t).powi(2)))
        };
        h.push(v);
    }
    let e: f64 = h.iter().map(|v| v * v).sum();
    let norm = e.sqrt();
    for v in h.iter_mut() {
        *v /= norm;
    }
    h
}

// ---------------------------------------------------------------- resampling

fn i0_bessel(x: f64) -> f64 {
    // modified Bessel I0 via series (enough for Kaiser windows)
    let mut sum = 1.0;
    let mut term = 1.0;
    let half = x / 2.0;
    for k in 1..40 {
        term *= (half / k as f64).powi(2);
        sum += term;
        if term < 1e-14 * sum {
            break;
        }
    }
    sum
}

fn firwin_kaiser(numtaps: usize, cutoff: f64, beta: f64) -> Vec<f64> {
    let m = numtaps - 1;
    let alpha = m as f64 / 2.0;
    let denom = i0_bessel(beta);
    let mut h = vec![0.0f64; numtaps];
    for (i, hi) in h.iter_mut().enumerate() {
        let n = i as f64 - alpha;
        let sinc = if n.abs() < 1e-9 {
            cutoff
        } else {
            (PI * cutoff * n).sin() / (PI * n)
        };
        let r = (i as f64 - alpha) / alpha;
        let w = i0_bessel(beta * (1.0 - r * r).max(0.0).sqrt()) / denom;
        *hi = sinc * w;
    }
    let s: f64 = h.iter().sum();
    for hi in h.iter_mut() {
        *hi /= s;
    }
    h
}

/// Rational resample by up/dn, mirroring scipy.signal.resample_poly with the
/// default Kaiser(5.0) window. Output length ceil(len*up/dn), aligned so output
/// sample 0 corresponds to input sample 0 (linear-phase group delay removed).
pub fn resample_poly(x: &[C32], up: usize, dn: usize) -> Vec<C32> {
    let g = gcd(up, dn);
    let (up, dn) = (up / g, dn / g);
    let max_rate = up.max(dn);
    let half_len = 10 * max_rate;
    let ntaps = 2 * half_len + 1;
    let mut h = firwin_kaiser(ntaps, 1.0 / max_rate as f64, 5.0);
    // scipy scales the filter by `up` so the interpolated samples keep amplitude
    for v in h.iter_mut() {
        *v *= up as f64;
    }
    // polyphase upfirdn: insert up-1 zeros, convolve, take every dn-th, with the
    // filter group delay (half_len) removed in the upsampled domain.
    let n = x.len();
    let up_len = n * up;
    // upsampled convolution output index m corresponds to upsampled-domain time
    // (m - half_len); we sample those at multiples of dn starting from 0.
    let n_out = (n * up).div_ceil(dn);
    let mut out = vec![C32::new(0.0, 0.0); n_out];
    for (oi, o) in out.iter_mut().enumerate() {
        // upsampled-domain center for this output sample
        let center = oi * dn + half_len;
        let mut acc = C32::new(0.0, 0.0);
        // h index j spans 0..ntaps; upsampled input index = center - j
        // input contributes only where (center - j) is a multiple of `up`
        let jmin = center.saturating_sub(up_len.saturating_sub(1));
        let jmax = center.min(ntaps - 1);
        let mut j = jmin;
        while j <= jmax {
            let up_idx = center - j;
            if up_idx % up == 0 {
                let xi = up_idx / up;
                if xi < n {
                    acc += x[xi] * (h[j] as f32);
                }
            }
            j += 1;
        }
        *o = acc;
    }
    out
}

fn gcd(a: usize, b: usize) -> usize {
    if b == 0 { a } else { gcd(b, a % b) }
}

// ---------------------------------------------------------------- stages

/// Burst envelope edges (start,end) sample indices, matching burst_edges.
pub fn burst_edges(y: &[C32], fsy: f64) -> Option<(usize, usize)> {
    let env: Vec<f64> = y.iter().map(|c| c.norm_sqr() as f64).collect();
    let w = (fsy * 0.0002).max(4.0) as usize;
    let sm = movavg_box_f(&env, w);
    let thr = 0.5 * (percentile(&sm, 95.0) + percentile(&sm, 10.0));
    let on: Vec<usize> = (0..sm.len()).filter(|&i| sm[i] > thr).collect();
    if on.len() < (fsy * 0.005) as usize {
        return None;
    }
    Some((*on.first().unwrap(), *on.last().unwrap()))
}

fn percentile(x: &[f64], p: f64) -> f64 {
    let mut v: Vec<f64> = x.to_vec();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    if v.is_empty() {
        return 0.0;
    }
    // linear interpolation, numpy default
    let rank = p / 100.0 * (v.len() - 1) as f64;
    let lo = rank.floor() as usize;
    let hi = rank.ceil() as usize;
    if lo == hi {
        v[lo]
    } else {
        v[lo] + (rank - lo as f64) * (v[hi] - v[lo])
    }
}

/// Carrier frequency of the unmodulated preamble, matching preamble_carrier with
/// ls=false, fwin=0. Returns (df_hz, snr_db).
pub fn preamble_carrier(y: &[C32], fsy: f64, i0: usize, pad: usize) -> Option<(f64, f64)> {
    let a = i0 + (fsy * 0.0004) as usize;
    let b = y.len().min(a + (fsy * 0.0018) as usize);
    if b < a + 64 {
        return None;
    }
    let seg = &y[a..b];
    let len = seg.len();
    let nf = (len * pad).next_power_of_two();
    // hanning window (numpy np.hanning) then zero-padded FFT
    let mut buf: Vec<Complex<f64>> = vec![Complex::new(0.0, 0.0); nf];
    for i in 0..len {
        let w = if len > 1 {
            0.5 - 0.5 * (2.0 * PI * i as f64 / (len as f64 - 1.0)).cos()
        } else {
            1.0
        };
        buf[i] = Complex::new(seg[i].re as f64 * w, seg[i].im as f64 * w);
    }
    let mut planner = rustfft::FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(nf);
    fft.process(&mut buf);
    let s: Vec<f64> = buf.iter().map(|c| c.norm_sqr()).collect();
    let mut sorted = s.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let nfloor = sorted[sorted.len() / 2].max(1e-30);
    let k = (0..nf).max_by(|&i, &j| s[i].partial_cmp(&s[j]).unwrap()).unwrap();
    // fftfreq(nf, 1/fsy): bin k -> k*fsy/nf for k<nf/2 else (k-nf)*fsy/nf
    let fk = if k < nf / 2 { k as f64 } else { k as f64 - nf as f64 } * fsy / nf as f64;
    let snr = 10.0 * (s[k].max(1e-30) / nfloor).log10();
    Some((fk, snr))
}

/// Oerder-Meyr square-law timing estimate (fractional sample phase in [0,SPS)).
pub fn timing_om(z2: &[C32], n0: usize, n1: usize) -> f64 {
    let mut c = Complex::<f64>::new(0.0, 0.0);
    for n in n0..n1 {
        let e = z2[n].norm_sqr() as f64;
        let ph = -2.0 * PI * n as f64 / SPS as f64;
        c += Complex::new(e * ph.cos(), e * ph.sin());
    }
    let tau = -c.arg() / (2.0 * PI) * SPS as f64;
    tau.rem_euclid(SPS as f64)
}

/// Linear interpolation of z2 at fractional indices (mode "lin").
fn interp_lin(z: &[C32], idx: &[f64]) -> Vec<C32> {
    idx.iter()
        .map(|&f| {
            let i = f.floor() as isize;
            let i = i.clamp(0, z.len() as isize - 2) as usize;
            let mu = (f - i as f64) as f32;
            z[i] * (1.0 - mu) + z[i + 1] * mu
        })
        .collect()
}

fn movavg_box_f(x: &[f64], l: usize) -> Vec<f64> {
    if l <= 1 {
        return x.to_vec();
    }
    let n = x.len();
    let mut out = vec![0.0; n];
    let off = (l - 1) / 2;
    let inv = 1.0 / l as f64;
    for i in 0..n {
        let mut acc = 0.0;
        for k in 0..l {
            let src = i as isize + k as isize - off as isize;
            if src >= 0 && (src as usize) < n {
                acc += x[src as usize];
            }
        }
        out[i] = acc * inv;
    }
    out
}

fn movavg_box_c(x: &[Complex<f64>], l: usize) -> Vec<Complex<f64>> {
    if l <= 1 {
        return x.to_vec();
    }
    let n = x.len();
    let mut out = vec![Complex::new(0.0, 0.0); n];
    let off = (l - 1) / 2;
    let inv = 1.0 / l as f64;
    for i in 0..n {
        let mut acc = Complex::new(0.0, 0.0);
        for k in 0..l {
            let src = i as isize + k as isize - off as isize;
            if src >= 0 && (src as usize) < n {
                acc += x[src as usize];
            }
        }
        out[i] = acc * inv;
    }
    out
}

fn unwrap(p: &[f64]) -> Vec<f64> {
    let mut out = Vec::with_capacity(p.len());
    if p.is_empty() {
        return out;
    }
    out.push(p[0]);
    let mut prev = p[0];
    let mut offset = 0.0;
    for &cur in &p[1..] {
        let mut d = cur - prev;
        while d > PI {
            d -= 2.0 * PI;
        }
        while d < -PI {
            d += 2.0 * PI;
        }
        let unwrapped = out.last().unwrap() + d;
        let _ = offset;
        offset = 0.0;
        out.push(unwrapped);
        prev = cur;
    }
    out
}

/// Near-coherent DQPSK detection (det="coh"), returns (differential symbols,
/// per-symbol quality) matching demod3.detect with default params.
pub fn detect(sym: &[C32], l: usize, ddit: usize) -> (Vec<u8>, Vec<f64>) {
    let symd: Vec<Complex<f64>> = sym.iter().map(|c| Complex::new(c.re as f64, c.im as f64)).collect();
    // 4th-power phase reference
    let r: Vec<Complex<f64>> = symd.iter().map(|c| c.powu(4)).collect();
    let rr = movavg_box_c(&r, l);
    let ph: Vec<f64> = unwrap(&rr.iter().map(|c| c.arg()).collect::<Vec<_>>())
        .iter()
        .map(|a| a / 4.0)
        .collect();
    let mut v: Vec<Complex<f64>> = symd
        .iter()
        .zip(&ph)
        .map(|(s, p)| s * Complex::from_polar(1.0, -p))
        .collect();
    for _ in 0..ddit {
        // decision-directed reference
        let ideal: Vec<Complex<f64>> = v
            .iter()
            .map(|vi| {
                let k = (vi.arg() / (PI / 2.0)).round();
                Complex::from_polar(1.0, k * (PI / 2.0))
            })
            .collect();
        let c: Vec<Complex<f64>> = symd.iter().zip(&ideal).map(|(s, id)| s * id.conj()).collect();
        let cm = movavg_box_c(&c, l);
        let ph2 = unwrap(&cm.iter().map(|c| c.arg()).collect::<Vec<_>>());
        v = symd
            .iter()
            .zip(&ph2)
            .map(|(s, p)| s * Complex::from_polar(1.0, -p))
            .collect();
    }
    let ang: Vec<f64> = v.iter().map(|c| c.arg()).collect();
    let k: Vec<i64> = ang
        .iter()
        .map(|a| (a / (PI / 2.0)).round().rem_euclid(4.0) as i64)
        .collect();
    let ds: Vec<u8> = (1..k.len())
        .map(|i| (k[i] - k[i - 1]).rem_euclid(4) as u8)
        .collect();
    let q: Vec<f64> = ang
        .iter()
        .map(|a| {
            let dev = a - (a / (PI / 2.0)).round() * (PI / 2.0);
            (2.0 * dev).cos().abs()
        })
        .collect();
    // q for a differential symbol is min of the two symbols it spans
    let qd: Vec<f64> = (1..q.len()).map(|i| q[i].min(q[i - 1])).collect();
    (ds, qd)
}

/// Locate the frame start by correlating the known preamble+UW differential
/// pattern, matching find_frame. Returns (start_index, uw_bit_errors, score).
pub fn find_frame(ds: &[u8], hint: usize, win: usize, prewin: usize, maxd: usize) -> Option<(usize, usize, usize)> {
    if ds.len() < 12 {
        return None;
    }
    let n = ds.len() - 12;
    let lo = hint.saturating_sub(win);
    let hi = (hint + win).min(n);
    if hi < lo {
        return None;
    }
    // bits of each symbol (2 per symbol)
    let bits: Vec<[u8; 2]> = ds.iter().map(|&s| SYM2BITS[s as usize]).collect();
    let uwbits: Vec<u8> = UW_SYMS.iter().flat_map(|&s| SYM2BITS[s as usize]).collect();
    let mut best: Option<(usize, usize, usize)> = None; // (score, d, p)
    for p in lo..=hi {
        // UW bit errors over 12 symbols (24 bits)
        let mut d = 0usize;
        for i in 0..12 {
            if p + i < bits.len() {
                d += (bits[p + i][0] != uwbits[2 * i]) as usize;
                d += (bits[p + i][1] != uwbits[2 * i + 1]) as usize;
            }
        }
        // preamble: bits before p should be zero
        let j = p.saturating_sub(prewin);
        let mut pd = 0usize;
        for x in j..p {
            pd += bits[x][0] as usize + bits[x][1] as usize;
        }
        let sc = d + pd;
        if best.is_none() || sc < best.unwrap().0 {
            best = Some((sc, d, p));
        }
    }
    let (sc, d, p) = best?;
    if d > maxd {
        return None;
    }
    Some((p, d, sc))
}

/// Intermediate results of `demod_snippet`, for stage-by-stage validation.
pub struct Demod3Debug {
    pub fsy: f64,
    pub i0: usize,
    pub i1: usize,
    pub df: f64,
    pub psnr: f64,
    pub frac: f64,
    pub hint: usize,
    pub conf: i32,
    pub ds: Vec<u8>,
    pub uw_pos: Option<usize>,
    pub z2: Vec<C32>,
    pub sym: Vec<C32>,
    pub rwa: Option<String>,
}

/// Full single-burst demodulation of a captured snippet. Returns the RWA line
/// (ready for iridium-parser) or None. Mirrors demod3.demod + to_line defaults.
pub fn demod_snippet<S: IqSample>(raw: &[S], fcen: f64, fc: f64, fs: f64) -> Option<String> {
    demod_snippet_debug(raw, fcen, fc, fs).and_then(|d| d.rwa)
}

/// As `demod_snippet` but returns all intermediates. None only if the front end
/// cannot form a burst at all. A snippet is a burst that starts ~2 ms in.
pub fn demod_snippet_debug<S: IqSample>(raw: &[S], fcen: f64, fc: f64, fs: f64) -> Option<Demod3Debug> {
    demod_burst_debug(raw, 0.002, 0.0204, fcen, fc, fs)
}

/// Decode all bursts in a capture: find_bursts2 then demod each. Returns RWA
/// lines ready for iridium-parser, mirroring demod3.run_file.
pub fn run_capture<S: IqSample>(raw: &[S], fc: f64, fs: f64, dur: f64, lo: f64, hi: f64, thr_db: f64) -> Vec<String> {
    find_bursts2(raw, fc, fs, dur, lo, hi, thr_db)
        .iter()
        .filter_map(|b| demod_burst_debug(raw, b.t, b.dur, b.fcen, fc, fs).and_then(|d| d.rwa))
        .collect()
}

/// Demodulate one burst at absolute capture time `t0` (seconds). None only if
/// the front end cannot form a burst there.
pub fn demod_burst_debug<S: IqSample>(raw: &[S], t0: f64, dur: f64, fcen: f64, fc: f64, fs: f64) -> Option<Demod3Debug> {
    let (y, fsy) = load_bb(raw, t0, dur, fcen, fc, fs, 0.0015)?;
    let (i0, i1) = burst_edges(&y, fsy)?;
    let (df, psnr) = preamble_carrier(&y, fsy, i0, 16)?;

    // de-rotate by df
    let w = -2.0 * PI * df / fsy;
    let z: Vec<C32> = y
        .iter()
        .enumerate()
        .map(|(k, v)| v * C32::new((w * k as f64).cos() as f32, (w * k as f64).sin() as f32))
        .collect();
    let g = (fsy * 0.0002) as usize;
    let a0 = i0.saturating_sub(g);
    let lead = i0 - a0;
    let z = &z[a0..];
    let nb = i1 - a0;

    let up = (BAUD * SPS as f64) as usize;
    let dn = fsy.round() as usize;
    let z2 = resample_poly(z, up, dn);
    let h = rrc(0.4, SPS, 8);
    let z2 = convolve_same_rf(&z2, &h);

    let sc = (BAUD * SPS as f64) / fsy;
    let n1 = ((z2.len() as isize - SPS as isize - 4).max(0) as usize).min((nb as f64 * sc) as usize);
    let n0t = (lead as f64 * sc) as usize;
    if n1 <= n0t + 100 * SPS {
        return None;
    }
    let frac = timing_om(&z2, n0t, n1);

    let k0 = 1usize;
    let kmax = (((n1 as f64 - frac) / SPS as f64) as isize - 1).max(0) as usize;
    if kmax <= k0 + 130 {
        return None;
    }
    let idx: Vec<f64> = (k0..kmax).map(|k| k as f64 * SPS as f64 + frac).collect();
    let sym = interp_lin(&z2, &idx);
    let s0 = (lead as f64 * sc / SPS as f64).round() as usize;
    let b0 = (s0 as isize - k0 as isize - 4).max(0) as usize;
    let b1 = (s0 + NSYM_BURST + 4 - k0).min(sym.len());
    if b1 <= b0 {
        return None;
    }
    let (ds, q) = detect(&sym[b0..b1], 33, 2);
    let conf = if q.is_empty() {
        0
    } else {
        (100.0 * q.iter().filter(|&&x| x > 0.5).count() as f64 / q.len() as f64) as i32
    };
    let hint = (s0 - k0 - b0) + PREAMBLE - 2;
    let uw_pos = find_uw(&ds);

    // find_frame + to_line (maxd=3, win=24, prewin=20)
    let rwa = find_frame(&ds, hint, 24, 20, 3).and_then(|(p, _d, _sc)| {
        let bits: String = ds[p..]
            .iter()
            .flat_map(|&s| SYM2BITS[s as usize].iter().map(|b| char::from(b'0' + b)))
            .collect();
        if bits.len() < 120 {
            return None;
        }
        let fabs = (fcen + df).round() as i64;
        Some(format!(
            "RWA: i-0-t1 {:012.4} {} N:{:.2}-100.00 I:00000000 {:3}% 0.500 {} {}",
            2.0,
            fabs,
            psnr.max(1.0),
            conf.min(99),
            bits.len() / 2,
            bits
        ))
    });

    Some(Demod3Debug {
        fsy, i0, i1, df, psnr, frac, hint, conf, ds, uw_pos,
        z2, sym: sym[b0..b1].to_vec(), rwa,
    })
}

/// Position of the unique word in a differential-symbol stream, if present.
fn find_uw(ds: &[u8]) -> Option<usize> {
    if ds.len() < 12 {
        return None;
    }
    (0..=ds.len() - 12).find(|&i| ds[i..i + 12] == UW_SYMS)
}

/// Real-valued FIR convolution of complex data, numpy 'same' alignment.
fn convolve_same_rf(x: &[C32], h: &[f64]) -> Vec<C32> {
    let n = x.len();
    let l = h.len();
    let off = (l - 1) / 2;
    let mut out = vec![C32::new(0.0, 0.0); n];
    for i in 0..n {
        let mut acc = C32::new(0.0, 0.0);
        for (k, &hk) in h.iter().enumerate() {
            let src = i as isize + off as isize - k as isize;
            if src >= 0 && (src as usize) < n {
                acc += x[src as usize] * (hk as f32);
            }
        }
        out[i] = acc;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn firwin_is_symmetric_and_unity_dc() {
        let b = firwin_hamming(81, 0.25);
        let s: f64 = b.iter().sum();
        assert!((s - 1.0).abs() < 1e-9, "DC gain {s}");
        for i in 0..b.len() / 2 {
            assert!((b[i] - b[b.len() - 1 - i]).abs() < 1e-12, "asymmetric");
        }
    }

    #[test]
    fn gcd_and_bessel() {
        assert_eq!(gcd(200000, 500000), 100000);
        assert_eq!(gcd(12, 8), 4);
        assert_eq!(gcd(7, 0), 7);
        assert!((i0_bessel(0.0) - 1.0).abs() < 1e-12);
        assert!(i0_bessel(2.0) > i0_bessel(1.0) && i0_bessel(1.0) > 1.0);
    }

    #[test]
    fn kaiser_is_symmetric_and_unity_dc() {
        let b = firwin_kaiser(65, 0.2, 5.0);
        assert!((b.iter().sum::<f64>() - 1.0).abs() < 1e-9);
        for i in 0..b.len() / 2 {
            assert!((b[i] - b[b.len() - 1 - i]).abs() < 1e-12);
        }
    }

    #[test]
    fn percentile_matches_numpy() {
        let v = [1.0, 2.0, 3.0, 4.0, 5.0];
        assert!((percentile(&v, 0.0) - 1.0).abs() < 1e-9);
        assert!((percentile(&v, 50.0) - 3.0).abs() < 1e-9);
        assert!((percentile(&v, 100.0) - 5.0).abs() < 1e-9);
        assert!((percentile(&v, 25.0) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn movavg_and_interp_and_unwrap() {
        // box average of a constant is the constant (interior)
        let x = vec![3.0f64; 20];
        let y = movavg_box_f(&x, 5);
        for v in &y[3..17] {
            assert!((v - 3.0).abs() < 1e-9);
        }
        // complex box average of a constant
        let xc = vec![Complex::new(2.0f64, -1.0); 16];
        let yc = movavg_box_c(&xc, 5);
        assert!((yc[8].re - 2.0).abs() < 1e-9 && (yc[8].im + 1.0).abs() < 1e-9);
        // linear interpolation halfway
        let z = vec![C32::new(0.0, 0.0), C32::new(2.0, 4.0), C32::new(4.0, 0.0)];
        let o = interp_lin(&z, &[0.5]);
        assert!((o[0].re - 1.0).abs() < 1e-5 && (o[0].im - 2.0).abs() < 1e-5);
        // unwrap continues past a wrap instead of jumping
        let u = unwrap(&[0.0, 3.0, -3.0]);
        assert!(u[2] > u[1] && (u[2] - 3.283).abs() < 0.01);
    }

    #[test]
    fn resample_poly_preserves_a_slow_tone() {
        let fsy = 500_000.0;
        let f = 3000.0;
        let x: Vec<C32> = (0..5000)
            .map(|k| {
                let p = 2.0 * PI * f * k as f64 / fsy;
                C32::new(p.cos() as f32, p.sin() as f32)
            })
            .collect();
        let y = resample_poly(&x, 2, 5); // -> 200 kHz
        assert!((y.len() as isize - (5000 * 2 / 5)).abs() <= 1);
        let m: f32 = y[200..1800].iter().map(|v| v.norm()).sum::<f32>() / 1600.0;
        assert!((m - 1.0).abs() < 0.05, "tone magnitude {m}");
    }

    #[test]
    fn convolve_same_keeps_length_and_centers() {
        let mut x = vec![C32::new(0.0, 0.0); 9];
        x[4] = C32::new(1.0, 0.0); // impulse at centre
        let h = vec![0.25, 0.5, 0.25];
        let y = convolve_same_rf(&x, &h);
        assert_eq!(y.len(), 9);
        // 'same' convolution of a centred impulse reproduces the (reversed) kernel around centre
        assert!((y[4].re - 0.5).abs() < 1e-6);
        assert!((y[3].re - 0.25).abs() < 1e-6 && (y[5].re - 0.25).abs() < 1e-6);
    }

    #[test]
    fn timing_om_recovers_the_sampling_phase() {
        // energy peaks every SPS samples at a known fractional offset
        let n = SPS * 200;
        let off = 3.0f64;
        let z2: Vec<C32> = (0..n)
            .map(|i| {
                // energy peaks when the sampling phase equals `off` (circular)
                let mut d = ((i as f64).rem_euclid(SPS as f64) - off).abs();
                d = d.min(SPS as f64 - d);
                C32::new((1.0 / (1.0 + d)) as f32, 0.0)
            })
            .collect();
        let frac = timing_om(&z2, 0, n);
        let err = (frac - off).abs().min(SPS as f64 - (frac - off).abs());
        assert!(err < 0.6, "recovered {frac} vs {off}");
    }

    #[test]
    fn preamble_carrier_finds_a_tone() {
        let fsy = 500_000.0;
        let f = 4200.0;
        let y: Vec<C32> = (0..2000)
            .map(|k| {
                let p = 2.0 * PI * f * k as f64 / fsy;
                C32::new(p.cos() as f32, p.sin() as f32)
            })
            .collect();
        let (df, snr) = preamble_carrier(&y, fsy, 0, 16).expect("carrier");
        assert!((df - f).abs() < 60.0, "df {df} vs {f}");
        assert!(snr > 10.0, "snr {snr}");
    }

    #[test]
    fn burst_edges_bracket_an_amplitude_burst() {
        let fsy = 500_000.0;
        let n = 12000;
        let (b0, b1) = (3000usize, 9000usize);
        let y: Vec<C32> = (0..n)
            .map(|i| {
                let a = if i >= b0 && i < b1 { 8.0 } else { 0.4 };
                C32::new(a, 0.0)
            })
            .collect();
        let (i0, i1) = burst_edges(&y, fsy).expect("edges");
        assert!((i0 as isize - b0 as isize).abs() < 200, "i0 {i0}");
        assert!((i1 as isize - (b1 as isize - 1)).abs() < 200, "i1 {i1}");
    }

    #[test]
    fn detect_recovers_differential_symbols() {
        // clean DQPSK: absolute symbols k, sym = unit phasor; detect differences them
        let ks: [i64; 40] = [
            0, 1, 3, 2, 0, 0, 1, 1, 2, 3, 3, 0, 2, 1, 0, 3, 2, 2, 1, 0, 1, 2, 3, 0, 1, 1, 3, 2, 0,
            2, 3, 1, 0, 0, 2, 3, 1, 2, 0, 1,
        ];
        let sym: Vec<C32> = ks
            .iter()
            .map(|&k| {
                let ph = k as f64 * PI / 2.0;
                C32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        let (ds, q) = detect(&sym, 33, 2);
        assert_eq!(ds.len(), ks.len() - 1);
        for i in 0..ds.len() {
            let want = (ks[i + 1] - ks[i]).rem_euclid(4) as u8;
            assert_eq!(ds[i], want, "differential symbol {i}");
        }
        assert!(q.iter().all(|&x| x > 0.9), "clean signal -> high quality");
    }

    #[test]
    fn find_frame_and_find_uw_locate_the_unique_word() {
        // preamble of differential-zero, then the unique word, then payload
        let hint = 30usize;
        let mut ds = vec![0u8; hint];
        ds.extend_from_slice(&UW_SYMS);
        ds.extend_from_slice(&[1, 3, 2, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2]);
        assert_eq!(find_uw(&ds), Some(hint));
        let (p, d, _sc) = find_frame(&ds, hint, 24, 20, 3).expect("frame");
        assert_eq!(p, hint);
        assert_eq!(d, 0, "clean unique word has zero bit errors");
    }

    #[test]
    fn rrc_matches_python_center_and_energy() {
        let h = rrc(0.4, 8, 8);
        assert_eq!(h.len(), 2 * 8 * 8 + 1);
        let e: f64 = h.iter().map(|v| v * v).sum();
        assert!((e - 1.0).abs() < 1e-9, "unit energy {e}");
        // peak at center
        let mid = h.len() / 2;
        assert!(h[mid] > h[mid + 1] && h[mid] > h[mid - 1]);
    }

    #[test]
    fn find_bursts2_locates_injected_bursts() {
        // noise plus three ~20 ms tone bursts at known times and channels
        let fs = 4.0e6;
        let fc = 1626.25e6;
        let nsamp = (1.2 * fs) as usize;
        let mut state = 0x1234_5678_9abc_def0u64;
        let mut nz = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (((state >> 40) as i64 & 0xff) - 128) as f64 / 20.0
        };
        let mut iq = vec![0i8; nsamp * 2];
        for k in 0..nsamp {
            iq[2 * k] = nz().clamp(-127.0, 127.0) as i8;
            iq[2 * k + 1] = nz().clamp(-127.0, 127.0) as i8;
        }
        let inject = |iq: &mut [i8], t: f64, fcen: f64, amp: f64| {
            let start = (t * fs) as usize;
            let len = (0.0203 * fs) as usize;
            let w = 2.0 * PI * (fcen - fc) / fs;
            for k in 0..len {
                let ph = w * (start + k) as f64;
                let i = (iq[2 * (start + k)] as f64 + amp * ph.cos()).clamp(-127.0, 127.0);
                let q = (iq[2 * (start + k) + 1] as f64 + amp * ph.sin()).clamp(-127.0, 127.0);
                iq[2 * (start + k)] = i as i8;
                iq[2 * (start + k) + 1] = q as i8;
            }
        };
        inject(&mut iq, 0.30, 1626.100e6, 30.0);
        inject(&mut iq, 0.60, 1626.400e6, 30.0);
        inject(&mut iq, 0.90, 1626.250e6, 30.0);

        let bursts = find_bursts2(&iq, fc, fs, 1.2, 1626.0e6, 1626.5e6, 4.0);
        // each injected burst must be found at the right time and frequency
        for &(t, f) in &[(0.30, 1626.100e6), (0.60, 1626.400e6), (0.90, 1626.250e6)] {
            let hit = bursts
                .iter()
                .any(|b| (b.t - t).abs() < 0.01 && (b.fcen - f).abs() < 25e3);
            assert!(hit, "missed burst at t={t} f={f}; found {bursts:?}");
        }
        // and it must not invent bursts away from an injection (a pure CW tone
        // can split across adjacent channels, so allow several per injection but
        // none at an unrelated time/frequency)
        let inj = [(0.30, 1626.100e6), (0.60, 1626.400e6), (0.90, 1626.250e6)];
        for b in &bursts {
            let near = inj
                .iter()
                .any(|&(t, f)| (b.t - t).abs() < 0.03 && (b.fcen - f).abs() < 60e3);
            assert!(near, "spurious burst away from any injection: {b:?}");
        }
    }

    #[test]
    fn decimate_preserves_a_low_tone() {
        // a slow complex tone should survive decimation nearly unchanged
        let fs = 4.0e6;
        let f = 5000.0;
        let x: Vec<C32> = (0..8000)
            .map(|k| {
                let p = 2.0 * PI * f * k as f64 / fs;
                C32::new(p.cos() as f32, p.sin() as f32)
            })
            .collect();
        let y = decimate(&x, 4);
        assert_eq!(y.len(), 2000);
        // interior magnitude ~1
        let m: f32 = y[500..1500].iter().map(|v| v.norm()).sum::<f32>() / 1000.0;
        assert!((m - 1.0).abs() < 0.05, "tone magnitude {m}");
    }
}
