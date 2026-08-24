use num_complex::Complex;

/// Target standard deviation for ADC dynamic range optimization (HackRF 8-bit ADC).
/// Keeps signals well above quantization noise floor while providing headroom against ADC clipping.
pub const TARGET_STD_DEV: f32 = 12.0;

/// Default baseline LNA gain (dB) used when evaluating RF gain level adjustments.
pub const BASELINE_LNA_GAIN: u32 = 32;

/// Default baseline VGA gain (dB) used when evaluating RF gain level adjustments.
pub const BASELINE_VGA_GAIN: u32 = 40;

/// Subtracts the DC offset (mean of I and Q samples) from the given IQ buffer.
///
/// This eliminates the 0 Hz DC spike caused by local oscillator leakage or ADC DC bias
/// in direct-conversion SDR receivers like the HackRF.
///
/// # Arguments
/// * `iq_data` - Mutable slice of complex floating-point IQ samples.
pub fn remove_dc_offset(iq_data: &mut [Complex<f32>]) {
    if iq_data.is_empty() {
        return;
    }

    // Accumulate sums in f64 to avoid floating-point loss of precision on large buffers.
    let (sum_re, sum_im) = iq_data.iter().fold((0.0f64, 0.0f64), |(re, im), sample| {
        (re + sample.re as f64, im + sample.im as f64)
    });

    let n = iq_data.len() as f64;
    let mean_re = (sum_re / n) as f32;
    let mean_im = (sum_im / n) as f32;
    let mean = Complex::new(mean_re, mean_im);

    for sample in iq_data.iter_mut() {
        *sample -= mean;
    }
}

/// Analyzes ADC power variance/standard deviation of IQ samples and calculates optimal
/// HackRF LNA (IF) and VGA (Baseband) gain settings to maintain target standard deviation around ~12.0.
///
/// HackRF gain constraints:
/// - LNA gain: 0 to 40 dB in steps of 8 dB (0, 8, 16, 24, 32, 40).
/// - VGA gain: 0 to 62 dB in steps of 2 dB (0, 2, 4, ..., 62).
///
/// # Arguments
/// * `iq_data` - Slice of complex floating-point IQ samples.
/// * `current_lna` - Current LNA gain in dB (0..=40).
/// * `current_vga` - Current VGA gain in dB (0..=62).
///
/// # Returns
/// A tuple `(lna_gain, vga_gain)` with recommended gain values in dB.
pub fn calibrate_gain_levels(iq_data: &[Complex<f32>], current_lna: u32, current_vga: u32) -> (u32, u32) {
    if iq_data.is_empty() {
        return (current_lna.clamp(0, 40), current_vga.clamp(0, 62));
    }

    // Calculate mean for DC-debiased variance calculation
    let (sum_re, sum_im) = iq_data.iter().fold((0.0f64, 0.0f64), |(re, im), sample| {
        (re + sample.re as f64, im + sample.im as f64)
    });
    let n = iq_data.len() as f64;
    let mean_re = sum_re / n;
    let mean_im = sum_im / n;

    // Variance per real component: Var(I) + Var(Q) divided by 2
    let var_sum = iq_data.iter().fold(0.0f64, |acc, sample| {
        let diff_re = sample.re as f64 - mean_re;
        let diff_im = sample.im as f64 - mean_im;
        acc + diff_re * diff_re + diff_im * diff_im
    });

    // Component variance is total power variance divided by 2N (for I and Q components)
    let component_var = var_sum / (2.0 * n);
    let current_std_dev = component_var.sqrt() as f32;

    // If signal power is virtually zero, return max clean gain settings
    if current_std_dev < 1e-6 {
        return (40, 62);
    }

    // Gain change in dB required to scale current_std_dev to TARGET_STD_DEV (~12.0)
    let delta_g_db = 20.0 * (TARGET_STD_DEV / current_std_dev).log10();

    let current_total_gain = (current_lna + current_vga) as f32;
    let target_total_gain = (current_total_gain + delta_g_db).clamp(0.0, 102.0);

    quantize_gain_levels(target_total_gain)
}

/// Convenience function using baseline gains (32 dB LNA, 40 dB VGA).
pub fn calibrate_gain_baseline(iq_data: &[Complex<f32>]) -> (u32, u32) {
    calibrate_gain_levels(iq_data, BASELINE_LNA_GAIN, BASELINE_VGA_GAIN)
}

/// Helper function to quantize total target gain into valid HackRF LNA and VGA step levels.
/// Prioritizes LNA gain for low Noise Figure (NF) in GNSS reception.
fn quantize_gain_levels(target_total_gain: f32) -> (u32, u32) {
    // Allowed LNA gain values: 0, 8, 16, 24, 32, 40 (steps of 8)
    let lna_options = [40u32, 32, 24, 16, 8, 0];

    let mut best_lna = BASELINE_LNA_GAIN;
    let mut best_vga = BASELINE_VGA_GAIN;
    let mut min_error = f32::MAX;

    for &lna in &lna_options {
        let rem_vga = target_total_gain - lna as f32;
        // Quantize VGA to even integer between 0 and 62
        let vga_quant = (rem_vga / 2.0).round() as i32 * 2;
        let vga_clamped = vga_quant.clamp(0, 62) as u32;

        let total_achieved = lna + vga_clamped;
        let error = (total_achieved as f32 - target_total_gain).abs();

        if error < min_error {
            min_error = error;
            best_lna = lna;
            best_vga = vga_clamped;
        }
    }

    (best_lna, best_vga)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_remove_dc_offset() {
        let mut samples = vec![
            Complex::new(10.0, -5.0),
            Complex::new(12.0, -3.0),
            Complex::new(8.0, -7.0),
        ];

        remove_dc_offset(&mut samples);

        // Mean was (10.0, -5.0)
        assert!((samples[0].re - 0.0).abs() < 1e-5);
        assert!((samples[0].im - 0.0).abs() < 1e-5);
        assert!((samples[1].re - 2.0).abs() < 1e-5);
        assert!((samples[1].im - 2.0).abs() < 1e-5);
        assert!((samples[2].re - (-2.0)).abs() < 1e-5);
        assert!((samples[2].im - (-2.0)).abs() < 1e-5);
    }

    #[test]
    fn test_calibrate_gain_levels_at_target() {
        // Generate pseudo-Gaussian noise with std dev ~ 12.0
        let mut samples = Vec::new();
        for i in 0..1000 {
            let val = if i % 2 == 0 { 12.0 } else { -12.0 };
            samples.push(Complex::new(val, val));
        }

        let (lna, vga) = calibrate_gain_baseline(&samples);
        // Total gain should remain around 72 dB
        assert_eq!(lna + vga, 72);
    }

    #[test]
    fn test_calibrate_gain_levels_too_weak() {
        // Std dev = 6.0 (half of target 12.0) -> needs +6 dB gain
        let mut samples = Vec::new();
        for i in 0..1000 {
            let val = if i % 2 == 0 { 6.0 } else { -6.0 };
            samples.push(Complex::new(val, val));
        }

        let (lna, vga) = calibrate_gain_baseline(&samples);
        // Standard dev is 6.0, so needs +6 dB gain -> total 78 dB
        assert_eq!(lna + vga, 78);
    }

    #[test]
    fn test_calibrate_gain_levels_with_current_gains() {
        // Current gain: LNA 24, VGA 20 (44 dB)
        // Std dev = 6.0 -> needs +6 dB gain -> target total 50 dB
        let mut samples = Vec::new();
        for i in 0..1000 {
            let val = if i % 2 == 0 { 6.0 } else { -6.0 };
            samples.push(Complex::new(val, val));
        }

        let (lna, vga) = calibrate_gain_levels(&samples, 24, 20);
        assert_eq!(lna + vga, 50);
    }

    #[test]
    fn test_calibrate_gain_levels_too_strong() {
        // Std dev = 24.0 (double target 12.0) -> needs -6 dB gain
        let mut samples = Vec::new();
        for i in 0..1000 {
            let val = if i % 2 == 0 { 24.0 } else { -24.0 };
            samples.push(Complex::new(val, val));
        }

        let (lna, vga) = calibrate_gain_baseline(&samples);
        // Standard dev is 24.0, so needs -6 dB gain -> total 66 dB
        assert_eq!(lna + vga, 66);
    }

    #[test]
    fn test_empty_buffer() {
        let mut empty: Vec<Complex<f32>> = Vec::new();
        remove_dc_offset(&mut empty);
        let (lna, vga) = calibrate_gain_baseline(&empty);
        assert_eq!((lna, vga), (BASELINE_LNA_GAIN, BASELINE_VGA_GAIN));
    }
}
