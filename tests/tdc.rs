//! Tests for the TDC host side: thermometer popcount decoding and
//! code-density calibration (see firmware/fpga/dsp/tdc.py for the gateware).

use hackrf_gnss::tdc::{TdcCal, phase_ns, popcount_thermo};

#[test]
fn popcount_full_and_empty() {
    assert_eq!(popcount_thermo(&[0xff; 16]), 128);
    assert_eq!(popcount_thermo(&[0x00; 16]), 0);
}

#[test]
fn popcount_is_bubble_tolerant() {
    // 0b10111 has a bubble at bit 3: a plain left/right-edge position would
    // report 5 or 0; the popcount must report exactly 4.
    let mut bytes = [0u8; 16];
    bytes[0] = 0b0001_0111;
    assert_eq!(popcount_thermo(&bytes), 4);
}

#[test]
fn popcount_counts_across_byte_lanes() {
    // Byte 0 = taps 0-7, byte 1 = taps 8-15, ... (LSB-first within a byte).
    let mut bytes = [0u8; 16];
    bytes[0] = 0xff; // taps 0-7 set
    bytes[1] = 0x01; // tap 8 set
    bytes[7] = 0x80; // tap 63 set
    assert_eq!(popcount_thermo(&bytes), 10);
}

#[test]
fn uniform_histogram_gives_uniform_bins() {
    // Code-density calibration: a phase source uniform over the clock
    // period fills each bin in proportion to its width. A uniform
    // histogram over 100 populated bins of a 31.25 ns period must
    // therefore yield 312.5 ps per bin, +/-5%.
    let mut cal = TdcCal::new(31.25e-9);
    for pop in 0..100 {
        for _ in 0..1000 {
            cal.ingest(pop);
        }
    }
    let widths = cal.bin_widths_ps();
    assert_eq!(widths.len(), 100);
    for w in &widths {
        assert!(
            (*w - 312.5).abs() < 312.5 * 0.05,
            "bin width {w} outside 312.5 ps +/-5%"
        );
    }
    // Prefix sums: lut[0] = 0, lut[k] = sum of widths[0..k]; the 100
    // uniform bins tile the 31.25 ns period exactly.
    let lut = cal.lut_ps();
    assert_eq!(lut[0], 0.0);
    assert!((lut[99] - 99.0 * 312.5).abs() < 1.0, "lut[99] {} ps", lut[99]);
    assert!((lut[50] - 15625.0).abs() < 1.0);
    let total_ps: f64 = cal.bin_widths_ps().iter().sum();
    assert!((total_ps - 31250.0).abs() < 1.0, "period coverage {total_ps} ps");
    assert!((cal.widest_bin_ps() - 312.5).abs() < 312.5 * 0.05);
}

#[test]
fn empty_bins_have_zero_width_and_lut_stays_monotone() {
    // Bins the sampler never produced (e.g. past the end of a short chain)
    // get zero width; the LUT must remain monotone non-decreasing.
    let mut cal = TdcCal::new(25.0e-9);
    for _ in 0..500 {
        cal.ingest(10);
        cal.ingest(30);
    }
    let lut = cal.lut_ps();
    assert_eq!(lut.len(), 31);
    for w in lut.windows(2) {
        assert!(w[1] >= w[0]);
    }
    // All mass in two equal bins of 12.5 ns each.
    assert!((cal.bin_widths_ps()[10] - 12500.0).abs() < 1.0);
    assert!((cal.widest_bin_ps() - 12500.0).abs() < 1.0);
}

#[test]
fn phase_combines_coarse_ticks_and_fine_lut() {
    // lut: 100 uniform bins of 312.5 ps over a 31.25 ns period (32 MHz).
    let mut cal = TdcCal::new(31.25e-9);
    for pop in 0..100 {
        for _ in 0..100 {
            cal.ingest(pop);
        }
    }
    let lut = cal.lut_ps();
    // 1_000_000 ticks at 32 MHz = 31.25 ms; pop 40 -> 40 * 312.5 ps = 12.5 ns.
    let ph = phase_ns(1_000_000, 32e6, 40, &lut);
    assert!((ph - (31_250_000.0 + 12.5)).abs() < 0.5, "phase {ph} ns");
    // pop beyond the LUT clamps to the last entry instead of panicking.
    let ph_clamped = phase_ns(0, 32e6, 200, &lut);
    assert!((ph_clamped - 31.25).abs() < 0.5);
}
