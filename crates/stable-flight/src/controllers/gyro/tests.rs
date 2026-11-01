use super::*;
use assert2::check;

/// With the notch off the chain is the low-pass alone, bit for bit, so airframes that
/// leave it off fly as before (#597).
#[test]
fn an_off_notch_leaves_the_low_pass_alone() {
    const DT: f32 = 0.004;
    let mut chain = GyroFilter::default();
    let mut low_pass = LowPass::default();
    let filters = GyroFilters::low_pass(40.0);
    for tick in 0..500 {
        let rate_radps = libm::sinf(0.37 * tick as f32) + 0.1 * libm::sinf(2.1 * tick as f32);
        check!(chain.apply(rate_radps, DT, &filters) == low_pass.apply(rate_radps, DT, 40.0));
    }
}
