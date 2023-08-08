use core::f32::consts::PI;

use fixed::S15x16;

/// A simple Biquad filter.
///
/// Originally created by Created by Nigel Redmon on 11/24/12 using a permissive license.
///
///  For a complete explanation of the Biquad code:
///  http://www.earlevel.com/main/2012/11/26/biquad-c-source-code/
///

/// The type of a filter
#[derive(Debug, Clone, Copy)]
pub enum FilterType {
    Lowpass,
    Highpass,
    // Bandpass,
    // Notch,
    // Peak,
    // Lowshelf,
    // Highshelf,
}
pub struct Biquad {
    r#type: FilterType,
    a0: S15x16,
    a1: S15x16,
    a2: S15x16,
    b1: S15x16,
    b2: S15x16,
    z1: S15x16,
    z2: S15x16,
}

impl Biquad {
    pub fn new(r#type: FilterType, fc: f32, q: f32, peak_gain_dB: f32) -> Self {
        //let V: f32 = (peak_gain_dB.abs() / 20.0).powf(10.0);
        //let k: f32 = (PI * fc).tan();
        let k: f32 = libm::tan((PI * fc) as f64) as f32;

        let k: S15x16 = k.into();

        match r#type {
            FilterType::Lowpass => {
                let norm = 1.0 / (1.0 + k / q + k * k);
                let a0 = k * k * norm;

                Self {
                    r#type,
                    a0: a0.into(),
                    a1: (2.0 * a0).into(),
                    a2: a0.into(),
                    b1: (2.0 * (k * k - 1.0) * norm).into(),
                    b2: ((1.0 - k / q + k * k) * norm).into(),
                    z1: 0.0.into(),
                    z2: 0.0.into(),
                }
            }
            FilterType::Highpass => {
                let norm = 1.0 / (1.0 + k / q + k * k);
                let a0 = 1.0 * norm;
                Self {
                    r#type,
                    a0,
                    a1: -2.0 * a0,
                    a2: a0,
                    b1: 2.0 * (k * k - 1.0) * norm,
                    b2: (1.0 - k / q + k * k) * norm,
                    z1: 0.0.into(),
                    z2: 0.0.into(),
                }
            } // FilterType::Bandpass => todo!(),
              // FilterType::Notch => todo!(),
              // FilterType::Peak => todo!(),
              // FilterType::Lowshelf => todo!(),
              // FilterType::Highshelf => todo!(),
        }

        // switch (this->type) {
        //     case bq_type_lowpass:
        //         norm = 1 / (1 + K / Q + K * K);
        //         a0 = K * K * norm;
        //         a1 = 2 * a0;
        //         a2 = a0;
        //         b1 = 2 * (K * K - 1) * norm;
        //         b2 = (1 - K / Q + K * K) * norm;
        //         break;

        //     case bq_type_highpass:
        //         norm = 1 / (1 + K / Q + K * K);
        //         a0 = 1 * norm;
        //         a1 = -2 * a0;
        //         a2 = a0;
        //         b1 = 2 * (K * K - 1) * norm;
        //         b2 = (1 - K / Q + K * K) * norm;
        //         break;

        //     case bq_type_bandpass:
        //         norm = 1 / (1 + K / Q + K * K);
        //         a0 = K / Q * norm;
        //         a1 = 0;
        //         a2 = -a0;
        //         b1 = 2 * (K * K - 1) * norm;
        //         b2 = (1 - K / Q + K * K) * norm;
        //         break;

        //     case bq_type_notch:
        //         norm = 1 / (1 + K / Q + K * K);
        //         a0 = (1 + K * K) * norm;
        //         a1 = 2 * (K * K - 1) * norm;
        //         a2 = a0;
        //         b1 = a1;
        //         b2 = (1 - K / Q + K * K) * norm;
        //         break;

        //     case bq_type_peak:
        //         if (peakGain >= 0) {    // boost
        //             norm = 1 / (1 + 1/Q * K + K * K);
        //             a0 = (1 + V/Q * K + K * K) * norm;
        //             a1 = 2 * (K * K - 1) * norm;
        //             a2 = (1 - V/Q * K + K * K) * norm;
        //             b1 = a1;
        //             b2 = (1 - 1/Q * K + K * K) * norm;
        //         }
        //         else {    // cut
        //             norm = 1 / (1 + V/Q * K + K * K);
        //             a0 = (1 + 1/Q * K + K * K) * norm;
        //             a1 = 2 * (K * K - 1) * norm;
        //             a2 = (1 - 1/Q * K + K * K) * norm;
        //             b1 = a1;
        //             b2 = (1 - V/Q * K + K * K) * norm;
        //         }
        //         break;
        //     case bq_type_lowshelf:
        //         if (peakGain >= 0) {    // boost
        //             norm = 1 / (1 + sqrt(2) * K + K * K);
        //             a0 = (1 + sqrt(2*V) * K + V * K * K) * norm;
        //             a1 = 2 * (V * K * K - 1) * norm;
        //             a2 = (1 - sqrt(2*V) * K + V * K * K) * norm;
        //             b1 = 2 * (K * K - 1) * norm;
        //             b2 = (1 - sqrt(2) * K + K * K) * norm;
        //         }
        //         else {    // cut
        //             norm = 1 / (1 + sqrt(2*V) * K + V * K * K);
        //             a0 = (1 + sqrt(2) * K + K * K) * norm;
        //             a1 = 2 * (K * K - 1) * norm;
        //             a2 = (1 - sqrt(2) * K + K * K) * norm;
        //             b1 = 2 * (V * K * K - 1) * norm;
        //             b2 = (1 - sqrt(2*V) * K + V * K * K) * norm;
        //         }
        //         break;
        //     case bq_type_highshelf:
        //         if (peakGain >= 0) {    // boost
        //             norm = 1 / (1 + sqrt(2) * K + K * K);
        //             a0 = (V + sqrt(2*V) * K + K * K) * norm;
        //             a1 = 2 * (K * K - V) * norm;
        //             a2 = (V - sqrt(2*V) * K + K * K) * norm;
        //             b1 = 2 * (K * K - 1) * norm;
        //             b2 = (1 - sqrt(2) * K + K * K) * norm;
        //         }
        //         else {    // cut
        //             norm = 1 / (V + sqrt(2*V) * K + K * K);
        //             a0 = (1 + sqrt(2) * K + K * K) * norm;
        //             a1 = 2 * (K * K - 1) * norm;
        //             a2 = (1 - sqrt(2) * K + K * K) * norm;
        //             b1 = 2 * (K * K - V) * norm;
        //             b2 = (V - sqrt(2*V) * K + K * K) * norm;
        //         }
        //         break;
        // }
    }

    /// Apply the filter to the sample
    pub fn process(&mut self, sample: S15x16) -> S15x16 {
        let out = sample * self.a0 + self.z1;
        self.z1 = sample * self.a1 + self.z2 - self.b1 * out;
        self.z2 = sample * self.a2 - self.b2 * out;
        out
    }
}
