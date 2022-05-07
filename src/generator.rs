use crate::waveforms::SINE_TABLE;

///
///
/// Implementation based on James Munns [blog post]
///
/// [blog post]: https://jamesmunns.com/blog/fixed-point-math/
pub struct SineWave {
    incr: i32,
    cur_offset: i32,
}

impl SineWave {
    pub fn new(sample_rate: f32, frequency: f32) -> Self {
        let samp_per_cycle = sample_rate / frequency;

        let fincr = 256.0 / samp_per_cycle;

        let incr: i32 = (((1 << 24) as f32) * fincr) as i32;

        SineWave {
            incr,
            cur_offset: 0,
        }
    }

    pub fn next(&mut self) -> i16 {
        let val = self.cur_offset as u32;

        let idx_now = ((val >> 24) & 0xFF) as u8;

        let idx_nxt = idx_now.wrapping_add(1);

        let base_val = SINE_TABLE[idx_now as usize] as i32;
        let next_val = SINE_TABLE[idx_nxt as usize] as i32;

        let off = ((val >> 16) & 0xFF) as i32;

        let cur_weight = base_val.wrapping_mul(256i32.wrapping_sub(off));
        let nxt_weight = next_val.wrapping_mul(off);
        let ttl_weight = cur_weight.wrapping_add(nxt_weight);
        let ttl_val = ttl_weight >> 8; // div 256

        // Un-sign-extend this back to an i16, to use as a sample
        let ttl_val = ttl_val as i16;

        self.cur_offset = self.cur_offset.wrapping_add(self.incr);

        ttl_val
    }
}
