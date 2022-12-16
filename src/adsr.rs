#![allow(dead_code)]

#[derive(Debug, Clone, Copy)]
pub struct Parameters {
    attack_time_samples: u32, // time is in samples
    decay_time_samples: u32,
    sustain_level: f32,
    release_time_samples: u32,
}

impl Parameters {
    pub(crate) fn new(
        attack_time: f32,
        decay_time: f32,
        sustain_level: f32,
        release_time: f32,
        sample_rate: f32,
    ) -> Self {
        Self {
            attack_time_samples: (attack_time * sample_rate) as u32,
            decay_time_samples: (decay_time * sample_rate) as u32,
            sustain_level,
            release_time_samples: (release_time * sample_rate) as u32,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Idle,
    Attack,
    Decay,
    Sustain,
    Release,
    Done,
}

#[derive(Debug, Clone, Copy)]
pub struct PianoEnvelope {
    params: Parameters,
    state: State,
    value: f32,
    attack_step: f32,
    decay_step: f32,
    sustain_level: f32,
    hold_step: f32,
    release_step: f32,
    // sample_rate: f32,
}

impl PianoEnvelope {
    pub(crate) fn new(sample_rate: f32, params: Parameters, hold_decay_time: f32) -> Self {
        Self {
            params,
            state: State::Idle,
            value: 0.0, // start at 0
            attack_step: (1.0 - 0.0) / params.attack_time_samples as f32,
            decay_step: (1.0 - params.sustain_level) / params.decay_time_samples as f32,
            sustain_level: params.sustain_level,
            // hold_step: (params.sustain_level - 0.0) / (hold_decay_time * sample_rate),
            // hold_step: (-(2.0f32.ln() / hold_decay_time) * 1.0 / sample_rate).exp(),
            hold_step: 0.99998,
            release_step: (params.sustain_level - 0.0) / params.release_time_samples as f32,
        }
    }

    pub(crate) fn next_sample(&mut self, sustain: bool) -> f32 {
        use State::*;

        // self.counter += 1;

        match self.state {
            Idle => {
                self.value = 0.0;
            }
            Attack => {
                self.value += self.attack_step;

                if self.value >= 1.0 {
                    self.value = 1.0;
                    self.state = Decay;
                }
            }
            Decay => {
                self.value -= self.decay_step;

                if self.value <= self.sustain_level {
                    self.value = self.sustain_level;
                    self.state = Sustain;
                }
            }
            Sustain => {
                // self.value -= self.hold_step;

                self.value *= self.hold_step;

                if self.value <= 0.0 {
                    self.value = 0.0;
                    // We are done, set to Done
                    self.state = Done;
                }
            }
            Release => {
                if !sustain {
                    self.value -= self.release_step;
                } else {
                    // decay as in Sustain above
                    // self.value -= self.hold_step;
                    self.value *= self.hold_step;
                }

                if self.value <= 0.0 {
                    self.value = 0.0;
                    // We are done, set to Done
                    self.state = Idle;
                }
            }
            Done => {
                // TODO: remove as duplicate of Idle and never used
                self.value = 0.0;
            }
        }

        self.value
    }

    pub(crate) fn key_down(&mut self, velocity: u8) {
        // recalculate the attach speed based on the velocity
        // TODO: make this better
        //mod env attack duration scales with velocity (velocity of 1 is full duration, max velocity is 0.125 times duration)
        self.attack_step = (1.0 - 0.0)
            / (self.params.attack_time_samples as f32 * ((145.0 - velocity as f32) / 144.0));

        self.state = State::Attack;
    }

    pub(crate) fn key_up(&mut self) {
        if self.state != State::Release && self.state != State::Done {
            self.state = State::Release;
        }
    }

    pub(crate) fn is_active(&self) -> bool {
        self.state != State::Idle
    }
}
