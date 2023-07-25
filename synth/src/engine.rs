use crate::{
    adsr::Parameters,
    generator::EnvelopedGenerator,
    midi::{self, MidiCommand, CC_SUSTAIN, CC_VOLUME, MIDI_KEYS},
};

pub trait MidiEngine {
    /// Returns the next sample from the engine
    fn next_sample(&mut self) -> i16;

    /// Processes a MIDI command
    fn process_command(&mut self, command: MidiCommand);
}

pub struct SimpleMidiEngine {
    generators: [EnvelopedGenerator; MIDI_KEYS],
    volume: i16,
    sustain: bool,
}

impl SimpleMidiEngine {
    pub fn new(sample_rate: f32) -> Self {
        // SAFETY: from https://doc.rust-lang.org/stable/std/mem/union.MaybeUninit.html#initializing-an-array-element-by-element
        let generators = {
            let mut generators: [core::mem::MaybeUninit<EnvelopedGenerator>; MIDI_KEYS] =
                unsafe { core::mem::MaybeUninit::uninit().assume_init() };

            for (key, g) in &mut generators[..].iter_mut().enumerate() {
                let freq = midi::KEY_CENTER_FREQ[key] as f32;

                g.write(EnvelopedGenerator::new(
                    sample_rate,
                    freq,
                    Parameters::new(0.2, 0.1, 0.9, 0.3, sample_rate),
                ));
            }

            unsafe { core::mem::transmute::<_, [EnvelopedGenerator; MIDI_KEYS]>(generators) }
        };

        Self {
            generators,
            volume: 64, // 50%
            sustain: false,
        }
    }
}

impl MidiEngine for SimpleMidiEngine {
    fn process_command(&mut self, command: MidiCommand) {
        use midi::MidiCommand::*;

        // update state based on commands
        match command {
            NoteOn(key, vel) => self.generators[key as usize].key_down(vel),
            NoteOff(key, _vel) => self.generators[key as usize].key_up(),
            ContinousController(CC_VOLUME, val) => self.volume = val as i16, // volume / modulation wheel
            ContinousController(CC_SUSTAIN, val) => self.sustain = val > 0,  // sustain pedal
            _ => {}
        }
    }
    fn next_sample(&mut self) -> i16 {
        let mut sample = 0i32;

        for gen in self.generators.iter_mut().filter(|g| g.is_active()) {
            sample += gen.next(self.sustain) as i32
        }

        // apply volume control: Volume is 0-127, so double that to get 0-254 and we can do fast scaling
        let sample = sample.wrapping_mul(self.volume as i32) >> 8;

        // generate the sample
        (sample >> 2) as i16
    }
}
