use std::{env, process::exit, str::FromStr, thread};
use std::{fs::File, io::Read, path::PathBuf};

use anyhow::{Error, Result};

use cpal::{
    traits::{DeviceTrait, HostTrait, StreamTrait},
    FromSample, Stream,
};
use crossbeam_channel::Sender;

use anyhow::Context;

use crossbeam_channel::Receiver;
use synth::engine::{MidiEngine, SimpleMidiEngine};
use synth::midi::MidiCommand;

struct Engine {
    engine: SimpleMidiEngine,
    rx: Receiver<MidiCommand>,
}

impl Engine {
    fn new(rx: Receiver<MidiCommand>, sample_rate: f32) -> Self {
        Self {
            engine: SimpleMidiEngine::new(sample_rate),
            rx,
        }
    }
    fn update_state(&mut self) {
        // handle all available incoming MIDI messages
        for cmd in self.rx.try_iter() {
            println!("Recv: {:?}", cmd);
            self.engine.process_command(cmd);
        }
    }
    /// Used to fill the data buffer of samples.
    /// Is genering to accomodate the different data types required by different platforms.
    pub(crate) fn write_data<T>(&mut self, output: &mut [T], channels: usize)
    where
        T: cpal::Sample + FromSample<i16>,
    {
        // process incoming messages, update state
        self.update_state();

        // do both channels (left & right) at the same time
        for frame in output.chunks_mut(channels) {
            // get next value

            let value: T = T::from_sample(self.engine.next_sample());

            // and fill both channels
            for sample in frame.iter_mut() {
                *sample = value;
            }
        }
    }
}
fn read_loop(midi_device: &PathBuf, tx: Sender<MidiCommand>) -> Result<()> {
    println!("Opening MIDI device: {}", midi_device.display());

    // Open the MIDI input and read bytes!
    let mut midi = File::open(midi_device).context("Open MIDI device")?;

    let mut buffer = vec![0; 32];

    loop {
        let n = midi.read(&mut buffer)?;

        // parse messages (seems we get a single MIDI message each time!)
        let data = &buffer[0..n];
        let cmd = synth::midi::parse_midi(data);
        tx.send(cmd)?;
        // TODO: add debug logging statement here instead of this println
        // println!("{:x?} -> {:?}", data, cmd);
    }
}

fn main() -> Result<()> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: midi [device]");
        exit(1);
    }

    let midi_device = PathBuf::from_str(&args[1]).unwrap();

    // create channel for MIDI messages
    let (tx, rx) = crossbeam_channel::unbounded();

    // create sound engine stream and start it
    let stream = setup_sound(rx)?;
    stream.play()?;

    // lauch the MIDI reading thread (writes into the channel that is read by the engine)
    let handle = thread::spawn(move || read_loop(&midi_device, tx).unwrap());

    // main thread does nothing!

    // wait for the MIDI reading thread to finish before exiting
    handle.join().unwrap();

    Ok(())
}

fn setup_sound(rx: crossbeam_channel::Receiver<MidiCommand>) -> Result<Stream> {
    let host = cpal::default_host();

    let device = host
        .default_output_device()
        .expect("no output device available");

    let config = device.default_output_config().unwrap();
    println!("Default output config: {:?}", config);

    // construct the sound engine instance
    let engine = Engine::new(rx, config.sample_rate().0 as f32);

    match config.sample_format() {
        cpal::SampleFormat::F32 => construct_stream::<f32>(&device, &config.into(), engine),
        cpal::SampleFormat::I16 => construct_stream::<i16>(&device, &config.into(), engine),
        cpal::SampleFormat::U16 => construct_stream::<u16>(&device, &config.into(), engine),
        _ => Err(anyhow::anyhow!("No matching sample format found!")),
    }
}

fn construct_stream<T>(
    device: &cpal::Device,
    config: &cpal::StreamConfig,
    mut engine: Engine,
) -> Result<Stream, anyhow::Error>
where
    T: cpal::Sample + cpal::SizedSample + FromSample<i16>,
{
    let channels = config.channels as usize;

    device
        .build_output_stream(
            config,
            move |data: &mut [T], _: &cpal::OutputCallbackInfo| engine.write_data(data, channels),
            move |err| eprintln!("an error occurred on stream: {}", err),
            None,
        )
        .map_err(Error::msg)
}
