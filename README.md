# Pico Synth

My take on trying to create a device that can be used to generate noise from MIDI messages!
Uses a Raspberry Pi Pico and MAX98357A I2S amplifier to process the messages and play _music_.

There is also a command line application available that does the same thing but through the computer instead.
That way, development cycle can be kept short!

To facilitate this, the project is divided into three crates:

- [`cli`](cli/) The command line application, using [`cpal`](https://crates.io/crates/cpal) for audio output.
- [`pico`](pico/) The firmware for the Raspberry Pi Pico board.
- [`synth`](synth/) A common `no_std` crate that contains the actual engine to generate audio samples.
  It is used by the other two binary crates.



## Features and TODOs

- [x] Basic sine wave generation.
- [x] MIDI input.
- [x] Note On/Off amplitude envelope.
- [ ] Other types of waves: square, saw, triangle.
- [ ] Low- and high-pass filters.
- [ ] Using fixed point integer math (currently uses `i16`, but the pico is a 32-bit device).
- [ ] Using MIDI control messages to change filter, envelope parameters and wave form.
