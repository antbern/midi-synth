use rp_pico as bsp;

use bsp::hal::pac::interrupt;

use defmt::*;
use synth::midi;

use crate::MIDI_COMMAND_BUFFER;

/// Internal shadow to add nice defmt printing
struct MidiCommandFmt(midi::MidiCommand);

impl defmt::Format for MidiCommandFmt {
    fn format(&self, fmt: Formatter) {
        match self.0 {
            midi::MidiCommand::NoteOff(a, b) => defmt::write!(fmt, "NoteOff ({},{})", a, b),
            midi::MidiCommand::NoteOn(a, b) => defmt::write!(fmt, "NoteOn ({},{})", a, b),
            midi::MidiCommand::Aftertouch(a, b) => defmt::write!(fmt, "Aftertouch ({},{})", a, b),
            midi::MidiCommand::ContinousController(a, b) => {
                defmt::write!(fmt, "ContinousController ({},{})", a, b)
            }
            midi::MidiCommand::PatchChange(a) => defmt::write!(fmt, "PatchChange ({})", a),
            midi::MidiCommand::ChannelPressure(a) => defmt::write!(fmt, "ChannelPressure ({})", a),
            midi::MidiCommand::PitchBend(a) => defmt::write!(fmt, "PitchBend ({})", a),
            midi::MidiCommand::Unknown(a) => defmt::write!(fmt, "Unknown ({})", a),
        }
    }
}

#[interrupt]
fn UART1_IRQ() {
    // todo: read the buffer values and parse the MIDI messages here
    debug!("UART1_IRQ");

    cortex_m::interrupt::free(|cs| {
        let uart1 = crate::GLOBAL_UART1.borrow(cs).borrow();

        if let Some(uart_midi) = uart1.as_ref() {
            // the messages we are interested in are all 3 bytes long, so let's only read 3 bytes
            let mut buffer = [0u8; 3];

            // try to read the bytes and parse them if available
            match uart_midi.read_raw(&mut buffer) {
                Ok(bytes) if bytes >= 3 => {
                    let msg = midi::parse_midi(&buffer);
                    // TODO: push the command into the command buffer that is processed by the main loop
                    debug!("Read: {=[u8]:#x} => {}", buffer, MidiCommandFmt(msg));

                    MIDI_COMMAND_BUFFER.put(cs, msg);
                }
                _ => {}
            }
        }
    });
}
