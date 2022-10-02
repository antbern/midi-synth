use rp_pico as bsp;

use bsp::{hal::pac, hal::pac::interrupt, pac::Peripherals};

use defmt::*;

#[derive(Clone, Copy, Debug, Format)]
pub enum MidiCommand {
    NoteOff(u8, u8),             // (key, velocity)
    NoteOn(u8, u8),              // (key, velocity)
    Aftertouch(u8, u8),          // (key, touch)
    ContinousController(u8, u8), // (controller #, value)
    PatchChange(u8),             // (instrument #)
    ChannelPressure(u8),         // (pressure)
    PitchBend(u16),              // (amount)
    Unknown(u8),                 // (unknown command byte)
}

/// Parses a MidiCommand from recieved Midi data.
/// See: https://ccrma.stanford.edu/~craig/articles/linuxmidi/misc/essenmidi.html
pub fn parse_midi(data: &[u8]) -> MidiCommand {
    use MidiCommand::*;
    match data[0] {
        0x80 if data.len() >= 3 => NoteOff(data[1], data[2]),
        0x90 if data.len() >= 3 => NoteOn(data[1], data[2]),
        0xA0 if data.len() >= 3 => Aftertouch(data[1], data[2]),
        0xB0 if data.len() >= 3 => ContinousController(data[1], data[2]),
        0xC0 if data.len() >= 2 => PatchChange(data[1]),
        0xD0 if data.len() >= 2 => ChannelPressure(data[1]),
        0xE0 if data.len() >= 3 => PitchBend(data[1] as u16 | (data[2] as u16) << 8),
        n => Unknown(n),
    }
}

#[interrupt]
fn UART1_IRQ() {
    // todo: read the buffer values and parse the MIDI messages here
    debug!("UART1_IRQ");

    cortex_m::interrupt::free(|cs| {
        let uart1 = crate::GLOBAL_UART1.borrow(cs).borrow();

        let uart_midi = uart1.as_ref().unwrap();

        // the messages we are interested in are all 3 bytes long, so let's only read 3 bytes
        let mut buffer = [0u8; 3];

        // try to read the bytes and parse them if available
        match uart_midi.read_raw(&mut buffer) {
            Ok(bytes) if bytes >= 3 => {
                let msg = parse_midi(&buffer);
                // TODO: push the command into the command buffer that is processed by the main loop
                debug!("Read: {=[u8]:#x} => {}", buffer, msg);
            }
            _ => {}
        }
    });
}
