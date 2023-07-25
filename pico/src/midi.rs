use rp_pico as bsp;

use bsp::{hal::pac, hal::pac::interrupt, pac::Peripherals};

use defmt::*;

use crate::MIDI_COMMAND_BUFFER;

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

        if let Some(uart_midi) = uart1.as_ref() {
            // the messages we are interested in are all 3 bytes long, so let's only read 3 bytes
            let mut buffer = [0u8; 3];

            // try to read the bytes and parse them if available
            match uart_midi.read_raw(&mut buffer) {
                Ok(bytes) if bytes >= 3 => {
                    let msg = parse_midi(&buffer);
                    // TODO: push the command into the command buffer that is processed by the main loop
                    debug!("Read: {=[u8]:#x} => {}", buffer, msg);

                    MIDI_COMMAND_BUFFER.put(cs, msg);
                }
                _ => {}
            }
        }
    });
}

/// This function calculates the center frequency for a given MIDI key
/// Source: https://www.inspiredacoustics.com/en/MIDI_note_numbers_and_center_frequencies
// fn key_center_frequency(key: u8) -> f64 {
//     440.0 * ((key as f64 - 69.0) / 12.0).exp2()
// }

// lazy_static! {
//     #[derive(Debug)]
//     /// Table of key center frequencies
//     pub(crate) static ref CENTER_FREQUENCIES: [f64; 0x80] = {
//         let mut freqs = [0.0; 0x80];

//         for (i, f) in freqs.iter_mut().enumerate() {
//             *f = key_center_frequency(i as u8);
//         }

//         freqs
//     };
// }

pub const KEY_CENTER_FREQ: [f64; 0x80] = [
    8.175798915643707,
    8.661957218027252,
    9.177023997418988,
    9.722718241315029,
    10.300861153527183,
    10.913382232281373,
    11.562325709738575,
    12.249857374429663,
    12.978271799373287,
    13.75,
    14.567617547440307,
    15.433853164253883,
    16.351597831287414,
    17.323914436054505,
    18.354047994837977,
    19.445436482630058,
    20.601722307054366,
    21.826764464562746,
    23.12465141947715,
    24.499714748859326,
    25.956543598746574,
    27.5,
    29.13523509488062,
    30.86770632850775,
    32.70319566257483,
    34.64782887210901,
    36.70809598967594,
    38.890872965260115,
    41.20344461410875,
    43.653528929125486,
    46.2493028389543,
    48.999429497718666,
    51.91308719749314,
    55.0,
    58.27047018976124,
    61.7354126570155,
    65.40639132514966,
    69.29565774421802,
    73.41619197935188,
    77.78174593052023,
    82.4068892282175,
    87.30705785825097,
    92.4986056779086,
    97.99885899543733,
    103.82617439498628,
    110.0,
    116.54094037952248,
    123.47082531403103,
    130.8127826502993,
    138.59131548843604,
    146.8323839587038,
    155.56349186104046,
    164.81377845643496,
    174.61411571650194,
    184.9972113558172,
    195.99771799087463,
    207.65234878997256,
    220.0,
    233.08188075904496,
    246.94165062806206,
    261.6255653005986,
    277.1826309768721,
    293.6647679174076,
    311.1269837220809,
    329.6275569128699,
    349.2282314330039,
    369.9944227116344,
    391.99543598174927,
    415.3046975799451,
    440.0,
    466.1637615180899,
    493.8833012561241,
    523.2511306011972,
    554.3652619537442,
    587.3295358348151,
    622.2539674441618,
    659.2551138257398,
    698.4564628660078,
    739.9888454232688,
    783.9908719634985,
    830.6093951598903,
    880.0,
    932.3275230361799,
    987.7666025122483,
    1046.5022612023945,
    1108.7305239074883,
    1174.6590716696303,
    1244.5079348883237,
    1318.5102276514797,
    1396.9129257320155,
    1479.9776908465376,
    1567.981743926997,
    1661.2187903197805,
    1760.0,
    1864.6550460723597,
    1975.533205024496,
    2093.004522404789,
    2217.4610478149766,
    2349.31814333926,
    2489.0158697766474,
    2637.02045530296,
    2793.825851464031,
    2959.955381693075,
    3135.9634878539946,
    3322.437580639561,
    3520.0,
    3729.3100921447194,
    3951.066410048992,
    4186.009044809578,
    4434.922095629953,
    4698.63628667852,
    4978.031739553295,
    5274.04091060592,
    5587.651702928062,
    5919.91076338615,
    6271.926975707989,
    6644.875161279122,
    7040.0,
    7458.620184289437,
    7902.132820097988,
    8372.018089619156,
    8869.844191259906,
    9397.272573357044,
    9956.06347910659,
    10548.081821211836,
    11175.303405856126,
    11839.8215267723,
    12543.853951415975,
];
