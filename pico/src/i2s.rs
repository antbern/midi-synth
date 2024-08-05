use rp_pico::{
    hal::{
        clocks::ClocksManager,
        gpio::{
            bank0::{Gpio2, Gpio3, Gpio4},
            FunctionPio0, Pin, PullDown,
        },
        pio::{
            Buffers, PIOBuilder, PinDir, Running, ShiftDirection, StateMachine, Tx,
            UninitStateMachine, PIO, SM0,
        },
        Clock,
    },
    pac::PIO0,
};

/// An object capable of outputing I2S sound data.
/// Currently hardcoded to use PIO0 with the following pinout:
///  * DATA:   GPIO2
///  * BCLK:   GPIO3
///  * LRCLCK: GPIO4
///
/// Most of the I2S implementation (including the PIO program) is taken from the
/// [`pico_audio_i2s`] example in [`pico_extras`].
///
/// [`pico_audio_i2s`]: (https://github.com/raspberrypi/pico-extras/blob/master/src/rp2_common/pico_audio_i2s)
/// [`pico_extras`]: (https://github.com/raspberrypi/pico-extras)
pub struct I2SOutput {
    _pio: PIO<PIO0>,
    _sm: StateMachine<(PIO0, SM0), Running>,
    tx: Tx<(PIO0, SM0)>,
}

impl I2SOutput {
    pub fn new(
        _pins: (
            Pin<Gpio2, FunctionPio0, PullDown>,
            Pin<Gpio3, FunctionPio0, PullDown>,
            Pin<Gpio4, FunctionPio0, PullDown>,
        ),
        sampling_freq: u32,
        clocks: &ClocksManager,
        mut pio: PIO<PIO0>,
        sm0: UninitStateMachine<(PIO0, SM0)>,
    ) -> I2SOutput {
        // pins are hard-wired for now
        let data_pin = 2;
        let bclk_pin = 3;
        let lrclk_pin = 4;

        assert_eq!(lrclk_pin, bclk_pin + 1); // pins need to be consecutive for it to work with the PIO

        // Create and start the I2S pio program
        let program = pio_proc::pio_file!("audio_i2s.pio", options());

        // Initialize and start PIO
        let installed = pio.install(&program.program).unwrap();

        let offset = installed.wrap_target();
        let entry_point = program.public_defines.entry_point as u8;

        // calculate the PIO frequency required for the desired sampling_frequency
        // taken from the official example
        let system_clock_frequency = clocks.system_clock.freq().to_Hz();
        core::assert!(system_clock_frequency < 0x40000000);
        let divider = system_clock_frequency * 4 / sampling_freq; // avoid arithmetic overflow
        core::assert!(divider < 0x1000000);

        // finally setup the PIO program with program-specific details
        let (mut sm, _rx, tx) = PIOBuilder::from_program(installed)
            .side_set_pin_base(bclk_pin)
            .out_pins(data_pin, 1)
            .out_shift_direction(ShiftDirection::Left)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point((divider >> 8) as u16, (divider & 0xff) as u8)
            .buffers(Buffers::OnlyTx)
            .build(sm0);

        // The GPIO pins need to be configured as outputs
        sm.set_pindirs([
            (data_pin, PinDir::Output),
            (bclk_pin, PinDir::Output),
            (lrclk_pin, PinDir::Output),
        ]);

        // needed to get the execution to start at the right point (as done in the example)
        sm.exec_instruction(pio::Instruction {
            operands: pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: offset + entry_point,
            },
            delay: 0,
            side_set: Some(0),
        });

        let sm = sm.start();

        I2SOutput {
            _pio: pio,
            _sm: sm,
            tx,
        }
    }

    // pub fn write(&mut self, sample: i16) -> bool {
    //     self.tx.write_u16_replicated(sample as u16)
    // }

    pub fn tx_addr(&self) -> *const u32 {
        self.tx.fifo_address()
    }

    pub fn tx_dreq_value(&self) -> u8 {
        self.tx.dreq_value()
    }
}
