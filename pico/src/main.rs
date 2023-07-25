#![no_std]
#![no_main]

mod defmt_uart;
mod dma;
mod i2s;
mod midi;
mod queue;
mod util;

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use defmt::*;
use embedded_hal::digital::v2::OutputPin;

use panic_probe as _;

use queue::Queue;
use rp_pico as bsp;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    gpio::{
        bank0::{Gpio0, Gpio1, Gpio8, Gpio9},
        FunctionUart,
    },
    pac,
    pio::PIOExt,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    watchdog::Watchdog,
};
use fugit::RateExtU32;
use util::GlobalCell;

use synth::{adsr::Parameters, generator::EnvelopedGenerator, generator::SineWave};

/// Alias the type for our UART pins to make things clearer.
type Uart0Pins = (
    hal::gpio::Pin<Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
    hal::gpio::Pin<Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
);

type Uart1Pins = (
    hal::gpio::Pin<Gpio8, hal::gpio::Function<hal::gpio::Uart>>,
    hal::gpio::Pin<Gpio9, hal::gpio::Function<hal::gpio::Uart>>,
);

/// Alias the type for our UART to make things clearer.
type Uart0 = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, Uart0Pins>;
type Uart1 = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART1, Uart1Pins>;

/// Shared UART object so that it can be accessed from defmt
static GLOBAL_UART0: Mutex<RefCell<Option<Uart0>>> = Mutex::new(RefCell::new(None));
static GLOBAL_UART1: Mutex<RefCell<Option<Uart1>>> = Mutex::new(RefCell::new(None));

static MIDI_COMMAND_BUFFER: Queue<32, synth::midi::MidiCommand> = Queue::new();

static mut DMA_BUFFER: [i16; 16_000] = [0; 16_000];

const MIDI_KEYS: usize = 127;
const CC_SUSTAIN: u8 = 64u8;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (
        pins.gpio0.into_mode::<FunctionUart>(),
        pins.gpio1.into_mode::<FunctionUart>(),
    );

    let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.write_full_blocking(b"Hello World!=n");

    // put the UART into the shared global
    cortex_m::interrupt::free(|cs| {
        GLOBAL_UART0.borrow(cs).replace(Some(uart));
    });

    // Open UART for incoming MIDI messages
    let uart_pins = (
        pins.gpio8.into_mode::<FunctionUart>(),
        pins.gpio9.into_mode::<FunctionUart>(),
    );

    let mut uart_midi = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(31250.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart_midi.enable_rx_interrupt();

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::UART1_IRQ);
    }
    // put the UART into the shared global
    cortex_m::interrupt::free(|cs| {
        GLOBAL_UART1.borrow(cs).replace(Some(uart_midi));
    });

    // UART is now initialized, and we can start using defmt macros!
    info!("Program start");
    let mut led_pin = pins.led.into_push_pull_output();

    let sampling_freq = 16_000;

    let (pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut i2s = i2s::I2SOutput::new(
        (
            pins.gpio2.into_mode(),
            pins.gpio3.into_mode(),
            pins.gpio4.into_mode(),
        ),
        sampling_freq,
        &clocks,
        pio,
        sm0,
    );

    // fill the buffer first

    let mut generator = SineWave::new(sampling_freq as f32, 440_f32);

    for i in 0..unsafe { DMA_BUFFER.len() } {
        let sample = generator.next();
        let sample = sample >> 5; // adjust volume

        // SAFETY: we are only writing to this once, while no one else is doing it
        unsafe { DMA_BUFFER[i] = sample };
    }

    //// test direct I2S output first
    let mut next_sample = 0;
    for _ in 0..sampling_freq {
        // try to write sample but retry if buffer is full
        while !i2s.write(next_sample >> 5) {}

        next_sample = generator.next();
    }

    led_pin.set_high().unwrap();
    delay.delay_ms(1000);
    led_pin.set_low().unwrap();

    // setup one generator for each key

    // SAFETY: from https://doc.rust-lang.org/stable/std/mem/union.MaybeUninit.html#initializing-an-array-element-by-element
    let generators = {
        let mut generators: [core::mem::MaybeUninit<EnvelopedGenerator>; MIDI_KEYS] =
            unsafe { core::mem::MaybeUninit::uninit().assume_init() };

        for (key, g) in &mut generators[..].iter_mut().enumerate() {
            let freq = synth::midi::KEY_CENTER_FREQ[key as usize] as f32;

            g.write(EnvelopedGenerator::new(
                sampling_freq as f32,
                freq as f32,
                Parameters::new(0.2, 0.1, 0.9, 0.3, sampling_freq as f32),
            ));
        }

        unsafe { core::mem::transmute::<_, [EnvelopedGenerator; MIDI_KEYS]>(generators) }
    };

    // SAFETY: we access this before the interrupt that uses this has a chance to get called
    unsafe { GENERATORS.replace(generators) };

    //// now configure and do it with the DMA instead!

    dma::setup_double_buffered(&mut pac.RESETS, &pac.DMA, &i2s);

    while pac.DMA.ch[0].ch_ctrl_trig.read().busy().bit_is_set() {
        led_pin.set_high().unwrap();
        delay.delay_ms(50);
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
    }

    // cortex_m::asm::wfi();

    // big loop for keeping track of pressed keys
    // let mut key_on = [false; MIDI_KEYS];

    // put something into the state
    cortex_m::interrupt::free(|cs| {
        STATE.put_cs(
            cs,
            MidiState {
                volume: 64, // 50%
                sustain: false,
            },
        )
    });

    loop {
        if let Some(mut state) =
            cortex_m::interrupt::free(|cs| STATE.try_borrow_mut(cs, |s| Some(s.clone())))
        {
            // handle all pending Midi Commands
            while let Some(cmd) = cortex_m::interrupt::free(|cs| MIDI_COMMAND_BUFFER.take(cs)) {
                use synth::midi::MidiCommand::*;
                // update state based on commants
                match cmd {
                    NoteOn(key, vel) => {
                        if let Some(g) = unsafe { &mut GENERATORS } {
                            g[key as usize].key_down(vel)
                        }
                    }
                    NoteOff(key, _vel) => {
                        if let Some(g) = unsafe { &mut GENERATORS } {
                            g[key as usize].key_up()
                        }
                    }
                    ContinousController(1, val) => state.volume = val as i16, // volume / modulation wheel
                    ContinousController(CC_SUSTAIN, val) => state.sustain = val > 0, // sustain pedal
                    _ => {}
                }
            }

            // update global version of the state here
            cortex_m::interrupt::free(|cs| {
                STATE.try_borrow_mut(cs, |s| {
                    s.sustain = state.sustain;
                    s.volume = state.volume;
                    Some(0)
                });
            });
        }

        // use the current state to generate samples?
    }

    delay.delay_ms(1000);

    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(100);
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
    }
}

// struct MidiEngine {}

static STATE: GlobalCell<MidiState> = GlobalCell::new(None);

static mut GENERATORS: Option<[EnvelopedGenerator; MIDI_KEYS]> = None;

#[derive(Clone)]
struct MidiState {
    volume: i16,
    sustain: bool,
}
fn FILL_BUFFER(buffer: &mut [i16]) {
    // get the current state of the midi engine
    if let Some(state) =
        cortex_m::interrupt::free(|cs| STATE.try_borrow_mut(cs, |s| Some(s.clone())))
    {
        if let Some(g) = unsafe { &mut GENERATORS } {
            for i in 0..buffer.len() {
                // and compute the next sample

                let mut sample = 0i32;

                for gen in g.iter_mut().filter(|g| g.is_active()) {
                    sample += gen.next(state.sustain) as i32
                }

                // apply volume control: Volume is 0-127, so double that to get 0-254 and we can do fast scaling
                let sample = sample.wrapping_mul(state.volume as i32) >> 8;

                // generate the samples
                buffer[i] = (sample >> 2) as i16;
            }
        }
    }
}
