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
use fugit::{Instant, RateExtU32};
use util::GlobalCell;

use synth::engine::{MidiEngine, SimpleMidiEngine};

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

static mut TIMER: Option<hal::Timer> = None;

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

    unsafe {
        TIMER = Some(hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS));
    }

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

    // Open UART for incoming MIDI messages (uses pin 9 for RX only)
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

    // put the UART into the shared global
    cortex_m::interrupt::free(|cs| {
        GLOBAL_UART1.borrow(cs).replace(Some(uart_midi));
    });

    // finally enable the interrupt in the NVIC
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::UART1_IRQ);
    }

    // UART is now initialized, and we can start using defmt macros!
    info!("Program start");
    let mut led_pin = pins.led.into_push_pull_output();

    let sampling_freq = 24_000;

    let (pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let i2s = i2s::I2SOutput::new(
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

    dma::setup_double_buffered(&mut pac.RESETS, &pac.DMA, &i2s);

    while pac.DMA.ch[0].ch_ctrl_trig.read().busy().bit_is_set() {
        led_pin.set_high().unwrap();
        delay.delay_ms(50);
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
    }

    // initialize the engine
    cortex_m::interrupt::free(|cs| ENGINE.put_cs(cs, SimpleMidiEngine::new(sampling_freq as f32)));

    loop {
        // handle all pending Midi Commands
        while let Some(cmd) = cortex_m::interrupt::free(|cs| MIDI_COMMAND_BUFFER.take(cs)) {
            cortex_m::interrupt::free(|cs| {
                ENGINE.try_borrow_mut(cs, |s| {
                    s.process_command(cmd);
                    Some(0)
                });
            });
        }
    }
}

static ENGINE: GlobalCell<SimpleMidiEngine> = GlobalCell::new(None);

fn FILL_BUFFER(buffer: &mut [i16]) {
    // get the current state of the midi engine]

    let t = unsafe { &TIMER.as_ref().unwrap_unchecked() };
    let start = t.get_counter();

    cortex_m::interrupt::free(|cs| {
        ENGINE.try_borrow_mut(cs, |engine| {
            (0..buffer.len()).for_each(|i| {
                // generate the samples
                buffer[i] = engine.next_sample();
            });
            Some(0)
        });
    });

    let diff = t.get_counter() - start;

    debug!(
        "FILL_BUFFER took {:?} microseconds to fill {} samples",
        diff.ticks(),
        buffer.len()
    );
}
