#![no_std]
#![no_main]

mod defmt_uart;

use core::cell::RefCell;

use cortex_m::interrupt::{self, Mutex};
use cortex_m_rt::entry;
use defmt::*;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    gpio::{
        bank0::{Gpio0, Gpio1},
        FunctionPio0, FunctionUart, Pin,
    },
    pac,
    pio::{Buffers, PIOExt},
    sio::Sio,
    uart::{self, UartPeripheral},
    watchdog::Watchdog,
};

/// Alias the type for our UART pins to make things clearer.
type UartPins = (
    hal::gpio::Pin<Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
    hal::gpio::Pin<Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
);

/// Alias the type for our UART to make things clearer.
type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;

/// Shared UART object so that it can be accessed from defmt
static GLOBAL_UART0: Mutex<RefCell<Option<Uart>>> = Mutex::new(RefCell::new(None));

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

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
            uart::common_configs::_115200_8_N_1,
            clocks.peripheral_clock.into(),
        )
        .unwrap();

    uart.write_full_blocking(b"Hello World!=n");

    // put the UART into the shared global
    interrupt::free(|cs| {
        GLOBAL_UART0.borrow(cs).replace(Some(uart));
    });

    // UART is now initialized, and we can start using defmt macros!
    info!("Program start");

    // DATA: Pin GPIO2
    // BCLK: Pin GPIO3
    // LRCLCK: Pin GPIO4

    let data_pin = 2;
    let bclk_pin = 3;
    let lrclk_pin = 4;

    // configure pins for Pio0
    let _data: Pin<_, FunctionPio0> = pins.gpio2.into_mode();
    let _bclk: Pin<_, FunctionPio0> = pins.gpio3.into_mode();
    let _lrclk: Pin<_, FunctionPio0> = pins.gpio4.into_mode();

    // Create and start the I2S pio program
    let program = pio_proc::pio_file!("audio_i2s.pio", options());
    let _s = program.public_defines.entry_point;

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();

    let _d = installed.wrap_target();

    debug!("sysclk = {:?}", clocks.system_clock.freq().0);

    let sample_freq: u32 = 16_000; //32_000; // 16khz

    let system_clock_frequency = clocks.system_clock.freq().0;

    core::assert!(system_clock_frequency < 0x40000000);
    let divider = system_clock_frequency * 4 / sample_freq; // avoid arithmetic overflow
    core::assert!(divider < 0x1000000);

    debug!("divider >> 8 = {:?}", divider >> 8);
    debug!("divider & 0xff = {:?}", divider & 0xff);

    // construct the input the clock_divisor() call expects...
    let divisor = (divider >> 8) as f32 + (divider & 0xff) as f32 / 256.0;

    // just double check with the logic from clock_divisor()
    let int = divisor as u16;
    let frac = ((divisor - int as f32) * 256.0) as u8;

    debug!("int = {:?}", int);
    debug!("frac = {:?}", frac);

    let (mut sm, _rx, mut tx) = hal::pio::PIOBuilder::from_program(installed)
        .side_set_pin_base(bclk_pin)
        .out_pins(data_pin, 1)
        .out_shift_direction(hal::pio::ShiftDirection::Left)
        .autopull(true)
        .pull_threshold(32)
        .clock_divisor(divisor)
        .buffers(Buffers::OnlyTx)
        .build(sm0);

    // The GPIO pins need to be configured as an output.
    sm.set_pindirs([
        (data_pin, hal::pio::PinDir::Output),
        (bclk_pin, hal::pio::PinDir::Output),
        (lrclk_pin, hal::pio::PinDir::Output),
    ]);

    sm.exec_instruction(
        pio::InstructionOperands::JMP {
            condition: pio::JmpCondition::Always,
            address: 0x18 + 0x7 as u8,
        }
        .encode(),
    );

    sm.start();

    let mut led_pin = pins.led.into_push_pull_output();

    //generate a square wave output
    let freq = 440 * 2;

    let sample_counter_wrap = sample_freq / freq;
    let mut sample_counter = 0u32;
    loop {
        let sample = if sample_counter < sample_counter_wrap / 2 {
            i16::MAX
        } else {
            i16::MIN
        };

        // let sample = 0u32;
        //let lr_sample: u32 = ((sample as u16 as u32) << 16) | (sample as u16) as u32;
        // debug!("{:x}", sample);

        // push samples but do not advance if buffer is full
        if tx.write(sample >> 8) {
            sample_counter = (sample_counter + 1) % sample_counter_wrap;
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }

        // tx.write(0u32);
        // info!("on!");

        // delay.delay_ms(500);
        // info!("off!");

        // delay.delay_ms(500);
    }
}
