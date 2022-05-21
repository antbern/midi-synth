#![no_std]
#![no_main]

mod defmt_uart;
mod dma;
mod generator;
mod i2s;
mod waveforms;

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
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
        FunctionUart,
    },
    pac,
    pio::PIOExt,
    sio::Sio,
    uart::{self, UartPeripheral},
    watchdog::Watchdog,
};

use crate::{generator::SineWave, i2s::I2SOutput};

/// Alias the type for our UART pins to make things clearer.
type UartPins = (
    hal::gpio::Pin<Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
    hal::gpio::Pin<Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
);

/// Alias the type for our UART to make things clearer.
type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;

/// Shared UART object so that it can be accessed from defmt
static GLOBAL_UART0: Mutex<RefCell<Option<Uart>>> = Mutex::new(RefCell::new(None));

static mut DMA_BUFFER: [i16; 16_000] = [0; 16_000];

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
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.write_full_blocking(b"Hello World!=n");

    // put the UART into the shared global
    cortex_m::interrupt::free(|cs| {
        GLOBAL_UART0.borrow(cs).replace(Some(uart));
    });

    // UART is now initialized, and we can start using defmt macros!
    info!("Program start");
    let mut led_pin = pins.led.into_push_pull_output();

    let sampling_freq = 16_000;

    let (pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut i2s = I2SOutput::new(
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

    let mut generator = SineWave::new(sampling_freq as f32, 440 as f32);

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

    //// now configure and do it with the DMA instead!

    dma::setup_double_buffered(&mut pac.RESETS, &pac.DMA, &i2s);

    // pac.DMA.ints0.modify(f)

    /*

    // unreset the DMA peripheral & wait for it to become available
    pac.RESETS.reset.modify(|_, w| w.dma().clear_bit());
    while !pac.RESETS.reset_done.read().dma().bit_is_set() {}

    // DMA channel can be 0 or 1, let's use channel 0 for now
    // #define PICO_AUDIO_I2S_DMA_IRQ 0
    let dma = &pac.DMA.ch[0];

    // setup read & write address
    dma.ch_write_addr
        .write(|w| unsafe { w.bits(i2s.tx_addr() as u32) });
    dma.ch_read_addr
        .write(|w| unsafe { w.bits((&DMA_BUFFER as *const i16) as u32) });

    // number of samples to take
    // dma.ch_trans_count
    //     .write(|w| unsafe { w.bits(16000 as u32) });

    // setup the rest of the parameters
    // dma.ch_ctrl_trig

    // writing to a non-triggering register to not trigger a new transfer
    dma.ch_al1_ctrl.modify(|_, w| {
        w.en()
            .bit(true) // enable chanel
            .data_size()
            .size_halfword() // we are sending i16
            .incr_write()
            .clear_bit() // do not increment write address
            .incr_read()
            .set_bit(); // increment read address

        unsafe { w.chain_to().bits(0) }; // Needs to be the same as the channel we are configuring

        unsafe { w.treq_sel().bits(i2s.tx_dreq_value()) };

        w
    });

    // write number of bytes and trigger!
    dma.ch_al1_trans_count_trig
        .write(|w| unsafe { w.bits(16000 as u32) });
        */

    // debug!("read_addr: {:?}", dma.ch_read_addr.read().bits());
    // debug!("read_addr(desired): {:?}", i2s.tx_addr() as u32);
    // debug!("write_addr: {:?}", dma.ch_write_addr.read().bits());
    // debug!("incr_read: {:?}", dma.ch_ctrl_trig.read().incr_read().bit());
    // debug!(
    //     "incr_write: {:?}",
    //     dma.ch_ctrl_trig.read().incr_write().bit()
    // );
    // debug!("en: {:?}", dma.ch_ctrl_trig.read().en().bit());
    // debug!("treq: {:?}", dma.ch_al1_ctrl.read().treq_sel().bits());
    // debug!("busy: {:?}", dma.ch_ctrl_trig.read().busy().bit());
    // debug!("ahb_error: {:?}", dma.ch_ctrl_trig.read().ahb_error().bit());

    while pac.DMA.ch[0].ch_ctrl_trig.read().busy().bit_is_set() {
        led_pin.set_high().unwrap();
        delay.delay_ms(50);
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
    }

    delay.delay_ms(1000);

    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(100);
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
    }
}

fn FILL_BUFFER(buffer: &mut [i16]) {
    for i in 0..buffer.len() {
        buffer[i] = unsafe { DMA_BUFFER[i] };
    }
}
