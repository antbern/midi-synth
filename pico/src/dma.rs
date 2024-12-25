use rp_pico as bsp;

use bsp::{hal::pac, hal::pac::interrupt, pac::Peripherals};

use crate::i2s;

use defmt::*;

/// which DMA channels to use for buffer A and B, respectively
const DMA_A: u8 = 0;
const DMA_B: u8 = 1;

/// The number of samples in each of the two buffers used to double-buffer the audio output
pub const BUFFER_LENGTH: usize = 64 * 2;

/// Private buffers used by the DMA's
static mut DMA_BUFFER_A: [i16; BUFFER_LENGTH] = [0; BUFFER_LENGTH];
static mut DMA_BUFFER_B: [i16; BUFFER_LENGTH] = [0; BUFFER_LENGTH];

pub fn setup_double_buffered(resets: &mut pac::RESETS, dma: &pac::DMA, i2s: &i2s::I2SOutput) {
    // unreset the DMA peripheral & wait for it to become available

    resets.reset().modify(|_, w| w.dma().clear_bit());
    while !resets.reset_done().read().dma().bit_is_set() {}

    setup_channel(
        dma.ch(DMA_A as usize),
        (core::ptr::addr_of_mut!(DMA_BUFFER_A) as *const i16) as u32,
        i2s.tx_addr() as u32,
        BUFFER_LENGTH as u32,
        DMA_B,
        i2s.tx_dreq_value(),
    );

    setup_channel(
        dma.ch(DMA_B as usize),
        (core::ptr::addr_of_mut!(DMA_BUFFER_B) as *const i16) as u32,
        i2s.tx_addr() as u32,
        BUFFER_LENGTH as u32,
        DMA_A,
        i2s.tx_dreq_value(),
    );

    // make sure buffers are initialized
    crate::FILL_BUFFER(&mut unsafe { DMA_BUFFER_A });
    crate::FILL_BUFFER(&mut unsafe { DMA_BUFFER_B });

    // enable interrupts
    dma.inte0()
        .modify(|_, w| unsafe { w.inte0().bits(1u16 << DMA_A | 1u16 << DMA_B) });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    // trigger DMA A (should cause ping-pong )
    dma.ch(DMA_A as usize).ch_ctrl_trig().modify(|_, w| {
        w.en().bit(true) // enable chanel
    });
}

fn setup_channel(
    dma: &pac::dma::CH,
    read_addr: u32,
    write_addr: u32,
    trans_count: u32,
    chain_to: u8,
    treq_value: u8,
) {
    // setup read & write address
    dma.ch_write_addr().write(|w| unsafe { w.bits(write_addr) });
    dma.ch_read_addr().write(|w| unsafe { w.bits(read_addr) });

    // number of samples to take
    dma.ch_trans_count()
        .write(|w| unsafe { w.bits(trans_count) });

    // setup the rest of the parameters
    // writing to a non-triggering register to not trigger a new transfer
    dma.ch_al1_ctrl().modify(|_, w| {
        w.en()
            .bit(true) // enable chanel
            .data_size()
            .size_halfword() // we are sending i16
            .incr_write()
            .clear_bit() // do not increment write address
            .incr_read()
            .set_bit(); // increment read address
                        // enable IRQ
        w.irq_quiet().clear_bit();

        unsafe { w.chain_to().bits(chain_to) };
        unsafe { w.treq_sel().bits(treq_value) };

        w
    });
}

#[interrupt]
fn DMA_IRQ_0() {
    // check which DMA was triggered and reconfigure the other one + start filling in the relevant buffer

    // SAFETY: we steal and write to a registers not written to anywhere else
    let p = unsafe { Peripherals::steal() };

    // check which DMA channel that triggered the interrupt to reconfigure the other one

    let ints0 = p.DMA.ints0().read().ints0().bits();
    // if it was not triggered by any of our DMA channels, just return
    if ints0 & (1u16 << DMA_A | 1u16 << DMA_B) == 0 {
        debug!("DMA_IRQ_0 interrupt happened, but not by DMA_A or DMA_B!");
        return;
    }

    // extract which DMA that completed so that it can be reconfigured
    let (dma, buffer) = if ints0 & (1u16 << DMA_A) != 0 {
        (DMA_A, core::ptr::addr_of_mut!(DMA_BUFFER_A))
    } else {
        (DMA_B, core::ptr::addr_of_mut!(DMA_BUFFER_B))
    };

    debug!("IRQ0 TRIGGERED by DMA {}", dma);

    // reconfigure the read addres of the completed DMA channel
    p.DMA
        .ch(dma as usize)
        .ch_read_addr()
        .write(|w| unsafe { w.bits((buffer as *const i16) as u32) });

    // reset the interrupt
    p.DMA
        .ints0()
        .write(|w| unsafe { w.ints0().bits(1u16 << dma) });

    // fill the next buffer with the data HERE
    // PROBLEM: if this call takes longer than then number of samples we are generating, then
    // the DMA ping-pong means that we do not get to reset the read address in time,
    // causing the DMA to read random (garbage) bytes and writing them out to the
    // I2S interface causing loud white noise to appear in the speaker. In the future,
    // we should look into using a third DMA channel that can automatically restart
    // the channels at the appropriate addresses.
    crate::FILL_BUFFER(unsafe { &mut *buffer });
}
