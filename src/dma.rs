use rp_pico as bsp;

use bsp::{hal::pac, hal::pac::interrupt, pac::Peripherals};

use crate::i2s;

use defmt::*;

/// which DMA channels to use for buffer A and B, respectively
const DMA_A: u8 = 0;
const DMA_B: u8 = 1;

/// The number of samples in each of the two buffers used to double-buffer the audio output
pub const BUFFER_LENGTH: usize = 16_000;

static mut DMA_BUFFER_A: [i16; BUFFER_LENGTH] = [0; BUFFER_LENGTH];
static mut DMA_BUFFER_B: [i16; BUFFER_LENGTH] = [0; BUFFER_LENGTH];

// Need to use two DMA channels and two buffers, chained to trigger each other on completion. This way we get an interrupt "half-way" and can start filling the
// unused buffer-half with new samples

pub fn setup_double_buffered(resets: &mut pac::RESETS, dma: &pac::DMA, i2s: &i2s::I2SOutput) {
    // unreset the DMA peripheral & wait for it to become available
    resets.reset.modify(|_, w| w.dma().clear_bit());
    while !resets.reset_done.read().dma().bit_is_set() {}

    setup_channel(
        &dma.ch[DMA_A as usize],
        (unsafe { &DMA_BUFFER_A } as *const i16) as u32,
        i2s.tx_addr() as u32,
        BUFFER_LENGTH as u32,
        DMA_B,
        i2s.tx_dreq_value(),
    );

    setup_channel(
        &dma.ch[DMA_B as usize],
        (unsafe { &DMA_BUFFER_B } as *const i16) as u32,
        i2s.tx_addr() as u32,
        BUFFER_LENGTH as u32,
        DMA_A,
        i2s.tx_dreq_value(),
    );

    // enable interrupts
    dma.inte0
        .modify(|_, w| unsafe { w.inte0().bits(1u16 << DMA_A | 1u16 << DMA_B) });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    // trigger DMA A (should cause ping-pong )
    dma.ch[DMA_A as usize].ch_ctrl_trig.modify(|_, w| {
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
    dma.ch_write_addr.write(|w| unsafe { w.bits(write_addr) });
    dma.ch_read_addr.write(|w| unsafe { w.bits(read_addr) });

    // number of samples to take
    dma.ch_trans_count.write(|w| unsafe { w.bits(trans_count) });

    // setup the rest of the parameters
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

    debug!("{:#02x}", p.DMA.ints0.read().ints0().bits());

    // check which DMA channel that triggered the interrupt to reconfigure the other one

    let ints0 = p.DMA.ints0.read().ints0().bits();

    if ints0 & (1u16 << DMA_A) != 0 {
        // DMA A completed, reconfigure it by updating the read register
        debug!("IRQ0 TRIGGERED by DMA A");

        p.DMA.ch[DMA_A as usize]
            .ch_read_addr
            .write(|w| unsafe { w.bits((&DMA_BUFFER_A as *const i16) as u32) });

        // reset the interrupt

        p.DMA
            .ints0
            .write(|w| unsafe { w.ints0().bits(1u16 << DMA_A) });

        for i in 0..BUFFER_LENGTH {
            unsafe {
                DMA_BUFFER_A[i] = crate::DMA_BUFFER[i];
            }
        }
    } else if ints0 & (1u16 << DMA_B) != 0 {
        // DMA B completed, reconfigure it by updating the read register
        debug!("IRQ0 TRIGGERED by DMA B");

        p.DMA.ch[DMA_B as usize]
            .ch_read_addr
            .write(|w| unsafe { w.bits((&DMA_BUFFER_B as *const i16) as u32) });

        p.DMA
            .ints0
            .write(|w| unsafe { w.ints0().bits(1u16 << DMA_B) });

        for i in 0..BUFFER_LENGTH {
            unsafe {
                DMA_BUFFER_B[i] = crate::DMA_BUFFER[i] >> 1;
            }
        }
    }

    // TODO: fill the next buffer with the data HERE

    // FIXME: copy from the global buffer
    // for i in 0..BUFFER_LENGTH {
    //     unsafe {
    //         DMA_BUFFER_A[i] = crate::DMA_BUFFER[i];
    //         DMA_BUFFER_B[i] = crate::DMA_BUFFER[i];
    //     }
    // }
}
