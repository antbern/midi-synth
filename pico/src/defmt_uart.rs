//! Module contains an implementation of `defmt::Logger` that prints to the UART.
//! Based heavily on the `defmt-rtt` implementation for various versions, including
//! [0.2.0](https://docs.rs/defmt-rtt/0.2.0/src/defmt_rtt/lib.rs.html#25-64) and
//! [0.3.2](https://docs.rs/defmt-rtt/0.3.2/src/defmt_rtt/lib.rs.html#35-83)
///
use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m::{
    interrupt::{self, free},
    register,
};

use crate::GLOBAL_UART0;

#[defmt::global_logger]
struct UartLogger;

/// Global logger lock.
static TAKEN: AtomicBool = AtomicBool::new(false);
static INTERRUPTS_ACTIVE: AtomicBool = AtomicBool::new(false);
static mut ENCODER: defmt::Encoder = defmt::Encoder::new();

unsafe impl defmt::Logger for UartLogger {
    fn acquire() {
        // get interrupt active status before disabling interrupts
        let primask = register::primask::read();
        // SAFETY: must be paired with a call to enable() (if applicable)
        interrupt::disable();

        if TAKEN.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }

        // no need for CAS because interrupts are disabled
        TAKEN.store(true, Ordering::Relaxed);

        INTERRUPTS_ACTIVE.store(primask.is_active(), Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        unsafe { ENCODER.start_frame(do_write) }
    }

    unsafe fn flush() {
        // SAFETY: if we get here, the global logger mutex is currently acquired

        // no flush needed (only blocking IO for now)
    }

    unsafe fn release() {
        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        ENCODER.end_frame(do_write);

        TAKEN.store(false, Ordering::Relaxed);

        // safety: Must be paired with corresponding call to disable(), see above
        if INTERRUPTS_ACTIVE.load(Ordering::Relaxed) {
            interrupt::enable();
        }
    }

    unsafe fn write(bytes: &[u8]) {
        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        ENCODER.write(bytes, do_write);
    }
}

fn do_write(bytes: &[u8]) {
    // write to the UART here
    free(|cs| {
        GLOBAL_UART0
            .borrow(cs)
            .borrow()
            .as_ref()
            .unwrap()
            .write_full_blocking(bytes);
    })
}
