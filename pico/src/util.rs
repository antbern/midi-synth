use core::{cell::RefCell, ops::DerefMut};

use cortex_m::interrupt::{free, CriticalSection, Mutex};

// Struct for easing the usage of mutexes and refcells to contain global variables
pub struct GlobalCell<T> {
    cell: Mutex<RefCell<Option<T>>>,
}

impl<T> GlobalCell<T> {
    pub const fn new(value: Option<T>) -> Self {
        GlobalCell {
            cell: Mutex::new(RefCell::new(value)),
        }
    }

    #[allow(dead_code)]
    pub fn put(&self, value: T) {
        free(|cs| self.put_cs(cs, value));
    }

    pub fn put_cs(&self, cs: &CriticalSection, value: T) {
        // *self.cell.borrow(cs).borrow_mut() = Some(value);
        self.cell.borrow(cs).replace(Some(value)); //
    }

    pub fn try_borrow_mut<F, A>(&self, cs: &CriticalSection, mut f: F) -> Option<A>
    where
        F: FnMut(&mut T) -> Option<A>,
    {
        if let Some(ref mut value) = self.cell.borrow(cs).borrow_mut().deref_mut() {
            f(value)
        } else {
            None
        }
    }

    // pub fn borrow_mut<'cs>(&'cs self, cs: &'cs CriticalSection) -> &'cs mut Option<T>{
    //     &mut self.cell.borrow(cs).borrow_mut()
    // }
    // pub fn borrow_mut(&self, cs: &CriticalSection) -> & mut Option<T>{
    //     self.cell.borrow(cs).borrow_mut().deref_mut()
    // }
}
