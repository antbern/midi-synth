use core::cell::RefCell;

use cortex_m::interrupt::{CriticalSection, Mutex};

/// Inner implementation for EventQueue protected by a Mutex for inner mutability
#[derive(Debug)]
struct QueueInner<const N: usize, T> {
    queue: [Option<T>; N],
    head: usize,
    tail: usize,
}
/// A queue that stores `T`s and provides inner mutability to the
/// queue, as long as a `CriticalSection` is passed as an argument to most functions.
pub struct Queue<const N: usize, T> {
    inner: Mutex<RefCell<QueueInner<N, T>>>,
}

#[allow(dead_code)]
impl<const N: usize, T: Sized + Copy> Queue<N, T> {
    pub const fn new() -> Self {
        Queue {
            inner: Mutex::new(RefCell::new(QueueInner {
                queue: [None; N],
                head: 0,
                tail: 0,
            })),
        }
    }

    pub fn take(&self, cs: &CriticalSection) -> Option<T> {
        let mut inner = self.inner.borrow(cs).borrow_mut();

        let head = inner.head;
        let evt = inner.queue[head].take()?;

        inner.head += 1;
        if inner.head >= inner.queue.len() {
            inner.head = 0;
        }
        Some(evt)
    }

    pub fn put(&self, cs: &CriticalSection, evt: T) {
        let mut inner = self.inner.borrow(cs).borrow_mut();

        let tail = inner.tail;
        inner.queue[tail] = Some(evt);
        inner.tail += 1;
        if inner.tail >= inner.queue.len() {
            inner.tail = 0;
        }
    }

    pub fn count(&self, cs: &CriticalSection) -> usize {
        let inner = self.inner.borrow(cs).borrow();
        inner.tail.wrapping_sub(inner.head)
    }
}
