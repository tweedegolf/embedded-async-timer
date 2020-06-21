use crate::CapacityError;
use crate::CAPACITY;
use core::mem::MaybeUninit;

const PADS: usize = ((CAPACITY - 1) / 8) + 1; // ceil

/// An array equivalent to [Option<T>; CAPACITY], backed by a bitarray.
pub struct PackedSlab<T> {
    buffer: [MaybeUninit<T>; CAPACITY],
    pads: [u8; PADS],
}

impl<T> PackedSlab<T> {
    pub fn new() -> Self {
        Self {
            buffer: unsafe { MaybeUninit::<[MaybeUninit<T>; CAPACITY]>::uninit().assume_init() },
            pads: [0u8; PADS],
        }
    }

    /// Find an uninitialized cell and if available claim it.
    ///
    /// Returns `None` when there is no uninitialized cell available.
    fn find_uninitialized_and_claim(&mut self) -> Option<usize> {
        for (pad_i, pad) in self.pads.iter_mut().enumerate() {
            if !*pad != 0u8 {
                // Here's hoping there is an intrinsic that can do it better.
                let bit_i = (!*pad).trailing_zeros();
                let res = pad_i * 8 + bit_i as usize;
                *pad |= 1 << bit_i;
                return Some(res);
            }
        }

        None
    }

    /// Inserts data into the first uninitialized cell.
    ///
    /// Yields `CapacityError` if the PackedSlab is full.
    /// Yields the cell index and a mutable reference to the cell on success.
    pub fn insert(&mut self, x: T) -> Result<(usize, &mut T), CapacityError> {
        if let Some(i) = self.find_uninitialized_and_claim() {
            let ptr = self.buffer[i].as_mut_ptr();
            unsafe {
                ptr.write(x);
                Ok((i, &mut *ptr))
            }
        } else {
            Err(CapacityError)
        }
    }

    fn idx(i: usize) -> (usize, usize) {
        (i / 8, i % 8)
    }

    /// Checks whether the cell indicated by the index is occupied.
    pub fn exists(&self, i: usize) -> bool {
        let (pad_i, bit_i) = Self::idx(i);
        self.pads[pad_i] & 1 << bit_i != 0
    }

    /// Get a mutable reference to a cell indicated by the index, if it is occupied.
    pub fn get_mut(&mut self, i: usize) -> Option<&mut T> {
        if i > CAPACITY {
            return None;
        }

        if self.exists(i) {
            Some(unsafe { &mut *self.buffer[i].as_mut_ptr() })
        } else {
            None
        }
    }

    /// Get a reference to a cell indicated by the index, if it is occupied.
    #[allow(unused)]
    pub fn get_ref(&self, i: usize) -> Option<&T> {
        if i > CAPACITY {
            return None;
        }

        if self.exists(i) {
            Some(unsafe { &*self.buffer[i].as_ptr() })
        } else {
            None
        }
    }

    pub unsafe fn get_ref_unchecked(&self, i: usize) -> &T {
        &*self.buffer[i].as_ptr()
    }

    #[allow(unused)]
    pub unsafe fn get_mut_unchecked(&mut self, i: usize) -> &mut T {
        &mut *self.buffer[i].as_mut_ptr()
    }

    /// Remove an element from the PackedSlab.
    ///
    /// Returns `Some(())` on success, or `None` if the cell was not occupied.
    pub fn remove(&mut self, i: usize) -> Option<()> {
        if i > CAPACITY {
            return None;
        }

        let (pad_i, bit_i) = Self::idx(i);
        if self.pads[pad_i] & 1 << bit_i != 0 {
            drop(unsafe { &mut *self.buffer[i].as_mut_ptr() });
            self.pads[pad_i] &= !(1 << bit_i);
            Some(())
        } else {
            None
        }
    }
}
