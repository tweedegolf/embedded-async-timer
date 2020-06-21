use arraydeque::ArrayDeque;

use crate::util::packed_slab::PackedSlab;
use crate::CapacityError;

pub type Index = u8;

//// A sorted store of indexable elements.
///
/// Prioritizes insertion at the end and draining the lowest elements.
/// Guarantees that the elements themselves do not move iff the PriorityQueue itself is not moved.
pub struct PriorityQueue<T> {
    // TODO: some data structure for which finding an empty slot is easier.
    /// Pinned buffer with actual elements, unsorted.
    buffer: PackedSlab<T>,
    /// Sorted (lowest to highest) catalogue that references the elements in the buffer.
    catalogue: ArrayDeque<[Index; crate::CAPACITY]>,
}

impl<T: core::cmp::PartialOrd> PriorityQueue<T> {
    pub fn new() -> Self {
        Self {
            buffer: PackedSlab::new(),
            catalogue: ArrayDeque::new(),
        }
    }

    /// Find the first element that is smaller than the reference.
    ///
    /// Yields the index of the catalogue element.
    fn find_first_larger<U: PartialOrd<T>>(&self, x: &U) -> Option<usize> {
        // TODO do binary search instead of linear search
        for (catalogue_i, buffer_y_i) in self.catalogue.iter().enumerate() {
            let y = unsafe { self.buffer.get_ref_unchecked(*buffer_y_i as usize) };

            if x < y {
                return Some(catalogue_i);
            }
        }

        None
    }

    /// Insert a new entry to be pinned.
    pub fn insert(&mut self, x: T) -> Result<Index, CapacityError> {
        let catalogue_ref = self.find_first_larger(&x);

        let (buffer_i, _) = self.buffer.insert(x)?;
        let buffer_i = buffer_i as Index;

        match catalogue_ref {
            Some(catalogue_i) => self.catalogue.insert(catalogue_i, buffer_i),
            None => self.catalogue.push_back(buffer_i),
        }
        .or(Err(CapacityError))?;

        Ok(buffer_i)
    }

    /// Iterate over all entries in-order.
    ///
    /// Also yields the stored index key, which can be used to remove each item.
    pub fn iter<'a>(&'a self) -> impl Iterator<Item = (Index, &'a T)> + 'a {
        self.catalogue
            .iter()
            .map(move |i| (*i, unsafe { self.buffer.get_ref_unchecked(*i as usize) }))
    }

    #[allow(unused)]
    pub fn get_ref(&self, i: Index) -> Option<&T> {
        self.buffer.get_ref(i as usize)
    }

    pub fn get_mut(&mut self, i: Index) -> Option<&mut T> {
        self.buffer.get_mut(i as usize)
    }

    /// Remove an item from the priority queue, given its internal buffer index.
    pub fn remove(&mut self, i: Index) -> Option<()> {
        if self.buffer.remove(i as usize).is_some() {
            // Hopefully the element we are searching for is in most cases the first one.
            if let Some((catalogue_i, _)) = self
                .catalogue
                .iter()
                .enumerate()
                .find(|(_, buffer_i)| i == **buffer_i)
            {
                self.catalogue.remove(catalogue_i).unwrap();
                return Some(());
            }
        } else {
            return None;
        }

        panic!("PinPriorityQueue discrepancy");
    }
}
