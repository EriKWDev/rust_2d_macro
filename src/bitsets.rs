/*
    BitSets
*/

type BitSetImpl = i8;
pub type Flag = BitSetImpl;

#[derive(Clone, Copy)]
pub struct BitSet {
    bits: BitSetImpl,
}

impl BitSet {
    pub const fn new(bits: BitSetImpl) -> Self {
        Self { bits }
    }

    pub const fn with(&self, flag: Flag) -> Self {
        Self {
            bits: self.bits | flag,
        }
    }

    pub const fn empty() -> Self {
        Self { bits: 0 }
    }

    #[inline]
    pub fn include_flag(&mut self, flag: Flag) {
        self.bits |= flag
    }

    #[inline]
    pub fn exclude_flag(&mut self, flag: Flag) {
        self.bits &= !flag
    }

    #[inline]
    pub const fn contains(&self, flag: Flag) -> bool {
        (self.bits & flag) != 0
    }

    #[inline]
    pub const fn is_subset_of(&self, other: &Self) -> bool {
        (other.bits & self.bits) == self.bits
    }
}
