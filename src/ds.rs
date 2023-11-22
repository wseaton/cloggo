use fixed::types::I32F32;

pub struct CalibrationMatrix {
    data: [(I32F32, I32F32); 10],
    current_index: usize,
    is_full: bool,
}

impl CalibrationMatrix {
    pub const CAPACITY: usize = 10; // Use associated constants for the capacity

    pub fn new() -> Self {
        Self {
            data: [(I32F32::from_num(0), I32F32::from_num(0)); Self::CAPACITY],
            current_index: 0,
            is_full: false,
        }
    }

    pub fn push(&mut self, reading: (I32F32, I32F32)) {
        self.data[self.current_index] = reading;
        self.current_index = (self.current_index + 1) % Self::CAPACITY;
        if self.current_index == 0 {
            self.is_full = true;
        }
    }

    pub fn is_full(&self) -> bool {
        self.is_full
    }

    pub fn len(&self) -> usize {
        if self.is_full {
            Self::CAPACITY
        } else {
            self.current_index
        }
    }

    pub fn capacity(&self) -> usize {
        Self::CAPACITY
    }

    pub fn clear(&mut self) {
        self.current_index = 0;
        self.is_full = false;
    }

    pub fn iter(&self) -> impl Iterator<Item = &(I32F32, I32F32)> {
        self.data.iter()
    }

    pub fn get(&self, index: usize) -> Option<&(I32F32, I32F32)> {
        if index < self.len() {
            let idx = if self.is_full {
                (self.current_index + index) % Self::CAPACITY
            } else {
                index
            };
            Some(&self.data[idx])
        } else {
            None
        }
    }

    pub fn latest(&self) -> Option<&(I32F32, I32F32)> {
        if self.current_index == 0 && self.is_full {
            Some(&self.data[Self::CAPACITY - 1])
        } else if self.current_index > 0 {
            Some(&self.data[self.current_index - 1])
        } else {
            None
        }
    }
}
