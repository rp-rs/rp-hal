use const_random::const_random;
use embedded_dma::{ReadBuffer, WriteBuffer};

pub struct SliceWrapperPointer<T> {
    pointer: *const T,
    length: usize,
    id: u32,
}

unsafe impl<T> ReadBuffer for SliceWrapperPointer<T> {
    type Word = T;
    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (self.pointer, self.length)
    }
}

#[derive(Debug)]
pub struct SliceWrapper<'a, T> {
    inner: &'a [T],
    pointer_alive: bool,
    id: u32,
}

impl<'a, T> SliceWrapper<'a, T> {
    pub fn new(inner: &'a [T]) -> Self {
        Self {
            inner,
            pointer_alive: false,
            id: const_random!(u32),
        }
    }

    pub fn get_pointer(&mut self) -> SliceWrapperPointer<T> {
        if self.pointer_alive {
            panic!("SliceWrapperPointer already referenced in scope");
        }
        self.pointer_alive = true;

        SliceWrapperPointer {
            pointer: self.inner.as_ptr(),
            length: self.inner.len(),
            id: self.id,
        }
    }

    pub fn feed_pointer(&mut self, pointer: SliceWrapperPointer<T>) {
        if pointer.id != self.id {
            panic!("Attempted to feed SliceWrapper a pointer with an invalid ID");
        }

        self.pointer_alive = false;
    }
}

impl<'a, T> Drop for SliceWrapper<'a, T> {
    fn drop(&mut self) {
        if self.pointer_alive {
            panic!("SliceWrapper has not been fed it's pointer");
        }
    }
}

pub struct SliceWrapperPointerMut<T> {
    pointer: *mut T,
    length: usize,
    id: u32,
}

unsafe impl<T> ReadBuffer for SliceWrapperPointerMut<T> {
    type Word = T;
    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (self.pointer, self.length)
    }
}

unsafe impl<T> WriteBuffer for SliceWrapperPointerMut<T> {
    type Word = T;
    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        (self.pointer, self.length)
    }
}

#[derive(Debug)]
pub struct SliceWrapperMut<'a, T> {
    inner: &'a mut [T],
    pointer_alive: bool,
    id: u32,
}

impl<'a, T> SliceWrapperMut<'a, T> {
    pub fn new(inner: &'a mut [T]) -> Self {
        Self {
            inner,
            pointer_alive: false,
            id: const_random!(u32),
        }
    }

    pub fn get_pointer(&mut self) -> SliceWrapperPointerMut<T> {
        if self.pointer_alive {
            panic!("SliceWrapperPointer already referenced in scope");
        }
        self.pointer_alive = true;

        SliceWrapperPointerMut {
            pointer: self.inner.as_mut_ptr(),
            length: self.inner.len(),
            id: self.id,
        }
    }

    pub fn feed_pointer(&mut self, pointer: SliceWrapperPointerMut<T>) {
        if pointer.id != self.id {
            panic!("Attempted to feed SliceWrapper a pointer with an invalid ID");
        }

        self.pointer_alive = false;
    }
}

impl<'a, T> Drop for SliceWrapperMut<'a, T> {
    fn drop(&mut self) {
        if self.pointer_alive {
            panic!("SliceWrapper has not been fed it's pointer");
        }
    }
}
