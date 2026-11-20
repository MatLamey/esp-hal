#[cfg(place_rmt_driver_in_ram)]
use procmacros::ram;

use super::{DynChannelAccess, Error, PulseCode, Tx};

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) enum WriterState {
    // There's still data left to write
    Active,

    // An error occurred, and either
    // - no data was written (and transmission was never started), or
    // - some data was written and the last code written is an end marker, such that transmission
    //   will eventually stop
    Error(Error),

    // Completed without errors
    Done,
}

impl WriterState {
    pub(super) fn is_ok(self) -> bool {
        !matches!(self, Self::Error(_))
    }

    pub(super) fn to_result(self) -> Result<(), Error> {
        match self {
            // tx ended but writer wasn't done yet
            Self::Active => Err(Error::TransmissionError),
            Self::Error(e) => Err(e),
            Self::Done => Ok(()),
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) struct RmtWriter {
    // The position in channel RAM to continue writing at; must be either
    // 0 or half the available RAM size if there's further data.
    // The position may be invalid if there's no data left.
    offset: u16,

    written: usize,

    last_code: PulseCode,

    state: WriterState,
}

impl RmtWriter {
    pub(super) fn new() -> Self {
        Self {
            offset: 0,
            written: 0,
            last_code: PulseCode::default(),
            state: WriterState::Active,
        }
    }

    #[inline]
    pub(super) fn state(&self) -> WriterState {
        self.state
    }

    // Copy from `data` to the hardware buffer, advancing the `data` slice accordingly.
    //
    // If `initial` is set, fill the entire buffer. Otherwise, append half the buffer's length from
    // `data`.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub(super) fn write(
        &mut self,
        data: &mut &[PulseCode],
        raw: DynChannelAccess<Tx>,
        initial: bool,
    ) {
        if self.state != WriterState::Active {
            return;
        }

        debug_assert!(!initial || self.offset == 0);

        let ram_start = raw.channel_ram_start();
        let memsize = raw.memsize().codes();
        let ram_end = unsafe { ram_start.add(memsize) };

        let max_count = if initial { memsize } else { memsize / 2 };
        let count = data.len().min(max_count);

        let mut ram_ptr = unsafe { ram_start.add(self.offset as usize) };

        let mut data_ptr = data.as_ptr();
        let data_end = unsafe { data_ptr.add(count) };

        let mut last_code = self.last_code;
        while data_ptr < data_end {
            // SAFETY: The iteration `count` is smaller than both `max_count` and `data.len()` such
            // that incrementing both pointers cannot advance them beyond their allocation's end.
            unsafe {
                last_code = data_ptr.read();
                ram_ptr.write_volatile(last_code);
                ram_ptr = ram_ptr.add(1);
                data_ptr = data_ptr.add(1);
            }
        }

        self.last_code = last_code;
        self.written += count;

        // If the input data was not exhausted, update offset as
        //
        // | initial | offset      | max_count   | new offset  |
        // | ------- + ----------- + ----------- + ----------- |
        // | true    | 0           | memsize     | 0           |
        // | false   | 0           | memsize / 2 | memsize / 2 |
        // | false   | memsize / 2 | memsize / 2 | 0           |
        //
        // Otherwise, the new position is invalid but the new slice is empty and we won't use the
        // offset again. In either case, the unsigned subtraction will not underflow.
        self.offset = memsize as u16 - max_count as u16 - self.offset;

        // The panic can never trigger since count <= data.len()!
        data.split_off(..count).unwrap();
        if data.is_empty() {
            self.state = if self.written == 0 {
                // data was empty
                WriterState::Error(Error::InvalidArgument)
            } else if last_code.is_end_marker() {
                // Do not check for end markers in the inner loop above since this would
                // substantially increase the instruction count there. Instead, only check the last
                // code to report on error.
                WriterState::Done
            } else {
                // Write an extra end marker to prevent looping forever with wrapping tx.
                if ram_ptr < ram_end {
                    unsafe { ram_ptr.write_volatile(PulseCode::end_marker()) };
                    WriterState::Error(Error::EndMarkerMissing)
                } else {
                    // The buffer is full, and we can't easily write an end marker. (Short of
                    // overwriting some other code, which might be ok since hitting this error case
                    // always indicates a bug in user code.)
                    // Thus, remain in Active state. On the next Event::Threshold, this function
                    // will be called again, with data already exhausted, and hit the other arm of
                    // this conditional.
                    WriterState::Active
                }
            };
        }

        debug_assert!(self.offset == 0 || self.offset as usize == memsize / 2);
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    Initial, // whole RAM, 0..memsize
    Lower,   // 0..memsize/2
    Upper,   // memsize/2..memsize
}

impl Phase {
    // The phase that follows after this one has been fully written.
    fn next(self) -> Self {
        match self {
            Phase::Initial | Phase::Upper => Phase::Lower,
            Phase::Lower => Phase::Upper,
        }
    }
}

/// Cyclic iterator over the slots of a channel's RAM.
/// Never ends; bound it with `.take(n)`.
#[derive(Debug)]
pub(super) struct RamPingPong {
    start: *mut PulseCode,
    memsize: usize, // in codes
    pos: usize,     // next slot, always < memsize
    phase: Phase,
}

impl RamPingPong {
    /// # Safety
    /// `start..start.add(memsize)` must be valid channel RAM, with `memsize > 0`.
    #[inline]
    pub(super) unsafe fn new(start: *mut PulseCode, memsize: usize) -> Self {
        Self {
            start,
            memsize,
            pos: 0,
            phase: Phase::Initial,
        }
    }
}

impl RamPingPong {
    /// Slots for the next chunk. Advances the phase.
    pub(super) fn next_chunk(&mut self) -> impl ExactSizeIterator<Item = *mut PulseCode> {
        let (from, len) = match self.phase {
            Phase::Initial => (0, self.memsize),
            Phase::Lower => (0, self.memsize / 2),
            Phase::Upper => (self.memsize / 2, self.memsize / 2),
        };
        self.phase = self.phase.next();
        let base = self.start;
        (from..from + len).map(move |i| unsafe { base.add(i) })
    }
}

impl Iterator for RamPingPong {
    type Item = *mut PulseCode;

    #[inline]
    fn next(&mut self) -> Option<*mut PulseCode> {
        let slot = unsafe { self.start.add(self.pos) };
        self.pos += 1;
        if self.pos == self.memsize {
            self.pos = 0;
        }
        Some(slot)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        match self.phase{
            Phase::Initial | Phase::Upper => {
                let size = self.memsize-self.pos;
                (size, Some(size))
            },
            Phase::Lower => {
                let size = self.memsize / 2 - self.pos;
                (size, Some(size))
            },
            
        }
        
    }
}


pub struct RmtWriterIter<'a> {
    ram: RamPingPong,
    state: WriterState,
    data: &'a mut dyn Iterator<Item = PulseCode>,
}

impl<'a> RmtWriterIter<'a> {
    pub fn new<I: Iterator<Item = PulseCode>>(raw: DynChannelAccess<Tx>, data: &'a mut I) -> Self {
        let memsize = raw.memsize();
        let ram =
            unsafe { RamPingPong::new(raw.channel_ram_start() as *mut PulseCode, memsize.codes()) };
        Self {
            ram,
            state: WriterState::Active,
            data,
        }
    }

    pub fn write(&mut self) {
        let mut ram_slots = self.ram.next_chunk();

        self.data.zip(ram_slots.by_ref()).for_each(|(src, dst)| unsafe {
            dst.write_volatile(src);
        });

        if ram_slots.len() != 0 {
            if let Some(slot) = ram_slots.next() {
                unsafe {slot.write_volatile(PulseCode::end_marker());}
            };
            self.state = WriterState::Done;            
        }      
    }
    
    #[inline]
    pub(super) fn state(&self) -> WriterState {
        self.state
    }
}

