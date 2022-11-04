mod convert {
    #[inline]
    pub unsafe fn as_bytes<T>(t: &T) -> &[u8] {
        core::slice::from_raw_parts((t as *const T) as *const u8, core::mem::size_of::<T>())
    }

    #[inline]
    pub unsafe fn as_struct<T>(buf: &[u8]) -> T {
        core::ptr::read(buf.as_ptr() as *const _)
    }
}

#[repr(C, packed)]
struct FrameData {
    ty: u8,
    id: u32,
    len: u8,
    data: [u8; 48],
}

#[derive(Debug)]
enum FrameType {
    /// CAN bus frame.
    Can,
    /// CAN FD bus frame.
    CanFd,
}

pub struct Frame {
    inner: FrameData,
}

impl Frame {
    /// CAN bus frame.
    const TYPE_CAN: u8 = 0x8;
    /// CAN FD bus frame.
    const TYPE_CAN_FD: u8 = 0x9;

    const EXTENDED_ID_FLAG: u32 = 0x80000000;

    pub fn parse(buf: &[u8]) -> Self {
        let inner: FrameData = unsafe { convert::as_struct(buf) };

        Self { inner }
    }

    #[inline]
    pub fn id(&self) -> u32 {
        if self.has_extended_id() {
            self.inner.id & !Self::EXTENDED_ID_FLAG
        } else {
            self.inner.id
        }
    }

    pub fn has_extended_id(&self) -> bool {
        self.inner.id & Self::EXTENDED_ID_FLAG == Self::EXTENDED_ID_FLAG
    }

    #[inline]
    pub fn len(&self) -> u8 {
        self.inner.len
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.inner.len == 0
    }

    #[inline]
    pub fn data(&self) -> &[u8] {
        &self.inner.data[..self.inner.len as usize]
    }

    fn ty(&self) -> FrameType {
        match self.inner.ty {
            Self::TYPE_CAN => FrameType::Can,
            Self::TYPE_CAN_FD => FrameType::CanFd,
            _ => panic!(),
        }
    }
}

impl From<&[u8]> for Frame {
    fn from(value: &[u8]) -> Self {
        Self::parse(value)
    }
}

impl core::fmt::Debug for Frame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Frame")
            .field("type", &self.ty())
            .field("id", &self.id())
            .field("data", &self.data())
            .finish()
    }
}

impl AsRef<[u8]> for Frame {
    fn as_ref(&self) -> &[u8] {
        unsafe { convert::as_bytes(&self.inner) }
    }
}

pub struct FeameBuilder {
    frame: FrameData,
}

impl FeameBuilder {
    pub fn with_standard_id(id: u16) -> Self {
        Self {
            frame: FrameData {
                ty: Frame::TYPE_CAN,
                id: id as u32,
                len: 0,
                data: [0; 48],
            },
        }
    }

    pub fn with_extended_id(id: u32) -> Self {
        Self {
            frame: FrameData {
                ty: Frame::TYPE_CAN,
                id: id | Frame::EXTENDED_ID_FLAG,
                len: 0,
                data: [0; 48],
            },
        }
    }

    pub fn data(mut self, buffer: &[u8]) -> Self {
        self.frame.data[..buffer.len()].copy_from_slice(buffer);
        self.frame.len = buffer.len() as u8;
        self
    }

    pub fn build(self) -> Frame {
        Frame { inner: self.frame }
    }
}
