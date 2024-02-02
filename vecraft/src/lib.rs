// #![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

pub use fdcan;
pub use j1939;

pub mod can;
pub mod console;
pub mod led;
pub mod lsgc;
pub mod state;
pub mod usb_avic;
pub mod usb_frame;

/// Trigger a full system reset.
#[inline]
pub fn sys_reset() {
    rtic::export::SCB::sys_reset();
}
