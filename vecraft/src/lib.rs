#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

pub use j1939;

pub mod console;
pub mod led;

/// Trigger a full system reset.
#[inline]
pub fn sys_reset() {
    rtic::export::SCB::sys_reset();
}
