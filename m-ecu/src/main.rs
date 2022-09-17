#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_halt as _;

use stm32h7xx_hal::time::Hertz;

const PKG_NAME: &str = env!("CARGO_PKG_NAME");
const PKG_VERSION: &str = env!("CARGO_PKG_VERSION");
const PKG_VERSION_MAJOR: &str = env!("CARGO_PKG_VERSION_MAJOR");
const PKG_VERSION_MINOR: &str = env!("CARGO_PKG_VERSION_MINOR");
const PKG_VERSION_PATCH: &str = env!("CARGO_PKG_VERSION_PATCH");

/// High speed external clock frequency.
const HSE: Hertz = Hertz::MHz(25);
/// System clock frequency.
const SYS_CLOCK: Hertz = Hertz::MHz(400);
/// FDCAN peripheral clock.
const FDCAN_CLOCK: Hertz = Hertz::MHz(32);
/// USART peripheral clock.
const USART_CLOCK: Hertz = Hertz::MHz(48);

const NET_ADDRESS: u8 = 0x6B;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [USART1, USART2])]
mod app {
    use stm32h7xx_hal::gpio::{self};
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc;
    use systick_monotonic::Systick;

    /// 1000 Hz / 1 ms granularity
    #[monotonic(binds = SysTick, default = true)]
    type Monotonic = Systick<1000>;

    #[derive(PartialEq)]
    pub enum State {
        Nominal,
        Ident,
        Faulty,
    }

    #[shared]
    struct SharedResources {
        state: State,
        console: vecraft::console::Console,
        canbus1: vecraft::can::J1939Session,
    }

    #[local]
    struct LocalResources {
        led: vecraft::led::Led,
    }

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
        let mono = Systick::new(ctx.core.SYST, crate::SYS_CLOCK.to_Hz());

        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = pwr.freeze();

        // RCC
        let rcc = ctx.device.RCC.constrain();
        let mut ccdr = rcc
            .use_hse(crate::HSE)
            .sys_ck(crate::SYS_CLOCK)
            .per_ck(32.MHz())
            .pll1_strategy(rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(crate::FDCAN_CLOCK)
            .pll3_strategy(rcc::PllConfigStrategy::Iterative)
            .pll3_p_ck(crate::SYS_CLOCK)
            .pll3_q_ck(crate::USART_CLOCK)
            .freeze(pwrcfg, &ctx.device.SYSCFG);

        ccdr.peripheral
            .kernel_usart234578_clk_mux(rcc::rec::Usart234578ClkSel::PLL3_Q);

        // GPIO
        // let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpiod = ctx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        // let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        // let gpioe = ctx.device.GPIOE.split(ccdr.peripheral.GPIOE);

        // UART
        let console = vecraft::console::Console::new(
            ctx.device
                .USART2
                .serial(
                    (gpiod.pd5.into_alternate(), gpiod.pd6.into_alternate()),
                    115200.bps(),
                    ccdr.peripheral.USART2,
                    &ccdr.clocks,
                )
                .unwrap(),
        );

        // FDCAN
        let canbus1 = {
            use fdcan::config::NominalBitTiming;
            use fdcan::filter::{ExtendedFilter, ExtendedFilterSlot};
            // use stm32h7xx_hal::rcc::ResetEnable;

            let fdcan_prec = ccdr
                .peripheral
                .FDCAN
                .kernel_clk_mux(stm32h7xx_hal::rcc::rec::FdcanClkSel::PLL1_Q);

            let mut pd3 = gpiod.pd3.into_push_pull_output();
            pd3.set_low();
            let mut pd7 = gpiod.pd7.into_push_pull_output();
            pd7.set_low();

            let mut fdcan1 = {
                let rx = gpiod.pd0.into_alternate().speed(gpio::Speed::VeryHigh);
                let tx = gpiod.pd1.into_alternate().speed(gpio::Speed::VeryHigh);

                ctx.device.FDCAN1.fdcan(tx, rx, fdcan_prec)
            };

            use core::num::{NonZeroU16, NonZeroU8};

            fdcan1.set_protocol_exception_handling(false);
            fdcan1.set_nominal_bit_timing(NominalBitTiming {
                prescaler: NonZeroU16::new(8).unwrap(),
                seg1: NonZeroU8::new(13).unwrap(),
                seg2: NonZeroU8::new(2).unwrap(),
                sync_jump_width: NonZeroU8::new(1).unwrap(),
            });

            fdcan1.set_extended_filter(
                ExtendedFilterSlot::_0,
                ExtendedFilter::accept_all_into_fifo0(),
            );

            fdcan1.enable_interrupt_line(fdcan::interrupt::InterruptLine::_0, true);
            fdcan1.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
            fdcan1.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0Full);

            vecraft::can::J1939Session::new(
                crate::NET_ADDRESS,
                vecraft::can::Bus::new(fdcan1.into_normal()),
            )
        };

        motd::spawn().ok();
        status_led::spawn().ok();
        status_print::spawn().ok();
        status_bus::spawn().ok();

        (
            SharedResources {
                state: State::Nominal,
                console,
                canbus1,
            },
            LocalResources {
                led: vecraft::led::Led::new(
                    gpiob.pb13.into_push_pull_output(),
                    gpiob.pb14.into_push_pull_output(),
                    gpiob.pb12.into_push_pull_output(),
                ),
            },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [console])]
    fn motd(mut ctx: motd::Context) {
        ctx.shared.console.lock(|console| {
            use core::fmt::Write;

            writeln!(console, "========================").ok();
            writeln!(console, r#"   //\\  ___"#).ok();
            writeln!(console, r#"   Y  \\/_/=|"#).ok();
            writeln!(console, r#"  _L  ((|_L_|"#).ok();
            writeln!(console, r#" (/\)(__(____)"#).ok();
            writeln!(console, "").ok();

            writeln!(console, "Firmware: {}", crate::PKG_NAME).ok();
            writeln!(console, "Version: {}", crate::PKG_VERSION).ok();
            writeln!(console, "Address: 0x{:X?}", crate::NET_ADDRESS).ok();

            writeln!(console, "").ok();
            writeln!(console, r#"« System operational »"#).ok();
            writeln!(console, "========================").ok();
        });
    }

    #[task(shared = [state], local = [led])]
    fn status_led(mut ctx: status_led::Context) {
        ctx.shared.state.lock(|state| match *state {
            State::Nominal => {
                ctx.local.led.set_green(vecraft::led::LedState::Toggle);
                ctx.local.led.set_blue(vecraft::led::LedState::Off);
                ctx.local.led.set_red(vecraft::led::LedState::Off);
            }
            State::Ident => {
                ctx.local.led.set_green(vecraft::led::LedState::Off);
                ctx.local.led.set_blue(vecraft::led::LedState::Toggle);
                ctx.local.led.set_red(vecraft::led::LedState::Off);
            }
            State::Faulty => {
                ctx.local.led.set_green(vecraft::led::LedState::Off);
                ctx.local.led.set_blue(vecraft::led::LedState::Off);
                ctx.local.led.set_red(vecraft::led::LedState::Toggle);
            }
        });

        status_led::spawn_after(200.millis().into()).unwrap();
    }

    #[task(shared = [console, state])]
    fn status_print(mut ctx: status_print::Context) {
        ctx.shared.console.lock(|console| {
            use core::fmt::Write;

            ctx.shared.state.lock(|state| match *state {
                State::Nominal => {
                    writeln!(console, "State: nominal").ok();
                }
                State::Ident => {
                    writeln!(console, "State: ident").ok();
                }
                State::Faulty => {
                    writeln!(console, "State: faulty").ok();
                }
            });
        });

        status_print::spawn_after(1.secs().into()).unwrap();
    }

    #[task(shared = [canbus1, state])]
    fn status_bus(mut ctx: status_bus::Context) {
        let state = ctx.shared.state.lock(|state| match *state {
            State::Nominal => 0x14,
            State::Ident => 0x16,
            State::Faulty => 0xfa,
        });

        ctx.shared.canbus1.lock(|canbus1| {
            let major: u8 = crate::PKG_VERSION_MAJOR.parse().unwrap();
            let minor: u8 = crate::PKG_VERSION_MINOR.parse().unwrap();
            let patch: u8 = crate::PKG_VERSION_PATCH.parse().unwrap();

            let last_error = 0_u16;
            let (last_error_high, last_error_low) = if last_error > 0 {
                (last_error.to_le_bytes()[0], last_error.to_le_bytes()[1])
            } else {
                (0xff, 0xff)
            };

            let message = vecraft::can::J1939Message {
                pgn: 65_282,
                data: [
                    0xff,
                    state,
                    major,
                    minor,
                    patch,
                    0xff,
                    last_error_high,
                    last_error_low,
                ],
            };

            canbus1.send(message)
        });

        status_bus::spawn_after(1.secs().into()).unwrap();
    }

    #[task(binds = FDCAN1_IT0, priority = 2, shared = [canbus1, state, console])]
    fn bus_event(mut ctx: bus_event::Context) {
        let mut message = vecraft::can::J1939Message::default();

        ctx.shared.canbus1.lock(|canbus1| {
            canbus1.recv(&mut message);
        });

        match message.pgn {
            45_312 => {
                if message.data[0] == b'Z' && message.data[1] == b'C' {
                    if message.data[2] & 0b00000001 == 1 {
                        ctx.shared.state.lock(|stateq| *stateq = State::Ident);
                        ctx.shared.console.lock(|console| {
                            use core::fmt::Write;

                            writeln!(console, "Identification LED on").ok();
                        });
                    } else if message.data[2] & 0b00000001 == 0 {
                        ctx.shared.state.lock(|stateq| *stateq = State::Nominal);
                        ctx.shared.console.lock(|console| {
                            use core::fmt::Write;

                            writeln!(console, "Identification LED off").ok();
                        });
                    }

                    if message.data[3] == 0x69 {
                        vecraft::sys_reset();
                    }
                }
            }
            _ => {}
        }
    }
}
