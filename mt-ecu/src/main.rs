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

const NET_ADDRESS: u8 = 0x6E;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [USART1, USART2])]
mod app {
    use stm32h7xx_hal::gpio::{self};
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc;
    use stm32h7xx_hal::watchdog::SystemWindowWatchdog;
    use systick_monotonic::Systick;

    /// 100 Hz / 10 ms granularity
    #[monotonic(binds = SysTick, default = true)]
    type Monotonic = Systick<100>;

    #[shared]
    struct SharedResources {
        state: vecraft::state::System,
        console: vecraft::console::Console,
        canbus1: vecraft::can::Can<
            stm32h7xx_hal::can::Can<stm32h7xx_hal::stm32::FDCAN1>,
            fdcan::NormalOperationMode,
        >,
    }

    #[local]
    struct LocalResources {
        signal1: gpio::gpioc::PC11<gpio::Input>,
        led: vecraft::led::Led,
        watchdog: SystemWindowWatchdog,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
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

        // Switch adc_ker_ck_input multiplexer to per_ck
        ccdr.peripheral.kernel_adc_clk_mux(rcc::rec::AdcClkSel::PER);

        ccdr.peripheral
            .kernel_usart234578_clk_mux(rcc::rec::Usart234578ClkSel::PLL3_Q);

        let mut watchdog = SystemWindowWatchdog::new(ctx.device.WWDG, &ccdr);

        // GPIO
        // let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpiod = ctx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
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

        let fdcan_prec = ccdr
            .peripheral
            .FDCAN
            .kernel_clk_mux(rcc::rec::FdcanClkSel::PLL1_Q);

        let canbus1 = {
            let rx = gpiod.pd0.into_alternate().speed(gpio::Speed::VeryHigh);
            let tx = gpiod.pd1.into_alternate().speed(gpio::Speed::VeryHigh);

            let pd3 = gpiod.pd3.into_push_pull_output();

            vecraft::can::CanBuilder::new(ctx.device.FDCAN1.fdcan(tx, rx, fdcan_prec), pd3)
                .set_bit_timing(vecraft::can::BITRATE_250K)
                .set_net_address_filter(crate::NET_ADDRESS)
                .build()
        };

        // Button
        let mut b1 = gpioc.pc11.into_pull_up_input();
        // let b2 = gpioc.pc10.into_pull_up_input();

        use stm32h7xx_hal::gpio::{Edge, ExtiPin};

        b1.make_interrupt_source(&mut ctx.device.SYSCFG);
        b1.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);
        b1.enable_interrupt(&mut ctx.device.EXTI);

        motd::spawn().ok();
        firmware_state::spawn().ok();
        status_print::spawn().ok();

        watchdog.start(75.millis());

        (
            SharedResources {
                state: vecraft::state::System::boot(),
                console,
                canbus1,
            },
            LocalResources {
                signal1: b1,
                led: vecraft::led::Led::new(
                    gpiob.pb13.into_push_pull_output(),
                    gpiob.pb14.into_push_pull_output(),
                    gpiob.pb12.into_push_pull_output(),
                ),
                watchdog,
            },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [console])]
    fn motd(mut ctx: motd::Context) {
        ctx.shared.console.lock(|console| {
            use core::fmt::Write;

            writeln!(console, "==========================").ok();
            writeln!(console, r#"       //\\  ___"#).ok();
            writeln!(console, r#"       Y  \\/_/=|"#).ok();
            writeln!(console, r#"      _L  ((|_L_|"#).ok();
            writeln!(console, r#"     (/\)(__(____)"#).ok();
            writeln!(console).ok();
            writeln!(console, "    Firmware : {}", crate::PKG_NAME).ok();
            writeln!(console, "    Version  : {}", crate::PKG_VERSION).ok();
            writeln!(console, "    Address  : 0x{:X?}", crate::NET_ADDRESS).ok();
            writeln!(console).ok();
            writeln!(console, "  Laixer Equipment B.V.").ok();
            writeln!(console, "   Copyright (C) 2022").ok();
            writeln!(console, "==========================").ok();
        });
    }

    #[task(binds = EXTI15_10, shared = [canbus1, console], local = [signal1])]
    fn on_signal(mut ctx: on_signal::Context) {
        use stm32h7xx_hal::gpio::ExtiPin;

        ctx.shared.console.lock(|console| {
            use core::fmt::Write;

            writeln!(console, "Signal falling").ok();
        });

        // let id = vecraft::j1939::IdBuilder::from_pgn(64_261)
        //     .sa(crate::NET_ADDRESS)
        //     .build();

        // let frame = vecraft::j1939::FrameBuilder::new(id)
        //     .from_slice(&[])
        //     .build();

        // ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));

        ctx.local.signal1.clear_interrupt_pending_bit();
    }

    #[task(shared = [state, canbus1], local = [led, watchdog])]
    fn firmware_state(mut ctx: firmware_state::Context) {
        let is_bus_ok = ctx.shared.canbus1.lock(|canbus1| canbus1.is_bus_ok());

        let state = ctx.shared.state.lock(|state| {
            if is_bus_ok {
                state.set_bus_error(false);
            }

            state.state()
        });

        ctx.local
            .led
            .set_color(&state.as_led(), &vecraft::led::LedState::Toggle);

        ctx.local.watchdog.feed();

        firmware_state::spawn_after(50.millis().into()).unwrap();
    }

    #[task(shared = [state, canbus1, console], local = [])]
    fn status_print(mut ctx: status_print::Context) {
        let uptime = monotonics::now().duration_since_epoch();

        let seconds = uptime.to_secs() % 60;
        let minutes = uptime.to_minutes() % 60;
        let hours = uptime.to_hours() % 24;

        let state = ctx.shared.state.lock(|state| state.state());

        ctx.shared.console.lock(|console| {
            use core::fmt::Write;

            writeln!(
                console,
                "[{:02}:{:02}:{:02}] State: {}",
                hours, minutes, seconds, state
            )
            .ok();
        });

        let major: u8 = crate::PKG_VERSION_MAJOR.parse().unwrap();
        let minor: u8 = crate::PKG_VERSION_MINOR.parse().unwrap();
        let patch: u8 = crate::PKG_VERSION_PATCH.parse().unwrap();

        let last_error = 0_u16;
        let (last_error_high, last_error_low) = if last_error > 0 {
            (last_error.to_le_bytes()[0], last_error.to_le_bytes()[1])
        } else {
            (0xff, 0xff)
        };

        let id = vecraft::j1939::IdBuilder::from_pgn(65_282)
            .sa(crate::NET_ADDRESS)
            .build();

        let frame = vecraft::j1939::FrameBuilder::new(id)
            .copy_from_slice(&[
                0xff,
                state.as_byte(),
                major,
                minor,
                patch,
                0xff,
                last_error_high,
                last_error_low,
            ])
            .build();

        ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));

        status_print::spawn_after(1.secs().into()).unwrap();
    }

    #[task(binds = FDCAN1_IT0, priority = 2, shared = [state, canbus1])]
    fn can1_event(mut ctx: can1_event::Context) {
        let is_bus_error = ctx.shared.canbus1.lock(|canbus1| canbus1.is_bus_error());

        if is_bus_error {
            ctx.shared.state.lock(|state| state.set_bus_error(true));
        }

        if let Some(frame) = ctx.shared.canbus1.lock(|canbus1| canbus1.recv()) {
            match frame.id().pgn() {
                45_312 => {
                    if frame.pdu()[0] == b'Z' && frame.pdu()[1] == b'C' {
                        if frame.pdu()[2] & 0b00000001 == 1 {
                            ctx.shared.state.lock(|state| state.set_ident(true));
                        } else if frame.pdu()[2] & 0b00000001 == 0 {
                            ctx.shared.state.lock(|state| state.set_ident(false));
                        }

                        if frame.pdu()[3] == 0x69 {
                            vecraft::sys_reset();
                        }
                    }
                }
                _ => {}
            }
        }
    }
}
