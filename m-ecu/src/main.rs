#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_halt as _;

use stm32h7xx_hal::time::Hertz;

const PKG_NAME: &str = env!("CARGO_PKG_NAME");
const PKG_VERSION: &str = env!("CARGO_PKG_VERSION");

/// High speed external clock frequency.
const HSE: Hertz = Hertz::MHz(25);
/// System clock frequency.
const SYS_CLOCK: Hertz = Hertz::MHz(400);
/// FDCAN peripheral clock.
const FDCAN_CLOCK: Hertz = Hertz::MHz(32);
/// USART peripheral clock.
const USART_CLOCK: Hertz = Hertz::MHz(48);

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [USART1, USART2])]
mod app {
    use stm32h7xx_hal::gpio::{self};
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc;
    use systick_monotonic::Systick;

    /// 1000 Hz / 1 ms granularity
    #[monotonic(binds = SysTick, default = true)]
    type Monotonic = Systick<1000>;

    #[shared]
    struct SharedResources {
        console: vecraft::console::Console,
        fd_can: fdcan::FdCan<
            stm32h7xx_hal::can::Can<stm32h7xx_hal::stm32::FDCAN1>,
            fdcan::NormalOperationMode,
        >,
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
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpiod = ctx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe = ctx.device.GPIOE.split(ccdr.peripheral.GPIOE);

        // UART
        let console = vecraft::console::Console::new(
            ctx.device
                .UART7
                .serial(
                    (gpioe.pe8.into_alternate(), gpioe.pe7.into_alternate()),
                    115200.bps(),
                    ccdr.peripheral.UART7,
                    &ccdr.clocks,
                )
                .unwrap(),
        );

        // FDCAN
        let fdcan1 = {
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
            fdcan1
        };

        motd::spawn().ok();
        status_led::spawn().ok();
        status_print::spawn().ok();
        status_bus::spawn().ok();

        (
            SharedResources {
                console,
                fd_can: fdcan1.into_normal(),
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

            writeln!(console, "").ok();
            writeln!(console, r#"« System operational »"#).ok();
            writeln!(console, "========================").ok();
        });
    }

    #[task(local = [led])]
    fn status_led(ctx: status_led::Context) {
        ctx.local.led.set_green(vecraft::led::LedState::Toggle);

        status_led::spawn_after(200.millis().into()).unwrap();
    }

    #[task(shared = [console])]
    fn status_print(mut ctx: status_print::Context) {
        ctx.shared.console.lock(|console| {
            use core::fmt::Write;

            writeln!(console, "State: nominal").ok();
        });

        status_print::spawn_after(1.secs().into()).unwrap();
    }

    #[task(shared = [fd_can])]
    fn status_bus(mut ctx: status_bus::Context) {
        ctx.shared.fd_can.lock(|fd_can| {
            let buffer = [0xff, 0x14, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff];

            let address = 0x18FF027A;

            let header = fdcan::frame::TxFrameHeader {
                len: buffer.len() as u8,
                id: fdcan::id::ExtendedId::new(address).unwrap().into(),
                frame_format: fdcan::frame::FrameFormat::Standard,
                bit_rate_switching: false,
                marker: None,
            };

            fd_can.transmit(header, &buffer).ok();
        });

        status_bus::spawn_after(1.secs().into()).unwrap();
    }

    #[task(binds = FDCAN1_IT0, priority = 2, shared = [fd_can])]
    fn bus_event(mut ctx: bus_event::Context) {
        let mut buffer = [0; 64];

        ctx.shared.fd_can.lock(|fd_can| {
            while let Ok(rx_header) = fd_can.receive0(&mut buffer) {
                let _rx_header = rx_header.unwrap();
            }

            fd_can.clear_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
            fd_can.clear_interrupt(fdcan::interrupt::Interrupt::RxFifo0Full);
        });
    }
}
