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

/// J1939 network address.
const NET_ADDRESS: u8 = 0x4A;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [USART1, USART2])]
mod app {
    use stm32h7xx_hal::gpio::{self};
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc;
    use stm32h7xx_hal::watchdog::{Event::EarlyWakeup, SystemWindowWatchdog};
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
        gate_lock: vecraft::lsgc::GateLock,
    }

    #[local]
    struct LocalResources {
        led: vecraft::led::Led,
        gc: vecraft::lsgc::GateControl,
        watchdog: SystemWindowWatchdog,
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

        // Switch adc_ker_ck_input multiplexer to per_ck
        ccdr.peripheral.kernel_adc_clk_mux(rcc::rec::AdcClkSel::PER);

        ccdr.peripheral
            .kernel_usart234578_clk_mux(rcc::rec::Usart234578ClkSel::PLL3_Q);

        let mut watchdog = SystemWindowWatchdog::new(ctx.device.WWDG, &ccdr);

        // GPIO
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = ctx.device.GPIOD.split(ccdr.peripheral.GPIOD);
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

        let pwr_swtich1 = gpiod.pd11.into_push_pull_output();
        let pwr_swtich2 = gpioa.pa12.into_push_pull_output();

        let (_, (pwm_high1, pwm_low1, pwm_high2, pwm_low2)) = ctx
            .device
            .TIM2
            .pwm_advanced(
                (
                    gpioa.pa1.into_alternate(),
                    gpioa.pa0.into_alternate(),
                    gpioa.pa3.into_alternate(),
                    gpioa.pa2.into_alternate(),
                ),
                ccdr.peripheral.TIM2,
                &ccdr.clocks,
            )
            .frequency(100.Hz())
            .period(32_767)
            .finalize();

        let (_, (pwm_high0, pwm_low0, pwm_high3, pwm_low3)) = ctx
            .device
            .TIM4
            .pwm_advanced(
                (
                    gpiod.pd12.into_alternate(),
                    gpiod.pd13.into_alternate(),
                    gpiod.pd14.into_alternate(),
                    gpiod.pd15.into_alternate(),
                ),
                ccdr.peripheral.TIM4,
                &ccdr.clocks,
            )
            .frequency(100.Hz())
            .period(32_767)
            .finalize();

        let (_, (pwm_high4, pwm_low4, pwm_high7, pwm_low7)) = ctx
            .device
            .TIM3
            .pwm_advanced(
                (
                    gpioc.pc6.into_alternate(),
                    gpioc.pc7.into_alternate(),
                    gpioc.pc8.into_alternate(),
                    gpioc.pc9.into_alternate(),
                ),
                ccdr.peripheral.TIM3,
                &ccdr.clocks,
            )
            .frequency(100.Hz())
            .period(32_767)
            .finalize();

        let (_, (pwm_high5, pwm_low5, pwm_high6, pwm_low6)) = ctx
            .device
            .TIM1
            .pwm_advanced(
                (
                    gpioa.pa10.into_alternate(),
                    gpioa.pa11.into_alternate(),
                    gpioa.pa8.into_alternate(),
                    gpioa.pa9.into_alternate(),
                ),
                ccdr.peripheral.TIM1,
                &ccdr.clocks,
            )
            .frequency(100.Hz())
            .period(32_767)
            .finalize();

        let gate0 = vecraft::lsgc::Gate::new(pwm_high0, pwm_low0);
        let gate1 = vecraft::lsgc::Gate::new(pwm_high1, pwm_low1);
        let gate2 = vecraft::lsgc::Gate::new(pwm_high2, pwm_low2);
        let gate3 = vecraft::lsgc::Gate::new(pwm_high3, pwm_low3);
        let gate4 = vecraft::lsgc::Gate::new(pwm_high4, pwm_low4);
        let gate5 = vecraft::lsgc::Gate::new(pwm_high5, pwm_low5);
        let gate6 = vecraft::lsgc::Gate::new(pwm_high6, pwm_low6);
        let gate7 = vecraft::lsgc::Gate::new(pwm_high7, pwm_low7);

        let mut gl = vecraft::lsgc::GateLock {
            lockout0: pwr_swtich1,
            lockout1: pwr_swtich2,
        };

            gate0,
            gate1,
            gate2,
            gate3,
            gate4,
            gate5,
            gate6,
            gate7,
        };
        gl.lock();

        motd::spawn().ok();
        firmware_state::spawn().ok();
        status_print::spawn().ok();

        watchdog.start(75.millis());
        watchdog.listen(EarlyWakeup);

        (
            SharedResources {
                state: vecraft::state::System::boot(),
                console,
                canbus1,
                gate_lock: gl,
            },
            LocalResources {
                led: vecraft::led::Led::new(
                    gpiob.pb13.into_push_pull_output(),
                    gpiob.pb14.into_push_pull_output(),
                    gpiob.pb12.into_push_pull_output(),
                ),
                gc,
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

    #[task(shared = [state, canbus1, gate_lock], local = [led, watchdog])]
    fn firmware_state(mut ctx: firmware_state::Context) {
        let is_bus_ok = ctx.shared.canbus1.lock(|canbus1| canbus1.is_bus_ok());

        let state = ctx.shared.state.lock(|state| {
            if is_bus_ok {
                state.set_bus_error(false);
            } else {
                state.set_bus_error(true);
                ctx.shared.gate_lock.lock(|gate_lock| gate_lock.lock());
            }

            state.state()
        });

        ctx.local
            .led
            .set_color(&state.as_led(), &vecraft::led::LedState::On);

        ctx.local.watchdog.feed();

        firmware_state::spawn_after(50.millis().into()).unwrap();
    }

    #[task(binds = WWDG1, shared = [console, gate_lock])]
    fn watchdog_early_warning(mut ctx: watchdog_early_warning::Context) {
        ctx.shared.gate_lock.lock(|gate_lock| gate_lock.lock());

        ctx.shared.console.lock(|console| {
            use core::fmt::Write;

            writeln!(console, "Soft fault occurred; resetting hardware").ok();
        });
    }

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

        // let major: u8 = crate::PKG_VERSION_MAJOR.parse().unwrap();
        // let minor: u8 = crate::PKG_VERSION_MINOR.parse().unwrap();
        // let patch: u8 = crate::PKG_VERSION_PATCH.parse().unwrap();

        // let last_error = 0_u16;
        // let (last_error_high, last_error_low) = if last_error > 0 {
        //     (last_error.to_le_bytes()[0], last_error.to_le_bytes()[1])
        // } else {
        //     (0xff, 0xff)
        // };

        // let id = vecraft::j1939::IdBuilder::from_pgn(vecraft::j1939::PGN::Other(65_282))
        //     .sa(crate::NET_ADDRESS)
        //     .build();

        // let _frame = vecraft::j1939::FrameBuilder::new(id)
        //     .copy_from_slice(&[
        //         0xff,
        //         state.as_byte(),
        //         major,
        //         minor,
        //         patch,
        //         0xff,
        //         last_error_high,
        //         last_error_low,
        //     ])
        //     .build();

        // ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));

        status_print::spawn_after(1.secs().into()).unwrap();
    }

    fn valve_value(value: i16) -> vecraft::lsgc::GateSide<u16, u16> {
        match value {
            v if v == 0 => vecraft::lsgc::GateSide::Hold(0, 0),
            v if v.is_positive() => vecraft::lsgc::GateSide::Up(value as u16),
            _ => vecraft::lsgc::GateSide::Down((value + 1).abs() as u16),
        }
    }
    fn valve_value32(value: i16) -> vecraft::lsgc::GateSide<u32, u32> {
        match value {
            v if v == 0 => vecraft::lsgc::GateSide::Hold(0, 0),
            v if v.is_positive() => vecraft::lsgc::GateSide::Up(value as u32),
            _ => vecraft::lsgc::GateSide::Down((value + 1).abs() as u32),
        }
    }

    #[task(binds = FDCAN1_IT0, priority = 2, shared = [canbus1, state, console, gate_lock], local = [gc])]
    fn can1_event(mut ctx: can1_event::Context) {
        let is_bus_error = ctx.shared.canbus1.lock(|canbus1| canbus1.is_bus_error());

        if is_bus_error {
            ctx.shared.state.lock(|state| state.set_bus_error(true));

            ctx.shared.console.lock(|console| {
                use core::fmt::Write;

                writeln!(console, "Motion locked due to bus error").ok();
            });

            ctx.shared.gate_lock.lock(|gate_lock| gate_lock.lock());
        }

        while let Some(frame) = ctx.shared.canbus1.lock(|canbus1| canbus1.recv()) {
            match frame.id().pgn() {
                vecraft::j1939::PGN::Request => {
                    let pgn =
                        vecraft::j1939::PGN::from_le_bytes(frame.pdu()[0..3].try_into().unwrap());

                    let id = vecraft::j1939::IdBuilder::from_pgn(
                        vecraft::j1939::PGN::AcknowledgmentMessage,
                    )
                    .da(0xff)
                    .sa(crate::NET_ADDRESS)
                    .build();

                    let pgn_bytes = pgn.to_le_bytes();

                    let frame = vecraft::j1939::FrameBuilder::new(id)
                        .copy_from_slice(&[
                            0x01,
                            0xff,
                            0xff,
                            0xff,
                            0xff,
                            pgn_bytes[0],
                            pgn_bytes[1],
                            pgn_bytes[2],
                        ])
                        .build();

                    ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));
                }
                vecraft::j1939::PGN::ProprietarilyConfigurableMessage1 => {
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
                vecraft::j1939::PGN::ProprietarilyConfigurableMessage3 => {
                    if frame.pdu()[0] == b'Z' && frame.pdu()[1] == b'C' {
                        if frame.pdu()[3] & 0b11 == 0 {
                            ctx.shared.console.lock(|console| {
                                use core::fmt::Write;

                                writeln!(console, "Motion locked").ok();
                            });

                            ctx.shared.gate_lock.lock(|gate_lock| gate_lock.lock());
                        } else if frame.pdu()[3] & 0b11 == 1 {
                            let state = ctx.shared.state.lock(|state| state.state());
                            if state == vecraft::state::State::Nominal {
                                ctx.shared.console.lock(|console| {
                                    use core::fmt::Write;

                                    writeln!(console, "Motion unlocked").ok();
                                });

                                ctx.local.gc.reset();
                                ctx.shared.gate_lock.lock(|gate_lock| gate_lock.unlock());
                            }
                        }

                        if frame.pdu()[4] & 0b00000001 == 1 {
                            ctx.local.gc.reset();
                        }
                    }
                }
                vecraft::j1939::PGN::Other(40_960) => {
                    if frame.pdu()[0..2] != [0xff, 0xff] {
                        let gate_value = i16::from_le_bytes(frame.pdu()[0..2].try_into().unwrap());

                        ctx.local.gc.gate0.set_value(valve_value(gate_value));
                    }
                    if frame.pdu()[2..4] != [0xff, 0xff] {
                        let gate_value = i16::from_le_bytes(frame.pdu()[2..4].try_into().unwrap());

                        ctx.local.gc.gate1.set_value(valve_value32(gate_value));
                    }
                    if frame.pdu()[4..6] != [0xff, 0xff] {
                        let gate_value = i16::from_le_bytes(frame.pdu()[4..6].try_into().unwrap());

                        ctx.local.gc.gate2.set_value(valve_value32(gate_value));
                    }
                    if frame.pdu()[6..8] != [0xff, 0xff] {
                        let gate_value = i16::from_le_bytes(frame.pdu()[6..8].try_into().unwrap());

                        ctx.local.gc.gate3.set_value(valve_value(gate_value));
                    }
                }
                vecraft::j1939::PGN::Other(41_216) => {
                    if frame.pdu()[0..2] != [0xff, 0xff] {
                        let gate_value = i16::from_le_bytes(frame.pdu()[0..2].try_into().unwrap());

                        ctx.local.gc.gate4.set_value(valve_value(gate_value));
                    }
                    if frame.pdu()[2..4] != [0xff, 0xff] {
                        let gate_value = i16::from_le_bytes(frame.pdu()[2..4].try_into().unwrap());

                        ctx.local.gc.gate5.set_value(valve_value(gate_value));
                    }
                    if frame.pdu()[4..6] != [0xff, 0xff] {
                        let gate_value = i16::from_le_bytes(frame.pdu()[4..6].try_into().unwrap());

                        ctx.local.gc.gate6.set_value(valve_value(gate_value));
                    }
                    if frame.pdu()[6..8] != [0xff, 0xff] {
                        let gate_value = i16::from_le_bytes(frame.pdu()[6..8].try_into().unwrap());

                        ctx.local.gc.gate7.set_value(valve_value(gate_value));
                    }
                }
                _ => {}
            }
        }
    }
}
