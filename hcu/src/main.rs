#![no_main]
#![no_std]
#![deny(warnings)]
#![deny(unsafe_code)]
// #![deny(missing_docs)]

use vecraft::panic_halt as _;

use stm32h7xx_hal::time::Hertz;

/// Glonax firmware name.
const PKG_NAME: &str = env!("CARGO_PKG_NAME");
/// Glonax firmware version.
const PKG_VERSION: &str = env!("CARGO_PKG_VERSION");
/// Glonax firmware major version.
const PKG_VERSION_MAJOR: &str = env!("CARGO_PKG_VERSION_MAJOR");
/// Glonax firmware minor version.
const PKG_VERSION_MINOR: &str = env!("CARGO_PKG_VERSION_MINOR");
/// Glonax firmware patch version.
const PKG_VERSION_PATCH: &str = env!("CARGO_PKG_VERSION_PATCH");

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
    use stm32h7xx_hal::system_watchdog::{Event::EarlyWakeup, SystemWindowWatchdog};

    use vecraft::fdcan;
    use vecraft::j1939::{
        protocol, FrameBuilder, IdBuilder, FIELD_DELIMITER, PDU_NOT_AVAILABLE, PGN,
    };
    use vecraft::Systick;

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
        config: vecraft::VecraftConfig,
        last_recv_time: Option<u64>,
    }

    #[local]
    struct LocalResources {
        led: vecraft::RGBLed,
        gate_control: vecraft::lsgc::GateControl,
        watchdog: SystemWindowWatchdog,
        eeprom: vecraft::eeprom::Eeprom<stm32h7xx_hal::i2c::I2c<stm32h7xx_hal::pac::I2C1>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
        let mono = Systick::new(ctx.core.SYST, crate::SYS_CLOCK.to_Hz());

        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = pwr.freeze();

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
        ccdr.peripheral.kernel_adc_clk_mux(rcc::rec::AdcClkSel::Per);

        ccdr.peripheral
            .kernel_usart234578_clk_mux(rcc::rec::Usart234578ClkSel::Pll3Q);

        let mut watchdog = SystemWindowWatchdog::new(ctx.device.WWDG, &ccdr);

        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = ctx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        // let gpioe = ctx.device.GPIOE.split(ccdr.peripheral.GPIOE);

        let led = {
            let led_green = gpiob.pb13.into_push_pull_output();
            let led_blue = gpiob.pb14.into_push_pull_output();
            let led_red = gpiob.pb12.into_push_pull_output();

            vecraft::RGBLed::new_with_color(
                led_green,
                led_blue,
                led_red,
                &vecraft::state::State::ConfigurationError.as_led(),
            )
        };

        let mut eeprom = {
            let scl = gpiob.pb8.into_alternate_open_drain();
            let sda = gpiob.pb9.into_alternate_open_drain();

            #[rustfmt::skip]
            let i2c1 = ctx.device.I2C1.i2c((scl, sda), 400.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);

            vecraft::eeprom::Eeprom::new(i2c1)
        };

        // TODO: Remove this block
        // eeprom.write_page(
        //     vecraft::VECRAFT_CONFIG_PAGE + 250,
        //     &vecraft::VecraftConfig::default().to_bytes(),
        // );

        // TODO: Replace array size with a constant
        let mut vecraft_config = [0; 64];
        eeprom.read_page(vecraft::VECRAFT_CONFIG_PAGE, &mut vecraft_config);

        let config = match vecraft::VecraftConfig::try_from(&vecraft_config[..]) {
            Err(vecraft::ConfigError::InvalidHeader) => {
                let mut vecraft_config_default = [0; 64];
                eeprom.read_page(
                    vecraft::VECRAFT_CONFIG_PAGE + 250,
                    &mut vecraft_config_default,
                );

                vecraft::VecraftConfig::try_from(&vecraft_config_default[..])
                    .expect("No factory config");

                eeprom.write_page(vecraft::VECRAFT_CONFIG_PAGE, &vecraft_config_default);
                vecraft::sys_reboot();
                unreachable!();
            }
            Err(vecraft::ConfigError::InvalidVersion) => panic!("Invalid config"),
            Ok(config) => config,
        };

        let mut console = {
            let rx = gpiod.pd5.into_alternate();
            let tx = gpiod.pd6.into_alternate();

            let usart2 = ctx
                .device
                .USART2
                .serial(
                    (rx, tx),
                    config.uart_baudrate.bps(),
                    ccdr.peripheral.USART2,
                    &ccdr.clocks,
                )
                .expect("Failed to initialize USART2");

            vecraft::console::Console::new(usart2)
        };

        assert!(config.canbus1_bitrate == 250_000 || config.canbus1_bitrate == 500_000);

        let mut canbus1 = {
            let rx = gpiod.pd0.into_alternate().speed(gpio::Speed::VeryHigh);
            let tx = gpiod.pd1.into_alternate().speed(gpio::Speed::VeryHigh);
            let term = gpiod.pd3.into_push_pull_output();

            let fdcan_prec = ccdr
                .peripheral
                .FDCAN
                .kernel_clk_mux(rcc::rec::FdcanClkSel::Pll1Q);

            let fdcan1 = ctx.device.FDCAN1.fdcan(tx, rx, fdcan_prec);

            #[rustfmt::skip]
            let builder = vecraft::can::CanBuilder::new(fdcan1, term)
                    .set_bit_timing(vecraft::can::bit_timing_from_baudrate(config.canbus1_bitrate).unwrap_or(vecraft::can::BITRATE_250K))
                    .set_j1939_broadcast_filter()
                    .set_j1939_destination_address_filter(config.j1939_address)
                    .set_termination(config.canbus1_termination);

            let builder = if let Some(address) = config.j1939_source_address {
                builder.set_j1939_source_address_filter(address)
            } else {
                builder
            };

            builder.build()
        };

        //
        // From this point on, setup hardware and peripherals for this specific application
        //

        assert!(config.ecu_mode().try_into() == Ok(vecraft::EcuApplication::HydraulicControl));

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

        let power_swtich1 = gpiod.pd11.into_push_pull_output();
        let power_swtich2 = gpioa.pa12.into_push_pull_output();

        let gate0 = vecraft::lsgc::Gate::new(pwm_high0, pwm_low0);
        let gate1 = vecraft::lsgc::Gate::new(pwm_high1, pwm_low1);
        let gate2 = vecraft::lsgc::Gate::new(pwm_high2, pwm_low2);
        let gate3 = vecraft::lsgc::Gate::new(pwm_high3, pwm_low3);
        let gate4 = vecraft::lsgc::Gate::new(pwm_high4, pwm_low4);
        let gate5 = vecraft::lsgc::Gate::new(pwm_high5, pwm_low5);
        let gate6 = vecraft::lsgc::Gate::new(pwm_high6, pwm_low6);
        let gate7 = vecraft::lsgc::Gate::new(pwm_high7, pwm_low7);

        let mut gate_lock = vecraft::lsgc::GateLock {
            lockout0: power_swtich1,
            lockout1: power_swtich2,
        };
        gate_lock.lock();

        let mut gate_control = vecraft::lsgc::GateControl {
            gate0,
            gate1,
            gate2,
            gate3,
            gate4,
            gate5,
            gate6,
            gate7,
        };
        gate_control.reset();

        //
        // End of application specific setup. The application is ready to run.
        //

        firmware_state::spawn().ok();
        // commit_config::spawn_after(1.minutes().into()).ok();

        watchdog.start(75.millis());
        watchdog.listen(EarlyWakeup);

        // TOOD: Move to vecraft module
        canbus1.send(protocol::address_claimed(
            config.j1939_address,
            config.j1939_name(),
        ));

        // TOOD: Move to vecraft module
        {
            use core::fmt::Write;

            writeln!(console, "==========================").ok();
            writeln!(console, r#"       //\\  ___"#).ok();
            writeln!(console, r#"       Y  \\/_/=|"#).ok();
            writeln!(console, r#"      _L  ((|_L_|"#).ok();
            writeln!(console, r#"     (/\)(__(____)"#).ok();
            writeln!(console).ok();
            writeln!(console, "    Firmware : {}", crate::PKG_NAME).ok();
            writeln!(console, "    Version  : {}", crate::PKG_VERSION).ok();
            writeln!(console, "    Address  : 0x{:X?}", config.j1939_address).ok();
            writeln!(console).ok();
            writeln!(console, "  Laixer Equipment B.V.").ok();
            writeln!(console, "   Copyright (C) 2024").ok();
            writeln!(console, "==========================").ok();
        };

        (
            SharedResources {
                state: vecraft::state::System::boot(),
                console,
                canbus1,
                gate_lock,
                config,
                last_recv_time: None,
            },
            LocalResources {
                led,
                gate_control,
                watchdog,
                eeprom,
            },
            init::Monotonics(mono),
        )
    }

    #[task(priority = 2, shared = [config, state, canbus1, gate_lock, last_recv_time], local = [led, watchdog, eeprom])]
    fn firmware_state(mut ctx: firmware_state::Context) {
        let config = ctx.shared.config.lock(|config| *config);
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

        //
        // Act on state changes
        //

        const GATE_CONTROL_TIMEOUT: u64 = 1_500;
        let timestamp = monotonics::now().duration_since_epoch().to_millis();
        let recv_within_time = ctx.shared.last_recv_time.lock(|last_recv_time| {
            last_recv_time.map_or(false, |time| {
                // If last received time + 1.5s is greater than current time, we received a frame within 1.5s of each other
                time + GATE_CONTROL_TIMEOUT > timestamp
            })
        });

        let is_locked = ctx.shared.gate_lock.lock(|gate_lock| gate_lock.is_locked());
        if !recv_within_time && !is_locked {
            ctx.shared.gate_lock.lock(|gate_lock| gate_lock.lock());
        }

        if state == vecraft::state::State::Nominal {
            // TODO: Schedule via idle task
            if config.is_dirty {
                let config = ctx.shared.config.lock(|config| *config);

                #[rustfmt::skip]
                ctx.local.eeprom.write_page(vecraft::VECRAFT_CONFIG_PAGE, &config.to_bytes());

                #[rustfmt::skip]
                ctx.shared.config.lock(|config| config.is_dirty = false);
                vecraft::sys_reboot();
            }
            if config.is_factory_reset {
                let mut vecraft_config_default = [0; 64];

                #[rustfmt::skip]
                ctx.local.eeprom.read_page(vecraft::VECRAFT_CONFIG_PAGE + 250, &mut vecraft_config_default);

                vecraft::VecraftConfig::try_from(&vecraft_config_default[..])
                    .expect("No factory config");

                #[rustfmt::skip]
                ctx.local.eeprom.write_page(vecraft::VECRAFT_CONFIG_PAGE, &vecraft_config_default);

                #[rustfmt::skip]
                ctx.shared.config.lock(|config| config.is_factory_reset = false);
                vecraft::sys_reboot();
            }
        }

        //
        // Report state via LED and CAN
        //

        ctx.local
            .led
            .set_color(&state.as_led(), &vecraft::LedState::On);

        let id = IdBuilder::from_pgn(PGN::Other(65_288))
            .sa(config.j1939_address)
            .build();

        let is_locked = ctx.shared.gate_lock.lock(|gate_lock| gate_lock.is_locked());
        let uptime = monotonics::now().duration_since_epoch();
        let timestamp = uptime.to_secs() as u32;

        let frame = FrameBuilder::new(id)
            .copy_from_slice(&[
                state.as_byte(),
                PDU_NOT_AVAILABLE,
                is_locked as u8,
                PDU_NOT_AVAILABLE,
                timestamp.to_le_bytes()[0],
                timestamp.to_le_bytes()[1],
                timestamp.to_le_bytes()[2],
                timestamp.to_le_bytes()[3],
            ])
            .build();

        ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));

        //
        // Feed the watchdog and schedule the next state check
        //

        ctx.local.watchdog.feed();

        firmware_state::spawn_after(50.millis().into()).expect("Fail to schedule");
    }

    #[task(binds = WWDG1, shared = [console, gate_lock])]
    fn watchdog_early_warning(mut ctx: watchdog_early_warning::Context) {
        ctx.shared.gate_lock.lock(|gate_lock| gate_lock.lock());

        #[cfg(debug_assertions)]
        ctx.shared.console.lock(|console| {
            use core::fmt::Write;

            writeln!(console, "Soft fault occurred; resetting hardware").ok();
        });
    }

    fn valve_value(value: i16) -> vecraft::lsgc::GateSide<u16, u16> {
        match value {
            0 => vecraft::lsgc::GateSide::Hold(0, 0),
            v if v.is_positive() => vecraft::lsgc::GateSide::Up(value as u16),
            _ => vecraft::lsgc::GateSide::Down((value + 1).unsigned_abs()),
        }
    }
    fn valve_value32(value: i16) -> vecraft::lsgc::GateSide<u32, u32> {
        match value {
            0 => vecraft::lsgc::GateSide::Hold(0, 0),
            v if v.is_positive() => vecraft::lsgc::GateSide::Up(value as u32),
            _ => vecraft::lsgc::GateSide::Down((value + 1).unsigned_abs() as u32),
        }
    }

    #[task(binds = FDCAN1_IT0, priority = 2, shared = [config, canbus1, state, console, gate_lock, last_recv_time], local = [gate_control])]
    fn can1_event(mut ctx: can1_event::Context) {
        let config = ctx.shared.config.lock(|config| *config);

        let is_bus_error = ctx.shared.canbus1.lock(|canbus1| canbus1.is_bus_error());

        if is_bus_error {
            ctx.shared.state.lock(|state| state.set_bus_error(true));
            ctx.shared.gate_lock.lock(|gate_lock| gate_lock.lock());

            #[cfg(debug_assertions)]
            ctx.shared.console.lock(|console| {
                use core::fmt::Write;

                writeln!(console, "Motion locked due to bus error").ok();
            });

            return;
        }

        while let Some(frame) = ctx.shared.canbus1.lock(|canbus1| canbus1.recv()) {
            match frame.id().pgn() {
                PGN::Request => {
                    let pgn = protocol::request_from_pdu(frame.pdu());

                    match pgn {
                        PGN::SoftwareIdentification => {
                            let id = IdBuilder::from_pgn(PGN::SoftwareIdentification)
                                .sa(config.j1939_address)
                                .build();

                            let frame = FrameBuilder::new(id)
                                .copy_from_slice(&[
                                    0x01,
                                    crate::PKG_VERSION_MAJOR.parse::<u8>().unwrap(),
                                    crate::PKG_VERSION_MINOR.parse::<u8>().unwrap(),
                                    crate::PKG_VERSION_PATCH.parse::<u8>().unwrap(),
                                    FIELD_DELIMITER,
                                ])
                                .build();

                            ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));
                        }
                        PGN::ComponentIdentification => {
                            let id = IdBuilder::from_pgn(PGN::ComponentIdentification)
                                .sa(config.j1939_address)
                                .build();

                            // TODO: Get the serial number from the EEPROM
                            let frame = FrameBuilder::new(id).build();

                            ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));
                        }
                        PGN::AddressClaimed => {
                            ctx.shared.canbus1.lock(|canbus1| {
                                canbus1.send(protocol::address_claimed(
                                    config.j1939_address,
                                    config.j1939_name(),
                                ))
                            });
                        }
                        _ => {
                            ctx.shared.canbus1.lock(|canbus1| {
                                canbus1.send(protocol::acknowledgement(config.j1939_address, pgn))
                            });
                        }
                    }
                }
                PGN::ProprietarilyConfigurableMessage1 => {
                    // TODO: Remove the header
                    if frame.pdu()[0] == b'Z' && frame.pdu()[1] == b'C' {
                        if frame.pdu()[2] & 0b1 == 1 {
                            ctx.shared.gate_lock.lock(|gate_lock| gate_lock.lock());
                            ctx.shared.state.lock(|state| state.set_ident(true));
                        } else if frame.pdu()[2] & 0b1 == 0 {
                            ctx.shared.state.lock(|state| state.set_ident(false));
                        }

                        if frame.pdu()[3] == 0x69 {
                            vecraft::sys_reboot();
                        }
                    }
                }
                PGN::ProprietarilyConfigurableMessage2 => {
                    if frame.pdu() == [0x01; 8] {
                        ctx.shared
                            .config
                            .lock(|config| config.is_factory_reset = true);
                    }
                }
                PGN::ProprietarilyConfigurableMessage3 => {
                    if frame.pdu()[0] == b'Z' && frame.pdu()[1] == b'C' {
                        if frame.pdu()[3] & 0b11 == 0 {
                            ctx.shared.gate_lock.lock(|gate_lock| gate_lock.lock());

                            #[cfg(debug_assertions)]
                            ctx.shared.console.lock(|console| {
                                use core::fmt::Write;

                                writeln!(console, "Motion locked").ok();
                            });
                        } else if frame.pdu()[3] & 0b11 == 1 {
                            let state = ctx.shared.state.lock(|state| state.state());
                            if state == vecraft::state::State::Nominal {
                                ctx.shared.gate_lock.lock(|gate_lock| gate_lock.unlock());
                                ctx.local.gate_control.reset();

                                let timestamp =
                                    monotonics::now().duration_since_epoch().to_millis();
                                ctx.shared
                                    .last_recv_time
                                    .lock(|last_recv_time| *last_recv_time = Some(timestamp));

                                #[cfg(debug_assertions)]
                                ctx.shared.console.lock(|console| {
                                    use core::fmt::Write;

                                    writeln!(console, "Motion unlocked").ok();
                                });
                            }
                        }

                        if frame.pdu()[4] & 0b11 == 1 {
                            ctx.local.gate_control.reset();
                        }
                    }
                }
                PGN::Other(40_960) => {
                    let timestamp = monotonics::now().duration_since_epoch().to_millis();
                    ctx.shared
                        .last_recv_time
                        .lock(|last_recv_time| *last_recv_time = Some(timestamp));

                    if frame.pdu()[0..2] != [PDU_NOT_AVAILABLE, PDU_NOT_AVAILABLE] {
                        let gate_value = i16::from_le_bytes([frame.pdu()[0], frame.pdu()[1]]);

                        ctx.local
                            .gate_control
                            .gate0
                            .set_value(valve_value(gate_value));
                    }
                    if frame.pdu()[2..4] != [PDU_NOT_AVAILABLE, PDU_NOT_AVAILABLE] {
                        let gate_value = i16::from_le_bytes([frame.pdu()[2], frame.pdu()[3]]);

                        ctx.local
                            .gate_control
                            .gate1
                            .set_value(valve_value32(gate_value));
                    }
                    if frame.pdu()[4..6] != [PDU_NOT_AVAILABLE, PDU_NOT_AVAILABLE] {
                        let gate_value = i16::from_le_bytes([frame.pdu()[4], frame.pdu()[5]]);

                        ctx.local
                            .gate_control
                            .gate2
                            .set_value(valve_value32(gate_value));
                    }
                    if frame.pdu()[6..8] != [PDU_NOT_AVAILABLE, PDU_NOT_AVAILABLE] {
                        let gate_value = i16::from_le_bytes([frame.pdu()[6], frame.pdu()[7]]);

                        ctx.local
                            .gate_control
                            .gate3
                            .set_value(valve_value(gate_value));
                    }
                }
                PGN::Other(41_216) => {
                    let timestamp = monotonics::now().duration_since_epoch().to_millis();
                    ctx.shared
                        .last_recv_time
                        .lock(|last_recv_time| *last_recv_time = Some(timestamp));

                    if frame.pdu()[0..2] != [PDU_NOT_AVAILABLE, PDU_NOT_AVAILABLE] {
                        let gate_value = i16::from_le_bytes([frame.pdu()[0], frame.pdu()[1]]);

                        ctx.local
                            .gate_control
                            .gate4
                            .set_value(valve_value(gate_value));
                    }
                    if frame.pdu()[2..4] != [PDU_NOT_AVAILABLE, PDU_NOT_AVAILABLE] {
                        let gate_value = i16::from_le_bytes([frame.pdu()[2], frame.pdu()[3]]);

                        ctx.local
                            .gate_control
                            .gate5
                            .set_value(valve_value(gate_value));
                    }
                    if frame.pdu()[4..6] != [PDU_NOT_AVAILABLE, PDU_NOT_AVAILABLE] {
                        let gate_value = i16::from_le_bytes([frame.pdu()[4], frame.pdu()[5]]);

                        ctx.local
                            .gate_control
                            .gate6
                            .set_value(valve_value(gate_value));
                    }
                    if frame.pdu()[6..8] != [PDU_NOT_AVAILABLE, PDU_NOT_AVAILABLE] {
                        let gate_value = i16::from_le_bytes([frame.pdu()[6], frame.pdu()[7]]);

                        ctx.local
                            .gate_control
                            .gate7
                            .set_value(valve_value(gate_value));
                    }
                }
                _ => {}
            }
        }
    }
}
