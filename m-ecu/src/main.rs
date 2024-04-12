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

// /// J1939 network address.
// // const J1939_ADDRESS: u8 = 0x12;
/// J1939 name manufacturer code.
///
const J1939_NAME_MANUFACTURER_CODE: u16 = 0x717;
/// J1939 name function instance.
const J1939_NAME_FUNCTION_INSTANCE: u8 = 1;
/// J1939 name ECU instance.
const J1939_NAME_ECU_INSTANCE: u8 = 1;
/// J1939 name function.
const J1939_NAME_FUNCTION: u8 = 0x11;
/// J1939 name vehicle system.
const J1939_NAME_VEHICLE_SYSTEM: u8 = 9;

/// Engine RPM minimum.
// const ENGINE_RPM_MIN: u16 = 700;
// /// Engine RPM maximum.
// const ENGINE_RPM_MAX: u16 = 2300;
mod protocol;

#[derive(Copy, Clone, Debug)]
pub struct Config {
    pub is_dirty: bool,
    pub is_factory_reset: bool,
    pub sa: u8,
    pub da: u8,
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [USART1, USART2])]
mod app {
    use stm32h7xx_hal::gpio::{self};
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc;
    use stm32h7xx_hal::system_watchdog::SystemWindowWatchdog;

    use vecraft::fdcan;
    use vecraft::j1939::{
        protocol, FrameBuilder, IdBuilder, NameBuilder, FIELD_DELIMITER, PDU_NOT_AVAILABLE, PGN,
    };
    use vecraft::Systick;

    /// 100 Hz / 10 ms granularity
    #[monotonic(binds = SysTick, default = true)]
    type Monotonic = Systick<100>;

    #[shared]
    struct SharedResources {
        in2: gpio::Pin<'B', 0, gpio::Output>,
        in1: gpio::Pin<'B', 1, gpio::Output>,
        state: vecraft::state::System,
        config: crate::Config,
        console: vecraft::console::Console,
        canbus1: vecraft::can::Can<
            stm32h7xx_hal::can::Can<stm32h7xx_hal::stm32::FDCAN1>,
            fdcan::NormalOperationMode,
        >,
    }

    #[local]
    struct LocalResources {
        is_starting: bool,
        // pwm0: stm32h7xx_hal::pwm::Pwm<
        //     stm32h7xx_hal::stm32::TIM3,
        //     3,
        //     stm32h7xx_hal::pwm::ComplementaryImpossible,
        // >,
        // pwm1: stm32h7xx_hal::pwm::Pwm<
        //     stm32h7xx_hal::stm32::TIM3,
        //     2,
        //     stm32h7xx_hal::pwm::ComplementaryImpossible,
        // >,
        led: vecraft::RGBLed,
        watchdog: SystemWindowWatchdog,
        eeprom: vecraft::eeprom::Eeprom,
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
        ccdr.peripheral.kernel_adc_clk_mux(rcc::rec::AdcClkSel::Per);

        ccdr.peripheral
            .kernel_usart234578_clk_mux(rcc::rec::Usart234578ClkSel::Pll3Q);

        let mut watchdog = SystemWindowWatchdog::new(ctx.device.WWDG, &ccdr);

        // GPIO
        // let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpiod = ctx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        // let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpioe = ctx.device.GPIOE.split(ccdr.peripheral.GPIOE);

        let mut led = vecraft::RGBLed::new(
            gpiob.pb13.into_push_pull_output(),
            gpiob.pb14.into_push_pull_output(),
            gpiob.pb12.into_push_pull_output(),
        );

        // Configure the SCL and the SDA pin for our I2C bus
        let scl = gpiob.pb8.into_alternate_open_drain();
        let sda = gpiob.pb9.into_alternate_open_drain();

        let i2c = ctx
            .device
            .I2C1
            .i2c((scl, sda), 400.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);

        let mut eeprom = vecraft::eeprom::Eeprom::new(i2c);

        led.set_color(
            &vecraft::state::State::ConfigurationError.as_led(),
            &vecraft::LedState::On,
        );

        let mut vecraft_config = [0; 64];
        eeprom.read_page(vecraft::VECRAFT_CONFIG_PAGE, &mut vecraft_config);

        let config = match vecraft::VecraftConfig::try_from(&vecraft_config[..]) {
            Err(1) => {
                let mut vecraft_config_default = [0; 64];
                eeprom.read_page(
                    vecraft::VECRAFT_CONFIG_PAGE + 250,
                    &mut vecraft_config_default,
                );
                eeprom.write_page(vecraft::VECRAFT_CONFIG_PAGE, &vecraft_config_default);
                vecraft::sys_reset();
                unreachable!();
            }
            Err(_) => panic!("Invalid config"),
            Ok(config) => config,
        };

        // UART
        let mut console = vecraft::console::Console::new(
            ctx.device
                .USART2
                .serial(
                    (gpiod.pd5.into_alternate(), gpiod.pd6.into_alternate()),
                    config.uart_baudrate.bps(),
                    ccdr.peripheral.USART2,
                    &ccdr.clocks,
                )
                .unwrap(),
        );

        let fdcan_prec = ccdr
            .peripheral
            .FDCAN
            .kernel_clk_mux(rcc::rec::FdcanClkSel::Pll1Q);

        let mut canbus1 = {
            let rx = gpiod.pd0.into_alternate().speed(gpio::Speed::VeryHigh);
            let tx = gpiod.pd1.into_alternate().speed(gpio::Speed::VeryHigh);

            let pd3 = gpiod.pd3.into_push_pull_output();

            // TODO: Add filter
            vecraft::can::CanBuilder::new(ctx.device.FDCAN1.fdcan(tx, rx, fdcan_prec), pd3)
                .set_bit_timing(vecraft::can::BITRATE_250K)
                .set_default_filter(config.j1939_address())
                .set_termination(config.canbus1_termination)
                .build()
        };

        let mut power2_enable = gpioe.pe2.into_push_pull_output();

        // let (_, (_, _, mut pwm1, mut pwm0)) = ctx
        //     .device
        //     .TIM3
        //     .pwm_advanced(
        //         (
        //             gpioc.pc6.into_alternate(),
        //             gpioc.pc7.into_alternate(),
        //             gpiob.pb0.into_alternate(),
        //             gpiob.pb1.into_alternate(),
        //         ),
        //         ccdr.peripheral.TIM3,
        //         &ccdr.clocks,
        //     )
        //     .frequency(100.Hz())
        //     .period(32_767)
        //     .finalize();

        // pwm0.set_duty(0);
        // pwm0.enable();

        // pwm1.set_duty(0);
        // pwm1.enable();

        let mut in2 = gpiob.pb0.into_push_pull_output();
        let mut in1 = gpiob.pb1.into_push_pull_output();

        in1.set_low();
        in2.set_low();

        power2_enable.set_high();

        firmware_state::spawn().ok();
        // firmware_test::spawn().ok();

        watchdog.start(75.millis());

        {
            // TODO: Make an identity number based on debug and firmware version
            let name = NameBuilder::default()
                .identity_number(config.serial_number().1)
                .manufacturer_code(config.j1939_name().manufacturer_code)
                .function_instance(config.j1939_name().function_instance)
                .ecu_instance(config.j1939_name().ecu_instance)
                .function(config.j1939_name().function)
                .vehicle_system(config.j1939_name().vehicle_system)
                .vehicle_system_instance(config.j1939_name().vehicle_system_instance)
                .industry_group(config.j1939_name().industry_group)
                .arbitrary_address(config.j1939_name().arbitrary_address)
                .build();

            canbus1.send(protocol::address_claimed(config.j1939_address(), name));
        }

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
            writeln!(console, "    Address  : 0x{:X?}", config.j1939_address()).ok();
            writeln!(console).ok();
            writeln!(console, "  Laixer Equipment B.V.").ok();
            writeln!(console, "   Copyright (C) 2024").ok();
            writeln!(console, "==========================").ok();
        };

        (
            SharedResources {
                in2,
                in1,
                state: vecraft::state::System::boot(),
                config: crate::Config {
                    is_dirty: false,
                    is_factory_reset: false,
                    sa: config.j1939_address(),
                    da: 0x11,
                },
                console,
                canbus1,
            },
            LocalResources {
                is_starting: false,
                // pwm0,
                // pwm1,
                led,
                watchdog,
                eeprom,
            },
            init::Monotonics(mono),
        )
    }

    #[task(priority = 2, shared = [config, state, canbus1], local = [led, watchdog, eeprom])]
    fn firmware_state(mut ctx: firmware_state::Context) {
        let config = ctx.shared.config.lock(|config| *config);

        let is_bus_ok = ctx.shared.canbus1.lock(|canbus1| canbus1.is_bus_ok());

        let state = ctx.shared.state.lock(|state| {
            if is_bus_ok {
                state.set_bus_error(false);
            } else {
                state.set_bus_error(true);
            }

            state.state()
        });

        if state == vecraft::state::State::Nominal {
            if config.is_dirty {
                // TODO: Write the config to the EEPROM
                ctx.shared.config.lock(|config| config.is_dirty = false);
            }
            if config.is_factory_reset {
                let mut vecraft_config_default = [0; 64];

                #[rustfmt::skip]
                ctx.local.eeprom.read_page(vecraft::VECRAFT_CONFIG_PAGE + 250, &mut vecraft_config_default);
                #[rustfmt::skip]
                ctx.local.eeprom.write_page(vecraft::VECRAFT_CONFIG_PAGE, &vecraft_config_default);
                vecraft::sys_reset();
            }
        }

        ctx.local
            .led
            .set_color(&state.as_led(), &vecraft::LedState::On);

        let id = IdBuilder::from_pgn(PGN::Other(65_288))
            .sa(config.sa)
            .build();

        let uptime = monotonics::now().duration_since_epoch();
        let timestamp = uptime.to_secs() as u32;
        let state_subclass = 0;

        let frame = FrameBuilder::new(id)
            .copy_from_slice(&[
                state.as_byte(),
                state_subclass,
                PDU_NOT_AVAILABLE,
                PDU_NOT_AVAILABLE,
                timestamp.to_le_bytes()[0],
                timestamp.to_le_bytes()[1],
                timestamp.to_le_bytes()[2],
                timestamp.to_le_bytes()[3],
            ])
            .build();

        ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));

        ctx.local.watchdog.feed();

        firmware_state::spawn_after(50.millis().into()).unwrap();
    }

    // #[task(priority = 2, shared = [state, canbus1], local = [toggle])]
    // fn firmware_test(mut ctx: firmware_test::Context) {
    //     if *ctx.local.toggle {
    //         *ctx.local.toggle = false;
    //         ctx.shared.state.lock(|state| state.set_ident(false));
    //     } else {
    //         *ctx.local.toggle = true;
    //         ctx.shared.state.lock(|state| state.set_ident(true));
    //     }

    //     firmware_test::spawn_after(500.millis().into()).unwrap();
    // }

    #[task(priority = 2, shared = [in1, in2, state], local = [])]
    fn start_timeout(mut ctx: start_timeout::Context) {
        ctx.shared.in1.lock(|in1| in1.set_low());
        ctx.shared.in2.lock(|in2| in2.set_low());
        ctx.shared.state.lock(|state| state.set_ident(false));
    }

    #[task(binds = FDCAN1_IT0, priority = 2, shared = [config, canbus1, state, console, in1, in2], local = [is_starting])]
    fn can1_event(mut ctx: can1_event::Context) {
        let config = ctx.shared.config.lock(|config| *config);

        let is_bus_error = ctx.shared.canbus1.lock(|canbus1| canbus1.is_bus_error());

        if is_bus_error {
            ctx.shared.state.lock(|state| state.set_bus_error(true));

            return;
        }

        while let Some(frame) = ctx.shared.canbus1.lock(|canbus1| canbus1.recv()) {
            match frame.id().pgn() {
                PGN::Request => {
                    let pgn = protocol::request_from_pdu(frame.pdu());

                    match pgn {
                        PGN::SoftwareIdentification => {
                            let id = IdBuilder::from_pgn(PGN::SoftwareIdentification)
                                .sa(config.sa)
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
                        PGN::AddressClaimed => {
                            // TODO: Get the name from the EEPROM
                            // TODO: Make an identity number based on debug and firmware version
                            let name = NameBuilder::default()
                                .identity_number(0x1)
                                .manufacturer_code(crate::J1939_NAME_MANUFACTURER_CODE)
                                .function_instance(crate::J1939_NAME_FUNCTION_INSTANCE)
                                .ecu_instance(crate::J1939_NAME_ECU_INSTANCE)
                                .function(crate::J1939_NAME_FUNCTION)
                                .vehicle_system(crate::J1939_NAME_VEHICLE_SYSTEM)
                                .build();

                            ctx.shared.canbus1.lock(|canbus1| {
                                canbus1.send(protocol::address_claimed(config.sa, name))
                            });
                        }
                        _ => {
                            ctx.shared.canbus1.lock(|canbus1| {
                                canbus1.send(protocol::acknowledgement(config.sa, pgn))
                            });
                        }
                    }
                }
                PGN::ProprietarilyConfigurableMessage1 => {
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
                PGN::ElectronicEngineController1 => {
                    // Used for hydraulic pump control
                    // if let Some(rpm) = message.rpm {
                    //     let duty = match rpm {
                    //         ..=1049 => 24_500,
                    //         1050..=1549 => 22_500,
                    //         1550..=u16::MAX => 20_500,
                    //     };

                    //     ctx.local.pwm0.set_duty(duty);
                    // }
                }
                PGN::ProprietaryB(65_282) => {
                    if frame.id().sa() == 0x11 {
                        let message =
                            crate::protocol::VolvoSpeedRequestMessage::from_pdu(frame.pdu());

                        if message.engine_mode == Some(crate::protocol::EngineMode::Starting) {
                            if !*ctx.local.is_starting {
                                *ctx.local.is_starting = true;
                                ctx.shared.in1.lock(|in1| in1.set_high());
                                ctx.shared.in2.lock(|in2| in2.set_low());
                                ctx.shared.state.lock(|state| state.set_ident(true));
                                start_timeout::spawn_after(1_500.millis().into()).unwrap();
                            }
                        } else {
                            *ctx.local.is_starting = false;
                            ctx.shared.in1.lock(|in1| in1.set_low());
                            ctx.shared.in2.lock(|in2| in2.set_low());
                            ctx.shared.state.lock(|state| state.set_ident(false));
                        }
                    }
                }
                _ => {}
            }
        }
    }
}
