#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use vecraft::panic_halt as _;

use stm32h7xx_hal::time::Hertz;

/// Glonax firmware name.
#[cfg(debug_assertions)]
const PKG_NAME: &str = env!("CARGO_PKG_NAME");
/// Glonax firmware version.
#[cfg(debug_assertions)]
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

/// J1939 network address.
const J1939_ADDRESS: u8 = 0x0;
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
const ENGINE_RPM_MIN: u16 = 700;
/// Engine RPM maximum.
const ENGINE_RPM_MAX: u16 = 2300;

mod protocol;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [USART1, USART2])]
mod app {
    use stm32h7xx_hal::gpio::{self};
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc;
    use stm32h7xx_hal::system_watchdog::SystemWindowWatchdog;

    use vecraft::fdcan;
    use vecraft::j1939::{
        protocol, spn, FrameBuilder, IdBuilder, NameBuilder, FIELD_DELIMITER, PDU_NOT_AVAILABLE,
        PGN,
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
    }

    #[local]
    struct LocalResources {
        in2: gpio::Pin<'B', 0, gpio::Output>,
        in1: gpio::Pin<'B', 1, gpio::Output>,
        led: vecraft::led::Led,
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
            .kernel_clk_mux(rcc::rec::FdcanClkSel::Pll1Q);

        let canbus1 = {
            let rx = gpiod.pd0.into_alternate().speed(gpio::Speed::VeryHigh);
            let tx = gpiod.pd1.into_alternate().speed(gpio::Speed::VeryHigh);

            let pd3 = gpiod.pd3.into_push_pull_output();

            vecraft::can::CanBuilder::new(ctx.device.FDCAN1.fdcan(tx, rx, fdcan_prec), pd3)
                .set_bit_timing(vecraft::can::BITRATE_250K)
                .build()
        };

        let mut power2_enable = gpioe.pe2.into_push_pull_output();

        let in2 = gpiob.pb0.into_push_pull_output();
        let in1 = gpiob.pb1.into_push_pull_output();

        power2_enable.set_high();

        bootstrap::spawn().ok();
        firmware_state::spawn().ok();

        watchdog.start(75.millis());

        (
            SharedResources {
                state: vecraft::state::System::boot(),
                console,
                canbus1,
            },
            LocalResources {
                in2,
                in1,
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

    #[task(shared = [canbus1, console])]
    fn bootstrap(mut ctx: bootstrap::Context) {
        // TODO: Make an identity number based on debug and firmware version
        let name = NameBuilder::default()
            .identity_number(0x1)
            .manufacturer_code(crate::J1939_NAME_MANUFACTURER_CODE)
            .function_instance(crate::J1939_NAME_FUNCTION_INSTANCE)
            .ecu_instance(crate::J1939_NAME_ECU_INSTANCE)
            .function(crate::J1939_NAME_FUNCTION)
            .vehicle_system(crate::J1939_NAME_VEHICLE_SYSTEM)
            .build();

        ctx.shared
            .canbus1
            .lock(|canbus1| canbus1.send(protocol::address_claimed(crate::J1939_ADDRESS, name)));

        #[cfg(debug_assertions)]
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
            writeln!(console, "    Address  : 0x{:X?}", crate::J1939_ADDRESS).ok();
            writeln!(console).ok();
            writeln!(console, "  Laixer Equipment B.V.").ok();
            writeln!(console, "   Copyright (C) 2024").ok();
            writeln!(console, "==========================").ok();
        });
    }

    #[task(shared = [state, canbus1], local = [led, watchdog])]
    fn firmware_state(mut ctx: firmware_state::Context) {
        let is_bus_ok = ctx.shared.canbus1.lock(|canbus1| canbus1.is_bus_ok());

        let state = ctx.shared.state.lock(|state| {
            if is_bus_ok {
                state.set_bus_error(false);
            } else {
                state.set_bus_error(true);
            }

            state.state()
        });

        ctx.local
            .led
            .set_color(&state.as_led(), &vecraft::led::LedState::On);

        ctx.local.watchdog.feed();

        let id = IdBuilder::from_pgn(PGN::Other(65_288))
            .sa(crate::J1939_ADDRESS)
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

        firmware_state::spawn_after(50.millis().into()).unwrap();
    }

    #[task(binds = FDCAN1_IT0, priority = 2, shared = [canbus1, state, console], local = [in1, in2])]
    fn can1_event(mut ctx: can1_event::Context) {
        let is_bus_error = ctx.shared.canbus1.lock(|canbus1| canbus1.is_bus_error());

        if is_bus_error {
            ctx.shared.state.lock(|state| state.set_bus_error(true));
        }

        while let Some(frame) = ctx.shared.canbus1.lock(|canbus1| canbus1.recv()) {
            match frame.id().pgn() {
                PGN::Request => {
                    let pgn = protocol::request_from_pdu(frame.pdu());

                    match pgn {
                        PGN::SoftwareIdentification => {
                            let id = IdBuilder::from_pgn(PGN::SoftwareIdentification)
                                .sa(crate::J1939_ADDRESS)
                                .build();

                            let frame = FrameBuilder::new(id)
                                .copy_from_slice(&[
                                    0x01,
                                    crate::PKG_VERSION_MAJOR.parse::<u8>().unwrap(),
                                    crate::PKG_VERSION_MINOR.parse::<u8>().unwrap(),
                                    crate::PKG_VERSION_PATCH.parse::<u8>().unwrap(),
                                    FIELD_DELIMITER,
                                    PDU_NOT_AVAILABLE,
                                    PDU_NOT_AVAILABLE,
                                    PDU_NOT_AVAILABLE,
                                ])
                                .build();

                            ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));
                        }
                        PGN::AddressClaimed => {
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
                                canbus1.send(protocol::address_claimed(crate::J1939_ADDRESS, name))
                            });
                        }
                        _ => {
                            ctx.shared.canbus1.lock(|canbus1| {
                                canbus1.send(protocol::acknowledgement(crate::J1939_ADDRESS, pgn))
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
                PGN::ElectronicBrakeController1 => {
                    if frame.pdu()[3] != PDU_NOT_AVAILABLE
                        && 0b0001_0000 & frame.pdu()[3] == 0b0001_0000
                    {
                        ctx.local.in1.set_low();
                        ctx.local.in2.set_low();

                        let frame = crate::protocol::volvo_speed_request(
                            crate::protocol::EngineMode::Shutdown,
                            crate::ENGINE_RPM_MIN,
                        );

                        ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));
                    }
                }
                PGN::TorqueSpeedControl1 => {
                    if frame.pdu()[0] != PDU_NOT_AVAILABLE {
                        let control_mode = frame.pdu()[0];
                        let rpm = spn::rpm::dec(&frame.pdu()[1..3])
                            .unwrap_or(crate::ENGINE_RPM_MIN)
                            .clamp(crate::ENGINE_RPM_MIN, crate::ENGINE_RPM_MAX);

                        let mode = match 0b11 & control_mode {
                            0b01 => crate::protocol::EngineMode::Nominal,
                            0b11 => crate::protocol::EngineMode::Starting,
                            _ => crate::protocol::EngineMode::Locked,
                        };

                        if mode == crate::protocol::EngineMode::Starting {
                            ctx.local.in1.set_high();
                            ctx.local.in2.set_low();
                        } else {
                            ctx.local.in1.set_low();
                            ctx.local.in2.set_low();
                        }

                        let frame = crate::protocol::volvo_speed_request(mode, rpm);

                        ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));
                    }
                }
                _ => {}
            }
        }
    }
}
