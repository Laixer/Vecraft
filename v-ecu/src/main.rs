// #![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_halt as _;

use stm32h7xx_hal::time::Hertz;

const PKG_NAME: &str = env!("CARGO_PKG_NAME");
const PKG_VERSION: &str = env!("CARGO_PKG_VERSION");

const AVIC_BRIDGE_VENDOR_ID: u16 = 0x1d6b;
const AVIC_BRIDGE_PRODUCT_ID: u16 = 0x27dd;

const AVIC_BRIDGE_CAN_DEV_CLASS: u8 = 0xff;
const AVIC_BRIDGE_CAN_DEV_SUBCLASS: u8 = 0;
const AVIC_BRIDGE_CAN_DEV_PROTO: u8 = 0;

const DEVICE_MANUFACTURER: &str = "Laixer Equipment B.V.";
const DEVICE_PRODUCT: &str = "AVIC CAN Bridge";

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
        canbus2: vecraft::can::Can<
            stm32h7xx_hal::can::Can<stm32h7xx_hal::stm32::FDCAN2>,
            fdcan::NormalOperationMode,
        >,
        usb: usb_device::prelude::UsbDevice<
            'static,
            stm32h7xx_hal::usb_hs::UsbBus<stm32h7xx_hal::usb_hs::USB2>,
        >,
        avic: vecraft::usb_avic::AvicClass<
            'static,
            stm32h7xx_hal::usb_hs::UsbBus<stm32h7xx_hal::usb_hs::USB2>,
        >,
    }

    #[local]
    struct LocalResources {
        led: vecraft::led::Led,
    }

    static mut USB2_BUS: Option<
        usb_device::class_prelude::UsbBusAllocator<
            stm32h7xx_hal::usb_hs::UsbBus<stm32h7xx_hal::usb_hs::USB2>,
        >,
    > = None;

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
            .kernel_usb_clk_mux(rcc::rec::UsbClkSel::PLL3_Q);

        ccdr.peripheral
            .kernel_usart234578_clk_mux(rcc::rec::Usart234578ClkSel::PLL3_Q);

        // GPIO
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
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
                .build()
        };

        let canbus2 = {
            let rx = gpiob.pb5.into_alternate().speed(gpio::Speed::VeryHigh);
            let tx = gpiob.pb6.into_alternate().speed(gpio::Speed::VeryHigh);

            let pd7 = gpiod.pd7.into_push_pull_output();

            vecraft::can::CanBuilder::new(ctx.device.FDCAN2.fdcan_simple(tx, rx), pd7)
                .set_bit_timing(vecraft::can::BITRATE_250K)
                .build()
        };

        use usb_device::prelude::*;

        static mut EP_MEMORY: [u32; 1024] = [0; 1024];

        let usb2 = stm32h7xx_hal::usb_hs::USB2::new(
            ctx.device.OTG2_HS_GLOBAL,
            ctx.device.OTG2_HS_DEVICE,
            ctx.device.OTG2_HS_PWRCLK,
            gpioa.pa11.into_alternate(),
            gpioa.pa12.into_alternate(),
            ccdr.peripheral.USB2OTG,
            &ccdr.clocks,
        );

        unsafe { USB2_BUS = Some(stm32h7xx_hal::usb_hs::UsbBus::new(usb2, &mut EP_MEMORY)) };

        let usb_bus = unsafe { USB2_BUS.as_ref().unwrap() };

        let inner = vecraft::usb_avic::AvicClass::new(usb_bus);

        let device = UsbDeviceBuilder::new(
            usb_bus,
            UsbVidPid(crate::AVIC_BRIDGE_VENDOR_ID, crate::AVIC_BRIDGE_PRODUCT_ID),
        )
        .manufacturer(crate::DEVICE_MANUFACTURER)
        .product(crate::DEVICE_PRODUCT)
        .serial_number(crate::PKG_VERSION)
        .device_class(crate::AVIC_BRIDGE_CAN_DEV_CLASS)
        .device_sub_class(crate::AVIC_BRIDGE_CAN_DEV_SUBCLASS)
        .device_protocol(crate::AVIC_BRIDGE_CAN_DEV_PROTO)
        .device_release(0x100)
        .build();

        motd::spawn().ok();
        firmware_state::spawn().ok();
        status_print::spawn().ok();

        (
            SharedResources {
                state: vecraft::state::System::boot(),
                console,
                canbus1,
                canbus2,
                usb: device,
                avic: inner,
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

            writeln!(console, "==========================").ok();
            writeln!(console, r#"       //\\  ___"#).ok();
            writeln!(console, r#"       Y  \\/_/=|"#).ok();
            writeln!(console, r#"      _L  ((|_L_|"#).ok();
            writeln!(console, r#"     (/\)(__(____)"#).ok();
            writeln!(console).ok();
            writeln!(console, "    Firmware : {}", crate::PKG_NAME).ok();
            writeln!(console, "    Version  : {}", crate::PKG_VERSION).ok();
            writeln!(console).ok();
            writeln!(console, "  Laixer Equipment B.V.").ok();
            writeln!(console, "   Copyright (C) 2022").ok();
            writeln!(console, "==========================").ok();
        });
    }

    #[task(shared = [state, canbus1], local = [led])]
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
            .set_color(&state.as_led(), &vecraft::led::LedState::On);

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

        status_print::spawn_after(1.secs().into()).unwrap();
    }

    #[task(binds = FDCAN1_IT0, priority = 2, shared = [canbus1, usb, avic, state])]
    fn can1_event(mut ctx: can1_event::Context) {
        let is_bus_error = ctx.shared.canbus1.lock(|canbus1| canbus1.is_bus_error());

        if is_bus_error {
            ctx.shared.state.lock(|state| state.set_bus_error(true));
        }

        while let Some(frame) = ctx.shared.canbus1.lock(|canbus1| canbus1.recv()) {
            ctx.shared.usb.lock(|usb| {
                if usb.state() == usb_device::device::UsbDeviceState::Configured {
                    ctx.shared.avic.lock(|avic| {
                        let frame =
                            vecraft::usb_frame::FeameBuilder::with_extended_id(frame.id().as_raw())
                                .data(frame.pdu())
                                .build();

                        let _ = avic.write_frame(
                            vecraft::usb_avic::ClassInterface::Interface0,
                            frame.as_ref(),
                        );
                    });
                }
            });
        }
    }

    #[task(binds = FDCAN2_IT0, priority = 3, shared = [canbus2, usb, avic, state])]
    fn can2_event(mut ctx: can2_event::Context) {
        let is_bus_error = ctx.shared.canbus2.lock(|canbus2| canbus2.is_bus_error());

        if is_bus_error {
            ctx.shared.state.lock(|state| state.set_bus_error(true));
        }

        while let Some(frame) = ctx.shared.canbus2.lock(|canbus1| canbus1.recv()) {
            ctx.shared.usb.lock(|usb| {
                if usb.state() == usb_device::device::UsbDeviceState::Configured {
                    ctx.shared.avic.lock(|avic| {
                        let frame =
                            vecraft::usb_frame::FeameBuilder::with_extended_id(frame.id().as_raw())
                                .data(frame.pdu())
                                .build();

                        let _ = avic.write_frame(
                            vecraft::usb_avic::ClassInterface::Interface1,
                            frame.as_ref(),
                        );
                    });
                }
            });
        }
    }

    #[task(binds = OTG_FS, priority = 2, shared = [canbus1, usb, avic, state, console])]
    fn usb_event(mut ctx: usb_event::Context) {
        use usb_device::prelude::UsbDeviceState;

        let data_available = ctx.shared.usb.lock(|usb| loop {
            let data_available = ctx.shared.avic.lock(|avic| usb.poll(&mut [avic]));

            match usb.state() {
                UsbDeviceState::Configured | UsbDeviceState::Suspend => break data_available,
                _ => {}
            }
        });

        if data_available {
            loop {
                match ctx.shared.avic.lock(|avic| avic.read_frame()) {
                    Ok(usb_frame) => {
                        let id = vecraft::j1939::Id::new(usb_frame.id());

                        let frame = vecraft::j1939::FrameBuilder::new(id)
                            .copy_from_slice(&usb_frame.data()[..usb_frame.len() as usize])
                            .build();

                        ctx.shared.canbus1.lock(|canbus1| canbus1.send(frame));
                    }
                    Err(usb_device::UsbError::WouldBlock) => break,
                    Err(e) => {
                        ctx.shared.console.lock(|console| {
                            use core::fmt::Write;

                            writeln!(console, "USB Error: {:?}", e).ok();
                        });
                        break;
                    }
                }
            }
        }
    }
}
