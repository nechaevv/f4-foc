#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

mod peripherals;
mod foc;

// Halt on panic
use panic_halt as _; // panic handler

#[rtic::app(device = pac, peripherals = true, dispatchers = [SDIO])]
mod app {
    use stm32f4xx_hal::{timer, gpio::{Output, Pin}, pac, prelude::*, spi};
    use stm32f4xx_hal::gpio::Speed;
    use stm32f4xx_hal::timer::CounterHz;
    use crate::peripherals::{
        at5048a,
        pwm_driver,
        usbserial,
        usbserial::Write
    };
    use crate::foc;
    use crate::foc::{Control, Driver};

    const POLE_COUNT: u16 = 14;
    const THROTTLE: i32 = i32::MAX / 20; //0x07FF;

    #[shared]
    struct Shared {
        angle_sensor: at5048a::At5048aDaisyChain<pac::SPI1, Pin<'B', 2, Output>, 1>,
        control: foc::TorqueControl,
        state: foc::MotorState,
        usb_serial: usbserial::Port
    }

    #[local]
    struct Local {
        led: Pin<'C', 13, Output>,
        motor_driver: pwm_driver::Driver3PWM,
        motor: foc::Motor,
        tick_timer: CounterHz<pac::TIM2>,
        comm_timer: CounterHz<pac::TIM3>,
        usb_dev: usbserial::Device
    }

    #[init(local = [usb_ep_mem: usbserial::EpMemory = usbserial::init_ep_memory(),
                    usb_bus: usbserial::UsbBusStore = usbserial::init_usb_bus_store()])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let dp = cx.device;
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr
            .sysclk(96.MHz())
            .use_hse(25.MHz())
            .hclk(96.MHz())
            .pclk1(48.MHz())
            .pclk2(96.MHz())
            .require_pll48clk()
            .freeze();

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        let motor_driver = pwm_driver::init(dp.TIM1, gpioa.pa8, gpioa.pa9, gpioa.pa10, &clocks);

        let mut tick_timer = dp.TIM2.counter_hz(&clocks);
        tick_timer.start(10.kHz()).unwrap();
        tick_timer.listen(timer::Event::Update);

        let mut comm_timer = dp.TIM3.counter_hz(&clocks);
        comm_timer.start(1.Hz()).unwrap();
        comm_timer.listen(timer::Event::Update);

        let mut led: Pin<'C', 13, Output> = gpioc.pc13.into_push_pull_output();
        led.set_low();

        let spi_mode = spi::Mode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnSecondTransition
        };

        let spi_cs = gpiob.pb2.into_push_pull_output().speed(Speed::VeryHigh);
        let spi_clk = gpiob.pb3.into_push_pull_output().speed(Speed::VeryHigh);
        let spi_cipo = gpiob.pb4.into_floating_input();

        let mut spi =
            spi::Spi::new(dp.SPI1,(spi_clk, spi_cipo, spi::NoMosi::new()), spi_mode, 1.MHz(), &clocks)
                .frame_size_16bit();
        spi.listen(spi::Event::RxNotEmpty);

        let angle_sensor = at5048a::DaisyChain::<1>::init(spi, spi_cs);

        let usb = usbserial::USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into(),
            pin_dp: gpioa.pa12.into(),
            hclk: clocks.hclk(),
        };
        let (usb_dev, usb_serial) = usbserial::init(usb, cx.local.usb_ep_mem, cx.local.usb_bus);

        let motor = foc::Motor::new(POLE_COUNT);
        let state = foc::MotorState { position: 0 };
        let control = foc::TorqueControl { throttle: THROTTLE };

        (Shared { angle_sensor, state, control, usb_serial }, Local { led, motor_driver, motor, tick_timer, comm_timer, usb_dev })
    }

    #[task(binds = TIM2, shared = [angle_sensor, control, state], local = [tick_timer, motor, motor_driver])]
    fn tick(mut cx: tick::Context) {
        cx.local.tick_timer.clear_all_flags();
        cx.shared.angle_sensor.lock(|angle_sensor| {
           angle_sensor.read_start();
        });

        (cx.shared.control, cx.shared.state).lock(|control, state| {
            let phase_vector = control.phase_vector(cx.local.motor, state);
            cx.local.motor_driver.set_voltages(phase_vector);
        });

//        ctx.shared.motor.lock(|motor| {
//            let [a, b, c] = motor.torque_vector(ctx.local.state)
//        });

        // let a = foc::sinusoid(OPENLOOP_THROTTLE, ctx.local.state.phase);
        // let b = foc::sinusoid(OPENLOOP_THROTTLE, ctx.local.state.phase.wrapping_add(PHASE_SHIFT));
        // let c = foc::sinusoid(OPENLOOP_THROTTLE, ctx.local.state.phase.wrapping_add(PHASE_SHIFT * 2));
        //
        // ctx.shared.board.lock(|board| {
        //     board.driver.set_voltages(a, b, c);
        //     if should_toggle_led {
        //         board.led.toggle();
        //         *ctx.local.phase_cycle_count += 1
        //     }
        //     board.phase_loop_timer
        // });

    }
    #[task(binds=SPI1, shared = [angle_sensor, state])]
    fn spi_rxne(cx: spi_rxne::Context) {
        (cx.shared.angle_sensor, cx.shared.state).lock(|angle_sensor, state| {
           match angle_sensor.on_spi_rxne() {
               Err(_) => {}
               Ok((position, _)) => {
                   state.position = position;
               }
           }
        });
    }

    #[task(binds = TIM3, shared = [state, usb_serial], local = [comm_timer, led])]
    fn comm(cx: comm::Context) {
        cx.local.comm_timer.clear_all_flags();
        (cx.shared.state, cx.shared.usb_serial).lock(|state, serial| {
            if serial.dtr() {
                let percent_value = ((state.position >> 16) * 100) >> 15;
                write!(serial, "{}\r\n", percent_value).unwrap();
            }
        });
        cx.local.led.toggle();
    }

    #[task(binds = OTG_FS, shared = [usb_serial], local = [usb_dev], priority=2)]
    fn otg_fs(mut cx: otg_fs::Context) {
        cx.shared.usb_serial.lock(|usb_serial| {
            if cx.local.usb_dev.poll(&mut [usb_serial]) {
                let mut buf = [0u8; 64];
                usb_serial.read(&mut buf).ok();
            }
        });
    }
}