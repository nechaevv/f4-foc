use stm32f4xx_hal::gpio::{Pin, Output};

pub mod at5048a;
pub mod usbserial;
pub mod pwm_driver;

pub trait OutputPin {
    fn set_high(&mut self);
    fn set_low(&mut self);
    fn toggle(&mut self);
}

impl <const C:char, const N:u8, OM> OutputPin for Pin<C, N, Output<OM>> {
    fn set_high(&mut self) {
        Pin::set_high(self);
    }

    fn set_low(&mut self) {
        Pin::set_low(self);
    }

    fn toggle(&mut self) {
        Pin::toggle(self);
    }
}

