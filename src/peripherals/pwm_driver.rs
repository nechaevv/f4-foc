use stm32f4xx_hal::{
    gpio::{PA8, PA9, PA10},
    pac::TIM1,
    prelude::*,
    rcc::Clocks,
    timer::{Channel, ChannelBuilder, PwmHz, Channel1, Channel2, Channel3}
};
use crate::foc::Driver;

pub type Driver3PWM = PwmHz<TIM1, (ChannelBuilder<TIM1, 0>, ChannelBuilder<TIM1, 1>, ChannelBuilder<TIM1, 2>)>;

impl Driver for Driver3PWM {
    fn set_voltages(&mut self, phase_vector: [u16; 3]) {
        let max_duty = self.get_max_duty();
        let to_duty = |value: u16| -> u16 {
            if max_duty == 0 { value } else {
                ((value as u32) << 16 / max_duty) as u16
            }
        };
        self.set_duty(Channel::C1, to_duty(phase_vector[0]));
        self.set_duty(Channel::C2, to_duty(phase_vector[1]));
        self.set_duty(Channel::C3, to_duty(phase_vector[2]));
    }
}

pub fn init(tim1: TIM1, pa8: PA8, pa9: PA9, pa10: PA10, clocks: &Clocks) -> Driver3PWM {
    let channels = (
        Channel1::new(pa8),
        Channel2::new(pa9),
        Channel3::new(pa10)
    );
    let mut pwm = tim1.pwm_hz(channels, 20.kHz(), &clocks);
    pwm.enable(Channel::C1);
    pwm.enable(Channel::C2);
    pwm.enable(Channel::C3);
    pwm
}