use esp_hal::{
    gpio::{Output, OutputConfig},
    ledc::{
        channel::{self, ChannelIFace},
        LowSpeed,
    },
};

#[derive(Clone, Copy)]
pub enum MotorDirection {
    Clockwise,
    CounterClockwise,
    Brake,
}

pub struct MotorController<'a> {
    dir: Output<'a>,
    pwm: channel::Channel<'a, LowSpeed>,
}

impl<'a> MotorController<'a> {
    pub fn new(dir: Output<'a>, pwm: channel::Channel<'a, LowSpeed>) -> Self {
        Self { dir, pwm }
    }

    pub fn set_motion(&mut self, direction: MotorDirection, speed: u16, fade_ms: u16) {
        let speed_clamped = speed.min(100) as u8;

        match direction {
            MotorDirection::Clockwise => {
                self.dir.set_low();
                self.pwm.start_duty_fade(0, speed as u8, fade_ms);
            }
            MotorDirection::CounterClockwise => {
                self.dir.set_high();
                loop {
                    self.pwm.set_duty(0);
                    break;
                }
            }
            MotorDirection::Brake => {
                self.dir.set_low();
                loop {
                    self.pwm.set_duty(0);
                    break;
                }
            }
        }
    }
}
