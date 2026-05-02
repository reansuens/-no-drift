#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is unsafe with esp_hal types holding buffers."
)]

use defmt::info;
use drift::{
    differential_drive::{DifferentialDrive, VehicleMotion},
    encoders::Encoders,
    forward_kalman::forward_one_kalman,
    mpu::{ImuBias, Mpu6050},
};
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    main,
    time::Rate,
};
use esp_println as _;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        info!("FAULT: UNRECOVERABLE_EXCEPTION_DETECTED");
        let delay = Delay::new();
        delay.delay_millis(1500);
    }
}


fn turn_90_ccw(
    encoders: &Encoders,
    mpu: &mut Mpu6050,
    bias: &ImuBias,
    drive: &mut DifferentialDrive,
    delay: &mut Delay,
) {
    let mut heading: f32 = 0.0;
    let mut angular_v: f32 = 0.0;
    let dt_ms: f32 = 10.0;
    let gz_bias = bias.gz_bias.abs();
    let mut prev_la = encoders.left_a.is_high();
    let mut prev_lb = encoders.left_b.is_low();
    let mut prev_ra = encoders.right_a.is_high();
    let mut prev_rb = encoders.right_b.is_low();
    let mut edges_l: u32 = 0;
    let mut edges_r: u32 = 0;
    let mut gz: f32 = 0.0;
    drive.execute(VehicleMotion::SpinCCW, 100, 0);
    loop {
        let (_, _, _, gz_new) = mpu.read_corrected(bias);
        gz = gz_new;
        angular_v += gz * (dt_ms / 1000.0);
        heading += angular_v * (dt_ms / 1000.0);
        let now_la = encoders.left_a.is_high();
        let now_lb = encoders.left_b.is_low();
        let now_ra = encoders.right_a.is_high();
        let now_rb = encoders.right_b.is_low();
        if now_la != prev_la {
            edges_l += 1;
            prev_la = now_la;
        }
        if now_lb != prev_lb {
            edges_l += 1;
            prev_lb = now_lb;
        }
        if now_ra != prev_ra {
            edges_r += 1;
            prev_ra = now_ra;
        }
        if now_rb != prev_rb {
            edges_r += 1;
            prev_rb = now_rb;
        }
        info!(
            "TURN_CCW: heading={} eL={} eR={}",
            heading, edges_l, edges_r
        );
        if (heading >= 0.90 - gz_bias / 1000000.0) || (edges_l as f32 + edges_r as f32) / 2.0 >= 7.0
        {
            drive.execute(VehicleMotion::Stop, 0, 0);
            delay.delay_millis(150);
            info!(
                "TURN_COMPLETE: heading={} eL={} eR={}",
                heading, edges_l, edges_r
            );

            while heading < 0.90 && heading > 0.80 {
                let (_, _, _, gz_c) = mpu.read_corrected(bias);
                heading += gz_c.abs() * (dt_ms / 1000.0);
                for _ in 0..6 {
                    drive.execute(VehicleMotion::SpinCCW, 30, 0);
                    delay.delay_millis(100);
                    info!("UNDERSHOOT_CORRECT: heading={}", heading);
                }
            }
            while heading > 0.92 {
                let (_, _, _, gz_c) = mpu.read_corrected(bias);
                heading -= gz_c.abs() * (dt_ms / 1000.0);
                drive.execute(VehicleMotion::SpinCW, 30, 0);
                delay.delay_millis(10);
                drive.execute(VehicleMotion::Stop, 0, 0);
                delay.delay_millis(10);
                info!("OVERSHOOT_CORRECT: heading={}", heading);
            }
            break;
        }
        delay.delay_millis(50);
    }
}

fn execute_square(
    encoders: &Encoders,
    mpu: &mut Mpu6050,
    bias: &ImuBias,
    drive: &mut DifferentialDrive,
    delay: &mut Delay,
) {
    for leg in 0..4u8 {
        info!("SQUARE KALMAN: LEG {}", leg);
        forward_one_kalman(encoders, mpu, bias, drive, delay);

        delay.delay_millis(500);
        if leg < 4 {
            info!("SQUARE: TURN {}", leg);
            turn_90_ccw(encoders, mpu, bias, drive, delay);
            delay.delay_millis(500);
        }
    }
    info!("SQUARE KALAMN: COMPLETE — MEASURE RETURN ERROR NOW");
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let outconfig = OutputConfig::default();
    let inconfig = InputConfig::default();
    let l_dir = Output::new(peripherals.GPIO2, Level::Low, outconfig);
    let l_pwm = peripherals.GPIO3;
    let r_dir = Output::new(peripherals.GPIO7, Level::Low, outconfig);
    let r_pwm = peripherals.GPIO6;
    let left_a = Input::new(peripherals.GPIO9, inconfig);
    let left_b = Input::new(peripherals.GPIO4, inconfig);
    let right_a = Input::new(peripherals.GPIO21, inconfig);
    let right_b = Input::new(peripherals.GPIO5, inconfig);
    let encoders = Encoders::new(left_a, left_b, right_a, right_b);

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0.configure(timer::config::Config {
        duty: timer::config::Duty::Duty10Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(20),
    });

    let mut channel0 = ledc.channel(channel::Number::Channel0, r_pwm);
    channel0.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    });

    let mut channel1 = ledc.channel(channel::Number::Channel1, l_pwm);
    channel1.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    });

    let motor_right = drift::differential_drive::MotorController::new(r_dir, channel0);
    let motor_left = drift::differential_drive::MotorController::new(l_dir, channel1);
    let mut drive = DifferentialDrive::new(motor_left, motor_right);
    let mut delay = Delay::new();

    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .expect("FAULT: I2C_INIT_FAILED")
    .with_sda(peripherals.GPIO10)
    .with_scl(peripherals.GPIO8);
    let mut mpu = Mpu6050::new(i2c);

    delay.delay_millis(2000);

    mpu.init(&mut delay);
    if mpu.verify() {
        info!("MPU6050: HANDSHAKE_OK");
    } else {
        info!("FAULT: MPU6050_NOT_FOUND");
        loop {
            delay.delay_millis(1000);
        }
    }
    delay.delay_millis(500);
    let bias = ImuBias::calibrate(&mut mpu, &mut delay);
    delay.delay_millis(2000);

    execute_square(&encoders, &mut mpu, &bias, &mut drive, &mut delay);
    info!("SQUARE COMPLETE");
    loop {
        delay.delay_millis(1000);
    }
}
