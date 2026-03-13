#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is unsafe with esp_hal types holding buffers."
)]

use defmt::info;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c}, // NEW
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

#[derive(Clone, Copy)]
enum MotorDirection {
    Clockwise,
    CounterClockwise,
    Brake,
}

#[derive(Clone, Copy)]
enum VehicleMotion {
    Forward,
    Backward,
    Right,
    Left,
    SpinCW,
    SpinCCW,
    Stop,
}

struct MotorController<'a> {
    dir: Output<'a>,
    pwm: channel::Channel<'a, LowSpeed>,
}

impl<'a> MotorController<'a> {
    fn new(dir: Output<'a>, pwm: channel::Channel<'a, LowSpeed>) -> Self {
        Self { dir, pwm }
    }

    fn set_motion(&mut self, direction: MotorDirection, speed: u16, fade_ms: u16) {
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

struct DifferentialDrive<'a> {
    motor_left: MotorController<'a>,
    motor_right: MotorController<'a>,
}

impl<'a> DifferentialDrive<'a> {
    fn new(motor_left: MotorController<'a>, motor_right: MotorController<'a>) -> Self {
        Self {
            motor_left,
            motor_right,
        }
    }

    fn execute(&mut self, motion: VehicleMotion, speed: u16, fade_ms: u16) {
        let speed_reduced = (speed / 3).min(100);
        let speed_reduced1 = speed.min(100);
        match motion {
            VehicleMotion::Forward => {
                self.motor_left
                    .set_motion(MotorDirection::Clockwise, speed_reduced1, fade_ms);
                self.motor_right.set_motion(
                    MotorDirection::CounterClockwise,
                    speed_reduced1,
                    fade_ms,
                );
            }
            VehicleMotion::Backward => {
                self.motor_left
                    .set_motion(MotorDirection::CounterClockwise, speed, fade_ms);
                self.motor_right
                    .set_motion(MotorDirection::Clockwise, speed, fade_ms);
            }
            VehicleMotion::Right => {
                self.motor_left
                    .set_motion(MotorDirection::Brake, 0, fade_ms);
                self.motor_right
                    .set_motion(MotorDirection::CounterClockwise, speed, fade_ms);
            }
            VehicleMotion::Left => {
                self.motor_right
                    .set_motion(MotorDirection::Brake, 0, fade_ms);
                self.motor_left
                    .set_motion(MotorDirection::Clockwise, speed, fade_ms);
            }
            VehicleMotion::SpinCCW => {
                self.motor_left
                    .set_motion(MotorDirection::Clockwise, speed, fade_ms);
                self.motor_right
                    .set_motion(MotorDirection::Clockwise, speed, fade_ms);
            }
            VehicleMotion::SpinCW => {
                self.motor_left
                    .set_motion(MotorDirection::CounterClockwise, speed, fade_ms);
                self.motor_right
                    .set_motion(MotorDirection::CounterClockwise, speed, fade_ms);
            }
            VehicleMotion::Stop => {
                self.motor_left
                    .set_motion(MotorDirection::Brake, 0, fade_ms);
                self.motor_right
                    .set_motion(MotorDirection::Brake, 0, fade_ms);
            }
        }
    }
}

// TRACK
const MOTION: usize = 900;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Cell {
    distance: u8,
}

struct Encoders<'e> {
    zero_left: Input<'e>,
    zero_right: Input<'e>,
}

impl<'e> Encoders<'e> {
    fn new(zero_left: Input<'e>, zero_right: Input<'e>) -> Self {
        Self {
            zero_left,
            zero_right,
        }
    }
    fn forward_one(&self, drive: &mut DifferentialDrive) {
        let mut delay = Delay::new();
        let mut prev0 = self.zero_left.is_high();
        let mut prev1 = self.zero_right.is_high();
        let mut edge0 = 0;
        let mut edge1 = 0;
        drive.execute(VehicleMotion::Forward, 100, 300);
        for _ in 0..14 {
            delay.delay_millis(100);
            let now0 = self.zero_left.is_high();
            let now1 = self.zero_right.is_high();
            if (now0 != prev0) && (now1 != prev1) {
                edge0 += 1;
                prev0 = now0;
                edge1 += 1;
                prev1 = now1;

                if (edge0 == 8) | (edge1 == 8) {
                    drive.execute(VehicleMotion::Stop, 0, 0);
                    delay.delay_millis(150);
                }
            }
        }
    }
}

//const WALL_NORTH: u8 = 0b0001;
//const WALL_EAST: u8 = 0b0010;
//const WALL_SOUTH: u8 = 0b0100;
//const WALL_WEST: u8 = 0b1000;

struct HeadingState {
    angle_deg: f32,
    bias_dps: f32,
}

impl HeadingState {
    fn new(bias_dps: f32) -> Self {
        Self {
            angle_deg: 0.0,
            bias_dps,
        }
    }

    // dt_ms = milliseconds since last call
    fn update(&mut self, omega_dps: f32, dt_ms: f32) {
        let corrected = omega_dps - self.bias_dps;
        self.angle_deg += corrected * (dt_ms / 1000.0);
    }

    fn reset(&mut self) {
        self.angle_deg = 0.0;
    }

    fn angle(&self) -> f32 {
        self.angle_deg
    }
}
const MPU6050_ADDR: u8 = 0x68;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_GYRO_ZOUT_H: u8 = 0x47;
const REG_WHO_AM_I: u8 = 0x75;
const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_ACCEL_YOUT_H: u8 = 0x3D;
const REG_ACCEL_ZOUT_H: u8 = 0x3F;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const ACCEL_SENSITIVITY: f32 = 16384.0;
const GYRO_SENSITIVITY: f32 = 131.0;

use esp_hal::i2c::master::BusTimeout;
struct Mpu6050<'d> {
    i2c: I2c<'d, esp_hal::Blocking>,
}

impl<'d> Mpu6050<'d> {
    fn new(i2c: I2c<'d, esp_hal::Blocking>) -> Self {
        Self { i2c }
    }
    fn init(&mut self, delay: &mut Delay) {
        self.write_reg(REG_PWR_MGMT_1, 0x00); // wake
        delay.delay_millis(100);
        self.write_reg(REG_GYRO_CONFIG, 0x00); // ±250°/s range
        delay.delay_millis(10);
        self.write_reg(REG_ACCEL_CONFIG, 0x08); // ±4g range
        delay.delay_millis(10);
    }
    fn verify(&mut self) -> bool {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_WHO_AM_I], &mut buf)
            .ok();
        buf[0] == 0x68
    }

    fn read_gyro_z_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_GYRO_ZOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
    }

    // CONVERTED — degrees per second
    // ω_z [°/s] = raw / 131.0
    fn read_gyro_z_dps(&mut self) -> f32 {
        self.read_gyro_z_raw() as f32 / GYRO_SENSITIVITY
    }

    // CALIBRATION — robot must be STATIONARY
    // Returns bias in °/s units
    // Call once at startup
    fn calibrate(&mut self, delay: &mut Delay, samples: u16) -> f32 {
        let mut sum: f32 = 0.0;
        for _ in 0..samples {
            sum += self.read_gyro_z_dps();
            delay.delay_millis(5);
        }
        sum / samples as f32
    }
    fn write_reg(&mut self, reg: u8, val: u8) {
        self.i2c.write(MPU6050_ADDR, &[reg, val]).ok();
    }
    fn read_accel_x_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_ACCEL_XOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
    }

    fn read_accel_y_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_ACCEL_YOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
    }

    fn read_accel_z_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_ACCEL_ZOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
    }

    // a [m/s²] = (raw / 16384.0) × 9.81
    fn read_accel_ms2(&mut self) -> (f32, f32, f32) {
        let ax = (self.read_accel_x_raw() as f32 / ACCEL_SENSITIVITY) * 9.81;
        let ay = (self.read_accel_y_raw() as f32 / ACCEL_SENSITIVITY) * 9.81;
        let az = (self.read_accel_z_raw() as f32 / ACCEL_SENSITIVITY) * 9.81;
        (ax, ay, az)
    }

    fn read_corrected(&mut self, bias: &ImuBias) -> (f32, f32, f32, f32) {
        // Returns (ax_true, ay_true, gz_true)
        let (ax_raw, ay_raw, az_raw) = self.read_accel_ms2();
        let gz_raw = self.read_gyro_z_dps();

        let ax_true = ax_raw - bias.ax_bias;
        let ay_true = ay_raw - bias.ay_bias;
        let az_true = az_raw - bias.az_bias;
        let gz_true = gz_raw - bias.gz_bias;

        (ax_true, ay_true, az_true, gz_true)
    }
}

struct ImuBias {
    ax_bias: f32,
    ay_bias: f32,
    az_bias: f32,
    gz_bias: f32,
}

impl ImuBias {
    fn calibrate(mpu: &mut Mpu6050, delay: &mut Delay) -> Self {
        let samples: u16 = 200;
        let mut ax_sum: f32 = 0.0;
        let mut ay_sum: f32 = 0.0;
        let mut az_sum: f32 = 0.0;
        let mut gz_sum: f32 = 0.0;

        for _ in 0..samples {
            let (ax, ay, az) = mpu.read_accel_ms2();
            let gz = mpu.read_gyro_z_dps();
            ax_sum += ax;
            ay_sum += ay;
            az_sum += az;
            gz_sum += gz;
            delay.delay_millis(2);
        }

        let ax_bias = ax_sum / samples as f32;
        let ay_bias = ay_sum / samples as f32;
        let az_bias = az_sum / samples as f32;
        let gz_bias = gz_sum / samples as f32;

        info!(
            "BIAS_CALIBRATED: AX={} AY={} AZ= {} GZ={}",
            ax_bias, ay_bias, az_bias, gz_bias
        );

        Self {
            ax_bias,
            ay_bias,
            az_bias,
            gz_bias,
        }
    }
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let outconfig = OutputConfig::default();
    let inconfig = InputConfig::default();

    // H-BRIDGE — unchanged
    let r_dir = Output::new(peripherals.GPIO7, Level::Low, outconfig);
    let r_pwm = peripherals.GPIO6;
    let l_dir = Output::new(peripherals.GPIO2, Level::Low, outconfig);
    let l_pwm = peripherals.GPIO3;

    let encoder1 = Input::new(peripherals.GPIO21, inconfig);
    let encoder0 = Input::new(peripherals.GPIO9, inconfig);
    let encoders = Encoders::new(encoder1, encoder0);

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

    let motor_right = MotorController::new(r_dir, channel0);
    let motor_left = MotorController::new(l_dir, channel1);
    let mut drive = DifferentialDrive::new(motor_left, motor_right);
    let mut delay = Delay::new();

    // ATTEMPT 1 — try this first
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .expect("FAULT: I2C_INIT_FAILED")
    .with_sda(peripherals.GPIO10)
    .with_scl(peripherals.GPIO8);
    let mut mpu = Mpu6050::new(i2c);

    delay.delay_millis(2000); // startup settling

    mpu.init(&mut delay);
    if mpu.verify() {
        // ← verify first
        info!("MPU6050: HANDSHAKE_OK");
    } else {
        info!("FAULT: MPU6050_NOT_FOUND");
        loop {
            delay.delay_millis(1000);
        }
    }
    delay.delay_millis(500);
    let bias = ImuBias::calibrate(&mut mpu, &mut delay);

    delay.delay_millis(2000); // pre-motion settling

    info!("MOTION: FORWARD");
    drive.execute(VehicleMotion::Forward, 100, 200);
    for _ in 0..50 {
        let (ax, ay, az, gz) = mpu.read_corrected(&bias);
        info!("GYRO_Z: {} | AX: {} | AY: {} | AZ: {}", gz, ax, ay, az);
        delay.delay_millis(100);
    }
    drive.execute(VehicleMotion::Stop, 0, 0);
    delay.delay_millis(500);

    info!("MOTION: SPIN_CW");
    drive.execute(VehicleMotion::SpinCW, 60, 0);
    for _ in 0..20 {
        let omega_z = mpu.read_gyro_z_dps();
        let (ax, ay, az, gz) = mpu.read_corrected(&bias);
        info!("GYRO_Z: {} | AX: {} | AY: {} | AZ: {}", gz, ax, ay, az);
        delay.delay_millis(100);
    }
    drive.execute(VehicleMotion::Stop, 0, 0);
    delay.delay_millis(500);

    info!("MOTION: BACKWARD");
    drive.execute(VehicleMotion::Backward, 70, 200);
    for _ in 0..50 {
        let omega_z = mpu.read_gyro_z_dps();
        let (ax, ay, az, gz) = mpu.read_corrected(&bias);
        info!("GYRO_Z: {} | AX: {} | AY: {} | AZ: {}", gz, ax, ay, az);
        delay.delay_millis(100);
    }
    drive.execute(VehicleMotion::Stop, 0, 0);
    delay.delay_millis(500);

    info!("MOTION: SPIN_CCW");
    drive.execute(VehicleMotion::SpinCCW, 60, 0);
    for _ in 0..20 {
        let omega_z = mpu.read_gyro_z_dps();
        let (ax, ay, az, gz) = mpu.read_corrected(&bias);
        info!("GYRO_Z: {} | AX: {} | AY: {} | AZ: {}", gz, ax, ay, az);
        delay.delay_millis(100);
    }
    drive.execute(VehicleMotion::Stop, 0, 0);
    delay.delay_millis(500);

    info!("SEQUENCE_COMPLETE: ENTERING_IDLE");
    loop {
        let omega_z = mpu.read_gyro_z_dps();
        let (ax, ay, az, gz) = mpu.read_corrected(&bias);
        info!("GYRO_Z: {} | AX: {} | AY: {} | AZ: {}", gz, ax, ay, az);
        delay.delay_millis(200);
    }
}
