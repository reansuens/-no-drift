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
                self.pwm.set_duty(speed as u8);
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
    fn set_speeds(&mut self, left: u16, right: u16) {
        self.motor_left
            .set_motion(MotorDirection::Clockwise, left, 0);
        self.motor_right
            .set_motion(MotorDirection::CounterClockwise, right, 0);
    }
}

// TRACK
const MOTION: usize = 800;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Cell {
    distance: u8,
}

struct Encoders<'e> {
    left_a: Input<'e>,  // GPIO9  — left
    left_b: Input<'e>,  // GPIO20 — left second
    right_a: Input<'e>, // GPIO21 — right
    right_b: Input<'e>, // GPIO4  — right second
}

impl<'e> Encoders<'e> {
    fn new(left_a: Input<'e>, left_b: Input<'e>, right_a: Input<'e>, right_b: Input<'e>) -> Self {
        Self {
            left_a,
            left_b,
            right_a,
            right_b,
        }
    }
}

fn forward_one(
    encoders: &Encoders,
    mpu: &mut Mpu6050,
    bias: &ImuBias,
    drive: &mut DifferentialDrive,
    delay: &mut Delay,
) {
    let mut prev_la = encoders.left_a.is_high();
    let mut prev_lb = encoders.left_b.is_high();
    let mut prev_ra = encoders.right_a.is_high();
    let mut prev_rb = encoders.right_b.is_high();

    let mut edges_l: u32 = 0;
    let mut edges_r: u32 = 0;

    // HEADING INTEGRATOR — tracks drift from straight
    let mut heading: f32 = 0.0;
    let dt_ms: f32 = 10.0;

    // PID STATE
    let mut integral: f32 = 0.0;
    let mut prev_err: f32 = 0.0;

    // PID GAINS — tune these
    const KP: f32 = 2.0;
    const KI: f32 = 0.03;
    const KD: f32 = 0.6;
    const BASE_SPEED: u16 = 200;

    loop {
        // ENCODER EDGE DETECTION
        let now_la = encoders.left_a.is_high();
        let now_lb = encoders.left_b.is_high();
        let now_ra = encoders.right_a.is_high();
        let now_rb = encoders.right_b.is_high();

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

        // GYRO HEADING UPDATE
        let (_, _, _, gz) = mpu.read_corrected(bias);
        heading += gz * (dt_ms / 1000.0);

        // PID — error = how far we drifted from straight (0°)
        let err = heading; // target = 0°
        integral += err * (dt_ms / 1000.0);
        let deriv = (err - prev_err) / (dt_ms / 1000.0);
        let u = KP * err + KI * integral + KD * deriv;
        prev_err = err;

        // DIFFERENTIAL PWM CORRECTION
        // positive heading (CCW drift) → increase right, decrease left
        let pwm_l = (BASE_SPEED as f32 - u).clamp(30.0, 100.0) as u16;
        let pwm_r = (BASE_SPEED as f32 + u).clamp(30.0, 100.0) as u16;

        drive
            .motor_left
            .set_motion(MotorDirection::Clockwise, pwm_l, 100);
        drive
            .motor_right
            .set_motion(MotorDirection::CounterClockwise, pwm_r, 100);

        info!(
            "heading={} u={} pwm_l={} pwm_r={} el={} er={}",
            heading, u, pwm_l, pwm_r, edges_l, edges_r
        );

        // STOP CONDITION
        if (edges_l + edges_r) / 2 >= TARGET_EDGES_800 {
            drive.execute(VehicleMotion::Stop, 0, 0);
            delay.delay_millis(200);
            info!(
                "LEG_COMPLETE: heading_final={} L={} R={}",
                heading, edges_l, edges_r
            );
            break;
        }

        delay.delay_millis(10);
    }
}

fn turn_90_ccw(
    mpu: &mut Mpu6050,
    bias: &ImuBias,
    drive: &mut DifferentialDrive,
    delay: &mut Delay,
) {
    let mut heading: f32 = 0.0;
    let mut angular_v: f32 = 0.0;
    let dt_ms: f32 = 50.0 / 2.33;
    let gz_bias = bias.gz_bias.abs();
    drive.execute(VehicleMotion::SpinCCW, 95, 0);

    loop {
        let (_, _, _, gz) = mpu.read_corrected(bias);
        angular_v += gz * (dt_ms / 1000.0);
        heading += angular_v * (dt_ms / 1000.0);
        info!("TURN_CCW: heading={}", heading);

        // CCW → positive gz → heading goes positive
        if heading >= 90.0 - gz_bias {
            drive.execute(VehicleMotion::Stop, 0, 0);
            delay.delay_millis(150);
            info!("TURN_COMPLETE: heading={}", heading);
            break;
        }

        delay.delay_millis(10);
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
        info!("SQUARE: LEG {}", leg);
        forward_one(encoders, mpu, bias, drive, delay);
        delay.delay_millis(400);

        if leg < 3 {
            info!("SQUARE: TURN {}", leg);
            turn_90_ccw(mpu, bias, drive, delay);
            delay.delay_millis(400);
        }
    }
    info!("SQUARE: COMPLETE — MEASURE RETURN ERROR NOW");
}
fn forward_one_open(encoders: &Encoders, drive: &mut DifferentialDrive, delay: &mut Delay) {
    let mut prev_la = encoders.left_a.is_high();
    let mut prev_lb = encoders.left_b.is_high();
    let mut prev_ra = encoders.right_a.is_high();
    let mut prev_rb = encoders.right_b.is_high();
    let mut edges_l: u32 = 0;
    let mut edges_r: u32 = 0;

    drive.execute(VehicleMotion::Forward, 200, 100);

    loop {
        let now_la = encoders.left_a.is_high();
        let now_lb = encoders.left_b.is_high();
        let now_ra = encoders.right_a.is_high();
        let now_rb = encoders.right_b.is_high();

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

        if edges_l >= TARGET_EDGES_800 && edges_r >= TARGET_EDGES_800 {
            drive.execute(VehicleMotion::Stop, 0, 0);
            delay.delay_millis(200);
            info!("OPEN_LEG_COMPLETE: L={} R={}", edges_l, edges_r);
            break;
        }
        delay.delay_millis(10);
    }
}

fn execute_square_open_loop(encoders: &Encoders, drive: &mut DifferentialDrive, delay: &mut Delay) {
    // Uses timer-based turns — no gyro
    // This is your baseline drift measurement
    const TURN_MS: u32 = 400; // tune this to approximate 90°

    for leg in 0..4u8 {
        info!("OPEN_LOOP: LEG {}", leg);
        forward_one_open(encoders, drive, delay);
        delay.delay_millis(400);

        if leg < 3 {
            info!("OPEN_LOOP: TURN {}", leg);
            drive.execute(VehicleMotion::SpinCCW, 160, 0);
            delay.delay_millis(TURN_MS);
            drive.execute(VehicleMotion::Stop, 0, 0);
            delay.delay_millis(400);
        }
    }
    info!("OPEN_LOOP: COMPLETE — MEASURE RETURN ERROR NOW");
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
const MM_PER_EDGE: f32 = 1.835;
const TARGET_EDGES_800: u32 = 440;

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
    // ── TEST: SINGLE TURN ONLY ───────────────────────────────────
    // Verify turn_90_ccw works before running full square
    info!("TEST: SINGLE CCW TURN — PLACE ROBOT, STEP BACK");
    delay.delay_millis(3000);

    turn_90_ccw(&mut mpu, &bias, &mut drive, &mut delay);

    info!("TURN_TEST_COMPLETE — CHECK PHYSICAL ANGLE");
    loop {
        delay.delay_millis(1000);
    }

    //// ── TEST: SINGLE FORWARD LEG ─────────────────────────────────
    //info!("TEST: SINGLE FORWARD LEG — PLACE ROBOT");
    //delay.delay_millis(3000);
    //
    //forward_one(&encoders, &mut mpu, &bias, &mut drive, &mut delay);
    //
    //info!("LEG_TEST_COMPLETE — MEASURE DISTANCE");
    //loop {
    //    delay.delay_millis(1000);
    //}
}
