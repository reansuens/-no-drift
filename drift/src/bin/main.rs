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
use libm;
use libm::expf;

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

    pub fn set_speeds(&mut self, left_pwm: u16, right_pwm: u16) {
        let l_duty = left_pwm.min(100);
        let r_duty = right_pwm.min(100);

        self.motor_left
            .set_motion(MotorDirection::CounterClockwise, l_duty, 0);

        self.motor_right
            .set_motion(MotorDirection::Clockwise, r_duty, 0);
    }

    fn execute(&mut self, motion: VehicleMotion, speed: u16, fade_ms: u16) {
        let speed_reduced = (speed / 3).min(100);
        let speed_reduced1 = speed.min(100);
        match motion {
            VehicleMotion::Forward => {
                self.motor_left.set_motion(
                    MotorDirection::CounterClockwise,
                    speed_reduced1,
                    fade_ms,
                );
                self.motor_right
                    .set_motion(MotorDirection::Clockwise, speed_reduced1, fade_ms);
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

struct KalmanHeading {
    angle: f32, // estimated heading [deg]
    bias: f32,  // estimated gyro bias [deg/s]
    p00: f32,   // covariance [0][0]
    p01: f32,   // covariance [0][1]
    p10: f32,   // covariance [1][0]
    p11: f32,   // covariance [1][1]
}

impl KalmanHeading {
    fn new() -> Self {
        Self {
            angle: 0.0,
            bias: 0.0,
            p00: 1.0,
            p01: 0.0,
            p10: 0.0,
            p11: 1.0,
        }
    }

    fn predict(&mut self, gz: f32, dt: f32) {
        // tune
        const Q_ANGLE: f32 = 0.1;
        const Q_BIAS: f32 = 0.6;

        let rate = gz - self.bias;
        self.angle += (dt * rate) / 100.0;

        // Update covariance
        self.p00 += dt * (dt * self.p11 - self.p01 - self.p10 + Q_ANGLE);
        self.p01 -= dt * self.p11;
        self.p10 -= dt * self.p11;
        self.p11 += Q_BIAS * dt;
    }

    fn update(&mut self, measurement: f32) -> f32 {
        const R_MEASURE: f32 = 3.0;

        let s = self.p00 + R_MEASURE;
        let k0 = self.p00 / s;
        let k1 = self.p10 / s;

        let y = measurement - self.angle;

        self.angle += k0 * y;
        self.bias += k1 * y;

        let p00_tmp = self.p00;
        let p01_tmp = self.p01;

        self.p00 -= k0 * p00_tmp;
        self.p01 -= k0 * p01_tmp;
        self.p10 -= k1 * p00_tmp;
        self.p11 -= k1 * p01_tmp;

        self.angle
    }

    fn angle(&self) -> f32 {
        self.angle
    }
}

struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    prev_err: f32,
    integral_limit: f32,
}

impl Pid {
    fn claculate(&mut self, target: f32, current: f32, dt: f32) -> f32 {
        let mut error = target - current;
        while error > 180.0 {
            error -= 360.0;
        }
        while error < -180.0 {
            error += 360.0;
        }

        let p_out = self.kp * error;
        self.integral += error * dt;
        self.integral = self
            .integral
            .clamp(-self.integral_limit, self.integral_limit);
        let i_out = self.ki * self.integral;
        let d_out = self.kd * (error - self.prev_err) / dt;
        self.prev_err = error;

        p_out + i_out + d_out
    }
}

fn forward_one(
    encoders: &Encoders,
    mpu: &mut Mpu6050,
    bias: &ImuBias,
    drive: &mut DifferentialDrive,
    delay: &mut Delay,
) {
    delay.delay_millis(100);
    let _ = mpu.read_corrected(bias);
    delay.delay_millis(50);

    let mut prev_la = encoders.left_a.is_high();
    let mut prev_lb = encoders.left_b.is_high();
    let mut prev_ra = encoders.right_a.is_high();
    let mut prev_rb = encoders.right_b.is_high();

    let mut edges_l: u32 = 0;
    let mut edges_r: u32 = 0;
    let mut heading: f32 = 0.0;
    let mut gz_filt: f32 = 0.0; // low pass on gz to kill spikes

    let mut pid = Pid {
        kp: 5.8,
        ki: 0.4,
        kd: 3.86,
        integral: 0.0,
        prev_err: 0.0,
        integral_limit: 1000.0,
    };

    const BASE_SPEED: f32 = 120.0;
    const TAU: f32 = 0.9;
    // ax removed from filter entirely 
    // gyro only complementary filter
    const ALPHA: f32 = 0.96;
    const GZ_LP: f32 = 0.6; // low pass on gz
    const GZ_GATE: f32 = 25.0; // deg/s 

    drive.set_speeds((BASE_SPEED - TRIM) as u16, BASE_SPEED as u16);

    loop {
        for _ in 0..800 {
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

            if (edges_l + edges_r) / 2 >= TARGET_EDGES_800 as u32 * 590 {
                drive.execute(VehicleMotion::Stop, 0, 0);
                info!("LEG_COMPLETE: h={} eL={} eR={}", heading, edges_l, edges_r);
                return;
            }

            delay.delay_millis(1);
        }

        let (_, _, _, gz_raw) = mpu.read_corrected(bias);

        // Gate: suppress vibration spikes on gz
        let gz_valid = if gz_raw.abs() < GZ_GATE {
            gz_raw
        } else {
            info!("GZ_SPIKE: gz={}", gz_raw);
            gz_filt // hold last clean value
        };

        gz_filt = GZ_LP * gz_valid + (1.0 - GZ_LP) * gz_filt;

        heading = ALPHA * (heading + gz_filt * 0.8) + (1.0 - ALPHA) * 0.0; // encoder diff could go here later

        let u = pid.claculate(0.0, heading, 0.8);

        let pwm_l = (BASE_SPEED - 0.9 * u - (4.5 * TRIM)).clamp(BASE_SPEED, 255.0) as u16;
        let pwm_r = (BASE_SPEED + 0.9 * u).clamp(BASE_SPEED, 255.0) as u16;
        drive.set_speeds(pwm_l, pwm_r);

        info!(
            "gz_r={} gz_f={} h={} u={} eL={} eR={} pL={} pR={}",
            gz_raw, gz_filt, heading, u, edges_l, edges_r, pwm_l, pwm_r
        );
    }
}
fn forward_one_kalman(
    encoders: &Encoders,
    mpu: &mut Mpu6050,
    bias: &ImuBias,
    drive: &mut DifferentialDrive,
    delay: &mut Delay,
) {
    delay.delay_millis(100);
    let _ = mpu.read_corrected(bias);
    delay.delay_millis(50);

    let mut prev_la = encoders.left_a.is_high();
    let mut prev_lb = encoders.left_b.is_high();
    let mut prev_ra = encoders.right_a.is_high();
    let mut prev_rb = encoders.right_b.is_high();

    let mut edges_l: u32 = 0;
    let mut edges_r: u32 = 0;
    let mut gz_filt: f32 = 0.0;

    let mut kalman = KalmanHeading::new();

    let mut pid = Pid {
        kp: 4.8,
        ki: 0.3,
        kd: 2.86,
        integral: 0.0,
        prev_err: 0.0,
        integral_limit: 1000.0,
    };

    const BASE_SPEED: f32 = 120.0;
    const GZ_LP: f32 = 0.6;
    const GZ_GATE: f32 = 25.0;

    drive.set_speeds((BASE_SPEED - TRIM) as u16, BASE_SPEED as u16);
    info!("STATE: FORWARD_KALMAN_STARTING");

    loop {
        // ── FAST ENCODER POLL — 800 × 1ms ────────────────────────
        for _ in 0..800 {
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

            if (edges_l + edges_r) / 2 >= TARGET_EDGES_800 as u32 * 610 {
                drive.execute(VehicleMotion::Stop, 0, 0);
                info!(
                    "LEG_COMPLETE: eL={} eR={} diff={}",
                    edges_l,
                    edges_r,
                    edges_r as i32 - edges_l as i32
                );
                return;
            }

            delay.delay_millis(1);
        }

        let (_, _, _, gz_raw) = mpu.read_corrected(bias);

        let gz_valid = if gz_raw.abs() < GZ_GATE {
            gz_raw
        } else {
            info!("GZ_SPIKE: gz={}", gz_raw);
            gz_filt
        };

        gz_filt = GZ_LP * gz_valid + (1.0 - GZ_LP) * gz_filt;

        // ── KALMAN PREDICT ────────────────────────────────────────
        kalman.predict(gz_filt, 0.4);

        // ── ENCODER DIFFERENTIAL MEASUREMENT ─────────────────────
        let edge_diff = edges_r as i32 - edges_l as i32;
        let enc_heading = (edge_diff as f32 / 46.0) * (180.0 / core::f32::consts::PI);

        // ── KALMAN UPDATE ─────────────────────────────────────────
        let heading_est = kalman.update(enc_heading);

        // ── PID ───────────────────────────────────────────────────
        let u = pid.claculate(0.0, heading_est, 0.4);

        let pwm_l = (BASE_SPEED - 0.9 * u - TRIM).clamp(BASE_SPEED, 255.0) as u16;
        let pwm_r = (BASE_SPEED + 0.9 * u).clamp(BASE_SPEED, 255.0) as u16;
        drive.set_speeds(pwm_l, pwm_r);

        info!(
            "gz_r={} gz_f={} kalman={} enc_h={} u={} eL={} eR={} pL={} pR={}",
            gz_raw, gz_filt, heading_est, enc_heading, u, edges_l, edges_r, pwm_l, pwm_r
        );
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
        if (heading >= 0.92 - gz_bias / 1000000.0)
            || (edges_l as f32 + edges_r as f32) / 2.0 >= 14.0
        {
            drive.execute(VehicleMotion::Stop, 0, 0);
            delay.delay_millis(150);
            info!(
                "TURN_COMPLETE: heading={} eL={} eR={}",
                heading, edges_l, edges_r
            );
            while heading > 0.7 && heading < 0.9 {
                let (_, _, _, gz_c) = mpu.read_corrected(bias);
                heading -= gz_c.abs() * (dt_ms / 1000.0);
                drive.execute(VehicleMotion::SpinCCW, 40, 0);
                delay.delay_millis(10);
                drive.execute(VehicleMotion::Stop, 0, 0);
                delay.delay_millis(10);
                info!("SLOW AND SLOW {}", heading);
            }

            while heading > 0.93 {
                let (_, _, _, gz_c) = mpu.read_corrected(bias);
                heading -= gz_c.abs() * (dt_ms / 1000.0);
                drive.execute(VehicleMotion::SpinCW, 40, 0);
                delay.delay_millis(10);
                drive.execute(VehicleMotion::Stop, 0, 0);
                delay.delay_millis(10);
                info!("OVERSHOOT_CORRECT: heading={}", heading);
            }

            //while heading < 0.90 {
            //    let (_, _, _, gz_c) = mpu.read_corrected(bias);
            //    heading += gz_c.abs() * (dt_ms / 1000.0);
            //    drive.execute(VehicleMotion::SpinCCW, 40, 0);
            //    delay.delay_millis(50);
            //    drive.execute(VehicleMotion::Stop, 0, 0);
            //    delay.delay_millis(50);
            //    info!("UNDERSHOOT_CORRECT: heading={}", heading);
            //}
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
        info!("SQUARE MADGWICK: LEG {}", leg);
        //forward_one_kalman(encoders, mpu, bias, drive, delay);

        forward_one_madgwick(encoders, mpu, bias, drive, delay);
        //forward_one(encoders, mpu, bias, drive, delay);
        delay.delay_millis(500);
        if leg < 4 {
            info!("SQUARE: TURN {}", leg);
            turn_90_ccw(encoders, mpu, bias, drive, delay);
            delay.delay_millis(500);
        }
    }
    info!("SQUARE MADGWICK: COMPLETE — MEASURE RETURN ERROR NOW");

    delay.delay_millis(10000);
    for leg in 0..4u8 {
        info!("SQUARE KALMAN: LEG {}", leg);
        forward_one_kalman(encoders, mpu, bias, drive, delay);

        //forward_one_madgwick(encoders, mpu, bias, drive, delay);
        //forward_one(encoders, mpu, bias, drive, delay);
        delay.delay_millis(500);
        if leg < 4 {
            info!("SQUARE: TURN {}", leg);
            turn_90_ccw(encoders, mpu, bias, drive, delay);
            delay.delay_millis(500);
        }
    }
    info!("SQUARE KALAMN: COMPLETE — MEASURE RETURN ERROR NOW");

    delay.delay_millis(10000);
    for leg in 0..4u8 {
        info!("SQUARE COMPLEMENTARY: LEG {}", leg);
        //forward_one_kalman(encoders, mpu, bias, drive, delay);

        //forward_one_madgwick(encoders, mpu, bias, drive, delay);
        forward_one(encoders, mpu, bias, drive, delay);
        delay.delay_millis(500);
        if leg < 4 {
            info!("SQUARE: TURN {}", leg);
            turn_90_ccw(encoders, mpu, bias, drive, delay);
            delay.delay_millis(500);
        }
    }
    info!("SQUARE COMPLEMENTARY: COMPLETE — MEASURE RETURN ERROR NOW");

    delay.delay_millis(10000);
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

        if (edges_l + edges_r) / 2 >= TARGET_EDGES_800 as u32 {
            drive.execute(VehicleMotion::Stop, 0, 0);
            delay.delay_millis(200);
            info!("OPEN_LEG_COMPLETE: L={} R={}", edges_l, edges_r);
            break;
        }
        delay.delay_millis(500);
    }
}

fn execute_square_open_loop(encoders: &Encoders, drive: &mut DifferentialDrive, delay: &mut Delay) {
    const TURN_MS: u32 = 1130;

    for leg in 0..4u8 {
        info!("OPEN_LOOP: LEG {}", leg);
        drive.execute(VehicleMotion::Forward, 200, 0);
        drive.set_speeds(200 - TRIM as u16, 200);
        delay.delay_millis(4500);

        if leg < 4 {
            info!("OPEN_LOOP: TURN {}", leg);
            drive.execute(VehicleMotion::SpinCCW, 70, 0);
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
const REG_GYRO_XOUT_H: u8 = 0x43;
const REG_GYRO_YOUT_H: u8 = 0x45;
const ACCEL_SENSITIVITY: f32 = 16384.0;
const GYRO_SENSITIVITY: f32 = 131.0;
const MM_PER_EDGE: f32 = 1.835;
const TARGET_EDGES_800: f32 = 4.0 * 2.4 + 1.0;
const YAW_PLAUSIBLE_MAX: f32 = 4.0; // degrees
const TRIM: f32 = 1.0;
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

    fn read_gyro_x_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_GYRO_XOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
    }

    fn read_gyro_y_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_GYRO_YOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
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
    fn read_all_corrected(&mut self, bias: &ImuBias) -> (f32, f32, f32, f32, f32, f32) {
        let (ax_raw, ay_raw, az_raw) = self.read_accel_ms2();
        let gx_raw = self.read_gyro_x_raw() as f32 / GYRO_SENSITIVITY;
        let gy_raw = self.read_gyro_y_raw() as f32 / GYRO_SENSITIVITY;
        let gz_raw = self.read_gyro_z_dps();

        (
            ax_raw - bias.ax_bias,
            ay_raw - bias.ay_bias,
            az_raw - bias.az_bias,
            gx_raw - bias.gx_bias,
            gy_raw - bias.gy_bias,
            gz_raw - bias.gz_bias,
        )
    }
}

struct ImuBias {
    ax_bias: f32,
    ay_bias: f32,
    az_bias: f32,
    gx_bias: f32,
    gy_bias: f32,
    gz_bias: f32,
}

impl ImuBias {
    fn calibrate(mpu: &mut Mpu6050, delay: &mut Delay) -> Self {
        let samples: u16 = 200;
        let mut ax_s = 0f32;
        let mut ay_s = 0f32;
        let mut az_s = 0f32;
        let mut gx_s = 0f32;
        let mut gy_s = 0f32;
        let mut gz_s = 0f32;

        for _ in 0..samples {
            let (ax, ay, az) = mpu.read_accel_ms2();
            let gx = mpu.read_gyro_x_raw() as f32 / GYRO_SENSITIVITY;
            let gy = mpu.read_gyro_y_raw() as f32 / GYRO_SENSITIVITY;
            let gz = mpu.read_gyro_z_dps();
            ax_s += ax;
            ay_s += ay;
            az_s += az;
            gx_s += gx;
            gy_s += gy;
            gz_s += gz;
            delay.delay_millis(2);
        }

        let n = samples as f32;
        let bias = Self {
            ax_bias: ax_s / n,
            ay_bias: ay_s / n,
            az_bias: az_s / n,
            gx_bias: gx_s / n,
            gy_bias: gy_s / n,
            gz_bias: gz_s / n,
        };
        info!(
            "BIAS: AX={} AY={} AZ={} GX={} GY={} GZ={}",
            bias.ax_bias, bias.ay_bias, bias.az_bias, bias.gx_bias, bias.gy_bias, bias.gz_bias
        );
        bias
    }
}

struct MadgwickFilter {
    q0: f32,
    q1: f32,
    q2: f32,
    q3: f32,
    beta: f32,
}

impl MadgwickFilter {
    fn new(beta: f32) -> Self {
        Self {
            q0: 1.0,
            q1: 0.0,
            q2: 0.0,
            q3: 0.0,
            beta,
        }
    }

    fn update(&mut self, ax: f32, ay: f32, az: f32, gx: f32, gy: f32, gz: f32, dt: f32) {
        // Convert gyro to rad/s
        const D2R: f32 = core::f32::consts::PI / 180.0;
        let gx = gx * D2R;
        let gy = gy * D2R;
        let gz = gz * D2R;

        let q0 = self.q0;
        let q1 = self.q1;
        let q2 = self.q2;
        let q3 = self.q3;

        let q_dot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        let q_dot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        let q_dot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        let q_dot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

        let a_norm = libm::sqrtf(ax * ax + ay * ay + az * az);
        if a_norm < 0.01 {
            // Accel unusable — gyro only update
            self.q0 += q_dot0 * dt;
            self.q1 += q_dot1 * dt;
            self.q2 += q_dot2 * dt;
            self.q3 += q_dot3 * dt;
            self.normalise();
            return;
        }
        let ax = ax / a_norm;
        let ay = ay / a_norm;
        let az = az / a_norm;

        // Gradient descent — objective: f(q) = q* ⊗ g ⊗ q - a_meas
        // Analytical Jacobian J^T * f
        let f0 = 2.0 * (q1 * q3 - q0 * q2) - ax;
        let f1 = 2.0 * (q0 * q1 + q2 * q3) - ay;
        let f2 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - az;

        let j00 = -2.0 * q2;
        let j01 = 2.0 * q3;
        let j02 = -2.0 * q0;
        let j03 = 2.0 * q1;
        let j10 = 2.0 * q1;
        let j11 = 2.0 * q0;
        let j12 = 2.0 * q3;
        let j13 = 2.0 * q2;
        let j20 = 0.0;
        let j21 = -4.0 * q1;
        let j22 = -4.0 * q2;
        let j23 = 0.0;

        let mut s0 = j00 * f0 + j10 * f1 + j20 * f2;
        let mut s1 = j01 * f0 + j11 * f1 + j21 * f2;
        let mut s2 = j02 * f0 + j12 * f1 + j22 * f2;
        let mut s3 = j03 * f0 + j13 * f1 + j23 * f2;

        // Normalise gradient
        let s_norm = libm::sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if s_norm > 0.001 {
            s0 /= s_norm;
            s1 /= s_norm;
            s2 /= s_norm;
            s3 /= s_norm;
        }

        // Apply gradient correction
        self.q0 += (q_dot0 - self.beta * s0) * dt;
        self.q1 += (q_dot1 - self.beta * s1) * dt;
        self.q2 += (q_dot2 - self.beta * s2) * dt;
        self.q3 += (q_dot3 - self.beta * s3) * dt;

        self.normalise();
    }

    fn normalise(&mut self) {
        let n = libm::sqrtf(
            self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3,
        );
        if n > 0.0 {
            self.q0 /= n;
            self.q1 /= n;
            self.q2 /= n;
            self.q3 /= n;
        }
    }

    // Returns yaw angle in degrees
    fn yaw_deg(&self) -> f32 {
        let yaw_rad = libm::atan2f(
            2.0 * (self.q0 * self.q3 + self.q1 * self.q2),
            1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3),
        );
        yaw_rad * (180.0 / core::f32::consts::PI)
    }

    // Returns yaw in radians
    fn yaw_rad(&self) -> f32 {
        libm::atan2f(
            2.0 * (self.q0 * self.q3 + self.q1 * self.q2),
            1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3),
        )
    }
}

fn forward_one_madgwick(
    encoders: &Encoders,
    mpu: &mut Mpu6050,
    bias: &ImuBias,
    drive: &mut DifferentialDrive,
    delay: &mut Delay,
) {
    delay.delay_millis(100);
    let _ = mpu.read_all_corrected(bias);
    delay.delay_millis(50);

    let mut prev_la = encoders.left_a.is_high();
    let mut prev_lb = encoders.left_b.is_high();
    let mut prev_ra = encoders.right_a.is_high();
    let mut prev_rb = encoders.right_b.is_high();

    let mut edges_l: u32 = 0;
    let mut edges_r: u32 = 0;
    let mut yaw_filtered: f32 = 0.0;

    const YAW_LP: f32 = 0.3;
    const YAW_GATE: f32 = 3.0;
    const BASE_SPEED: f32 = 120.0;

    let mut madgwick = MadgwickFilter::new(0.1);

    let mut pid = Pid {
        kp: 4.8,
        ki: 0.3,
        kd: 2.86,
        integral: 0.0,
        prev_err: 0.0,
        integral_limit: 1000.0,
    };

    info!("MADGWICK: WARMUP");
    for _ in 0..50 {
        let (ax, ay, az, gx, gy, gz) = mpu.read_all_corrected(bias);
        madgwick.update(ax, ay, az, gx, gy, gz, 0.01);
        delay.delay_millis(10);
    }
    let yaw_origin = madgwick.yaw_deg();
    info!("MADGWICK: ORIGIN yaw={}", yaw_origin);

    drive.set_speeds(BASE_SPEED as u16, BASE_SPEED as u16);

    loop {
        for _ in 0..800 {
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

            if (edges_l + edges_r) / 2 >= TARGET_EDGES_800 as u32 * 550 {
                drive.execute(VehicleMotion::Stop, 0, 0);
                info!(
                    "LEG_COMPLETE: yaw_f={} eL={} eR={}",
                    yaw_filtered, edges_l, edges_r
                );
                return;
            }

            delay.delay_millis(1);
        }

        let (ax, ay, az, gx, gy, gz) = mpu.read_all_corrected(bias);
        madgwick.update(ax, ay, az, gx, gy, gz, 0.8);

        let yaw_raw = madgwick.yaw_deg() - yaw_origin;

        let yaw_valid = if yaw_raw.abs() < YAW_GATE {
            yaw_raw
        } else {
            info!("SPIKE_SUPPRESSED: yaw_raw={}", yaw_raw);
            yaw_filtered
        };

        yaw_filtered = YAW_LP * yaw_valid + (1.0 - YAW_LP) * yaw_filtered;

        let u = pid.claculate(0.0, yaw_filtered, 0.8);

        let pwm_l = (BASE_SPEED - 0.9 * u - TRIM) as u16;
        let pwm_r = (BASE_SPEED + 0.9 * u) as u16;
        drive.set_speeds(pwm_l, pwm_r);

        info!(
            "yaw_raw={} yaw_f={} u={} eL={} eR={} pL={} pR={}",
            yaw_raw, yaw_filtered, u, edges_l, edges_r, pwm_l, pwm_r
        );
    }
}
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let outconfig = OutputConfig::default();
    let inconfig = InputConfig::default();
    // H bridge
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
    //info!("TEST: SINGLE CCW TURN — PLACE ROBOT, STEP BACK");
    //delay.delay_millis(3000);
    //
    //turn_90_ccw(&encoders, &mut mpu, &bias, &mut drive, &mut delay);

    info!("TURN_TEST_COMPLETE — CHECK PHYSICAL ANGLE");
    //loop {
    //    delay.delay_millis(1000);
    //}
    //drive.execute(VehicleMotion::Forward, 100, 50);
    //delay.delay_millis(10000);
    // TEST: SINGLE FORWARD LEG
    info!("TEST: SINGLE FORWARD LEG — PLACE ROBOT");
    execute_square(&encoders, &mut mpu, &bias, &mut drive, &mut delay);

    execute_square_open_loop(&encoders, &mut drive, &mut delay);
    info!("LEG_TEST_COMPLETE — MEASURE DISTANCE");
    loop {
        delay.delay_millis(1000);
    }
}
