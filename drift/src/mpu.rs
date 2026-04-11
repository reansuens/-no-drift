use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
};

pub const MPU6050_ADDR: u8 = 0x68;
pub const REG_PWR_MGMT_1: u8 = 0x6B;
pub const REG_GYRO_CONFIG: u8 = 0x1B;
pub const REG_GYRO_ZOUT_H: u8 = 0x47;
pub const REG_WHO_AM_I: u8 = 0x75;
pub const REG_ACCEL_XOUT_H: u8 = 0x3B;
pub const REG_ACCEL_YOUT_H: u8 = 0x3D;
pub const REG_ACCEL_ZOUT_H: u8 = 0x3F;
pub const REG_ACCEL_CONFIG: u8 = 0x1C;
pub const REG_GYRO_XOUT_H: u8 = 0x43;
pub const REG_GYRO_YOUT_H: u8 = 0x45;
pub const ACCEL_SENSITIVITY: f32 = 16384.0;
pub const GYRO_SENSITIVITY: f32 = 131.0;

pub struct Mpu6050<'d> {
    i2c: I2c<'d, esp_hal::Blocking>,
}

impl<'d> Mpu6050<'d> {
    pub fn new(i2c: I2c<'d, esp_hal::Blocking>) -> Self {
        Self { i2c }
    }
    pub fn init(&mut self, delay: &mut Delay) {
        self.write_reg(REG_PWR_MGMT_1, 0x00); // wake
        delay.delay_millis(100);
        self.write_reg(REG_GYRO_CONFIG, 0x00); // ±250°/s range
        delay.delay_millis(10);
        self.write_reg(REG_ACCEL_CONFIG, 0x08); // ±4g range
        delay.delay_millis(10);
    }
    pub fn verify(&mut self) -> bool {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_WHO_AM_I], &mut buf)
            .ok();
        buf[0] == 0x68
    }

    pub fn read_gyro_x_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_GYRO_XOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
    }

    pub fn read_gyro_y_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_GYRO_YOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
    }

    pub fn read_gyro_z_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_GYRO_ZOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
    }

    // CONVERTED — degrees per second
    // ω_z [°/s] = raw / 131.0
    pub fn read_gyro_z_dps(&mut self) -> f32 {
        self.read_gyro_z_raw() as f32 / GYRO_SENSITIVITY
    }

    // CALIBRATION — robot must be STATIONARY
    // Returns bias in °/s units
    // Call once at startup
    pub fn calibrate(&mut self, delay: &mut Delay, samples: u16) -> f32 {
        let mut sum: f32 = 0.0;
        for _ in 0..samples {
            sum += self.read_gyro_z_dps();
            delay.delay_millis(5);
        }
        sum / samples as f32
    }
    pub fn write_reg(&mut self, reg: u8, val: u8) {
        self.i2c.write(MPU6050_ADDR, &[reg, val]).ok();
    }
    pub fn read_accel_x_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_ACCEL_XOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
    }

    pub fn read_accel_y_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_ACCEL_YOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
    }

    pub fn read_accel_z_raw(&mut self) -> i16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MPU6050_ADDR, &[REG_ACCEL_ZOUT_H], &mut buf)
            .ok();
        (buf[0] as i16) << 8 | (buf[1] as i16)
    }

    // a [m/s^2] = (raw / 16384.0) × 9.81
    pub fn read_accel_ms2(&mut self) -> (f32, f32, f32) {
        let ax = (self.read_accel_x_raw() as f32 / ACCEL_SENSITIVITY) * 9.81;
        let ay = (self.read_accel_y_raw() as f32 / ACCEL_SENSITIVITY) * 9.81;
        let az = (self.read_accel_z_raw() as f32 / ACCEL_SENSITIVITY) * 9.81;
        (ax, ay, az)
    }

    pub fn read_corrected(&mut self, bias: &ImuBias) -> (f32, f32, f32, f32) {
        // Returns (ax_true, ay_true, az_true, gz_true)
        let (ax_raw, ay_raw, az_raw) = self.read_accel_ms2();
        let gz_raw = self.read_gyro_z_dps();

        let ax_true = ax_raw - bias.ax_bias;
        let ay_true = ay_raw - bias.ay_bias;
        let az_true = az_raw - bias.az_bias;
        let gz_true = gz_raw - bias.gz_bias;

        (ax_true, ay_true, az_true, gz_true)
    }
    pub fn read_all_corrected(&mut self, bias: &ImuBias) -> (f32, f32, f32, f32, f32, f32) {
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

pub struct ImuBias {
    pub ax_bias: f32,
    pub ay_bias: f32,
    pub az_bias: f32,
    pub gx_bias: f32,
    pub gy_bias: f32,
    pub gz_bias: f32,
}

impl ImuBias {
    pub fn calibrate(mpu: &mut Mpu6050, delay: &mut Delay) -> Self {
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
        defmt::info!(
            "BIAS: AX={} AY={} AZ={} GX={} GY={} GZ={}",
            bias.ax_bias,
            bias.ay_bias,
            bias.az_bias,
            bias.gx_bias,
            bias.gy_bias,
            bias.gz_bias
        );
        bias
    }
}
