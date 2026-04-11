use crate::constants::{BASE_SPEED, GZ_GATE, GZ_LP, TARGET_EDGES_800, TRIM};
use crate::differential_drive::{DifferentialDrive, VehicleMotion};
use crate::encoders::Encoders;
use crate::kalman::KalmanHeading;
use crate::mpu::{ImuBias, Mpu6050};
use crate::pid::Pid;
use defmt::info;

pub fn forward_one_kalman(
    encoders: &Encoders,
    mpu: &mut Mpu6050,
    bias: &ImuBias,
    drive: &mut DifferentialDrive,
    delay: &mut esp_hal::delay::Delay,
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

    drive.set_speeds((BASE_SPEED - TRIM) as u16, BASE_SPEED as u16);
    info!("STATE: FORWARD_KALMAN_STARTING");

    loop {
        // ENCODER POLL  800 × 1ms
        for _ in 0..1200 {
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

            if (edges_l + edges_r) / 2 >= (1.2 * TARGET_EDGES_800) as u32 * (420 / 6) {
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

        kalman.predict(gz_filt, 0.4);

        let edge_diff = edges_r as i32 - edges_l as i32;
        let enc_heading = (edge_diff as f32 / 46.0) * (180.0 / core::f32::consts::PI);

        let heading_est = kalman.update(enc_heading);

        let u = pid.claculate(0.0, heading_est, 0.4).clamp(-20.0, 20.0);

        let pwm_l = (BASE_SPEED - u - TRIM).clamp(BASE_SPEED, 255.0) as u16;
        let pwm_r = (BASE_SPEED + u).clamp(BASE_SPEED, 255.0) as u16;
        drive.set_speeds(pwm_l, pwm_r);

        info!(
            "gz_r={} gz_f={} kalman={} enc_h={} u={} eL={} eR={} pL={} pR={}",
            gz_raw, gz_filt, heading_est, enc_heading, u, edges_l, edges_r, pwm_l, pwm_r
        );
    }
}
