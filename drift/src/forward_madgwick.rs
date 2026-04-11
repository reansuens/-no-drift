use crate::constants::{BASE_SPEED, TARGET_EDGES_800, TRIM};
use crate::differential_drive::{DifferentialDrive, VehicleMotion};
use crate::encoders::Encoders;
use crate::madgwick::MadgwickFilter;
use crate::mpu::{ImuBias, Mpu6050};
use crate::pid::Pid;
use defmt::info;

const YAW_LP: f32 = 0.3;
const YAW_GATE: f32 = 3.0;

pub fn forward_one_madgwick(
    encoders: &Encoders,
    mpu: &mut Mpu6050,
    bias: &ImuBias,
    drive: &mut DifferentialDrive,
    delay: &mut esp_hal::delay::Delay,
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

            if (edges_l + edges_r) / 2 >= TARGET_EDGES_800 as u32 * 300 {
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

        let u = pid.claculate(0.0, yaw_filtered, 0.8).clamp(-20.0, 20.0);

        let pwm_l = (BASE_SPEED - u - TRIM) as u16;
        let pwm_r = (BASE_SPEED + u) as u16;
        drive.set_speeds(pwm_l, pwm_r);

        info!(
            "yaw_raw={} yaw_f={} u={} eL={} eR={} pL={} pR={}",
            yaw_raw, yaw_filtered, u, edges_l, edges_r, pwm_l, pwm_r
        );
    }
}
