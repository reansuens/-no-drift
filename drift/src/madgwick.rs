use libm;

pub struct MadgwickFilter {
    q0: f32,
    q1: f32,
    q2: f32,
    q3: f32,
    beta: f32,
}

impl MadgwickFilter {
    pub fn new(beta: f32) -> Self {
        Self {
            q0: 1.0,
            q1: 0.0,
            q2: 0.0,
            q3: 0.0,
            beta,
        }
    }

    pub fn update(&mut self, ax: f32, ay: f32, az: f32, gx: f32, gy: f32, gz: f32, dt: f32) {
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
    pub fn yaw_deg(&self) -> f32 {
        let yaw_rad = libm::atan2f(
            2.0 * (self.q0 * self.q3 + self.q1 * self.q2),
            1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3),
        );
        yaw_rad * (180.0 / core::f32::consts::PI)
    }

    // Returns yaw in radians
    pub fn yaw_rad(&self) -> f32 {
        libm::atan2f(
            2.0 * (self.q0 * self.q3 + self.q1 * self.q2),
            1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3),
        )
    }
}
