pub struct KalmanHeading {
    angle: f32, // estimated heading [deg]
    bias: f32,  // estimated gyro bias [deg/s]
    p00: f32,   // covariance [0][0]
    p01: f32,   // covariance [0][1]
    p10: f32,   // covariance [1][0]
    p11: f32,   // covariance [1][1]
}

impl KalmanHeading {
    pub fn new() -> Self {
        Self {
            angle: 0.0,
            bias: 0.0,
            p00: 1.0,
            p01: 0.0,
            p10: 0.0,
            p11: 1.0,
        }
    }

    pub fn predict(&mut self, gz: f32, dt: f32) {
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

    pub fn update(&mut self, measurement: f32) -> f32 {
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

    pub fn angle(&self) -> f32 {
        self.angle
    }
}
