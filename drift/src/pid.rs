pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral: f32,
    pub prev_err: f32,
    pub integral_limit: f32,
}

impl Pid {
    pub fn claculate(&mut self, target: f32, current: f32, dt: f32) -> f32 {
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
