use esp_hal::gpio::{Input, InputConfig};

pub struct Encoders<'e> {
    pub left_a: Input<'e>,  // GPIO9  — left
    pub left_b: Input<'e>,  // GPIO20 — left second
    pub right_a: Input<'e>, // GPIO21 — right
    pub right_b: Input<'e>, // GPIO4  — right second
}


impl<'e> Encoders<'e> {
    pub fn new(
        left_a: Input<'e>,
        left_b: Input<'e>,
        right_a: Input<'e>,
        right_b: Input<'e>,
    ) -> Self {
        Self {
            left_a,
            left_b,
            right_a,
            right_b,
        }
    }
}
