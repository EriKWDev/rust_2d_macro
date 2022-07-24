use std::ops::{Add, Mul};

#[inline]
pub fn lerp<T>(a: T, b: T, t: f32) -> T
where
    T: Add<T, Output = T>,
    T: Mul<f32, Output = T>,
    f32: Mul<T, Output = T>,
{
    (1.0 - t) * a + (t * b)
}
