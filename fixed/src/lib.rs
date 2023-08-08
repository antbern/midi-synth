#![no_std]
/// Module containing a simple 15x16 fixed point number supporting simple arithmetic and square
/// roots.
use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

/// A fixed point value with 15bits to the left of the decimal point and 16 bits for the decimal
/// part.
/// Resolution 2^-16 = 1.5e-5
/// Dynamic range is 32767/-32768.
///
///
///
/// Inspired by [Fixed Point Arithmetic Pi Pico RP2040](https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_fixed_pt/index_fixed.htmlhttps://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_fixed_pt/index_fixed.html)
/// and the [fixed point page](https://vanhunteradams.com/FixedPoint/FixedPoint.html) by Hunter Adams.
/// See also https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_floating_point/index_floating_point.html
#[derive(Debug, Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub struct S15x16(i32);

impl S15x16 {
    pub const fn from_int(value: i16) -> Self {
        Self((value as i32) << 16)
    }

    /// Return an integer by dropping the decimal part
    pub fn to_int(self) -> i16 {
        (self.0 >> 16) as i16
    }

    pub fn from_float(value: f32) -> Self {
        Self((value * 65536.0) as i32)
    }

    pub fn to_float(self) -> f32 {
        self.0 as f32 / 65536.0
    }

    // Compute square root of this number. Based on original implementation by Christophe Meessen
    // 1993, modified and available in https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_fixed_pt/speed_test/Fixed_point_test.c.
    pub fn sqrt(self) -> Self {
        assert!(self.0 >= 0);

        let mut t: u32;
        let mut b: u32 = 0x40000000;
        let mut q: u32 = 0;
        let mut r: u32 = self.0 as u32;

        while b > 0x40 {
            t = q + b;
            if r >= t {
                r -= t;
                q = t + b; // equivalent to q += 2*b
            }

            r <<= 1;
            b >>= 1;
        }

        q >>= 8;

        Self(q as i32)
    }
}

impl From<f32> for S15x16 {
    fn from(value: f32) -> Self {
        Self::from_float(value)
    }
}

impl From<i16> for S15x16 {
    fn from(value: i16) -> Self {
        Self::from_int(value)
    }
}

impl Add for S15x16 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl AddAssign for S15x16 {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Sub for S15x16 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl SubAssign for S15x16 {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl Mul for S15x16 {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self((((self.0 as i64) * (rhs.0 as i64)) >> 16) as i32)
    }
}

//impl MulAssign for S15x16 {
//    fn mul_assign(&mut self, rhs: Self) {
//        *self = *self * rhs;
//    }
//}

impl<T> MulAssign<T> for S15x16
where
    T: Into<S15x16>,
{
    fn mul_assign(&mut self, rhs: T) {
        *self = *self * rhs.into();
    }
}

//impl<T> Mul<T> for S15x16 where T: Into<S15x16>{
//    type Output = Self;
//
//    fn mul(self, rhs: T) -> Self::Output {
//       self * rhs.into()
//    }
//}

impl Div for S15x16 {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        Self((((self.0 as i64) << 16) / rhs.0 as i64) as i32)
    }
}

impl<T> DivAssign<T> for S15x16
where
    T: Into<S15x16>,
{
    fn div_assign(&mut self, rhs: T) {
        *self = *self / rhs.into();
    }
}

//impl<T> Mul<S15x16> for T where T: Into<S15x16> {
//    type Output = S15x16;
//
//    fn mul(self, rhs: S15x16) -> Self::Output {
//        T::into(self) * rhs
//    }
//}

// for reducing duplication when From implementations are available
macro_rules! aritmetic_from {
    ($type:ty) => {
        impl Mul<$type> for S15x16 {
            type Output = Self;

            fn mul(self, rhs: $type) -> Self::Output {
                self * Self::from(rhs)
            }
        }
        impl Mul<S15x16> for $type {
            type Output = S15x16;

            fn mul(self, rhs: S15x16) -> Self::Output {
                S15x16::from(self) * rhs
            }
        }
        impl Div<$type> for S15x16 {
            type Output = Self;

            fn div(self, rhs: $type) -> Self::Output {
                self / Self::from(rhs)
            }
        }
        impl Div<S15x16> for $type {
            type Output = S15x16;

            fn div(self, rhs: S15x16) -> Self::Output {
                S15x16::from(self) / rhs
            }
        }

        // Right and left hand add and subtract operations
        impl Add<$type> for S15x16 {
            type Output = Self;

            fn add(self, rhs: $type) -> Self::Output {
                self + Self::from(rhs)
            }
        }

        impl Add<S15x16> for $type {
            type Output = S15x16;

            fn add(self, rhs: S15x16) -> Self::Output {
                S15x16::from(self) + rhs
            }
        }
        impl Sub<$type> for S15x16 {
            type Output = Self;

            fn sub(self, rhs: $type) -> Self::Output {
                self - Self::from(rhs)
            }
        }

        impl Sub<S15x16> for $type {
            type Output = S15x16;

            fn sub(self, rhs: S15x16) -> Self::Output {
                S15x16::from(self) - rhs
            }
        }
    };
}

aritmetic_from!(f32);
aritmetic_from!(i16);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversion() {
        for i in i16::MIN..i16::MAX {
            assert_eq!(S15x16::from_int(i).to_int(), i);
        }
    }

    #[test]
    fn add_sub() {
        2.0 + S15x16::from_int(10);

        assert_eq!(
            S15x16::from_int(10) + S15x16::from_int(45),
            S15x16::from_int(55)
        );
        assert_eq!(
            S15x16::from_int(10) - S15x16::from_int(45),
            S15x16::from_int(-35)
        );

        let mut a = S15x16::from_int(34);
        a += S15x16::from_int(6);
        assert_eq!(a, S15x16::from_int(40));
        a -= S15x16::from_int(60);
        assert_eq!(a, S15x16::from_int(40 - 60));
        a /= S15x16::from_int(60);

        assert_eq!(S15x16::from_int(45) + 10.0, S15x16::from_float(55.0));
        assert_eq!(10.0 + S15x16::from_int(40), S15x16::from_float(50.0));
        assert_eq!(S15x16::from_int(45) - 10.0, S15x16::from_float(35.0));
        assert_eq!(10.0 - S15x16::from_int(40), S15x16::from_float(-30.0));
    }

    #[test]
    fn mul() {
        assert_eq!(
            S15x16::from_int(45) * S15x16::from_int(10),
            S15x16::from_int(450)
        );
        assert_eq!(
            S15x16::from_int(45) * S15x16::from_int(-10),
            S15x16::from_int(-450)
        );
        assert_eq!(
            S15x16::from_int(-45) * S15x16::from_int(10),
            S15x16::from_int(-450)
        );
        assert_eq!(
            S15x16::from_int(-45) * S15x16::from_int(-10),
            S15x16::from_int(450)
        );

        assert_eq!(S15x16::from_int(45) * 10.0, S15x16::from_float(450.0));
        assert_eq!(10.0 * S15x16::from_int(40), S15x16::from_float(400.0));
    }

    #[test]
    fn div() {
        assert_eq!(
            S15x16::from_int(45) / S15x16::from_int(10),
            S15x16::from_float(4.5)
        );
        assert_eq!(
            S15x16::from_int(45) / S15x16::from_int(-10),
            S15x16::from_float(-4.50)
        );
        assert_eq!(
            S15x16::from_int(-45) / S15x16::from_int(10),
            S15x16::from_float(-4.50)
        );
        assert_eq!(
            S15x16::from_int(-45) / S15x16::from_int(-10),
            S15x16::from_float(4.50)
        );

        assert_eq!(S15x16::from_int(45) / 10.0, S15x16::from_float(4.5));
        assert_eq!(10.0 / S15x16::from_int(40), S15x16::from_float(0.25));
    }

    #[test]
    fn sqrt() {
        assert_eq!(S15x16::from_int(4).sqrt(), S15x16::from_int(2));
        assert_eq!(S15x16::from_int(16).sqrt(), S15x16::from_int(4));
        assert_eq!(S15x16::from_int(100).sqrt(), S15x16::from_int(10));
        assert_eq!(
            S15x16::from_int(2).sqrt(),
            S15x16::from_float(2.0f32.sqrt())
        );
    }
}
