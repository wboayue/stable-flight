use std::ops::{Add, Div, Index, Mul, Neg, Sub};

#[cfg(test)]
mod tests;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

// Helper function to create Vec3 instances easily
pub fn vec3<T>(x: T, y: T, z: T) -> Vec3<T> {
    Vec3 { x, y, z }
}

impl<T: Sub<Output = T>> Sub for Vec3<T> {
    type Output = Vec3<T>;

    fn sub(self, rhs: Vec3<T>) -> Vec3<T> {
        Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl<T: Add<Output = T>> Add for Vec3<T> {
    type Output = Vec3<T>;

    fn add(self, rhs: Vec3<T>) -> Vec3<T> {
        Vec3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl<T: Mul<Output = T>> Mul for Vec3<T> {
    type Output = Vec3<T>;

    fn mul(self, rhs: Vec3<T>) -> Vec3<T> {
        Vec3 {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z,
        }
    }
}

// x * vec3
impl Mul<Vec3<f32>> for f32 {
    type Output = Vec3<f32>;

    fn mul(self, rhs: Vec3<f32>) -> Vec3<f32> {
        Vec3 {
            x: self * rhs.x,
            y: self * rhs.y,
            z: self * rhs.z,
        }
    }
}

// vec3 / x
impl<T: Div<Output = T> + Copy> Div<T> for Vec3<T> {
    type Output = Vec3<T>;

    fn div(self, rhs: T) -> Vec3<T> {
        Vec3 {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

impl<T: Default> Default for Vec3<T> {
    fn default() -> Self {
        Vec3 {
            x: T::default(),
            y: T::default(),
            z: T::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2<T: Default> {
    pub x: T,
    pub y: T,
}

// Helper function to create Vec3 instances easily
pub fn vec2<T: Default>(x: T, y: T) -> Vec2<T> {
    Vec2 { x, y }
}

impl<T: Sub<Output = T> + Default> Sub for Vec2<T> {
    type Output = Vec2<T>;

    fn sub(self, rhs: Vec2<T>) -> Vec2<T> {
        Vec2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl<T: Add<Output = T> + Default> Add for Vec2<T> {
    type Output = Vec2<T>;

    fn add(self, rhs: Vec2<T>) -> Vec2<T> {
        Vec2 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl<T: Mul<Output = T> + Default> Mul for Vec2<T> {
    type Output = Vec2<T>;

    fn mul(self, rhs: Vec2<T>) -> Vec2<T> {
        Vec2 {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
        }
    }
}

// x * vec2
impl Mul<Vec2<f32>> for f32 {
    type Output = Vec2<f32>;

    fn mul(self, rhs: Vec2<f32>) -> Vec2<f32> {
        Vec2 {
            x: self * rhs.x,
            y: self * rhs.y,
        }
    }
}

// vec3 / x
impl<T: Div<Output = T> + Copy + Default> Div<T> for Vec2<T> {
    type Output = Vec2<T>;

    fn div(self, rhs: T) -> Vec2<T> {
        Vec2 {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

impl<T: Default> Default for Vec2<T> {
    fn default() -> Self {
        Vec2 {
            x: T::default(),
            y: T::default(),
        }
    }
}

impl<T: Neg<Output = T> + Default> Neg for Vec2<T> {
    type Output = Vec2<T>;

    fn neg(self) -> Self::Output {
        Vec2 {
            x: -self.x,
            y: -self.y,
        }
    }
}

pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub fn rotation_matrix_i_wrt_b(&self) -> Mat3x3 {
        //     let qvec = Vec3 {
        //         x: self.data[1],
        //         y: self.data[2],
        //         z: self.data[3],
        //     };

        //     let skew_q = Mat3x3::skew_symmetric(qvec);

        //     let q0 = self.data[0];

        //     let ident = Mat3x3::identity();
        //     let qvec_mag_sq = qvec.x * qvec.x + qvec.y * qvec.y + qvec.z * qvec.z;

        //     let outer_product = Mat3x3::outer_product(qvec, qvec);

        //     ident * (q0 * q0 - qvec_mag_sq) + outer_product * 2.0 + skew_q * 2.0 * q0
        Mat3x3::identity()
    }
}

// Mat3x3F RotationMatrix_IwrtB(void) const
// {
//       //transformation matrix of inertial w.r.t. the body -- v_I = RotationMatrix_IwrtB*v_B
//   V3F qvec = V3F(_q[1],_q[2],_q[3]);
//   Mat3x3F Sk_q =  Mat3x3F::SkewSymmetric(qvec);// Build the skew symmetric matrix of the imaginary part of quaternion
//   return Mat3x3F::Ident()*(_q[0]*_q[0] - qvec.magSq()) + Mat3x3F::OuterProduct(qvec,qvec)*2  + Sk_q*2*_q[0];
// };

// Mat3x3F RotationMatrix_BwrtI(void) const
// {
//       //transformation matrix of body w.r.t. inertial
//   Mat3x3F temp = RotationMatrix_IwrtB();
//   temp.Transpose();
//   return temp;
// };

pub struct Mat3x3 {
    data: [f32; 9],
}

impl Mat3x3 {
    pub fn new(data: [f32; 9]) -> Self {
        Self { data }
    }

    pub fn zeros() -> Self {
        Self { data: [0.0; 9] }
    }

    pub fn identity() -> Self {
        Self {
            data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        }
    }

    pub fn transpose(&self) -> Self {
        Self {
            data: [
                self.data[0],
                self.data[3],
                self.data[6],
                self.data[1],
                self.data[4],
                self.data[7],
                self.data[2],
                self.data[5],
                self.data[8],
            ],
        }
    }

    pub fn determinant(&self) -> f32 {
        let data = &self.data;

        data[0] * (data[4] * data[8] - data[7] * data[5])
            - data[3] * (data[1] * data[8] - data[7] * data[2])
            + data[6] * (data[1] * data[5] - data[4] * data[2])
    }

    pub fn trace(&self) -> f32 {
        self.data[0] + self.data[4] + self.data[8]
    }
}

impl Index<[u32; 2]> for Mat3x3 {
    type Output = f32;

    fn index(&self, ndx: [u32; 2]) -> &Self::Output {
        &self.data[ndx[0] as usize * 3 + ndx[1] as usize]
    }
}
