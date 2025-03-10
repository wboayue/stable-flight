use std::ops::{Add, Div, Index, Mul, Neg, Sub};

#[cfg(test)]
mod tests;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl Vec3<f32> {
    pub fn clamp(&self, min: f32, max: f32) -> Vec3<f32> {
        Vec3 {
            x: self.x.clamp(min, max),
            y: self.y.clamp(min, max),
            z: self.z.clamp(min, max),
        }
    }
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

impl<T> Vec3<T> {
    pub fn magnitude_squared(&self) -> T
    where
        T: Mul<Output = T> + Add<Output = T> + Copy,
    {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn magnitude(&self) -> f32
    where
        T: Into<f32> + Mul<Output = T> + Add<Output = T> + Copy,
    {
        (self.magnitude_squared().into()).sqrt()
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

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Vec2<T: Default> {
    pub x: T,
    pub y: T,
}

impl Vec2<f32> {
    pub fn clamp(&self, min: f32, max: f32) -> Vec2<f32> {
        Vec2 {
            x: self.x.clamp(min, max),
            y: self.y.clamp(min, max),
        }
    }
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

/// A quaternion representing a 3D rotation or orientation.
///
/// Quaternions are mathematical objects of the form `w + xi + yj + zk`, where `w, x, y, z` are
/// real numbers, and `i, j, k` are imaginary units satisfying `i^2 = j^2 = k^2 = ijk = -1`.
/// In this implementation, quaternions are primarily used to represent 3D rotations, where a unit
/// quaternion (i.e., `w^2 + x^2 + y^2 + z^2 = 1`) encodes a rotation in 3D space.
///
/// # Fields
/// - `w`: The scalar (real) part of the quaternion.
/// - `x`: The coefficient of the `i` imaginary unit (first vector component).
/// - `y`: The coefficient of the `j` imaginary unit (second vector component).
/// - `z`: The coefficient of the `k` imaginary unit (third vector component).
///
/// # Examples
/// ```
/// use stable_flight_sitl::math::Quaternion;
///
/// let q = Quaternion::new(1.0, 0.0, 0.0, 0.0); // Identity quaternion
///
/// assert_eq!(q.w, 1.0);
/// assert_eq!(q.x, 0.0);
/// assert_eq!(q.y, 0.0);
/// assert_eq!(q.z, 0.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    /// Creates a new quaternion with the given components.
    ///
    /// # Arguments
    /// * `w` - The scalar part of the quaternion.
    /// * `x` - The `i` component (first vector part).
    /// * `y` - The `j` component (second vector part).
    /// * `z` - The `k` component (third vector part).
    ///
    /// # Returns
    /// A new `Quaternion` instance with the specified components.
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// Creates an identity quaternion representing no rotation.
    ///
    /// The identity quaternion is `(1, 0, 0, 0)`, which corresponds to a zero rotation.
    ///
    /// # Returns
    /// A `Quaternion` representing the identity (no rotation).
    ///
    /// # Examples
    /// ```
    /// use stable_flight_sitl::math::Quaternion;
    ///
    /// let q = Quaternion::identity();
    ///
    /// assert_eq!(q.w, 1.0);
    /// assert_eq!(q.x, 0.0);
    /// assert_eq!(q.y, 0.0);
    /// assert_eq!(q.z, 0.0);
    /// ```
    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Creates a quaternion from Euler angles (roll, pitch, yaw) in the ZYX convention.
    ///
    /// The Euler angles follow the "usual SLR sequence":
    /// 1. Rotate about the global Z-axis by `yaw`.
    /// 2. Rotate about the resulting Y-axis by `pitch`.
    /// 3. Rotate about the resulting X-axis by `roll`.
    ///
    /// The resulting quaternion represents the orientation of a body frame after applying these rotations.
    ///
    /// # Arguments
    /// * `yaw` - Rotation about the Z-axis (in radians).
    /// * `pitch` - Rotation about the Y-axis (in radians).
    /// * `roll` - Rotation about the X-axis (in radians).
    ///
    /// # Returns
    /// A `Quaternion` representing the combined rotation.
    ///
    /// # Examples
    /// ```
    /// use stable_flight_sitl::math::Quaternion;
    /// use std::f32::consts::PI;
    ///
    /// let q = Quaternion::from_euler(0.0, 0.0, PI / 2.0); // 90-degree yaw
    ///
    /// let expected = Quaternion::new((PI / 4.0).cos(), 0.0, 0.0, (PI / 4.0).sin());
    /// ```
    pub fn from_euler(yaw: f32, pitch: f32, roll: f32) -> Self {
        let roll = roll / 2.0;
        let pitch = pitch / 2.0;
        let yaw = yaw / 2.0;

        let q0 = roll.cos() * pitch.cos() * yaw.cos() + roll.sin() * pitch.sin() * yaw.sin();
        let q1 = -roll.cos() * pitch.sin() * yaw.sin() + pitch.cos() * yaw.cos() * roll.sin();
        let q2 = roll.cos() * yaw.cos() * pitch.sin() + roll.sin() * pitch.cos() * yaw.sin();
        let q3 = roll.cos() * pitch.cos() * yaw.sin() - roll.sin() * yaw.cos() * pitch.sin();

        Self {
            w: q0,
            x: q1,
            y: q2,
            z: q3,
        }
    }

    /// Creates a quaternion from a 3x3 rotation matrix.
    ///
    /// Converts a rotation matrix (assumed to be orthogonal with determinant 1) into its equivalent quaternion representation.
    /// This implementation uses a numerically stable algorithm that avoids division by zero by selecting the largest component.
    ///
    /// # Arguments
    /// * `rotation_matrix` - A `Mat3x3` representing a rotation (in row-major order).
    ///
    /// # Returns
    /// A `Quaternion` representing the same rotation as the input matrix.
    ///
    /// # Examples
    /// ```
    /// use stable_flight_sitl::math::{Quaternion, Mat3x3};
    /// use std::f32::consts::PI;
    ///
    /// let cos_90 = (PI / 2.0).cos();
    /// let sin_90 = (PI / 2.0).sin();
    /// let mat = Mat3x3::new([
    ///     cos_90, -sin_90, 0.0,
    ///     sin_90, cos_90,  0.0,
    ///     0.0,    0.0,     1.0,
    /// ]);
    /// let q = Quaternion::from_rotation_matrix(mat);
    ///
    /// let expected = Quaternion::new((PI / 4.0).cos(), 0.0, 0.0, (PI / 4.0).sin());
    /// ```
    pub fn from_rotation_matrix(rotation_matrix: Mat3x3) -> Self {
        let r = rotation_matrix.data;

        // Compute the four possible denominators
        let t0 = 1.0 + r[0] + r[4] + r[8]; // 1 + R11 + R22 + R33
        let t1 = 1.0 + r[0] - r[4] - r[8]; // 1 + R11 - R22 - R33
        let t2 = 1.0 - r[0] + r[4] - r[8]; // 1 - R11 + R22 - R33
        let t3 = 1.0 - r[0] - r[4] + r[8]; // 1 - R11 - R22 + R33

        // Find the largest value to determine which component to compute first
        if t0 >= t1 && t0 >= t2 && t0 >= t3 {
            // q0 is largest
            let q0 = 0.5 * t0.sqrt();
            let q1 = (r[7] - r[5]) / (4.0 * q0); // (R32 - R23) / (4 * q0)
            let q2 = (r[2] - r[6]) / (4.0 * q0); // (R13 - R31) / (4 * q0)
            let q3 = (r[3] - r[1]) / (4.0 * q0); // (R21 - R12) / (4 * q0)
            Self {
                w: q0,
                x: q1,
                y: q2,
                z: q3,
            }
        } else if t1 >= t2 && t1 >= t3 {
            // q1 is largest
            let q1 = 0.5 * t1.sqrt();
            let q0 = (r[7] - r[5]) / (4.0 * q1); // (R32 - R23) / (4 * q1)
            let q2 = (r[1] + r[3]) / (4.0 * q1); // (R12 + R21) / (4 * q1)
            let q3 = (r[2] + r[6]) / (4.0 * q1); // (R13 + R31) / (4 * q1)
            Self {
                w: q0,
                x: q1,
                y: q2,
                z: q3,
            }
        } else if t2 >= t3 {
            // q2 is largest
            let q2 = 0.5 * t2.sqrt();
            let q0 = (r[2] - r[6]) / (4.0 * q2); // (R13 - R31) / (4 * q2)
            let q1 = (r[1] + r[3]) / (4.0 * q2); // (R12 + R21) / (4 * q2)
            let q3 = (r[5] + r[7]) / (4.0 * q2); // (R23 + R32) / (4 * q2)
            Self {
                w: q0,
                x: q1,
                y: q2,
                z: q3,
            }
        } else {
            // q3 is largest
            let q3 = 0.5 * t3.sqrt();
            let q0 = (r[3] - r[1]) / (4.0 * q3); // (R21 - R12) / (4 * q3)
            let q1 = (r[2] + r[6]) / (4.0 * q3); // (R13 + R31) / (4 * q3)
            let q2 = (r[5] + r[7]) / (4.0 * q3); // (R23 + R32) / (4 * q3)
            Self {
                w: q0,
                x: q1,
                y: q2,
                z: q3,
            }
        }
    }

    /// Computes the conjugate of the quaternion.
    ///
    /// The conjugate of a quaternion \( q = (w, x, y, z) \) is \( q^* = (w, -x, -y, -z) \).
    /// For unit quaternions, the conjugate is also the inverse.
    ///
    /// # Returns
    /// The conjugate of this quaternion.
    ///
    /// # Examples
    /// ```
    /// use stable_flight_sitl::math::Quaternion;
    ///
    /// let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    /// let conj = q.conjugate();
    ///
    /// assert_eq!(conj.w, 1.0);
    /// assert_eq!(conj.x, -2.0);
    /// assert_eq!(conj.y, -3.0);
    /// assert_eq!(conj.z, -4.0);
    /// ```
    pub fn conjugate(&self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Computes the inverse of the quaternion, assuming it is a unit quaternion.
    ///
    /// For a unit quaternion (\( w^2 + x^2 + y^2 + z^2 = 1 \)), the inverse is equal to its conjugate.
    /// If the quaternion is not a unit quaternion, this method does not compute the correct inverse
    /// (which would require division by the norm).
    ///
    /// # Returns
    /// The inverse of this quaternion, assuming it is a unit quaternion.
    ///
    /// # Examples
    /// ```
    /// use stable_flight_sitl::math::Quaternion;
    /// use std::f32::consts::PI;
    ///
    /// let q = Quaternion::from_euler(0.0, 0.0, PI / 2.0); // Unit quaternion
    /// let inv = q.inverse();
    ///
    /// let expected = q.conjugate();
    /// assert_eq!(inv.w, expected.w);
    /// assert_eq!(inv.x, expected.x);
    /// assert_eq!(inv.y, expected.y);
    /// assert_eq!(inv.z, expected.z);
    /// ```
    pub fn inverse(&self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Computes the rotation matrix from inertial frame to body frame.
    ///
    /// The resulting matrix \( R \) transforms a vector from the body frame to the inertial frame:
    /// \( v_I = R v_B \). Assumes the quaternion is a unit quaternion.
    ///
    /// # Returns
    /// A `Mat3x3` representing the rotation from inertial to body frame.
    pub fn rotation_matrix_i_wrt_b(&self) -> Mat3x3 {
        let qvec = vec3(self.x, self.y, self.z);
        let sk_q = Mat3x3::skew_symmetric(qvec); // Build the skew symmetric matrix of the imaginary part of quaternion
        Mat3x3::identity() * (self.w * self.w - qvec.magnitude_squared())
            + Mat3x3::outer_product(qvec, qvec) * 2.0
            + sk_q * 2.0 * self.w
    }

    /// Computes the rotation matrix from body frame to inertial frame.
    ///
    /// The resulting matrix \( R \) transforms a vector from the inertial frame to the body frame:
    /// \( v_B = R v_I \). This is the transpose of `rotation_matrix_i_wrt_b`.
    ///
    /// # Returns
    /// A `Mat3x3` representing the rotation from body to inertial frame.
    pub fn rotation_matrix_b_wrt_i(&self) -> Mat3x3 {
        self.rotation_matrix_i_wrt_b().transpose()
    }

    /// Extracts the yaw angle (rotation about Z-axis) from the quaternion.
    ///
    /// Assumes the quaternion represents a rotation in the ZYX Euler angle convention.
    ///
    /// # Returns
    /// The yaw angle in radians.
    pub fn yaw(&self) -> f32 {
        let yaw_denominator = self.w * self.w + self.x * self.x - self.y * self.y - self.z * self.z;
        (2.0 * (self.x * self.y + self.w * self.z)).atan2(yaw_denominator)
    }

    /// Extracts the pitch angle (rotation about Y-axis) from the quaternion.
    ///
    /// Assumes the quaternion represents a rotation in the ZYX Euler angle convention.
    ///
    /// # Returns
    /// The pitch angle in radians.
    pub fn pitch(&self) -> f32 {
        (-2.0 * (self.x * self.z - self.w * self.y)).asin()
    }

    /// Extracts the roll angle (rotation about X-axis) from the quaternion.
    ///
    /// Assumes the quaternion represents a rotation in the ZYX Euler angle convention.
    ///
    /// # Returns
    /// The roll angle in radians.
    pub fn roll(&self) -> f32 {
        let roll_denominator =
            self.w * self.w - self.x * self.x - self.y * self.y + self.z * self.z;
        (2.0 * (self.y * self.z + self.w * self.x)).atan2(roll_denominator)
    }

    /// Converts the quaternion to a Gibbs vector (Rodrigues parameters).
    ///
    /// The Gibbs vector is defined as \( (x/w, y/w, z/w) \) if \( w \neq 0 \). If \( w = 0 \),
    /// a large scalar is used to approximate infinity.
    ///
    /// # Returns
    /// A `Vec3<f32>` representing the Gibbs vector.
    pub fn to_gibbs_vector(&self) -> Vec3<f32> {
        if self.w == 0.0 {
            1e20 * vec3(self.x, self.y, self.z)
        } else {
            vec3(self.x, self.y, self.z) / self.w
        }
    }

    /// Converts the quaternion to Euler angles in yaw-pitch-roll order.
    ///
    /// # Returns
    /// A `Vec3<f32>` containing `(yaw, pitch, roll)` in radians.
    pub fn to_euler(&self) -> Vec3<f32> {
        vec3(self.yaw(), self.pitch(), self.roll())
    }

    /// Converts the quaternion to Euler angles in roll-pitch-yaw order.
    ///
    /// # Returns
    /// A `Vec3<f32>` containing `(roll, pitch, yaw)` in radians.
    pub fn to_euler_rpy(&self) -> Vec3<f32> {
        vec3(self.roll(), self.pitch(), self.yaw())
    }
}

/// Implements quaternion multiplication.
///
/// Quaternion multiplication follows the standard Hamilton product rules:
/// - \( w = w_1 w_2 - x_1 x_2 - y_1 y_2 - z_1 z_2 \)
/// - \( x = w_1 x_2 + x_1 w_2 + y_1 z_2 - z_1 y_2 \)
/// - \( y = w_1 y_2 + y_1 w_2 + z_1 x_2 - x_1 z_2 \)
/// - \( z = w_1 z_2 + z_1 w_2 + x_1 y_2 - y_1 x_2 \)
impl Mul<Quaternion> for Quaternion {
    type Output = Quaternion;

    /// Multiplies two quaternions.
    ///
    /// # Arguments
    /// * `p` - The right-hand quaternion to multiply with.
    ///
    /// # Returns
    /// A new `Quaternion` representing the product of `self` and `p`.
    ///
    /// # Examples
    /// ```
    /// use stable_flight_sitl::math::Quaternion;
    ///
    /// let q1 = Quaternion::new(0.0, 1.0, 0.0, 0.0); // i
    /// let q2 = Quaternion::new(0.0, 0.0, 1.0, 0.0); // j
    /// let result = q1 * q2;
    ///
    /// assert_eq!(result.w, 0.0);
    /// assert_eq!(result.x, 0.0);
    /// assert_eq!(result.y, 0.0);
    /// assert_eq!(result.z, 1.0); // i * j = k
    /// ```
    fn mul(self, p: Quaternion) -> Quaternion {
        let q = self;

        let c0 = q.w * p.w - q.x * p.x - q.y * p.y - q.z * p.z; // w = w1w2 - x1x2 - y1y2 - z1z2
        let c1 = q.w * p.x + q.x * p.w + q.y * p.z - q.z * p.y; // x = w1x2 + x1w2 + y1z2 - z1y2
        let c2 = q.w * p.y + q.y * p.w + q.z * p.x - q.x * p.z; // y = w1y2 + y1w2 + z1x2 - x1z2
        let c3 = q.w * p.z + q.z * p.w + q.x * p.y - q.y * p.x; // z = w1z2 + z1w2 + x1y2 - y1x2

        Quaternion {
            w: c0,
            x: c1,
            y: c2,
            z: c3,
        }
    }
}

impl Default for Quaternion {
    /// Provides the default value for a `Quaternion`, which is the identity quaternion.
    ///
    /// # Returns
    /// The identity quaternion `(1, 0, 0, 0)`.
    fn default() -> Self {
        Quaternion {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

impl ToString for Quaternion {
    /// Formats the quaternion as a string.
    ///
    /// # Returns
    /// A string in the format `"w x y z"`, with each component formatted to three decimal places.
    ///
    /// # Examples
    /// ```
    /// use stable_flight_sitl::math::Quaternion;
    /// let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    /// assert_eq!(q.to_string(), "1.000 2.000 3.000 4.000");
    /// ```
    fn to_string(&self) -> String {
        format!("{:.3} {:.3} {:.3} {:.3}", self.w, self.x, self.y, self.z)
    }
}

/// A 3x3 matrix with single-precision floating-point elements, stored in row-major order.
///
/// The matrix is represented internally as a flat array of 9 `f32` values, where the elements
/// are ordered as `[a11, a12, a13, a21, a22, a23, a31, a32, a33]`. This struct provides basic
/// linear algebra operations for 3x3 matrices, such as transposition, determinant calculation,
/// and trace computation.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl::math::Mat3x3;
///
/// // Create an identity matrix
/// let identity = Mat3x3::identity();
/// assert_eq!(identity.data, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
///
/// // Create a custom matrix
/// let matrix = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
/// assert_eq!(matrix[[1, 2]], 6.0); // Access element at row 1, column 2
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat3x3 {
    pub data: [f32; 9],
}

impl Mat3x3 {
    /// Creates a new 3x3 matrix from a flat array of 9 elements in row-major order.
    ///
    /// The input array represents the matrix elements as `[a11, a12, a13, a21, a22, a23, a31, a32, a33]`.
    ///
    /// # Arguments
    ///
    /// * `data` - A 9-element array of `f32` values representing the matrix elements.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight_sitl::math::Mat3x3;
    /// let matrix = Mat3x3::new([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
    /// assert_eq!(matrix.data, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
    /// ```
    pub fn new(data: [f32; 9]) -> Self {
        Self { data }
    }

    /// Creates a 3x3 zero matrix, where all elements are 0.0.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight_sitl::math::Mat3x3;
    /// let zero = Mat3x3::zeros();
    /// assert_eq!(zero.data, [0.0; 9]);
    /// ```
    pub fn zeros() -> Self {
        Self { data: [0.0; 9] }
    }

    /// Creates a 3x3 identity matrix, where diagonal elements are 1.0 and all other elements are 0.0.
    ///
    /// The resulting matrix is:
    ///
    /// \[
    /// \begin{bmatrix}
    /// 1 & 0 & 0 \\
    /// 0 & 1 & 0 \\
    /// 0 & 0 & 1
    /// \end{bmatrix}
    /// \]
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight_sitl::math::Mat3x3;
    /// let identity = Mat3x3::identity();
    /// assert_eq!(identity.data, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
    /// ```
    pub fn identity() -> Self {
        Self {
            data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        }
    }

    /// Creates a skew-symmetric matrix from a 3D vector.
    ///
    /// A skew-symmetric matrix `S` has the property that `S^T = -S`. For a 3D vector
    /// `v = [x, y, z]`, the corresponding skew-symmetric matrix is:
    ///
    /// ```text
    /// [  0  -z   y ]
    /// [  z   0  -x ]
    /// [ -y   x   0 ]
    /// ```
    ///
    /// This matrix is useful in various applications:
    ///
    /// - Computing cross products: For vectors `a` and `b`, the cross product `a × b`
    ///   can be computed as `skew_symmetric(a) * b`
    /// - Representing infinitesimal rotations in SO(3)
    /// - Converting between quaternions and rotation matrices
    ///
    /// # Arguments
    ///
    /// * `q` - A 3D vector whose components will be used to construct the skew-symmetric matrix
    ///
    /// # Returns
    ///
    /// A 3×3 skew-symmetric matrix corresponding to the input vector
    pub fn skew_symmetric(q: Vec3<f32>) -> Mat3x3 {
        Self {
            data: [0.0, -q.z, q.y, q.z, 0.0, -q.x, -q.y, q.x, 0.0],
        }
    }

    /// Creates a matrix representing the outer product of two 3D vectors.
    ///
    /// The outer product of vectors `a` and `b` is the matrix `M = a ⊗ b` where
    /// each element `M[i,j] = a[i] * b[j]`. For 3D vectors, this produces a 3×3 matrix.
    ///
    /// The resulting matrix has the following structure:
    /// ```text
    /// [ a.x*b.x  a.x*b.y  a.x*b.z ]
    /// [ a.y*b.x  a.y*b.y  a.y*b.z ]
    /// [ a.z*b.x  a.z*b.y  a.z*b.z ]
    /// ```
    ///
    /// # Applications
    ///
    /// The outer product has various applications in physics, computer graphics, and numerical methods:
    /// - Constructing projection matrices
    /// - Forming part of tensor products
    /// - Computing dyadic products in physics
    /// - Creating rotation matrices from quaternions
    ///
    /// # Arguments
    ///
    /// * `a` - First 3D vector
    /// * `b` - Second 3D vector
    ///
    /// # Returns
    ///
    /// A 3×3 matrix representing the outer product of the input vectors
    pub fn outer_product(a: Vec3<f32>, b: Vec3<f32>) -> Mat3x3 {
        let mut data = [0.0; 9];

        data[0] = a.x * b.x;
        data[1] = a.x * b.y;
        data[2] = a.x * b.z;

        data[3] = a.y * b.x;
        data[4] = a.y * b.y;
        data[5] = a.y * b.z;

        data[6] = a.z * b.x;
        data[7] = a.z * b.y;
        data[8] = a.z * b.z;

        Self { data }
    }

    /// Computes the transpose of the matrix.
    ///
    /// The transpose of a matrix swaps its rows and columns. For a 3x3 matrix \( A \), the element
    /// at position \((i, j)\) in \( A \) becomes the element at position \((j, i)\) in the transpose \( A^T \).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight_sitl::math::Mat3x3;
    /// let matrix = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    /// let transposed = matrix.transpose();
    /// assert_eq!(transposed.data, [1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0]);
    /// ```
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

    /// Computes the determinant of the matrix.
    ///
    /// For a 3x3 matrix:
    ///
    /// \[
    /// A = \begin{bmatrix}
    /// a & b & c \\
    /// d & e & f \\
    /// g & h & i
    /// \end{bmatrix},
    /// \]
    ///
    /// the determinant is calculated as:
    ///
    /// \[
    /// \text{det}(A) = a(ei - fh) - d(bi - ch) + g(bf - ce)
    /// \]
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight_sitl::math::Mat3x3;
    /// let matrix = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    /// let det = matrix.determinant();
    /// assert_eq!(det, 0.0); // 1*(5*9 - 6*8) - 4*(2*9 - 3*8) + 7*(2*6 - 3*5) = 0
    /// ```
    pub fn determinant(&self) -> f32 {
        let data = &self.data;

        data[0] * (data[4] * data[8] - data[7] * data[5])
            - data[3] * (data[1] * data[8] - data[7] * data[2])
            + data[6] * (data[1] * data[5] - data[4] * data[2])
    }

    /// Computes the trace of the matrix.
    ///
    /// The trace of a matrix is the sum of the elements on its main diagonal, i.e., \( a_{11} + a_{22} + a_{33} \).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight_sitl::math::Mat3x3;
    /// let matrix = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    /// let trace = matrix.trace();
    /// assert_eq!(trace, 15.0); // 1 + 5 + 9 = 15
    /// ```
    pub fn trace(&self) -> f32 {
        self.data[0] + self.data[4] + self.data[8]
    }
}

/// Provides indexing into the matrix using `[row, col]` syntax.
///
/// The matrix is stored in row-major order, so the element at position \((i, j)\) corresponds to
/// the flat array index \( i \cdot 3 + j \). Indices must be in the range \([0, 2]\) for both row
/// and column; otherwise, the program will panic at runtime.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl::math::Mat3x3;
/// let matrix = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
/// assert_eq!(matrix[[0, 1]], 2.0); // Element at row 0, column 1
/// assert_eq!(matrix[[2, 2]], 9.0); // Element at row 2, column 2
/// ```
impl Index<[u32; 2]> for Mat3x3 {
    type Output = f32;

    fn index(&self, ndx: [u32; 2]) -> &Self::Output {
        &self.data[ndx[0] as usize * 3 + ndx[1] as usize]
    }
}

impl Mul<f32> for Mat3x3 {
    type Output = Mat3x3;

    fn mul(self, scalar: f32) -> Mat3x3 {
        let mut data = [0.0; 9];
        for i in 0..9 {
            data[i] = self.data[i] * scalar;
        }
        Mat3x3 { data }
    }
}

impl Add<Mat3x3> for Mat3x3 {
    type Output = Mat3x3;

    fn add(self, rhs: Mat3x3) -> Mat3x3 {
        let mut data = [0.0; 9];
        for (i, item) in data.iter_mut().enumerate() {
            *item = self.data[i] + rhs.data[i];
        }
        Mat3x3 { data }
    }
}
