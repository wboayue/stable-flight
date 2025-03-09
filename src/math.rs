use std::{
    ops::{Add, Div, Index, Mul, Neg, Sub},
    vec,
};

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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn from_euler(roll: f32, pitch: f32, yaw: f32) -> Self {
        // the roll,pitch,yaw angles are the "usual SLR sequence" Euler angles:
        // 1. rotate inertial frame about global-Z for yaw
        // 2. rotate resulting frame for pitch
        // 3. rotate resulting frame for roll

        let roll = roll / 2.0;
        let pitch = pitch / 2.0;
        let yaw = yaw / 2.0;

        let q0 = roll.cos() * pitch.cos() * yaw.cos() + roll.sin() * pitch.sin() * yaw.sin();
        let q1 = -roll.cos() * pitch.sin() * yaw.sin() + pitch.cos() * yaw.cos() * roll.sin();
        let q2 = roll.cos() * yaw.cos() * pitch.sin() + roll.sin() * pitch.cos() * yaw.sin();
        let q3 = roll.cos() * pitch.cos() * yaw.sin() - roll.sin() * yaw.cos() * pitch.sin();

        // The returned quaternion now represents the attitude of a vehicle with the above angles

        Self {
            w: q0,
            x: q1,
            y: q2,
            z: q3,
        }
    }

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

    // for unit quaternions adjoint == reciprocal == conjugate == inverse.
    pub fn conjugate(&self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    // ASSUME: Unit quaternion
    pub fn inverse(&self) -> Self {
        // divide by the magnitude norm
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn rotation_matrix_i_wrt_b(&self) -> Mat3x3 {
        //transformation matrix of inertial w.r.t. the body -- v_I = RotationMatrix_IwrtB*v_B
        let qvec = vec3(self.x, self.y, self.z);
        let sk_q = Mat3x3::skew_symmetric(qvec); // Build the skew symmetric matrix of the imaginary part of quaternion
        Mat3x3::identity() * (self.w * self.w - qvec.magnitude_squared())
            + Mat3x3::outer_product(qvec, qvec) * 2.0
            + sk_q * 2.0 * self.w
    }

    pub fn rotation_matrix_b_wrt_i(&self) -> Mat3x3 {
        self.rotation_matrix_i_wrt_b().transpose()
    }

    pub fn yaw(&self) -> f32 {
        let yaw_denominator = self.w * self.w + self.x * self.x - self.y * self.y - self.z * self.z;
        (2.0 * (self.x * self.y + self.w * self.z)).atan2(yaw_denominator)
    }

    pub fn pitch(&self) -> f32 {
        (-2.0 * (self.x * self.z - self.w * self.y)).asin()
    }

    pub fn roll(&self) -> f32 {
        let roll_denominator =
            self.w * self.w - self.x * self.x - self.y * self.y + self.z * self.z;
        (2.0 * (self.y * self.z + self.w * self.x)).atan2(roll_denominator)
    }

    pub fn to_gibbs_vector(&self) -> Vec3<f32> {
        if self.w == 0.0 {
            1e20 * vec3(self.x, self.y, self.z)
        } else {
            vec3(self.x, self.y, self.z) / self.w
        }
    }

    pub fn to_euler_ypr(&self) -> Vec3<f32> {
        vec3(self.yaw(), self.pitch(), self.roll())
    }

    pub fn to_euler(&self) -> Vec3<f32> {
        vec3(self.roll(), self.pitch(), self.yaw())
    }
}

impl Mul<Quaternion> for Quaternion {
    type Output = Quaternion;

    fn mul(self, p: Quaternion) -> Quaternion {
        let q = self;

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
/// use your_crate_name::Mat3x3; // Replace `your_crate_name` with the actual crate name
///
/// // Create an identity matrix
/// let identity = Mat3x3::identity();
/// assert_eq!(identity.data, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
///
/// // Create a custom matrix
/// let matrix = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
/// assert_eq!(matrix[[1, 2]], 6.0); // Access element at row 1, column 2
/// ```
pub struct Mat3x3 {
    data: [f32; 9],
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
    /// let identity = Mat3x3::identity();
    /// assert_eq!(identity.data, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
    /// ```
    pub fn identity() -> Self {
        Self {
            data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        }
    }

    pub fn skew_symmetric(q: Vec3<f32>) -> Mat3x3 {
        Self {
            data: [0.0, -q.z, q.y, q.z, 0.0, -q.x, -q.y, q.x, 0.0],
        }
    }

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
        for i in 0..9 {
            data[i] = self.data[i] + rhs.data[i];
        }
        Mat3x3 { data }
    }
}
