use std::f32::EPSILON;

// Helper function to compare floating-point values
fn assert_float_eq(a: f32, b: f32, epsilon: f32) {
    assert!(
        (a - b).abs() <= epsilon,
        "Expected {} to be approximately equal to {}, but difference is greater than epsilon {}",
        a,
        b,
        epsilon
    );
}

mod vec3 {
    use super::super::*;
    use super::*;

    #[test]
    fn test_default() {
        let v: Vec3<f32> = Vec3::default();
        assert_eq!(v.x, 0.0);
        assert_eq!(v.y, 0.0);
        assert_eq!(v.z, 0.0);

        let v_int: Vec3<i32> = Vec3::default();
        assert_eq!(v_int.x, 0);
        assert_eq!(v_int.y, 0);
        assert_eq!(v_int.z, 0);
    }

    #[test]
    fn test_addition() {
        let v1 = vec3(1.0, 2.0, 3.0);
        let v2 = vec3(4.0, 5.0, 6.0);
        let result = v1 + v2;
        assert_eq!(result.x, 5.0);
        assert_eq!(result.y, 7.0);
        assert_eq!(result.z, 9.0);

        // Test with integers
        let v1_int = vec3(1, 2, 3);
        let v2_int = vec3(4, 5, 6);
        let result_int = v1_int + v2_int;
        assert_eq!(result_int.x, 5);
        assert_eq!(result_int.y, 7);
        assert_eq!(result_int.z, 9);
    }

    #[test]
    fn test_subtraction() {
        let v1 = vec3(5.0, 7.0, 9.0);
        let v2 = vec3(1.0, 2.0, 3.0);
        let result = v1 - v2;
        assert_eq!(result.x, 4.0);
        assert_eq!(result.y, 5.0);
        assert_eq!(result.z, 6.0);

        // Test with integers
        let v1_int = vec3(5, 7, 9);
        let v2_int = vec3(1, 2, 3);
        let result_int = v1_int - v2_int;
        assert_eq!(result_int.x, 4);
        assert_eq!(result_int.y, 5);
        assert_eq!(result_int.z, 6);
    }

    #[test]
    fn test_multiplication_vec3() {
        let v1 = vec3(2.0, 3.0, 4.0);
        let v2 = vec3(5.0, 6.0, 7.0);
        let result = v1 * v2;
        assert_eq!(result.x, 10.0);
        assert_eq!(result.y, 18.0);
        assert_eq!(result.z, 28.0);

        // Test with integers
        let v1_int = vec3(2, 3, 4);
        let v2_int = vec3(5, 6, 7);
        let result_int = v1_int * v2_int;
        assert_eq!(result_int.x, 10);
        assert_eq!(result_int.y, 18);
        assert_eq!(result_int.z, 28);
    }

    #[test]
    fn test_scalar_multiplication() {
        let v = vec3(1.0, 2.0, 3.0);
        let scalar = 2.0;
        let result = scalar * v;
        assert_eq!(result.x, 2.0);
        assert_eq!(result.y, 4.0);
        assert_eq!(result.z, 6.0);
    }

    #[test]
    fn test_division_by_scalar() {
        let v = vec3(4.0, 6.0, 8.0);
        let scalar = 2.0;
        let result = v / scalar;
        assert_eq!(result.x, 2.0);
        assert_eq!(result.y, 3.0);
        assert_eq!(result.z, 4.0);

        // Test with integers
        let v_int = vec3(4, 6, 8);
        let scalar_int = 2;
        let result_int = v_int / scalar_int;
        assert_eq!(result_int.x, 2);
        assert_eq!(result_int.y, 3);
        assert_eq!(result_int.z, 4);
    }

    #[test]
    fn test_clone_and_copy() {
        let v1 = vec3(1.0, 2.0, 3.0);
        let v2 = v1; // Copy
        assert_eq!(v1.x, v2.x);
        assert_eq!(v1.y, v2.y);
        assert_eq!(v1.z, v2.z);

        let v3 = v1.clone(); // Explicit clone
        assert_eq!(v1.x, v3.x);
        assert_eq!(v1.y, v3.y);
        assert_eq!(v1.z, v3.z);
    }

    #[test]
    fn test_debug_output() {
        let v = vec3(1.0, 2.0, 3.0);
        let debug_str = format!("{:?}", v);
        assert!(debug_str.contains("Vec3"));
        assert!(debug_str.contains("x: 1"));
        assert!(debug_str.contains("y: 2"));
        assert!(debug_str.contains("z: 3"));
    }

    #[test]
    fn test_clamp() {
        // Test clamping within range
        let v = vec3(1.0, 2.0, 3.0);
        let result = v.clamp(0.0, 5.0);
        assert_eq!(result.x, 1.0);
        assert_eq!(result.y, 2.0);
        assert_eq!(result.z, 3.0);

        // Test clamping below minimum
        let v = vec3(-1.0, -2.0, -3.0);
        let result = v.clamp(0.0, 5.0);
        assert_eq!(result.x, 0.0);
        assert_eq!(result.y, 0.0);
        assert_eq!(result.z, 0.0);

        // Test clamping above maximum
        let v = vec3(6.0, 7.0, 8.0);
        let result = v.clamp(0.0, 5.0);
        assert_eq!(result.x, 5.0);
        assert_eq!(result.y, 5.0);
        assert_eq!(result.z, 5.0);

        // Test mixed clamping
        let v = vec3(-1.0, 3.0, 8.0);
        let result = v.clamp(0.0, 5.0);
        assert_eq!(result.x, 0.0);
        assert_eq!(result.y, 3.0);
        assert_eq!(result.z, 5.0);
    }

    #[test]
    fn test_magnitude_squared() {
        // Test with positive values
        let v = vec3(3.0, 4.0, 0.0);
        assert_eq!(v.magnitude_squared(), 25.0); // 3² + 4² + 0² = 9 + 16 + 0 = 25

        // Test with negative values
        let v = vec3(-3.0, -4.0, 0.0);
        assert_eq!(v.magnitude_squared(), 25.0); // (-3)² + (-4)² + 0² = 9 + 16 + 0 = 25

        // Test with mixed values
        let v = vec3(1.0, -2.0, 2.0);
        assert_eq!(v.magnitude_squared(), 9.0); // 1² + (-2)² + 2² = 1 + 4 + 4 = 9

        // Test with zero vector
        let v = vec3(0.0, 0.0, 0.0);
        assert_eq!(v.magnitude_squared(), 0.0);

        // Test with integers
        let v = vec3(3, 4, 0);
        assert_eq!(v.magnitude_squared(), 25); // 3² + 4² + 0² = 9 + 16 + 0 = 25
    }

    #[test]
    fn test_magnitude() {
        // Test with Pythagorean triple
        let v = vec3(3.0, 4.0, 0.0);
        assert_float_eq(v.magnitude(), 5.0, EPSILON); // √(3² + 4² + 0²) = √25 = 5

        // Test with negative values
        let v = vec3(-3.0, -4.0, 0.0);
        assert_float_eq(v.magnitude(), 5.0, EPSILON); // √((-3)² + (-4)² + 0²) = √25 = 5

        // Test with unit vector along x-axis
        let v = vec3(1.0, 0.0, 0.0);
        assert_float_eq(v.magnitude(), 1.0, EPSILON);

        // Test with zero vector
        let v = vec3(0.0, 0.0, 0.0);
        assert_float_eq(v.magnitude(), 0.0, EPSILON);

        // Test with 3D case
        let v = vec3(2.0, 3.0, 6.0);
        assert_float_eq(v.magnitude(), 7.0, EPSILON); // √(2² + 3² + 6²) = √(4 + 9 + 36) = √49 = 7
    }
}

mod vec2 {
    use super::super::*;

    #[test]
    fn test_creation_and_basic_ops() {
        // Test creation and default
        let v = vec2(1.0, 2.0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);

        let v_default: Vec2<f32> = Vec2::default();
        assert_eq!(v_default.x, 0.0);
        assert_eq!(v_default.y, 0.0);

        // Test addition
        let v1 = vec2(1.0, 2.0);
        let v2 = vec2(3.0, 4.0);
        let sum = v1 + v2;
        assert_eq!(sum.x, 4.0);
        assert_eq!(sum.y, 6.0);

        // Test subtraction
        let diff = v2 - v1;
        assert_eq!(diff.x, 2.0);
        assert_eq!(diff.y, 2.0);

        // Test multiplication
        let prod = v1 * v2;
        assert_eq!(prod.x, 3.0); // 1.0 * 3.0
        assert_eq!(prod.y, 8.0); // 2.0 * 4.0

        // Test scalar multiplication
        let scaled = 2.0 * v1;
        assert_eq!(scaled.x, 2.0);
        assert_eq!(scaled.y, 4.0);

        // Test division by scalar
        let divided = v2 / 2.0;
        assert_eq!(divided.x, 1.5);
        assert_eq!(divided.y, 2.0);
    }

    #[test]
    fn test_negation() {
        let v = vec2(1.0, -2.0);
        let neg = -v;
        assert_eq!(neg.x, -1.0);
        assert_eq!(neg.y, 2.0);

        // Test double negation
        let double_neg = -(-v);
        assert_eq!(double_neg.x, 1.0);
        assert_eq!(double_neg.y, -2.0);
    }

    #[test]
    fn test_clamp() {
        // Test clamping within range
        let v = vec2(1.0, 2.0);
        let result = v.clamp(0.0, 5.0);
        assert_eq!(result.x, 1.0);
        assert_eq!(result.y, 2.0);

        // Test clamping below minimum
        let v = vec2(-1.0, -2.0);
        let result = v.clamp(0.0, 5.0);
        assert_eq!(result.x, 0.0);
        assert_eq!(result.y, 0.0);

        // Test clamping above maximum
        let v = vec2(6.0, 7.0);
        let result = v.clamp(0.0, 5.0);
        assert_eq!(result.x, 5.0);
        assert_eq!(result.y, 5.0);

        // Test mixed clamping
        let v = vec2(-1.0, 8.0);
        let result = v.clamp(0.0, 5.0);
        assert_eq!(result.x, 0.0);
        assert_eq!(result.y, 5.0);
    }
}

mod mat3x3 {
    use super::super::*;
    use std::f32::EPSILON;

    // Helper function to compare floating-point numbers with an epsilon
    fn assert_float_eq(a: f32, b: f32, epsilon: f32) {
        assert!(
            (a - b).abs() <= epsilon,
            "Expected {} to be approximately equal to {}, but difference is greater than epsilon {}",
            a, b, epsilon
        );
    }

    #[test]
    fn test_new() {
        let data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let mat = Mat3x3::new(data);
        assert_eq!(mat.data, data);
    }

    #[test]
    fn test_zeros() {
        let mat = Mat3x3::zeros();
        let expected = [0.0; 9];
        assert_eq!(mat.data, expected);
    }

    #[test]
    fn test_identity() {
        let mat = Mat3x3::identity();
        let expected = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert_eq!(mat.data, expected);
    }

    #[test]
    fn test_index() {
        let mat = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        assert_eq!(mat[[0, 0]], 1.0);
        assert_eq!(mat[[0, 1]], 2.0);
        assert_eq!(mat[[0, 2]], 3.0);
        assert_eq!(mat[[1, 0]], 4.0);
        assert_eq!(mat[[1, 1]], 5.0);
        assert_eq!(mat[[1, 2]], 6.0);
        assert_eq!(mat[[2, 0]], 7.0);
        assert_eq!(mat[[2, 1]], 8.0);
        assert_eq!(mat[[2, 2]], 9.0);
    }

    #[test]
    #[should_panic]
    fn test_index_out_of_bounds() {
        let mat = Mat3x3::new([0.0; 9]);
        let _ = mat[[3, 0]]; // Should panic
    }

    #[test]
    fn test_transpose() {
        let mat = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        let transposed = mat.transpose();
        let expected = [1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0];
        assert_eq!(transposed.data, expected);

        // Test that transposing twice gives the original matrix
        let double_transposed = transposed.transpose();
        assert_eq!(double_transposed.data, mat.data);
    }

    #[test]
    fn test_determinant() {
        // Test with identity matrix (determinant should be 1.0)
        let mat = Mat3x3::identity();
        assert_float_eq(mat.determinant(), 1.0, EPSILON);

        // Test with a known matrix
        // Matrix: [1, 2, 3]
        //         [4, 5, 6]
        //         [7, 8, 9]
        // Determinant = 1*(5*9 - 6*8) - 2*(4*9 - 6*7) + 3*(4*8 - 5*7) = 0
        let mat = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        assert_float_eq(mat.determinant(), 0.0, EPSILON);

        // Test with another matrix
        // Matrix: [2, 0, 0]
        //         [0, 3, 0]
        //         [0, 0, 4]
        // Determinant = 2*(3*4 - 0*0) - 0*(0*4 - 0*0) + 0*(0*0 - 3*0) = 24
        let mat = Mat3x3::new([2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0]);
        assert_float_eq(mat.determinant(), 24.0, EPSILON);
    }

    #[test]
    fn test_trace() {
        // Test with identity matrix (trace should be 3.0)
        let mat = Mat3x3::identity();
        assert_float_eq(mat.trace(), 3.0, EPSILON);

        // Test with a known matrix
        // Matrix: [1, 2, 3]
        //         [4, 5, 6]
        //         [7, 8, 9]
        // Trace = 1 + 5 + 9 = 15
        let mat = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        assert_float_eq(mat.trace(), 15.0, EPSILON);

        // Test with zeros (trace should be 0.0)
        let mat = Mat3x3::zeros();
        assert_float_eq(mat.trace(), 0.0, EPSILON);
    }

    #[test]
    fn test_skew_symmetric() {
        // Test with unit vectors
        let v = vec3(1.0, 0.0, 0.0);
        let skew = Mat3x3::skew_symmetric(v);
        // Expected skew-symmetric matrix for [1,0,0]:
        // [ 0  0  0]
        // [ 0  0 -1]
        // [ 0  1  0]
        let expected = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0];
        assert_eq!(skew.data, expected);

        let v = vec3(0.0, 1.0, 0.0);
        let skew = Mat3x3::skew_symmetric(v);
        // Expected for [0,1,0]:
        // [ 0  0  1]
        // [ 0  0  0]
        // [-1  0  0]
        let expected = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0];
        assert_eq!(skew.data, expected);

        let v = vec3(0.0, 0.0, 1.0);
        let skew = Mat3x3::skew_symmetric(v);
        // Expected for [0,0,1]:
        // [ 0 -1  0]
        // [ 1  0  0]
        // [ 0  0  0]
        let expected = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert_eq!(skew.data, expected);

        // Test with arbitrary vector
        let v = vec3(1.0, 2.0, 3.0);
        let skew = Mat3x3::skew_symmetric(v);
        // Expected for [1,2,3]:
        // [ 0 -3  2]
        // [ 3  0 -1]
        // [-2  1  0]
        let expected = [0.0, -3.0, 2.0, 3.0, 0.0, -1.0, -2.0, 1.0, 0.0];
        assert_eq!(skew.data, expected);
    }

    #[test]
    fn test_outer_product() {
        // Test with unit vectors
        let a = vec3(1.0, 0.0, 0.0);
        let b = vec3(0.0, 1.0, 0.0);
        let outer = Mat3x3::outer_product(a, b);
        // Expected outer product for [1,0,0] ⊗ [0,1,0]:
        // [ 0  1  0]
        // [ 0  0  0]
        // [ 0  0  0]
        let expected = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert_eq!(outer.data, expected);

        // Test with same vector
        let a = vec3(1.0, 2.0, 3.0);
        let outer = Mat3x3::outer_product(a, a);
        // Expected outer product for [1,2,3] ⊗ [1,2,3]:
        // [ 1  2  3]
        // [ 2  4  6]
        // [ 3  6  9]
        let expected = [1.0, 2.0, 3.0, 2.0, 4.0, 6.0, 3.0, 6.0, 9.0];
        assert_eq!(outer.data, expected);

        // Test with arbitrary vectors
        let a = vec3(1.0, 2.0, 3.0);
        let b = vec3(4.0, 5.0, 6.0);
        let outer = Mat3x3::outer_product(a, b);
        // Expected outer product for [1,2,3] ⊗ [4,5,6]:
        // [ 4  5  6]
        // [ 8 10 12]
        // [12 15 18]
        let expected = [4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 12.0, 15.0, 18.0];
        assert_eq!(outer.data, expected);
    }

    #[test]
    fn test_scalar_multiplication() {
        let mat = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        let scaled = mat * 2.0;
        let expected = [2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0];
        assert_eq!(scaled.data, expected);

        // Test with zero scaling
        let scaled = mat * 0.0;
        let expected = [0.0; 9];
        assert_eq!(scaled.data, expected);

        // Test with negative scaling
        let scaled = mat * -1.0;
        let expected = [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0, -9.0];
        assert_eq!(scaled.data, expected);
    }

    #[test]
    fn test_addition() {
        let mat1 = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        let mat2 = Mat3x3::new([9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0]);
        let sum = mat1 + mat2;
        let expected = [10.0; 9]; // Each element should be 10.0
        assert_eq!(sum.data, expected);

        // Test addition with zero matrix
        let zero = Mat3x3::zeros();
        let sum = mat1 + zero;
        assert_eq!(sum.data, mat1.data);

        // Test addition with identity matrix
        let identity = Mat3x3::identity();
        let sum = mat1 + identity;
        let expected = [2.0, 2.0, 3.0, 4.0, 6.0, 6.0, 7.0, 8.0, 10.0];
        assert_eq!(sum.data, expected);
    }
}

mod quaternion {
    use super::super::*;
    use std::f32::consts::PI;
    use std::f32::EPSILON;

    // Helper function for approximate equality of f32 values
    fn assert_float_eq(a: f32, b: f32, epsilon: f32) {
        assert!(
            (a - b).abs() < epsilon,
            "Expected {} to be approximately equal to {}, difference: {}",
            a,
            b,
            (a - b).abs()
        );
    }

    // Helper function for quaternion equality
    fn assert_quaternion_eq(q1: &Quaternion, q2: &Quaternion, epsilon: f32) {
        assert_float_eq(q1.w, q2.w, epsilon);
        assert_float_eq(q1.x, q2.x, epsilon);
        assert_float_eq(q1.y, q2.y, epsilon);
        assert_float_eq(q1.z, q2.z, epsilon);
    }

    #[test]
    fn test_identity() {
        let q = Quaternion::identity();
        assert_quaternion_eq(&q, &Quaternion::new(1.0, 0.0, 0.0, 0.0), 1e-6);
    }

    #[test]
    fn test_default() {
        let q = Quaternion::default();
        assert_quaternion_eq(&q, &Quaternion::identity(), 1e-6);
    }

    #[test]
    fn test_conjugate() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let conj = q.conjugate();
        assert_quaternion_eq(&conj, &Quaternion::new(1.0, -2.0, -3.0, -4.0), 1e-6);
    }

    #[test]
    fn test_inverse() {
        // Test inverse for a unit quaternion
        let q = Quaternion::from_euler(0.0, 0.0, PI / 2.0); // 90-degree yaw
        let inv = q.inverse();
        let expected = q.conjugate(); // Since q is unit quaternion
        assert_quaternion_eq(&inv, &expected, 1e-6);
    }

    #[test]
    fn test_multiplication() {
        // Test quaternion multiplication with identity
        let q1 = Quaternion::identity();
        let q2 = Quaternion::new(0.0, 1.0, 0.0, 0.0);
        let result = q1 * q2;
        assert_quaternion_eq(&result, &q2, 1e-6);

        // Test with two pure quaternions
        let q1 = Quaternion::new(0.0, 1.0, 0.0, 0.0); // i
        let q2 = Quaternion::new(0.0, 0.0, 1.0, 0.0); // j
        let result = q1 * q2;
        // i * j = k
        assert_quaternion_eq(&result, &Quaternion::new(0.0, 0.0, 0.0, 1.0), 1e-6);
    }

    #[test]
    fn test_from_euler() {
        // Test quaternion from Euler angles (yaw = 90 degrees, pitch = 0, roll = 0)
        let q = Quaternion::from_euler(PI / 2.0, 0.0, 0.0);
        let expected = Quaternion::new((PI / 4.0).cos(), 0.0, 0.0, (PI / 4.0).sin());
        assert_quaternion_eq(&q, &expected, 1e-6);
    }

    #[test]
    fn test_to_euler() {
        // Test Euler angle extraction for a 90-degree yaw
        let q = Quaternion::from_euler(PI / 2.0, 0.0, 0.0);
        let euler = q.to_euler_rpy(); // (roll, pitch, yaw)
        assert_float_eq(euler.x, 0.0, 1e-6); // roll
        assert_float_eq(euler.y, 0.0, 1e-6); // pitch
        assert_float_eq(euler.z, PI / 2.0, 1e-6); // yaw
    }

    #[test]
    fn test_roll() {
        // Test roll extraction for a 90-degree roll
        let q = Quaternion::from_euler(0.0, 0.0, PI / 2.0);
        let roll = q.roll();
        assert_float_eq(roll, PI / 2.0, 1e-6);
    }

    #[test]
    fn test_pitch() {
        // Test pitch extraction for a 90-degree pitch
        let q = Quaternion::from_euler(0.0, PI / 2.0, 0.0);
        let pitch = q.pitch();
        assert_float_eq(pitch, PI / 2.0, 1e-3);
    }

    #[test]
    fn test_yaw() {
        // Test yaw extraction for a 90-degree yaw
        let q = Quaternion::from_euler(PI / 2.0, 0.0, 0.0);
        let yaw = q.yaw();
        assert_float_eq(yaw, PI / 2.0, 1e-6);
    }

    #[test]
    fn test_from_rotation_matrix() {
        // Test conversion from a rotation matrix (90-degree yaw)
        let cos_90 = (PI / 2.0).cos();
        let sin_90 = (PI / 2.0).sin();
        let mat = Mat3x3::new([cos_90, -sin_90, 0.0, sin_90, cos_90, 0.0, 0.0, 0.0, 1.0]);
        let q = Quaternion::from_rotation_matrix(mat);
        let expected = Quaternion::new((PI / 4.0).cos(), 0.0, 0.0, (PI / 4.0).sin());
        assert_quaternion_eq(&q, &expected, 1e-6);
    }

    #[test]
    fn test_rotation_matrix_i_wrt_b() {
        // Test rotation matrix for a 90-degree yaw
        let q = Quaternion::from_euler(PI / 2.0, 0.0, 0.0);
        let mat = q.rotation_matrix_i_wrt_b();
        let cos_90 = (PI / 2.0).cos();
        let sin_90 = (PI / 2.0).sin();
        let expected = Mat3x3::new([cos_90, -sin_90, 0.0, sin_90, cos_90, 0.0, 0.0, 0.0, 1.0]);
        for i in 0..9 {
            assert_float_eq(mat.data[i], expected.data[i], 1e-6);
        }
    }

    #[test]
    fn test_rotation_matrix_b_wrt_i() {
        // Test body-to-inertial rotation matrix (transpose of i_wrt_b)
        let q = Quaternion::from_euler(PI / 2.0, 0.0, 0.0);
        let mat = q.rotation_matrix_b_wrt_i();
        let cos_90 = (PI / 2.0).cos();
        let sin_90 = (PI / 2.0).sin();
        let expected = Mat3x3::new([cos_90, sin_90, 0.0, -sin_90, cos_90, 0.0, 0.0, 0.0, 1.0]);
        for i in 0..9 {
            assert_float_eq(mat.data[i], expected.data[i], 1e-6);
        }
    }

    #[test]
    fn test_to_gibbs_vector() {
        // Test conversion to Gibbs vector
        let q = Quaternion::new(1.0, 1.0, 0.0, 0.0);
        let gibbs = q.to_gibbs_vector();
        assert_float_eq(gibbs.x, 1.0, 1e-6);
        assert_float_eq(gibbs.y, 0.0, 1e-6);
        assert_float_eq(gibbs.z, 0.0, 1e-6);
    }

    #[test]
    fn test_to_string() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let s = q.to_string();
        assert_eq!(s, "1.000 2.000 3.000 4.000");
    }

    #[test]
    fn test_from_rotation_matrix_edge_cases() {
        // Test with identity matrix
        let mat = Mat3x3::identity();
        let q = Quaternion::from_rotation_matrix(mat);
        assert_quaternion_eq(&q, &Quaternion::identity(), 1e-6);

        // Test with rotation matrix where q0 is largest
        // 180-degree rotation about z-axis
        let mat = Mat3x3::new([-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0]);
        let q = Quaternion::from_rotation_matrix(mat);
        // Expected quaternion: (0, 0, 0, 1) or (0, 0, 0, -1)
        // But our code chooses the one with positive w
        assert!(
            (q.w.abs() < 1e-6 && (q.z - 1.0).abs() < 1e-6)
                || (q.w.abs() < 1e-6 && (q.z + 1.0).abs() < 1e-6)
        );

        // Test with rotation matrix where q1 is largest
        // 180-degree rotation about x-axis
        let mat = Mat3x3::new([1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0]);
        let q = Quaternion::from_rotation_matrix(mat);
        // Expected quaternion: (0, 1, 0, 0) or (0, -1, 0, 0)
        assert!(
            (q.w.abs() < 1e-6 && (q.x - 1.0).abs() < 1e-6)
                || (q.w.abs() < 1e-6 && (q.x + 1.0).abs() < 1e-6)
        );

        // Test with rotation matrix where q2 is largest
        // 180-degree rotation about y-axis
        let mat = Mat3x3::new([-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0]);
        let q = Quaternion::from_rotation_matrix(mat);
        // Expected quaternion: (0, 0, 1, 0) or (0, 0, -1, 0)
        assert!(
            (q.w.abs() < 1e-6 && (q.y - 1.0).abs() < 1e-6)
                || (q.w.abs() < 1e-6 && (q.y + 1.0).abs() < 1e-6)
        );
    }

    #[test]
    fn test_to_gibbs_vector_edge_cases() {
        // Test normal case
        let q = Quaternion::new(2.0, 1.0, 2.0, 3.0);
        let gibbs = q.to_gibbs_vector();
        assert_float_eq(gibbs.x, 0.5, EPSILON); // 1.0/2.0
        assert_float_eq(gibbs.y, 1.0, EPSILON); // 2.0/2.0
        assert_float_eq(gibbs.z, 1.5, EPSILON); // 3.0/2.0

        // Test with w = 0 (180-degree rotation, Gibbs vector goes to infinity)
        let q = Quaternion::new(0.0, 1.0, 0.0, 0.0);
        let gibbs = q.to_gibbs_vector();
        assert!(gibbs.x.abs() > 1e19); // Should be very large
        assert_float_eq(gibbs.y, 0.0, EPSILON);
        assert_float_eq(gibbs.z, 0.0, EPSILON);
    }

    #[test]
    fn test_euler_conversions() {
        // Test all three Euler angles together
        let yaw = PI / 4.0; // 45 degrees
        let pitch = PI / 6.0; // 30 degrees
        let roll = PI / 3.0; // 60 degrees

        let q = Quaternion::from_euler(yaw, pitch, roll);
        let euler = q.to_euler();

        assert_float_eq(euler.x, yaw, 1e-5); // yaw
        assert_float_eq(euler.y, pitch, 1e-5); // pitch
        assert_float_eq(euler.z, roll, 1e-5); // roll

        // Also test the reverse order (rpy)
        let rpy = q.to_euler_rpy();
        assert_float_eq(rpy.x, roll, 1e-5); // roll
        assert_float_eq(rpy.y, pitch, 1e-5); // pitch
        assert_float_eq(rpy.z, yaw, 1e-5); // yaw
    }
}
