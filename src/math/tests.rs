use super::Mat3x3;

mod vec3_tests {
    use super::super::{vec3, Vec3};

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
}

#[test]
fn test_mat_3x3_identity() {
    let m = Mat3x3::identity();
    assert_eq!(m.data, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
}

#[test]
fn test_mat_3x3_new() {
    // row-major order
    let m = Mat3x3::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    assert_eq!(m.data, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
}
