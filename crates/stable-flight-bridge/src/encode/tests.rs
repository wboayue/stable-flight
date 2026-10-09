use assert2::check;

use super::*;

#[test]
fn packs_flight_sw_version() {
    // (case, major, minor, patch, expected)
    let cases = [
        ("crate version", "0", "1", "0", 0x0001_0000),
        ("every part", "1", "2", "3", 0x0102_0300),
        ("unparsable part is 0", "1", "x", "3", 0x0100_0300),
    ];
    for (case, major, minor, patch, expected) in cases {
        let dev = FirmwareVersionType::FIRMWARE_VERSION_TYPE_DEV as u32;
        check!(
            flight_sw_version(major, minor, patch) == expected | dev,
            "{case}"
        );
    }
}
