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

#[test]
fn statustext_is_a_warning_cut_at_fifty_bytes_on_a_char_boundary() {
    // (text, expected text sent)
    let long = "x".repeat(60);
    let accented = format!("{}é", "x".repeat(49)); // `é` straddles byte 50
    let cases = [
        (
            "Link back: move the sticks to take control",
            "Link back: move the sticks to take control",
        ),
        (long.as_str(), &long[..50]),
        (accented.as_str(), &accented[..49]),
    ];
    for (text, expected) in cases {
        let data = statustext(text);
        check!(data.severity == MavSeverity::MAV_SEVERITY_WARNING);
        check!(data.text.to_str().ok() == Some(expected), "{text}");
    }
}
