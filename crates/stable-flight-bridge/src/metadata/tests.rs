use assert2::{assert, check};

use super::*;

const BASE: &str = "http://127.0.0.1:14580";

fn parse(file: &MetadataFile) -> Value {
    serde_json::from_slice(&file.bytes).expect("valid JSON")
}

/// The keys QGC's parameter schema accepts; any other drops that param's metadata.
const QGC_PARAM_KEYS: [&str; 8] = [
    "name",
    "type",
    "shortDesc",
    "category",
    "group",
    "units",
    "min",
    "max",
];

#[test]
fn describes_every_param_with_only_keys_qgc_accepts() {
    let files = MetadataFiles::build(BASE, copter_params()).expect("short URI");
    let json = parse(&files.parameters);

    check!(json["version"] == 1);
    let params = json["parameters"].as_array().expect("an array");
    check!(params.len() == copter_params().count());
    for (param, meta) in params.iter().zip(copter_params()) {
        let object = param.as_object().expect("an object");
        let name = meta.name();
        for key in object.keys() {
            check!(QGC_PARAM_KEYS.contains(&key.as_str()), "{name}: {key}");
        }
        check!(param["name"] == name);
        check!(param["type"] == "Float", "{name}: every param is REAL32");
        check!(param["shortDesc"] == meta.description());
        check!(param["category"] == meta.category());
        check!(param["group"] == meta.group());
    }
}

#[test]
fn writes_units_and_bounds_only_when_present() {
    let files = MetadataFiles::build(BASE, copter_params()).expect("short URI");
    let json = parse(&files.parameters);
    let param = |name: &str| {
        json["parameters"]
            .as_array()
            .expect("an array")
            .iter()
            .find(|p| p["name"] == name)
            .cloned()
            .expect("listed")
    };

    // (name, expected units, min, max); `Null` is absent.
    let cases = [
        ("STAB_RATE_RLL_P", Value::Null, Value::Null, Value::Null), // gain: none
        ("STAB_RATE_IMAX", json!("rad"), json!(0.0), Value::Null),
        ("GUID_ALT_HOVER", Value::Null, json!(0.0), json!(1.0)), // exclusive min as its value
        ("GUID_TILT_MAX", json!("deg"), json!(0.0), json!(90.0)),
    ];
    for (name, units, min, max) in cases {
        let p = param(name);
        check!(p["units"] == units, "{name}");
        check!(p["min"] == min, "{name}");
        check!(p["max"] == max, "{name}");
    }
}

#[test]
fn general_file_points_to_the_parameters_by_crc_versioned_uri() {
    let files = MetadataFiles::build(&format!("{BASE}/"), copter_params()).expect("short URI");
    let general = parse(&files.general);

    check!(general["version"] == 1);
    let types = general["metadataTypes"].as_array().expect("an array");
    let [parameters] = types.as_slice() else {
        panic!("expected one metadata type, got {types:?}");
    };
    check!(parameters["type"] == COMP_METADATA_TYPE_PARAMETER);
    check!(parameters["fileCrc"] == files.parameters.crc);
    let crc = files.parameters.crc;
    check!(
        parameters["uri"] == format!("{BASE}/{crc}/parameters.json"),
        "trailing / trimmed"
    );
    check!(files.parameters.path == format!("/{crc}/parameters.json"));
    check!(files.general_uri == format!("{BASE}{}", files.general.path));

    let metadata = files.component_metadata();
    check!(metadata.crc == files.general.crc);
    let uri = files.general_uri.as_bytes();
    check!(&metadata.uri[..uri.len()] == uri);
    check!(
        metadata.uri[uri.len()..].iter().all(|&b| b == 0),
        "NUL-padded"
    );
}

#[test]
fn crcs_and_paths_change_with_the_content() {
    let all = MetadataFiles::build(BASE, copter_params()).expect("short URI");
    let fewer = MetadataFiles::build(BASE, copter_params().skip(1)).expect("short URI");
    let again = MetadataFiles::build(BASE, copter_params()).expect("short URI");

    check!(all == again, "deterministic");
    check!(all.parameters.crc != fewer.parameters.crc);
    check!(all.parameters.path != fewer.parameters.path);
    check!(
        all.general.crc != fewer.general.crc,
        "the general file names the parameters' CRC"
    );
}

#[test]
fn advertises_the_bind_address_unless_it_is_every_interface() {
    // (http_bind, http_url, expected base URL; None: refused)
    let cases = [
        ("127.0.0.1:14580", None, Some("http://127.0.0.1:14580")),
        (
            "192.168.1.30:14580",
            None,
            Some("http://192.168.1.30:14580"),
        ),
        ("localhost:14580", None, Some("http://localhost:14580")), // a name: used as is
        ("0.0.0.0:14580", None, None),
        ("[::]:14580", None, None),
        (
            "0.0.0.0:14580",
            Some("http://192.168.1.30:14580"),
            Some("http://192.168.1.30:14580"),
        ),
        (
            "127.0.0.1:14580",
            Some("http://proxy/sf"),
            Some("http://proxy/sf"),
        ),
    ];
    for (bind, url, expected) in cases {
        let result = base_url(bind, url);
        check!(result.as_deref().ok() == expected, "{bind} {url:?}");
        if expected.is_none() {
            assert!(let Err(UnreachableUrl(rejected)) = result);
            check!(rejected == bind);
        }
    }
}

#[test]
fn rejects_a_base_url_too_long_for_component_metadata() {
    let long = format!("http://{}.example:14580", "a".repeat(60));
    let result = MetadataFiles::build(&long, copter_params());
    assert!(let Err(UriTooLong(uri)) = result);
    check!(uri.len() > MAX_URI_LEN);
}
