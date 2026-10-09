use assert2::check;

use super::*;

fn param(name: &str, value: f32, index: u16, count: u16) -> ParamValue {
    ParamValue {
        name: name.try_into().unwrap(),
        value,
        index,
        count,
    }
}

#[test]
fn param_id_converts_to_a_name() {
    // (case, param_id bytes, expected name)
    let cases: [(&str, &[u8], Option<&str>); 4] = [
        ("NUL-terminated", b"STAB_RATE_P\0", Some("STAB_RATE_P")),
        (
            "all 16 bytes, no NUL",
            b"ABCDEFGHIJKLMNOP",
            Some("ABCDEFGHIJKLMNOP"),
        ),
        ("empty", b"", None),
        ("not UTF-8", b"\xff\xfe", None),
    ];
    for (case, bytes, expected) in cases {
        let mut data = [0u8; 16];
        data[..bytes.len()].copy_from_slice(bytes);
        let name = param_name(&CharArray::new(data));
        check!(name.as_deref() == expected, "{case}");
    }
}

#[test]
fn cache_finds_params_by_index_and_name() {
    let mut cache = ParamCache::default();
    cache.insert(param("A", 1.0, 0, 2));
    cache.insert(param("B", 2.0, 1, 2));

    check!(cache.get(1).map(|p| p.value) == Some(2.0));
    check!(cache.find("A").map(|p| p.index) == Some(0));
    check!(cache.get(2).is_none());
    check!(cache.find("C").is_none());
}

#[test]
fn echo_replaces_the_cached_value() {
    let mut cache = ParamCache::default();
    cache.insert(param("A", 1.0, 0, 1));
    cache.insert(param("A", 5.0, 0, 1));

    check!(cache.find("A").map(|p| p.value) == Some(5.0));
}

#[test]
fn new_count_restarts_the_cache() {
    let mut cache = ParamCache::default();
    cache.insert(param("A", 1.0, 0, 2));
    cache.insert(param("B", 2.0, 1, 3));

    check!(cache.get(0).is_none());
    check!(cache.find("B").map(|p| p.index) == Some(1));
}

#[test]
fn index_beyond_count_is_ignored() {
    let mut cache = ParamCache::default();
    cache.insert(param("A", 1.0, 2, 2));

    check!(cache.find("A").is_none());
}

#[test]
fn clear_forgets_every_param() {
    let mut cache = ParamCache::default();
    cache.insert(param("A", 1.0, 0, 1));
    cache.clear();

    check!(cache.get(0).is_none());
}
