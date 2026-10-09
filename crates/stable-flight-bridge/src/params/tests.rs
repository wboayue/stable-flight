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

fn set(name: &str, value: f32, param_type: MavParamType) -> PARAM_SET_DATA {
    PARAM_SET_DATA {
        param_value: value,
        target_system: 1,
        target_component: 1,
        param_id: name.into(),
        param_type,
    }
}

fn read_index(index: i16) -> PARAM_REQUEST_READ_DATA {
    PARAM_REQUEST_READ_DATA {
        param_index: index,
        target_system: 1,
        target_component: 1,
        param_id: "".into(),
    }
}

fn read_name(name: &str) -> PARAM_REQUEST_READ_DATA {
    PARAM_REQUEST_READ_DATA {
        param_index: -1,
        target_system: 1,
        target_component: 1,
        param_id: name.into(),
    }
}

fn uplink_set(name: &str, value: f32) -> Uplink {
    Uplink::Param(ParamRequest::Set {
        name: name.try_into().unwrap(),
        value,
    })
}

/// `(name, value, index, count)` of a `PARAM_VALUE`.
fn param_value_of(msg: &MavMessage) -> Option<(String, f32, u16, u16)> {
    let MavMessage::PARAM_VALUE(data) = msg else {
        return None;
    };
    check!(data.param_type == MavParamType::MAV_PARAM_TYPE_REAL32);
    Some((
        data.param_id.to_str().unwrap().to_owned(),
        data.param_value,
        data.param_index,
        data.param_count,
    ))
}

/// A service with params `A` (index 0) = 1.0 and `B` = 2.0 cached.
fn service_with_params() -> ParamService {
    let mut service = ParamService::default();
    let mut out = Outbox::default();
    service.on_param(&param("A", 1.0, 0, 2), &mut out);
    service.on_param(&param("B", 2.0, 1, 2), &mut out);
    service
}

/// A ground-station request to the service.
enum Request {
    List,
    Read(PARAM_REQUEST_READ_DATA),
    Set(PARAM_SET_DATA),
}

#[test]
fn params_translate_between_mavlink_and_the_link() {
    let a = Some(("A".to_owned(), 1.0, 0, 2));
    // (case, from the ground station, expected uplinks, expected PARAM_VALUE)
    let cases = [
        (
            "list goes to the flight controller",
            Request::List,
            vec![Uplink::Param(ParamRequest::List)],
            None,
        ),
        (
            "read by index from the cache",
            Request::Read(read_index(0)),
            vec![],
            a.clone(),
        ),
        (
            "read by name from the cache",
            Request::Read(read_name("A")),
            vec![],
            a.clone(),
        ),
        (
            "read of an uncached index: no reply",
            Request::Read(read_index(5)),
            vec![],
            None,
        ),
        (
            "read of an uncached name: no reply",
            Request::Read(read_name("C")),
            vec![],
            None,
        ),
        (
            "REAL32 set goes to the flight controller",
            Request::Set(set("A", 30.0, MavParamType::MAV_PARAM_TYPE_REAL32)),
            vec![uplink_set("A", 30.0)],
            None,
        ),
        (
            "set of an uncached name still goes; the flight controller decides",
            Request::Set(set("C", 3.0, MavParamType::MAV_PARAM_TYPE_REAL32)),
            vec![uplink_set("C", 3.0)],
            None,
        ),
        (
            "non-REAL32 set is rejected with the current value",
            Request::Set(set("A", 30.0, MavParamType::MAV_PARAM_TYPE_INT32)),
            vec![],
            a.clone(),
        ),
        (
            "set without a name is ignored",
            Request::Set(set("", 30.0, MavParamType::MAV_PARAM_TYPE_REAL32)),
            vec![],
            None,
        ),
    ];
    for (case, request, to_fc, to_gcs) in cases {
        let service = service_with_params();
        let mut out = Outbox::default();
        match &request {
            Request::List => service.on_request_list(&mut out),
            Request::Read(data) => service.on_request_read(data, &mut out),
            Request::Set(data) => service.on_set(data, &mut out),
        }
        check!(out.to_fc == to_fc, "{case}");
        check!(
            out.to_gcs.iter().map(param_value_of).collect::<Vec<_>>()
                == Vec::from_iter(to_gcs.map(Some)),
            "{case}"
        );
    }
}

#[test]
fn every_flight_controller_param_goes_out_as_param_value() {
    // (case, param)
    let cases = [
        ("listing entry", param("A", 1.0, 0, 2)),
        (
            "accepted write's echo: the value sent",
            param("STAB_ATT_AMAX_X", 30.0, 4, 20),
        ),
        (
            "rejected write's echo: the current value",
            param("STAB_RATE_IMAX", 0.3, 7, 20),
        ),
    ];
    for (case, sent) in cases {
        let mut service = ParamService::default();
        let mut out = Outbox::default();
        service.on_param(&sent, &mut out);
        let expected = (sent.name.to_string(), sent.value, sent.index, sent.count);
        check!(
            out.to_gcs.iter().map(param_value_of).collect::<Vec<_>>() == [Some(expected)],
            "{case}"
        );
        check!(out.to_fc.is_empty(), "{case}");
    }
}

#[test]
fn echo_updates_the_cache() {
    let mut service = service_with_params();
    let mut out = Outbox::default();
    service.on_param(&param("A", 5.0, 0, 2), &mut out);
    out.to_gcs.clear();

    service.on_request_read(&read_name("A"), &mut out);

    check!(
        out.to_gcs.iter().map(param_value_of).collect::<Vec<_>>()
            == [Some(("A".to_owned(), 5.0, 0, 2))]
    );
}

#[test]
fn flight_controller_appearing_clears_the_cache_and_lists() {
    let mut service = service_with_params();
    let mut out = Outbox::default();

    service.fc_appeared(&mut out);
    check!(out.to_fc == [Uplink::Param(ParamRequest::List)]);

    service.on_request_read(&read_index(0), &mut out);
    check!(out.to_gcs.is_empty(), "cache cleared");
}
