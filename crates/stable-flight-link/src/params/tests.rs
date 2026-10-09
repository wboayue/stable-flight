use assert2::{assert, check};
use heapless::spsc::Queue;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};

use super::*;

fn stabilizer() -> CopterStabilizer {
    CopterStabilizer::new(CopterStabilizerConfig::default())
}

fn set(name: &str, value: f32) -> ParamRequest {
    ParamRequest::Set {
        name: ParamName::try_from(name).expect("name fits"),
        value,
    }
}

fn drain(replies: &mut Consumer<'_, ParamValue>) -> Vec<ParamValue> {
    core::iter::from_fn(|| replies.dequeue()).collect()
}

#[test]
fn burst_larger_than_the_write_limit_applies_over_successive_polls_in_order() {
    let mut requests = Queue::<ParamRequest, 16>::new();
    let mut replies = Queue::<ParamValue, 16>::new();
    let (mut commander, requests) = requests.split();
    let (replies, mut publisher) = replies.split();
    let mut server = ParamServer::new(requests, replies, 2);
    let mut params = stabilizer();

    let sent = [0.1, 0.2, 0.3, 0.4, 0.5];
    for value in sent {
        commander.enqueue(set("STAB_RATE_P", value)).unwrap();
    }

    let mut echoes = Vec::new();
    for expected_writes in [2, 2, 1, 0] {
        server.poll(&mut params);
        let batch = drain(&mut publisher);
        check!(batch.len() == expected_writes);
        echoes.extend(batch.into_iter().map(|reply| reply.value));
    }
    check!(echoes == sent);
    check!(params.param("STAB_RATE_P") == Some(0.5));
}

#[test]
fn listing_streams_every_param_in_index_order_as_replies_drain() {
    const ROOM: usize = 4;
    let mut requests = Queue::<ParamRequest, 4>::new();
    let mut replies = Queue::<ParamValue, { ROOM + 1 }>::new();
    let (mut commander, requests) = requests.split();
    let (replies, mut publisher) = replies.split();
    let mut server = ParamServer::new(requests, replies, 1);
    let mut params = stabilizer();
    let count = params.param_count();

    commander.enqueue(ParamRequest::List).unwrap();
    let mut listing = Vec::new();
    let mut polls = 0;
    loop {
        server.poll(&mut params);
        let batch = drain(&mut publisher);
        if batch.is_empty() {
            break;
        }
        check!(batch.len() <= ROOM);
        listing.extend(batch);
        polls += 1;
    }

    check!(polls == count.div_ceil(ROOM));
    check!(listing.len() == count);
    for (i, reply) in listing.iter().enumerate() {
        check!(usize::from(reply.index) == i);
        check!(usize::from(reply.count) == count);
        check!(Some(reply.name.as_str()) == params.param_name(i));
        check!(Some(reply.value) == params.param_at(i));
    }
}

#[test]
fn writes_echo_the_value_sent_or_the_current_value() {
    let default_imax = stabilizer().param("STAB_RATE_IMAX").expect("exists");
    // (case, request, echoed value: None = no reply)
    let cases = [
        (
            "accepted degree param echoes the value sent",
            set("STAB_ATT_AMAX_X", 30.0),
            Some(30.0),
        ),
        (
            "rejected value echoes the current value",
            set("STAB_RATE_IMAX", -1.0),
            Some(default_imax),
        ),
        (
            "non-finite value echoes the current value",
            set("STAB_RATE_IMAX", f32::NAN),
            Some(default_imax),
        ),
        (
            "unknown name gets no reply",
            set("NO_SUCH_PARAM", 1.0),
            None,
        ),
    ];
    for (case, request, echo) in cases {
        let mut requests = Queue::<ParamRequest, 4>::new();
        let mut replies = Queue::<ParamValue, 4>::new();
        let (mut commander, requests) = requests.split();
        let (replies, mut publisher) = replies.split();
        let mut server = ParamServer::new(requests, replies, 1);
        let mut params = stabilizer();

        commander.enqueue(request).unwrap();
        server.poll(&mut params);
        let reply = publisher.dequeue();
        check!(reply.as_ref().map(|r| r.value) == echo, "{case}");
        check!(publisher.dequeue() == None, "{case}");
    }
}

#[test]
fn degree_echo_is_exact_where_the_read_back_is_not() {
    let mut requests = Queue::<ParamRequest, 4>::new();
    let mut replies = Queue::<ParamValue, 4>::new();
    let (mut commander, requests) = requests.split();
    let (replies, mut publisher) = replies.split();
    let mut server = ParamServer::new(requests, replies, 1);
    let mut params = stabilizer();

    commander.enqueue(set("STAB_ATT_AMAX_X", 30.0)).unwrap();
    server.poll(&mut params);

    assert!(let Some(echo) = publisher.dequeue());
    check!(echo.value == 30.0);
    check!(params.param("STAB_ATT_AMAX_X") != Some(30.0)); // 29.999998 after deg → rad → deg
}

#[test]
fn full_reply_queue_holds_writes_back_until_it_drains() {
    let mut requests = Queue::<ParamRequest, 4>::new();
    let mut replies = Queue::<ParamValue, 2>::new(); // room for one
    let (mut commander, requests) = requests.split();
    let (replies, mut publisher) = replies.split();
    let mut server = ParamServer::new(requests, replies, 4);
    let mut params = stabilizer();

    commander.enqueue(set("STAB_RATE_P", 0.1)).unwrap();
    commander.enqueue(set("STAB_RATE_P", 0.2)).unwrap();

    server.poll(&mut params);
    check!(params.param("STAB_RATE_P") == Some(0.1));
    server.poll(&mut params); // reply queue still full: nothing applied
    check!(params.param("STAB_RATE_P") == Some(0.1));

    check!(publisher.dequeue().map(|r| r.value) == Some(0.1));
    server.poll(&mut params);
    check!(params.param("STAB_RATE_P") == Some(0.2));
    check!(publisher.dequeue().map(|r| r.value) == Some(0.2));
}

#[test]
fn list_request_restarts_a_listing_in_progress() {
    let mut requests = Queue::<ParamRequest, 4>::new();
    let mut replies = Queue::<ParamValue, 4>::new(); // room for three
    let (mut commander, requests) = requests.split();
    let (replies, mut publisher) = replies.split();
    let mut server = ParamServer::new(requests, replies, 1);
    let mut params = stabilizer();
    let indices = |batch: Vec<ParamValue>| batch.iter().map(|r| r.index).collect::<Vec<_>>();

    commander.enqueue(ParamRequest::List).unwrap();
    server.poll(&mut params);
    check!(indices(drain(&mut publisher)) == [0, 1, 2]);
    server.poll(&mut params);
    check!(indices(drain(&mut publisher)) == [3, 4, 5]);

    commander.enqueue(ParamRequest::List).unwrap();
    server.poll(&mut params);
    check!(indices(drain(&mut publisher)) == [0, 1, 2]);
}

#[test]
#[should_panic(expected = "max_writes_per_poll must be > 0")]
fn zero_write_limit_is_rejected_at_construction() {
    let mut requests = Queue::<ParamRequest, 2>::new();
    let mut replies = Queue::<ParamValue, 2>::new();
    let (_, requests) = requests.split();
    let (replies, _) = replies.split();
    ParamServer::new(requests, replies, 0);
}
