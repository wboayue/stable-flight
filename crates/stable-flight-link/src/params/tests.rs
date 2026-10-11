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

/// A server over a real stabilizer, with the commander's and publisher's queue ends.
struct Harness<'a> {
    commander: Producer<'a, ParamRequest>,
    server: ParamServer<'a>,
    publisher: Consumer<'a, ParamValue>,
    params: CopterStabilizer,
}

impl Harness<'_> {
    fn send(&mut self, request: ParamRequest) {
        self.commander
            .enqueue(request)
            .expect("request queue has room");
    }

    fn poll(&mut self) {
        self.server.poll(&mut self.params);
    }

    fn drain(&mut self) -> Vec<ParamValue> {
        core::iter::from_fn(|| self.publisher.dequeue()).collect()
    }
}

/// Runs `test` against a server applying `max_writes` per poll, whose reply queue holds
/// `REPLIES - 1` (heapless keeps one slot free).
fn with_server<const REPLIES: usize>(max_writes: usize, test: impl FnOnce(&mut Harness)) {
    let mut requests = Queue::<ParamRequest, 16>::new();
    let mut replies = Queue::<ParamValue, REPLIES>::new();
    let (commander, requests) = requests.split();
    let (replies, publisher) = replies.split();
    test(&mut Harness {
        commander,
        server: ParamServer::new(requests, replies, max_writes),
        publisher,
        params: stabilizer(),
    });
}

#[test]
fn burst_larger_than_the_write_limit_applies_over_successive_polls_in_order() {
    with_server::<16>(2, |h| {
        let sent = [0.1, 0.2, 0.3, 0.4, 0.5];
        for value in sent {
            h.send(set("STAB_RATE_RLL_P", value));
        }

        let mut echoes = Vec::new();
        for expected_writes in [2, 2, 1, 0] {
            h.poll();
            let batch = h.drain();
            check!(batch.len() == expected_writes);
            echoes.extend(batch.into_iter().map(|reply| reply.value));
        }
        check!(echoes == sent);
        check!(h.params.param("STAB_RATE_RLL_P") == Some(0.5));
    });
}

#[test]
fn listing_streams_every_param_in_index_order_as_replies_drain() {
    const ROOM: usize = 4;
    with_server::<{ ROOM + 1 }>(1, |h| {
        let count = h.params.param_count();
        h.send(ParamRequest::List);

        let mut listing = Vec::new();
        let mut polls = 0;
        loop {
            h.poll();
            let batch = h.drain();
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
            check!(Some(reply.name.as_str()) == h.params.param_name(i));
            check!(Some(reply.value) == h.params.param_at(i));
        }
    });
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
        with_server::<4>(1, |h| {
            h.send(request);
            h.poll();
            let echoes: Vec<_> = h.drain().into_iter().map(|reply| reply.value).collect();
            check!(echoes == echo.into_iter().collect::<Vec<_>>(), "{case}");
        });
    }
}

#[test]
fn degree_echo_is_exact_where_the_read_back_is_not() {
    with_server::<4>(1, |h| {
        h.send(set("STAB_ATT_AMAX_X", 30.0));
        h.poll();

        let replies = h.drain();
        assert!(let [echo] = replies.as_slice());
        check!(echo.value == 30.0);
        check!(h.params.param("STAB_ATT_AMAX_X") != Some(30.0)); // 29.999998 after deg → rad → deg
    });
}

#[test]
fn full_reply_queue_holds_writes_back_until_it_drains() {
    with_server::<2>(4, |h| {
        // Room for one reply.
        h.send(set("STAB_RATE_RLL_P", 0.1));
        h.send(set("STAB_RATE_RLL_P", 0.2));

        h.poll();
        check!(h.params.param("STAB_RATE_RLL_P") == Some(0.1));
        h.poll(); // reply queue still full: nothing applied
        check!(h.params.param("STAB_RATE_RLL_P") == Some(0.1));

        check!(h.publisher.dequeue().map(|r| r.value) == Some(0.1));
        h.poll();
        check!(h.params.param("STAB_RATE_RLL_P") == Some(0.2));
        check!(h.publisher.dequeue().map(|r| r.value) == Some(0.2));
    });
}

#[test]
fn list_request_restarts_a_listing_in_progress() {
    with_server::<4>(1, |h| {
        // Room for three replies.
        let next_indices = |h: &mut Harness| {
            h.poll();
            h.drain().iter().map(|r| r.index).collect::<Vec<_>>()
        };

        h.send(ParamRequest::List);
        check!(next_indices(h) == [0, 1, 2]);
        check!(next_indices(h) == [3, 4, 5]);

        h.send(ParamRequest::List);
        check!(next_indices(h) == [0, 1, 2]);
    });
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
