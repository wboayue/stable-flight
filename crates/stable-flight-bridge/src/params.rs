//! Params between MAVLink and the link: see [`ParamService`].

use mavlink::common::*;
use mavlink::types::CharArray;
use stable_flight_link::{ParamName, ParamRequest, ParamValue, Uplink};

use crate::encode;
use crate::outbox::Outbox;

#[cfg(test)]
mod tests;

/// The last [`ParamValue`] the flight controller sent for each index, to answer
/// `PARAM_REQUEST_READ` without a round trip. Filled from listings and write echoes.
#[derive(Debug, Default)]
struct ParamCache {
    /// One slot per index, `0..count`; `None` until that index arrives.
    values: Vec<Option<ParamValue>>,
}

impl ParamCache {
    /// Stores `param` at its index. A `count` other than the cached one means a different
    /// table (the flight controller restarted), so the cache starts over at the new size. An
    /// index outside `0..count` is ignored.
    fn insert(&mut self, param: ParamValue) {
        if usize::from(param.count) != self.values.len() {
            self.values = vec![None; usize::from(param.count)];
        }
        if let Some(slot) = self.values.get_mut(usize::from(param.index)) {
            *slot = Some(param);
        }
    }

    /// The param at `index`, if it has arrived.
    fn get(&self, index: usize) -> Option<&ParamValue> {
        self.values.get(index)?.as_ref()
    }

    /// The param named `name`, if it has arrived.
    fn find(&self, name: &str) -> Option<&ParamValue> {
        self.values.iter().flatten().find(|p| p.name == name)
    }

    /// Forgets every param.
    fn clear(&mut self) {
        self.values.clear();
    }
}

/// A MAVLink `param_id` as a link [`ParamName`]: the bytes up to the first NUL (all 16 when
/// there is none). `None` if they aren't UTF-8 or are empty.
fn param_name(param_id: &CharArray<16>) -> Option<ParamName> {
    let name = param_id.to_str().ok().filter(|name| !name.is_empty())?;
    ParamName::try_from(name).ok()
}

/// Translates params between the ground station and the flight controller, which is the only
/// judge of a value: listings and writes go to it, and every param it sends (a listing entry or
/// a write's echo, which carries the index) goes out unchanged as a `REAL32` `PARAM_VALUE`. A
/// cache of those answers `PARAM_REQUEST_READ`; a miss gets no reply, and the ground station
/// retries. A non-`REAL32` write is rejected by echoing the cached value.
#[derive(Debug, Default)]
pub(crate) struct ParamService {
    cache: ParamCache,
}

impl ParamService {
    /// The flight controller appeared (or came back): its table may have changed, so forget
    /// the cache and list again.
    pub(crate) fn fc_appeared(&mut self, out: &mut Outbox) {
        self.cache.clear();
        out.to_fc.push(Uplink::Param(ParamRequest::List));
    }

    /// A param from the flight controller: forward it and cache it.
    pub(crate) fn on_param(&mut self, param: &ParamValue, out: &mut Outbox) {
        out.to_gcs
            .push(MavMessage::PARAM_VALUE(encode::param_value(param)));
        self.cache.insert(param.clone());
    }

    /// `PARAM_REQUEST_LIST`: the flight controller streams every param.
    pub(crate) fn on_request_list(&self, out: &mut Outbox) {
        out.to_fc.push(Uplink::Param(ParamRequest::List));
    }

    /// `PARAM_REQUEST_READ`, by index or, with a negative index, by name: from the cache.
    pub(crate) fn on_request_read(&self, data: &PARAM_REQUEST_READ_DATA, out: &mut Outbox) {
        let cached = match usize::try_from(data.param_index) {
            Ok(index) => self.cache.get(index),
            Err(_) => param_name(&data.param_id).and_then(|name| self.cache.find(&name)),
        };
        out.to_gcs
            .extend(cached.map(|p| MavMessage::PARAM_VALUE(encode::param_value(p))));
    }

    /// `PARAM_SET`: a `REAL32` write goes to the flight controller, even for a name the cache
    /// hasn't seen (the flight controller decides); another type is rejected with the cached
    /// value. A write without a name is ignored.
    pub(crate) fn on_set(&self, data: &PARAM_SET_DATA, out: &mut Outbox) {
        let Some(name) = param_name(&data.param_id) else {
            return;
        };
        if data.param_type == MavParamType::MAV_PARAM_TYPE_REAL32 {
            out.to_fc.push(Uplink::Param(ParamRequest::Set {
                name,
                value: data.param_value,
            }));
        } else if let Some(current) = self.cache.find(&name) {
            out.to_gcs
                .push(MavMessage::PARAM_VALUE(encode::param_value(current)));
        }
    }
}
