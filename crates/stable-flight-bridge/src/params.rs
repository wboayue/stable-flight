//! The bridge's copy of the flight controller's params, and MAVLink `param_id` conversion.

use mavlink::types::CharArray;
use stable_flight_link::{ParamName, ParamValue};

#[cfg(test)]
mod tests;

/// The last [`ParamValue`] the flight controller sent for each index, to answer
/// `PARAM_REQUEST_READ` without a round trip. Filled from listings and write echoes.
#[derive(Debug, Default)]
pub(crate) struct ParamCache {
    /// One slot per index, `0..count`; `None` until that index arrives.
    values: Vec<Option<ParamValue>>,
}

impl ParamCache {
    /// Stores `param` at its index. A `count` other than the cached one means a different
    /// table (the flight controller restarted), so the cache starts over at the new size. An
    /// index outside `0..count` is ignored.
    pub(crate) fn insert(&mut self, param: ParamValue) {
        if usize::from(param.count) != self.values.len() {
            self.values = vec![None; usize::from(param.count)];
        }
        if let Some(slot) = self.values.get_mut(usize::from(param.index)) {
            *slot = Some(param);
        }
    }

    /// The param at `index`, if it has arrived.
    pub(crate) fn get(&self, index: usize) -> Option<&ParamValue> {
        self.values.get(index)?.as_ref()
    }

    /// The param named `name`, if it has arrived.
    pub(crate) fn find(&self, name: &str) -> Option<&ParamValue> {
        self.values.iter().flatten().find(|p| p.name == name)
    }

    /// Forgets every param.
    pub(crate) fn clear(&mut self) {
        self.values.clear();
    }
}

/// A MAVLink `param_id` as a link [`ParamName`]: the bytes up to the first NUL (all 16 when
/// there is none). `None` if they aren't UTF-8 or are empty.
pub(crate) fn param_name(param_id: &CharArray<16>) -> Option<ParamName> {
    let name = param_id.to_str().ok().filter(|name| !name.is_empty())?;
    ParamName::try_from(name).ok()
}
