//! QGC's component metadata for the flight controller's params, built from the core's tables:
//! see [`MetadataFiles`]. Pure: no I/O; [`crate::http`] serves the files.

use std::fmt;
use std::net::SocketAddr;

use crc::{CRC_32_ISO_HDLC, Crc};
use serde_json::{Map, Value, json};
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::params::metadata::{ParamMeta, ParamMetadata};
use stable_flight::stabilizers::copter::CopterStabilizerConfig;

#[cfg(test)]
mod tests;

/// The checksum QGC caches metadata files by. It never verifies it, so it only has to change
/// when the content does; this is the CRC-32 the MAVLink spec names.
const FILE_CRC: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

/// `COMP_METADATA_TYPE_PARAMETER` in `component_general.json`'s `metadataTypes`.
const COMP_METADATA_TYPE_PARAMETER: u32 = 1;

/// Longest URI `COMPONENT_METADATA` carries (its `uri` field).
pub(crate) const MAX_URI_LEN: usize = 100;

/// Every param a copter flight controller serves: its stabilizer's, then its guidance's. A
/// second airframe will pick its components' tables here.
pub(crate) fn copter_params() -> impl Iterator<Item = &'static ParamMeta> {
    CopterStabilizerConfig::METADATA
        .iter()
        .chain(CopterGuidanceConfig::METADATA)
}

/// One file to serve: where, what, and its CRC.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MetadataFile {
    /// URL path, `/<crc>/<name>`: versioned by content, because QGC caches an HTTP download by
    /// URL and never fetches the same URL again.
    pub(crate) path: String,
    pub(crate) bytes: Vec<u8>,
    pub(crate) crc: u32,
}

impl MetadataFile {
    fn new(name: &str, json: &Value) -> Self {
        let bytes = json.to_string().into_bytes();
        let crc = FILE_CRC.checksum(&bytes);
        Self {
            path: format!("/{crc}/{name}"),
            bytes,
            crc,
        }
    }
}

/// What the bridge serves so QGC can describe the params: `parameters.json` from the core's
/// metadata tables, and `component_general.json` pointing to it. QGC asks for the general
/// file through `COMPONENT_METADATA` ([`MetadataFiles::component_metadata`]).
///
/// Only the keys QGC's parameter schema accepts are written (it drops a param's metadata on any
/// other): `name`, `type` (`"Float"`: every param is `REAL32` on the wire), `shortDesc`,
/// `category`, `group`, and when present `units`, `min`, and `max`. An exclusive bound is sent
/// as its value; the flight controller still rejects the value itself. No defaults yet: TODO(#229).
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MetadataFiles {
    pub(crate) parameters: MetadataFile,
    pub(crate) general: MetadataFile,
    /// Where QGC fetches the general file.
    pub(crate) general_uri: String,
}

impl MetadataFiles {
    /// The files for `params`, with URIs under `base_url` (e.g. `http://127.0.0.1:14580`, as
    /// QGC reaches the bridge).
    ///
    /// # Errors
    ///
    /// [`UriTooLong`] if the general file's URI doesn't fit `COMPONENT_METADATA`.
    pub(crate) fn build<'a>(
        base_url: &str,
        params: impl IntoIterator<Item = &'a ParamMeta>,
    ) -> Result<Self, UriTooLong> {
        let base_url = base_url.trim_end_matches('/');
        let parameters = MetadataFile::new(
            "parameters.json",
            &json!({
                "version": 1,
                "parameters": params.into_iter().map(param_json).collect::<Vec<_>>(),
            }),
        );
        let general = MetadataFile::new(
            "component_general.json",
            &json!({
                "version": 1,
                "metadataTypes": [{
                    "type": COMP_METADATA_TYPE_PARAMETER,
                    "uri": format!("{base_url}{}", parameters.path),
                    "fileCrc": parameters.crc,
                }],
            }),
        );
        let general_uri = format!("{base_url}{}", general.path);
        if general_uri.len() > MAX_URI_LEN {
            return Err(UriTooLong(general_uri));
        }
        Ok(Self {
            parameters,
            general,
            general_uri,
        })
    }
}

impl MetadataFiles {
    /// What `COMPONENT_METADATA` carries: the general file's URI and CRC.
    pub(crate) fn component_metadata(&self) -> ComponentMetadata {
        // `build` checked the URI fits.
        let mut uri = [0u8; MAX_URI_LEN];
        uri[..self.general_uri.len()].copy_from_slice(self.general_uri.as_bytes());
        ComponentMetadata {
            uri,
            crc: self.general.crc,
        }
    }
}

/// `COMPONENT_METADATA`'s payload: where the general metadata file is, and its CRC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ComponentMetadata {
    /// The URI, NUL-padded, as the message's `uri` field.
    pub(crate) uri: [u8; MAX_URI_LEN],
    pub(crate) crc: u32,
}

/// One entry of `parameters.json`.
fn param_json(meta: &ParamMeta) -> Value {
    let mut param = Map::new();
    param.insert("name".into(), meta.name().into());
    param.insert("type".into(), "Float".into());
    param.insert("shortDesc".into(), meta.description().into());
    param.insert("category".into(), meta.category().into());
    param.insert("group".into(), meta.group().into());
    if !meta.units().is_empty() {
        param.insert("units".into(), meta.units().into());
    }
    if let Some(min) = meta.min() {
        param.insert("min".into(), min.value().into());
    }
    if let Some(max) = meta.max() {
        param.insert("max".into(), max.value().into());
    }
    Value::Object(param)
}

/// The base URL QGC fetches the metadata from: `http_url` if given, else the server's own
/// `http_bind` address.
///
/// # Errors
///
/// [`UnreachableUrl`] when `http_bind` listens on every interface (`0.0.0.0`, `::`) and no
/// `http_url` names one: QGC can't fetch from an unspecified address, and it only logs the
/// failure, so the params would silently show no descriptions.
pub(crate) fn base_url(http_bind: &str, http_url: Option<&str>) -> Result<String, UnreachableUrl> {
    if let Some(url) = http_url {
        return Ok(url.to_owned());
    }
    match http_bind.parse::<SocketAddr>() {
        Ok(addr) if addr.ip().is_unspecified() => Err(UnreachableUrl(http_bind.to_owned())),
        _ => Ok(format!("http://{http_bind}")),
    }
}

/// Returned by [`base_url`]: the server listens on every interface, so its address isn't one
/// QGC can fetch from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct UnreachableUrl(pub(crate) String);

impl fmt::Display for UnreachableUrl {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "--http-bind {} listens on every interface; set --http-url to an address QGC can reach, \
             e.g. http://192.168.1.30:14580",
            self.0
        )
    }
}

impl std::error::Error for UnreachableUrl {}

/// Returned by [`MetadataFiles::build`]: the general file's URI is longer than
/// `COMPONENT_METADATA`'s 100 bytes. Use a shorter `--http-url`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct UriTooLong(pub(crate) String);

impl fmt::Display for UriTooLong {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "metadata URI {} is longer than COMPONENT_METADATA's {MAX_URI_LEN} bytes",
            self.0
        )
    }
}

impl std::error::Error for UriTooLong {}
