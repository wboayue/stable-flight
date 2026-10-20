//! Command-line helpers shared by the simulator examples.

use core::num::ParseFloatError;

use thiserror::Error;

/// Why a `NAME=VALUE` param argument didn't parse.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum ParamArgError {
    /// The argument has no `=`.
    #[error("expected NAME=VALUE")]
    MissingEquals,
    /// The part after `=` isn't a number.
    #[error("invalid value: {0}")]
    InvalidValue(#[from] ParseFloatError),
}

/// Parses a `--param` argument, `NAME=VALUE`, into the param's name and value, trimming
/// whitespace around each; the examples set it before takeoff, as the link would. A clap
/// `value_parser`.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl::cli::{ParamArgError, parse_param};
///
/// assert_eq!(parse_param("GUID_TILT_MAX=35"), Ok(("GUID_TILT_MAX".to_owned(), 35.0)));
/// assert_eq!(parse_param("GUID_TILT_MAX"), Err(ParamArgError::MissingEquals));
/// ```
pub fn parse_param(arg: &str) -> Result<(String, f32), ParamArgError> {
    let (name, value) = arg.split_once('=').ok_or(ParamArgError::MissingEquals)?;
    let value = value.trim().parse::<f32>()?;
    Ok((name.trim().to_owned(), value))
}

#[cfg(test)]
mod tests;
