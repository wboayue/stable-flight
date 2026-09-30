//! Graceful-shutdown flag shared between an OS signal handler and a runner.

use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// A shutdown request flag: set by SIGINT (Unix) / Ctrl-C or Ctrl-Break (Windows) once
/// [`install`](ShutdownSignal::install)ed, or by hand with [`trigger`](ShutdownSignal::trigger).
///
/// Cheap to clone; clones share the same flag. Runners poll [`is_set`](ShutdownSignal::is_set)
/// between ticks. Keeping this separate from the runners means construction has no side
/// effects, tests can trip it directly, and every runner reuses one handler.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl::ShutdownSignal;
///
/// let shutdown = ShutdownSignal::new(); // not connected to any OS signal
/// assert!(!shutdown.is_set());
/// shutdown.trigger();
/// assert!(shutdown.is_set());
/// ```
#[derive(Debug, Clone, Default)]
pub struct ShutdownSignal {
    requested: Arc<AtomicBool>,
}

impl ShutdownSignal {
    /// A flag that only [`trigger`](ShutdownSignal::trigger) can set. Registers nothing.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns a flag that the process's interrupt signal sets: SIGINT on Unix, Ctrl-C or
    /// Ctrl-Break on Windows.
    ///
    /// The first interrupt only requests a graceful stop. A second one, while the flag is
    /// already set, terminates the process (exit status 1 on Unix; the default console
    /// handler on Windows), so a wedged loop can still be killed.
    ///
    /// On Unix each call registers an independent flag. On Windows the console handler is
    /// process-wide, so every call returns a handle to the same flag.
    ///
    /// # Errors
    ///
    /// Returns the OS error if the handler can't be registered.
    pub fn install() -> io::Result<Self> {
        platform::install()
    }

    /// Requests shutdown.
    pub fn trigger(&self) {
        self.requested.store(true, Ordering::Release);
    }

    /// Whether shutdown has been requested.
    pub fn is_set(&self) -> bool {
        self.requested.load(Ordering::Acquire)
    }
}

#[cfg(unix)]
mod platform {
    use std::io;

    use signal_hook::consts::SIGINT;

    use super::ShutdownSignal;

    pub(super) fn install() -> io::Result<ShutdownSignal> {
        let signal = ShutdownSignal::new();
        // Order matters: the conditional shutdown sees the flag before this SIGINT sets it,
        // so the first SIGINT only sets the flag and a second one exits.
        signal_hook::flag::register_conditional_shutdown(SIGINT, 1, signal.requested.clone())?;
        signal_hook::flag::register(SIGINT, signal.requested.clone())?;
        Ok(signal)
    }
}

#[cfg(windows)]
mod platform {
    use std::io;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, OnceLock};

    use windows_sys::Win32::System::Console::{
        CTRL_BREAK_EVENT, CTRL_C_EVENT, SetConsoleCtrlHandler,
    };

    use super::ShutdownSignal;

    /// The process-wide flag the console handler sets; created on first install.
    static REQUESTED: OnceLock<Arc<AtomicBool>> = OnceLock::new();
    /// Registration result (OS error code on failure), so the handler is added exactly once
    /// and later installs report the original failure.
    static REGISTERED: OnceLock<Result<(), i32>> = OnceLock::new();

    extern "system" fn handler(event: u32) -> i32 {
        if (event == CTRL_C_EVENT || event == CTRL_BREAK_EVENT)
            && let Some(requested) = REQUESTED.get()
            // A repeat while already requested falls through, so the default handler
            // terminates the process.
            && !requested.swap(true, Ordering::AcqRel)
        {
            return 1; // handled: graceful stop requested
        }
        0 // not handled: fall through to the default handler
    }

    pub(super) fn install() -> io::Result<ShutdownSignal> {
        let requested = REQUESTED.get_or_init(Default::default).clone();
        let registered = *REGISTERED.get_or_init(|| {
            // SAFETY: `handler` is a valid `extern "system"` function for the process lifetime.
            if unsafe { SetConsoleCtrlHandler(Some(handler), 1) } != 0 {
                Ok(())
            } else {
                Err(io::Error::last_os_error().raw_os_error().unwrap_or(0))
            }
        });
        registered.map_err(io::Error::from_raw_os_error)?;
        Ok(ShutdownSignal { requested })
    }
}
