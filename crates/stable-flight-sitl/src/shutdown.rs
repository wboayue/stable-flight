//! Graceful-shutdown flag shared between an OS signal handler and a runner.

use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// A shutdown request flag: set by SIGINT or SIGTERM (Unix) / Ctrl-C or Ctrl-Break (Windows) once
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

    /// Returns a flag that the process's stop signals set: SIGINT (Ctrl-C) and SIGTERM (`kill`,
    /// systemd, process managers) on Unix, Ctrl-C or Ctrl-Break on Windows.
    ///
    /// The first stop signal only requests a graceful stop, so the loop ends between ticks and
    /// everything it owns is dropped (e.g. a simulator vehicle hands control back, a capture
    /// file flushes). A second one, of either kind, while the flag is already set terminates
    /// the process (exit status 1 on Unix; the default console handler on Windows), so a
    /// wedged loop can still be killed.
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

    use signal_hook::consts::{SIGINT, SIGTERM};

    use super::ShutdownSignal;

    /// The signals that request a graceful stop: Ctrl-C, and what `kill` and process managers
    /// send.
    const STOP_SIGNALS: [i32; 2] = [SIGINT, SIGTERM];

    pub(super) fn install() -> io::Result<ShutdownSignal> {
        let signal = ShutdownSignal::new();
        // Order matters: every conditional shutdown is registered before any flag setter, so
        // each sees the flag before the signal sets it. The first stop signal only sets the
        // flag; a second one, of either kind, exits.
        for sig in STOP_SIGNALS {
            signal_hook::flag::register_conditional_shutdown(sig, 1, signal.requested.clone())?;
        }
        for sig in STOP_SIGNALS {
            signal_hook::flag::register(sig, signal.requested.clone())?;
        }
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
