//! Stable Flight firmware for the ARK FPV (STM32H743) — stub.
//!
//! Runs the core `FlightLoop` at [`LOOP_RATE_HZ`] on the SysTick monotonic with placeholder
//! components (see [`stub`]): no sensors, no motors. Board setup (clocks, LEDs, memory map)
//! comes from [ark-fpv-discovery](https://github.com/wboayue/ark-fpv-discovery).
//!
//! | Task          | Trigger          | Prio | Role                                          |
//! |---------------|------------------|------|-----------------------------------------------|
//! | `flight_step` | every 4 ms       | 2    | one `FlightLoop` tick                         |
//! | `heartbeat`   | every 1 s        | 1    | blink green; blue = loop on rate, red = off   |
//!
//! The IMU data-ready interrupt will pace `flight_step` once the sensors land (#88).

#![no_std]
#![no_main]

use panic_halt as _;

mod stub;

use core::sync::atomic::{AtomicU32, Ordering};
use core::time::Duration;

use stm32h7xx_hal::{
    gpio::{ErasedPin, Output, PinState, PushPull},
    pac,
    prelude::*,
};

use rtic_monotonics::systick::prelude::*;

use stable_flight::{
    ComposedVehicle, FlightLoop,
    guidance::copter::{CopterGuidance, CopterGuidanceConfig},
    mixers::QuadCopterMixer,
    stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig},
};

use stub::{HoldCommander, HoldEstimator, NullActuator};

systick_monotonic!(Mono, 1_000);

/// Core clock; SysTick runs from it.
const SYS_CLOCK_HZ: u32 = 400_000_000;
/// Flight loop rate.
const LOOP_RATE_HZ: u32 = 250;
const LOOP_PERIOD_MS: u32 = 1_000 / LOOP_RATE_HZ;
/// Ticks per heartbeat that still count as on-rate (one tick of phase slack either way).
const ON_RATE_TICKS: core::ops::RangeInclusive<u32> = LOOP_RATE_HZ - 1..=LOOP_RATE_HZ + 1;

/// Flight loop ticks since boot; `heartbeat` checks the rate from its per-second delta.
static TICKS: AtomicU32 = AtomicU32::new(0);

type Flight = FlightLoop<
    HoldCommander,
    ComposedVehicle<HoldEstimator, NullActuator>,
    CopterGuidance,
    CopterStabilizer,
    QuadCopterMixer,
    (),
>;

#[rtic::app(device = stm32h7xx_hal::pac, peripherals = true, dispatchers = [FDCAN1_IT0, FDCAN2_IT0])]
mod app {
    use super::*;

    // Status LED indices into `Local::leds` — ARK FPV pins PE3/PE4/PE5, active-low.
    const LED_RED: usize = 0;
    const LED_GREEN: usize = 1;
    const LED_BLUE: usize = 2;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        flight: Flight,
        leds: [ErasedPin<Output<PushPull>>; 3],
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let dp: pac::Peripherals = cx.device;
        Mono::start(cx.core.SYST, SYS_CLOCK_HZ);

        let pwr = dp.PWR.constrain();
        let vos = pwr.freeze();
        let ccdr = dp
            .RCC
            .constrain()
            .sys_ck(SYS_CLOCK_HZ.Hz())
            .freeze(vos, &dp.SYSCFG);

        // Active-low (LOW = lit): start all three HIGH (off).
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
        let leds = [
            gpioe
                .pe3
                .into_push_pull_output_in_state(PinState::High)
                .erase(),
            gpioe
                .pe4
                .into_push_pull_output_in_state(PinState::High)
                .erase(),
            gpioe
                .pe5
                .into_push_pull_output_in_state(PinState::High)
                .erase(),
        ];

        let mut flight = FlightLoop::new(
            HoldCommander,
            ComposedVehicle::new(HoldEstimator, NullActuator),
            CopterGuidance::new(CopterGuidanceConfig::default()),
            CopterStabilizer::new(CopterStabilizerConfig::default()),
            QuadCopterMixer::default(),
            (),
        );
        flight.prime();

        flight_step::spawn().ok();
        heartbeat::spawn().ok();

        (Shared {}, Local { flight, leds })
    }

    /// Steps the flight loop on an absolute schedule (`delay_until`), so work time doesn't
    /// stretch the period.
    #[task(priority = 2, local = [flight])]
    async fn flight_step(cx: flight_step::Context) {
        let period = LOOP_PERIOD_MS.millis();
        let dt = Duration::from_millis(u64::from(LOOP_PERIOD_MS));
        let mut next = Mono::now();

        loop {
            cx.local.flight.step(dt);
            TICKS.fetch_add(1, Ordering::Relaxed);

            next += period;
            Mono::delay_until(next).await;
        }
    }

    /// Blinks green once a second, and lights blue if the flight loop ran [`LOOP_RATE_HZ`]
    /// times in the last second, red if not.
    #[task(priority = 1, local = [leds])]
    async fn heartbeat(cx: heartbeat::Context) {
        let leds = cx.local.leds;
        let mut last = TICKS.load(Ordering::Relaxed);
        let mut green_on = false;

        loop {
            Mono::delay(1_000.millis()).await;

            let now = TICKS.load(Ordering::Relaxed);
            let on_rate = ON_RATE_TICKS.contains(&now.wrapping_sub(last));
            last = now;

            green_on = !green_on;
            // PinState::Low lights an LED.
            leds[LED_GREEN].set_state(PinState::from(!green_on));
            leds[LED_BLUE].set_state(PinState::from(!on_rate));
            leds[LED_RED].set_state(PinState::from(on_rate));
        }
    }
}
