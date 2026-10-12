//! Stable Flight firmware for the ARK FPV (STM32H743) — stub.
//!
//! Runs the core `FlightLoop` at [`LOOP_RATE_HZ`] on the SysTick monotonic with placeholder
//! components (see [`stub`]): no sensors, no motors. Board setup (clocks, LEDs, memory map)
//! comes from [ark-fpv-discovery](https://github.com/wboayue/ark-fpv-discovery).
//!
//! | Task          | Trigger                  | Prio | Role                                        |
//! |---------------|--------------------------|------|---------------------------------------------|
//! | `flight_step` | every [`LOOP_PERIOD_MS`] | 2    | one `FlightLoop` tick                       |
//! | `heartbeat`   | every 1 s                | 1    | toggle green; blue = on rate, red = off     |
//!
//! A panic disables interrupts and lights red alone. The IMU data-ready interrupt will pace
//! `flight_step` once the sensors land (#88).

#![no_std]
#![no_main]

mod stub;

use core::panic::PanicInfo;
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
    mixers::TableMixer,
    stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig},
};

use stub::{HoldCommander, HoldState, NullActuator};

systick_monotonic!(Mono, 1_000);

/// Core clock; SysTick runs from it.
const SYS_CLOCK_HZ: u32 = 400_000_000;
/// Flight loop rate. The 1 kHz monotonic schedules whole milliseconds, so it must divide 1000.
const LOOP_RATE_HZ: u32 = 250;
const LOOP_PERIOD_MS: u32 = 1_000 / LOOP_RATE_HZ;
const _: () = assert!(1_000 % LOOP_RATE_HZ == 0, "LOOP_RATE_HZ must divide 1000");
/// Ticks per heartbeat that still count as on-rate (one tick of phase slack either way).
const ON_RATE_TICKS: core::ops::RangeInclusive<u32> = LOOP_RATE_HZ - 1..=LOOP_RATE_HZ + 1;

/// Flight loop ticks since boot; `heartbeat` checks the rate from its per-second delta.
static TICKS: AtomicU32 = AtomicU32::new(0);
/// Ticks that finished after their next slot was due; any in a heartbeat window shows red.
static OVERRUNS: AtomicU32 = AtomicU32::new(0);

type Flight = FlightLoop<
    HoldCommander,
    ComposedVehicle<HoldState, NullActuator>,
    CopterGuidance,
    CopterStabilizer,
    TableMixer<4>,
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
        let mut core = cx.core;
        let dp: pac::Peripherals = cx.device;
        // Code runs from flash with wait states; the I-cache hides them. The D-cache stays
        // off until DMA buffers get a coherency plan.
        core.SCB.enable_icache();
        Mono::start(core.SYST, SYS_CLOCK_HZ);

        let pwr = dp.PWR.constrain();
        let vos = pwr.freeze();
        let ccdr = dp
            .RCC
            .constrain()
            .sys_ck(SYS_CLOCK_HZ.Hz())
            .freeze(vos, &dp.SYSCFG);

        // Active-low (LOW = lit): start all three HIGH (off). Red must stay on PE3: the panic
        // handler drives it directly.
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
            ComposedVehicle::new(HoldState, NullActuator),
            CopterGuidance::new(CopterGuidanceConfig::default()),
            CopterStabilizer::new(CopterStabilizerConfig::default()),
            TableMixer::quad_x(),
            (),
        );
        flight.prime();

        flight_step::spawn().ok();
        heartbeat::spawn().ok();

        (Shared {}, Local { flight, leds })
    }

    /// Steps the flight loop on an absolute schedule (`delay_until`), so work time doesn't
    /// stretch the period. A tick that runs into the next slot counts an overrun and drops the
    /// missed slots rather than running catch-up steps back to back, so the task always
    /// yields and a slow loop shows up as a low tick count.
    #[task(priority = 2, local = [flight])]
    async fn flight_step(cx: flight_step::Context) {
        let period = LOOP_PERIOD_MS.millis();
        let dt = Duration::from_millis(u64::from(LOOP_PERIOD_MS));
        let mut next = Mono::now();

        loop {
            cx.local.flight.step(dt);
            TICKS.fetch_add(1, Ordering::Relaxed);

            next += period;
            let now = Mono::now();
            if now >= next {
                OVERRUNS.fetch_add(1, Ordering::Relaxed);
                next = now + period;
            }
            Mono::delay_until(next).await;
        }
    }

    /// Toggles green every second, and lights blue if the flight loop ran [`LOOP_RATE_HZ`]
    /// times without an overrun in the last second, red if not.
    #[task(priority = 1, local = [leds])]
    async fn heartbeat(cx: heartbeat::Context) {
        let leds = cx.local.leds;
        let mut last_ticks = TICKS.load(Ordering::Relaxed);
        let mut last_overruns = OVERRUNS.load(Ordering::Relaxed);
        let mut green_on = false;
        let mut next = Mono::now();

        loop {
            next += 1_000.millis();
            Mono::delay_until(next).await;

            let ticks = TICKS.load(Ordering::Relaxed);
            let overruns = OVERRUNS.load(Ordering::Relaxed);
            let on_rate = ON_RATE_TICKS.contains(&ticks.wrapping_sub(last_ticks))
                && overruns == last_overruns;
            last_ticks = ticks;
            last_overruns = overruns;

            green_on = !green_on;
            // PinState::Low lights an LED.
            leds[LED_GREEN].set_state(PinState::from(!green_on));
            leds[LED_BLUE].set_state(PinState::from(!on_rate));
            leds[LED_RED].set_state(PinState::from(on_rate));
        }
    }
}

/// Stops everything and lights red alone (PE3 low, green/blue high), so a panic can't leave a
/// stale blue. Before `init` configures the LEDs these writes have no visible effect.
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    // SAFETY: interrupts are off and this never returns, so nothing else touches GPIOE; BSRR
    // writes are atomic set/reset of the named pins only.
    let gpioe = unsafe { &*pac::GPIOE::ptr() };
    gpioe
        .bsrr
        .write(|w| w.br3().set_bit().bs4().set_bit().bs5().set_bit());
    loop {
        cortex_m::asm::wfi();
    }
}
