/*
 * The status panel: pattern rendering, the boot lamp test, and the two things
 * that make it trustworthy rather than decorative -- that a state nobody has
 * reported reads as the bad case, and that a pattern keeps its phase when the
 * dispatcher pushes the same state every pass.
 *
 * Everything here is clock-driven, so the stub's HAL_GetTick() is moved by hand
 * (g_stub_tick). Nothing else in the suite touches it, so it starts at 0 and
 * stays wherever a test leaves it.
 */
#include "Indicator.hpp"
#include "main.h"
#include "test.hpp"

namespace {

// Distinct pins so the stub can record all three independently, and one spare
// for the active-low channel.
const IndicatorLed SYS { GPIOE, GPIO_PIN_0, true };
const IndicatorLed ARM { GPIOE, GPIO_PIN_1, true };
const IndicatorLed GPS { GPIOE, GPIO_PIN_2, true };

// Advance the clock and render. update() derives its slot from the clock rather
// than counting calls, so one call per step is enough.
void tickTo(Indicator &ind, uint32_t ms)
{
    g_stub_tick = ms;
    ind.update();
}

// Render one whole 16-slot cycle starting at `base`, sampling the middle of
// each slot so a boundary rounding error cannot be mistaken for a pattern bug.
// Returns the rendered bits, LSB = slot 0 -- the same encoding as the pattern.
uint16_t renderCycle(Indicator &ind, uint32_t base, Indicator::Channel ch)
{
    uint16_t got = 0;
    for (uint16_t s = 0; s < INDICATOR_SLOTS; s++) {
        tickTo(ind, base + s * INDICATOR_SLOT_MS + INDICATOR_SLOT_MS / 2u);
        if (ind.isLit(ch)) got = (uint16_t)(got | (uint16_t)(1u << s));
    }
    return got;
}

// Past the lamp test, on a phase boundary, so patterns render from slot 0.
uint32_t settle(Indicator &ind, uint32_t start)
{
    const uint32_t after = start + INDICATOR_LAMP_TEST_MS;
    tickTo(ind, after);
    return after;
}

} // namespace

int main()
{
    section("Unreported state reads as the bad case");
    {
        // An operator who sees "OK" because nothing has reported yet is being
        // lied to, and this is the one property that cannot be added later --
        // by the time something calls a setter the damage is done.
        Indicator ind { SYS, ARM, GPS };
        check(ind.getSystemState() == SystemState::ERROR, "system starts ERROR, not OK");
        check(ind.getArmState() == ArmState::DISARMED, "arm starts DISARMED");
        check(ind.getGPSState() == GPSState::UNLOCKED, "GPS starts UNLOCKED");
    }

    section("update() before init() is inert");
    {
        Indicator ind { SYS, ARM, GPS };
        g_stub_tick = 5000;
        stubGpioReset();
        ind.update();
        check(stubGpioWriteCount() == 0, "no pin is driven before init() claims it");
    }

    section("Boot lamp test");
    {
        // Every healthy pattern is mostly dark, so without this there is no way
        // to tell a working LED reporting "fine" from a dead one.
        Indicator ind { SYS, ARM, GPS };
        g_stub_tick = 1000;
        ind.init();
        check(ind.inLampTest(), "init() starts the lamp test");
        check(ind.isLit(Indicator::CHANNEL_SYSTEM)
              && ind.isLit(Indicator::CHANNEL_ARM)
              && ind.isLit(Indicator::CHANNEL_GPS), "all three light immediately");

        tickTo(ind, 1000 + INDICATOR_LAMP_TEST_MS - 1);
        check(ind.inLampTest(), "still lit one millisecond before the end");
        check(ind.isLit(Indicator::CHANNEL_GPS), "... and really still lit");

        tickTo(ind, 1000 + INDICATOR_LAMP_TEST_MS);
        check(!ind.inLampTest(), "and ends exactly on time");
    }

    section("Patterns render as documented");
    {
        Indicator ind { SYS, ARM, GPS };
        g_stub_tick = 0;
        ind.init();
        const uint32_t t0 = settle(ind, 0);

        // Default states: ERROR / DISARMED / UNLOCKED.
        check(renderCycle(ind, t0, Indicator::CHANNEL_SYSTEM) == INDICATOR_PAT_SYS_ERROR,
              "ERROR renders the 100 ms strobe");
        check(renderCycle(ind, t0, Indicator::CHANNEL_ARM) == INDICATOR_PAT_OFF,
              "DISARMED is DARK -- no light means no live props");
        check(renderCycle(ind, t0, Indicator::CHANNEL_GPS) == INDICATOR_PAT_GPS_SEARCH,
              "UNLOCKED renders the searching double blink");
    }
    {
        Indicator ind { SYS, ARM, GPS };
        g_stub_tick = 0;
        ind.init();
        settle(ind, 0);

        // A state change restarts the cycle, so render from the change onward.
        g_stub_tick = 2000;
        ind.setSystemState(SystemState::OK);
        ind.setArmState(ArmState::ARMED);
        ind.setGPSState(GPSState::LOCKED);
        check(renderCycle(ind, 2000, Indicator::CHANNEL_SYSTEM) == INDICATOR_PAT_OFF,
              "OK is DARK -- the system LED only ever means trouble");
        check(renderCycle(ind, 2000, Indicator::CHANNEL_ARM) == INDICATOR_PAT_ARM_LIVE,
              "ARMED is solid -- props live is never a dark LED");
        check(renderCycle(ind, 2000, Indicator::CHANNEL_GPS) == INDICATOR_PAT_GPS_FIX,
              "LOCKED is solid");
    }

    section("Phase handling");
    {
        // THE bug this class is shaped to avoid. The dispatcher pushes state
        // every pass, so if a setter restarted the cycle unconditionally the
        // LED would sit at slot 0 for ever and every pattern would render as a
        // solid light -- the failure looks like "the LED works", which is why
        // it would survive a bench test.
        Indicator ind { SYS, ARM, GPS };
        g_stub_tick = 0;
        ind.init();
        const uint32_t t0 = settle(ind, 0);

        uint16_t got = 0;
        for (uint16_t s = 0; s < INDICATOR_SLOTS; s++) {
            ind.setSystemState(SystemState::ERROR);   // same state, every pass
            tickTo(ind, t0 + s * INDICATOR_SLOT_MS + INDICATOR_SLOT_MS / 2u);
            if (ind.isLit(Indicator::CHANNEL_SYSTEM)) got = (uint16_t)(got | (uint16_t)(1u << s));
        }
        check(got == INDICATOR_PAT_SYS_ERROR,
              "re-pushing an unchanged state does not pin the pattern to slot 0");
    }
    {
        // A real change, though, must be visible at once rather than waiting
        // out whatever was left of the old cycle.
        Indicator ind { SYS, ARM, GPS };
        g_stub_tick = 0;
        ind.init();
        settle(ind, 0);

        // Land mid-cycle on a slot the OK pattern has dark, then switch to
        // ERROR, whose slot 0 is lit.
        tickTo(ind, 1550);
        ind.setSystemState(SystemState::OK);
        tickTo(ind, 1950);                       // slot 4 of OK: dark
        check(!ind.isLit(Indicator::CHANNEL_SYSTEM), "mid-cycle and dark, as set up");
        ind.setSystemState(SystemState::ERROR);
        ind.update();
        check(ind.isLit(Indicator::CHANNEL_SYSTEM),
              "a change restarts the cycle and lights slot 0 immediately");
    }

    section("The GPIO is only touched on a change");
    {
        // At 50 Hz across three channels this is 150 bus accesses a second for
        // a panel that changes twice a minute.
        Indicator ind { SYS, ARM, GPS };
        g_stub_tick = 0;
        ind.init();
        const uint32_t t0 = settle(ind, 0);

        ind.setSystemState(SystemState::OK);      // dark
        ind.setArmState(ArmState::ARMED);         // solid
        ind.setGPSState(GPSState::UNLOCKED);      // the only channel with edges
        tickTo(ind, t0 + 1);
        stubGpioReset();

        // 20 renders inside one slot: nothing can legitimately change.
        for (uint32_t i = 0; i < 20u; i++) tickTo(ind, t0 + 2u + i);
        check(stubGpioWriteCount() == 0, "no writes while the rendered level holds");

        // ... and a real edge still gets through. Two states are now flat --
        // OK is dark and ARMED is solid -- so the searching GPS pattern is what
        // proves rendering did not simply stop. It goes dark at slot 2.
        tickTo(ind, t0 + 2u * INDICATOR_SLOT_MS + 50u);
        check(stubGpioWriteCount() > 0, "but a genuine edge is written");
    }

    section("Active-low wiring inverts the pin, not the meaning");
    {
        // Getting this wrong does not break anything -- it silently inverts
        // every pattern, so "healthy" becomes a near-solid light.
        const IndicatorLed low { GPIOE, GPIO_PIN_3, false };
        Indicator ind { low, ARM, GPS };
        g_stub_tick = 0;
        ind.init();
        settle(ind, 0);

        // A genuine transition, not a re-push. The constructor already starts
        // at ERROR, so setting ERROR again is a no-op that leaves the phase
        // where the lamp test put it -- and the slot landed on would then be
        // whatever the clock happened to say. Go healthy first, so the failure
        // below really does restart the cycle at slot 0.
        g_stub_tick = 1000;
        ind.setSystemState(SystemState::OK);
        ind.update();
        check(!ind.isLit(Indicator::CHANNEL_SYSTEM), "healthy: the system LED is dark");

        g_stub_tick = 3000;
        ind.setSystemState(SystemState::ERROR);
        ind.update();   // slot 0 of the ERROR strobe: lit

        check(ind.isLit(Indicator::CHANNEL_SYSTEM), "the channel reports lit");
        check(stubGpioLevel(GPIO_PIN_3) == GPIO_PIN_RESET,
              "... and an active-low LED is driven LOW to light it");
        tickTo(ind, 3000 + INDICATOR_SLOT_MS);   // slot 1 of the strobe: dark
        check(!ind.isLit(Indicator::CHANNEL_SYSTEM), "the channel reports dark");
        check(stubGpioLevel(GPIO_PIN_3) == GPIO_PIN_SET, "... and is driven HIGH");
    }

    section("The clock wrap is survivable");
    {
        // HAL_GetTick() wraps every ~49 days. All the arithmetic is on unsigned
        // values, so this is a property of the code rather than a hope, but it
        // costs one test to say so.
        Indicator ind { SYS, ARM, GPS };
        g_stub_tick = 0xFFFFFF00u;
        ind.init();
        settle(ind, 0xFFFFFF00u);
        const uint32_t base = 0xFFFFFF00u + INDICATOR_LAMP_TEST_MS;  // wraps
        check(renderCycle(ind, base, Indicator::CHANNEL_SYSTEM) == INDICATOR_PAT_SYS_ERROR,
              "the pattern renders correctly straight through the wrap");
    }

    section("What the dark-resting scheme costs, and what still proves liveness");
    {
        // Two of the three channels now rest dark, so a healthy, disarmed
        // airframe shows almost nothing. Pin what remains.
        Indicator ind { SYS, ARM, GPS };
        g_stub_tick = 0;
        ind.init();
        const uint32_t t0 = settle(ind, 0);

        ind.setSystemState(SystemState::OK);
        ind.setArmState(ArmState::DISARMED);
        check(renderCycle(ind, t0, Indicator::CHANNEL_SYSTEM) == 0,
              "healthy + disarmed: the system LED is dark for a whole cycle");
        check(renderCycle(ind, t0, Indicator::CHANNEL_ARM) == 0,
              "... and so is the arm LED");

        // ... but the GPS channel has NO dark state: searching blinks, locked
        // is solid. So the panel is never completely unlit while the loop runs,
        // and that LED is the de-facto liveness indicator now that the other
        // two rest dark. Anything that makes a GPS state dark removes the last
        // signal that separates "healthy" from "unpowered".
        ind.setGPSState(GPSState::UNLOCKED);
        check(renderCycle(ind, t0, Indicator::CHANNEL_GPS) != 0,
              "UNLOCKED still shows light -- the panel is not fully dark");
        ind.setGPSState(GPSState::LOCKED);
        check(renderCycle(ind, t0, Indicator::CHANNEL_GPS) == INDICATOR_PAT_GPS_FIX,
              "and LOCKED is solid, so neither GPS state is ever unlit");
    }

    return testReport("Indicator");
}
