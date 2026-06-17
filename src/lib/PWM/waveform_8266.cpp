#if defined(PLATFORM_ESP8266)
/***
 * Slimmed-down version of esp8266_waveform for the ExpressLRS project
 * 2022-04-27 Created by CapnBry
 * - Adds the ability to change every servo no matter where it is in the
 *   cycle without blocking.
 * - Removes analogWrite() functionality. If this is used, it will break
 *   this code, as the standard core_esp8266_waveform_pwm will take over
 *   the timer.
 ***/
/*
  esp8266_waveform - General purpose waveform generation and control,
                     supporting outputs on all pins in parallel.

  Copyright (c) 2018 Earle F. Philhower, III.  All rights reserved.

  The core idea is to have a programmable waveform generator with a unique
  high and low period (defined in microseconds or CPU clock cycles).  TIMER1
  is set to 1-shot mode and is always loaded with the time until the next
  edge of any live waveforms.

  Up to one waveform generator per pin supported.

  Each waveform generator is synchronized to the ESP clock cycle counter, not
  the timer.  This allows for removing interrupt jitter and delay as the
  counter always increments once per 80MHz clock.  Changes to a waveform are
  contiguous and only take effect on the next waveform transition,
  allowing for smooth transitions.

  This replaces older tone(), analogWrite(), and the Servo classes.

  Everywhere in the code where "cycles" is used, it means ESP.getCycleCount()
  clock cycle count, or an interval measured in CPU clock cycles, but not
  TIMER1 cycles (which may be 2 CPU clock cycles @ 160MHz).

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/


#include <Arduino.h>
#include "ets_sys.h"
#include "user_interface.h"
#include "waveform_8266.h"

// Waveform generator can create tones, PWM, and servos
typedef struct {
  uint32_t nextServiceCycle;   // ESP cycle timer when a transition required
  uint32_t timeHighCycles;     // Ideal waveform period
  uint32_t timeLowCycles;      //
  uint32_t nextHighLowUs;      // Waveform ideal (us) "on deck", waiting to be changed next cycle
                               // packed into 32 bits to be atomic read/write, 65535us max
} Waveform;

class WVFState {
public:
  Waveform waveform[17]{};        // State of all possible pins
  uint32_t waveformState = 0;   // Is the pin high or low, updated in NMI so no access outside the NMI code
  uint32_t waveformEnabled = 0; // Is it actively running, updated in NMI so no access outside the NMI code

  // Enable lock-free by only allowing updates to waveformState and waveformEnabled from IRQ service routine
  uint32_t waveformToEnable = 0;  // Message to the NMI handler to start a waveform on a inactive pin
  uint32_t waveformToDisable = 0; // Message to the NMI handler to disable a pin from waveform generation

  // Optimize the NMI inner loop by keeping track of the min and max GPIO that we
  // are generating.  In the common case (1 PWM) these may be the same pin and
  // we can avoid looking at the other pins.
  uint16_t startPin = 0;
  uint16_t endPin = 0;
  bool timerRunning = false;
};
static WVFState wvfState;

static volatile uint32_t pwmIsrCalls;
static volatile uint32_t pwmIsrTotalCycles;
static volatile uint32_t pwmIsrMaxCycles;
static volatile uint32_t pwmIsrMaxDoWhileLoops;

enum : uint32_t {
  PWM_BREADCRUMB_IDLE = 0,
  PWM_BREADCRUMB_START_REQUESTED = 1,
  PWM_BREADCRUMB_TIMER_INITIALIZED = 2,
  PWM_BREADCRUMB_INTERRUPT_FORCED = 3,
  PWM_BREADCRUMB_WAITING_FOR_ENABLE = 4,
  PWM_BREADCRUMB_ACTIVE = 5,
  PWM_BREADCRUMB_STOP_REQUESTED = 6,
};

struct PWMBreadcrumbRtc {
  uint32_t magic;
  uint32_t version;
  uint32_t bootCount;
  uint32_t suspiciousResetCount;
  uint32_t resetReasonCode;
  uint32_t activeStage;
  uint32_t activeGpio;
  uint32_t activeHighUs;
  uint32_t activeLowUs;
  uint32_t lastResetStage;
  uint32_t lastResetGpio;
  uint32_t lastResetHighUs;
  uint32_t lastResetLowUs;
};

static constexpr uint32_t PWM_BREADCRUMB_MAGIC = 0x50574D42; // PWMB
static constexpr uint32_t PWM_BREADCRUMB_VERSION = 1;
static constexpr uint32_t PWM_BREADCRUMB_RTC_OFFSET = 40; // skip eboot RTC area
static PWMBreadcrumbRtc pwmBreadcrumbRtc = {};

static void savePwmBreadcrumbs()
{
  ESP.rtcUserMemoryWrite(PWM_BREADCRUMB_RTC_OFFSET, reinterpret_cast<uint32_t *>(&pwmBreadcrumbRtc), sizeof(pwmBreadcrumbRtc));
}

static void updatePwmBreadcrumbStage(uint32_t stage, uint32_t gpio, uint32_t highUs, uint32_t lowUs)
{
  pwmBreadcrumbRtc.activeStage = stage;
  pwmBreadcrumbRtc.activeGpio = gpio;
  pwmBreadcrumbRtc.activeHighUs = highUs;
  pwmBreadcrumbRtc.activeLowUs = lowUs;
  savePwmBreadcrumbs();
}

void initPwmCrashBreadcrumbs()
{
  PWMBreadcrumbRtc rtc = {};
  const bool ok = ESP.rtcUserMemoryRead(PWM_BREADCRUMB_RTC_OFFSET, reinterpret_cast<uint32_t *>(&rtc), sizeof(rtc));
  if (!ok || rtc.magic != PWM_BREADCRUMB_MAGIC || rtc.version != PWM_BREADCRUMB_VERSION)
  {
    rtc = {};
    rtc.magic = PWM_BREADCRUMB_MAGIC;
    rtc.version = PWM_BREADCRUMB_VERSION;
  }

  rtc.bootCount++;
  rtc.resetReasonCode = ESP.getResetInfoPtr()->reason;
  if (rtc.activeStage != PWM_BREADCRUMB_IDLE)
  {
    rtc.suspiciousResetCount++;
    rtc.lastResetStage = rtc.activeStage;
    rtc.lastResetGpio = rtc.activeGpio;
    rtc.lastResetHighUs = rtc.activeHighUs;
    rtc.lastResetLowUs = rtc.activeLowUs;
  }
  rtc.activeStage = PWM_BREADCRUMB_IDLE;
  rtc.activeGpio = 0;
  rtc.activeHighUs = 0;
  rtc.activeLowUs = 0;
  pwmBreadcrumbRtc = rtc;
  savePwmBreadcrumbs();
}

const char *getPwmBreadcrumbStageName(uint32_t stage)
{
  switch (stage) {
    case PWM_BREADCRUMB_IDLE: return "idle";
    case PWM_BREADCRUMB_START_REQUESTED: return "start_requested";
    case PWM_BREADCRUMB_TIMER_INITIALIZED: return "timer_initialized";
    case PWM_BREADCRUMB_INTERRUPT_FORCED: return "interrupt_forced";
    case PWM_BREADCRUMB_WAITING_FOR_ENABLE: return "waiting_for_enable";
    case PWM_BREADCRUMB_ACTIVE: return "active";
    case PWM_BREADCRUMB_STOP_REQUESTED: return "stop_requested";
    default: return "unknown";
  }
}

pwm_breadcrumbs_t getPwmCrashBreadcrumbs()
{
  pwm_breadcrumbs_t breadcrumbs = {};
  breadcrumbs.bootCount = pwmBreadcrumbRtc.bootCount;
  breadcrumbs.suspiciousResetCount = pwmBreadcrumbRtc.suspiciousResetCount;
  breadcrumbs.resetReasonCode = pwmBreadcrumbRtc.resetReasonCode;
  breadcrumbs.activeStage = pwmBreadcrumbRtc.activeStage;
  breadcrumbs.activeGpio = pwmBreadcrumbRtc.activeGpio;
  breadcrumbs.activeHighUs = pwmBreadcrumbRtc.activeHighUs;
  breadcrumbs.activeLowUs = pwmBreadcrumbRtc.activeLowUs;
  breadcrumbs.lastResetStage = pwmBreadcrumbRtc.lastResetStage;
  breadcrumbs.lastResetGpio = pwmBreadcrumbRtc.lastResetGpio;
  breadcrumbs.lastResetHighUs = pwmBreadcrumbRtc.lastResetHighUs;
  breadcrumbs.lastResetLowUs = pwmBreadcrumbRtc.lastResetLowUs;
  return breadcrumbs;
}

// Ensure everything is read/written to RAM
#define MEMBARRIER() { __asm__ volatile("" ::: "memory"); }

// Non-speed critical bits
#pragma GCC optimize ("Os")

// Interrupt on/off control
static void timer1Interrupt();

extern "C" IRAM_ATTR int __wrap_stopWaveform(uint8_t pin) { return true; }
extern "C" IRAM_ATTR bool __wrap__stopPWM(uint8_t pin) { return true; }

static __attribute__((noinline)) void initTimer() {
  if (!wvfState.timerRunning) {
    timer1_disable();
    ETS_FRC_TIMER1_INTR_ATTACH(NULL, NULL);
    ETS_FRC_TIMER1_NMI_INTR_ATTACH(timer1Interrupt);
    timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);
    wvfState.timerRunning = true;
    timer1_write(microsecondsToClockCycles(10));
  }
}

static void forceTimerInterrupt() {
  if (T1L > microsecondsToClockCycles(10)) {
    T1L = microsecondsToClockCycles(10);
  }
}

// If there are no more scheduled activities, shut down Timer 1.
// Otherwise, do nothing.
static void disableIdleTimer() {
  if (wvfState.timerRunning && !wvfState.waveformEnabled) {
    ETS_FRC_TIMER1_NMI_INTR_ATTACH(NULL);
    timer1_disable();
    timer1_isr_init();
    wvfState.timerRunning = false;
  }
}

// Start up a waveform on a pin, or change the current one.  Will change to the new
// waveform smoothly on next low->high transition.  For immediate change, stopWaveform()
// first, then it will immediately begin.
void startWaveform8266(uint8_t gpio, uint32_t timeHighUS, uint32_t timeLowUS) {
  if ((gpio > 16) || isFlashInterfacePin(gpio)) {
    return;
  }
  Waveform *wave = &wvfState.waveform[gpio];

  uint32_t mask = 1<<gpio;
  updatePwmBreadcrumbStage(PWM_BREADCRUMB_START_REQUESTED, gpio, timeHighUS, timeLowUS);
  MEMBARRIER();
  if (wvfState.waveformEnabled & mask) {
    wave->nextHighLowUs = (timeHighUS << 16) | timeLowUS;
    MEMBARRIER();
    updatePwmBreadcrumbStage(PWM_BREADCRUMB_ACTIVE, gpio, timeHighUS, timeLowUS);
    // The waveform will be updated some time in the future on the next period for the signal
  } else { //  if (!(wvfState.waveformEnabled & mask)) {
    wave->timeHighCycles = microsecondsToClockCycles(timeHighUS);
    wave->timeLowCycles = microsecondsToClockCycles(timeLowUS);
    wave->nextHighLowUs = 0;
    wave->nextServiceCycle = ESP.getCycleCount() + microsecondsToClockCycles(1);
    wvfState.waveformToEnable |= mask;
    MEMBARRIER();
    initTimer();
    updatePwmBreadcrumbStage(PWM_BREADCRUMB_TIMER_INITIALIZED, gpio, timeHighUS, timeLowUS);
    forceTimerInterrupt();
    updatePwmBreadcrumbStage(PWM_BREADCRUMB_INTERRUPT_FORCED, gpio, timeHighUS, timeLowUS);
    updatePwmBreadcrumbStage(PWM_BREADCRUMB_WAITING_FOR_ENABLE, gpio, timeHighUS, timeLowUS);
    while (wvfState.waveformToEnable) {
      delay(0); // Wait for waveform to update
      // No mem barrier here, the call to a global function implies global state updated
    }
    updatePwmBreadcrumbStage(PWM_BREADCRUMB_ACTIVE, gpio, timeHighUS, timeLowUS);
  }
}

// Stops a waveform on a pin
void stopWaveform8266(uint8_t gpio) {
  updatePwmBreadcrumbStage(PWM_BREADCRUMB_STOP_REQUESTED, gpio, 0, 0);
  // Can't possibly need to stop anything if there is no timer active
  if (!wvfState.timerRunning) {
    updatePwmBreadcrumbStage(PWM_BREADCRUMB_IDLE, gpio, 0, 0);
    return;
  }
  // If user sends in a pin >16 but <32, this will always point to a 0 bit
  // If they send >=32, then the shift will result in 0 and it will also return false
  uint32_t mask = 1<< gpio;
  if (wvfState.waveformEnabled & mask) {
    wvfState.waveformToDisable = mask;
    forceTimerInterrupt();
    while (wvfState.waveformToDisable) {
      MEMBARRIER(); // If it wasn't written yet, it has to be by now
      /* no-op */ // Can't delay() since stopWaveform may be called from an IRQ
    }
  }
  disableIdleTimer();
  updatePwmBreadcrumbStage(PWM_BREADCRUMB_IDLE, gpio, 0, 0);
}

// Speed critical bits
#pragma GCC optimize ("O2")

// Normally would not want two copies like this, but due to different
// optimization levels the inline attribute gets lost if we try the
// other version.
static inline IRAM_ATTR uint32_t GetCycleCountIRQ() {
  uint32_t ccount;
  __asm__ __volatile__("rsr %0,ccount":"=a"(ccount));
  return ccount;
}

// Find the earliest cycle as compared to right now
static inline IRAM_ATTR uint32_t earliest(uint32_t a, uint32_t b) {
    uint32_t now = GetCycleCountIRQ();
    int32_t da = a - now;
    int32_t db = b - now;
    return (da < db) ? a : b;
}

pwm_isr_profile_t getPwmIsrProfile()
{
  static uint32_t lastMillis = 0;
  static uint32_t lastCalls = 0;
  static uint32_t lastTotalCycles = 0;

  const uint32_t nowMs = millis();
  const uint32_t calls = pwmIsrCalls;
  const uint32_t totalCycles = pwmIsrTotalCycles;
  const uint32_t maxCycles = pwmIsrMaxCycles;
  const uint32_t maxDoWhileLoops = pwmIsrMaxDoWhileLoops;

  pwm_isr_profile_t profile = {};
  if (lastMillis == 0)
  {
    lastMillis = nowMs;
    lastCalls = calls;
    lastTotalCycles = totalCycles;
    return profile;
  }

  const uint32_t windowMs = nowMs - lastMillis;
  const uint32_t deltaCalls = calls - lastCalls;
  const uint32_t deltaCycles = totalCycles - lastTotalCycles;
  profile.sampleWindowMs = windowMs;
  profile.isrCalls = deltaCalls;
  if (windowMs != 0)
  {
    profile.isrRate = (uint32_t)(((uint64_t)deltaCalls * 1000ULL) / windowMs);
  }
  if (deltaCalls != 0)
  {
    profile.avgIsrUs = clockCyclesToMicroseconds(deltaCycles / deltaCalls);
  }
  profile.maxIsrUs = clockCyclesToMicroseconds(maxCycles);
  profile.maxDoWhileLoops = maxDoWhileLoops;

  lastMillis = nowMs;
  lastCalls = calls;
  lastTotalCycles = totalCycles;
  pwmIsrMaxCycles = 0;
  pwmIsrMaxDoWhileLoops = 0;
  return profile;
}

#if F_CPU == 80000000
#define adjust(x) ((x) << (turbo ? 1 : 0))
#else
#define adjust(x) ((x) >> 0)
#endif

static IRAM_ATTR void timer1Interrupt() {
  const uint32_t isrStartCs = GetCycleCountIRQ();
  // Maximum delay between IRQs. 25ms to guarantee no extra interrupts at 50Hz output (20ms)
  constexpr uint32_t MAXINTERVAL_CS = microsecondsToClockCycles(25000);
  // Keep running until the next event is at least this far in the future
#if F_CPU == 80000000
  constexpr int32_t DELTAIRQ_CS = microsecondsToClockCycles(8);
#else
  constexpr int32_t DELTAIRQ_CS = microsecondsToClockCycles(5);
#endif
  // Schedule the timer this much earlier to account for time to get to the first pin flip. Should be significantly lower than DELTAIRQ_CS
  constexpr int32_t PRESCHEDULE_CS = microsecondsToClockCycles(3);
  // Disable the timer while in the interrupt, even though it should be one-shot anyway
  T1C = 0;
  T1I = 0;
  int32_t cycleDeltaNextEvent = MAXINTERVAL_CS;

  if (wvfState.waveformToEnable || wvfState.waveformToDisable) {
    // Handle enable/disable requests from main app
    wvfState.waveformEnabled = (wvfState.waveformEnabled & ~wvfState.waveformToDisable) | wvfState.waveformToEnable; // Set the requested waveforms on/off
    wvfState.waveformState &= ~wvfState.waveformToEnable;  // And clear the state of any just started
    wvfState.waveformToEnable = 0;
    wvfState.waveformToDisable = 0;
    // No mem barrier.  Globals must be written to RAM on ISR exit.
    // Find the first GPIO being generated by checking GCC's find-first-set (returns 1 + the bit of the first 1 in an int32_t)
    wvfState.startPin = __builtin_ffs(wvfState.waveformEnabled) - 1;
    // Find the last bit by subtracting off GCC's count-leading-zeros (no offset in this one)
    wvfState.endPin = 32 - __builtin_clz(wvfState.waveformEnabled);
  }

  // Flag if the core is at 160 MHz, for use by adjust()
  #if F_CPU == 80000000
  const bool turbo = (*(uint32_t*)0x3FF00014) & 1 ? true : false;
  #else
  const bool turbo = true;
  #endif
  if (wvfState.waveformEnabled) {
    // Time the loop and use it to allow an edge to happen early if another round of loops would cause it to be late
    // For 160M clock and 10 pins checked with 1 flipping, this code takes ~250 clock cyles to run so start with an estimate
    int32_t lastLoopCs = (wvfState.endPin - wvfState.startPin) * (40 >> (turbo ? 1 : 0));
    uint32_t doWhileLoops = 0;
    do {
      doWhileLoops++;
      uint32_t loopStartCs = GetCycleCountIRQ();
      uint32_t nextEventCycle = loopStartCs + MAXINTERVAL_CS;

      for (auto gpio = wvfState.startPin; gpio < wvfState.endPin; gpio++) {
        const uint32_t mask = 1 << gpio;

        // If it's not on, ignore!
        if (!(wvfState.waveformEnabled & mask)) {
          continue;
        }

        Waveform *wave = &wvfState.waveform[gpio];

        uint32_t now = GetCycleCountIRQ();
        int32_t cyclesToGo = wave->nextServiceCycle - now;
        if (cyclesToGo < (lastLoopCs / 2)) {
          uint32_t nextEdgeCycles;
          if (wvfState.waveformState & mask) {
            GPOC = mask;
            if (gpio == 16) { // Special handling for GPIO16
              GP16O = 0;
            }
            nextEdgeCycles = wave->timeLowCycles;
          } else {
            GPOS = mask;
            if (gpio == 16) { // Special handling for GPIO16
              GP16O = 1;
            }

            if (wave->nextHighLowUs != 0) {
              // Copy over next full-cycle timings
              uint32_t next = wave->nextHighLowUs;
              wave->nextHighLowUs = 0; // indicate the change has taken place
              wave->timeHighCycles = microsecondsToClockCycles(next >> 16);
              wave->timeLowCycles = microsecondsToClockCycles(next & 0xffff);
            }
            nextEdgeCycles = wave->timeHighCycles;
          }
          wvfState.waveformState ^= mask;
          nextEdgeCycles = adjust(nextEdgeCycles);
          wave->nextServiceCycle = now + nextEdgeCycles;
        }
        nextEventCycle = earliest(nextEventCycle, wave->nextServiceCycle);
      }

      // Exit the loop if we've hit the fixed runtime limit or the next event is known to be after that timeout would occur
      uint32_t loopEndCs = GetCycleCountIRQ();
      cycleDeltaNextEvent = nextEventCycle - loopEndCs;
      // Save the duration of the loop for the next early timeout
      lastLoopCs = loopEndCs - loopStartCs;
    } while (cycleDeltaNextEvent < DELTAIRQ_CS);
    if (doWhileLoops > pwmIsrMaxDoWhileLoops) {
      pwmIsrMaxDoWhileLoops = doWhileLoops;
    }
  } // if (wvfState.waveformEnabled)

  // cycleDeltaNextEvent should be pretty close to or above DELTAIRQ_CS
  // schedule the timer a little early to allow time to get to the pin switch code before the deadline
  T1L = (cycleDeltaNextEvent - PRESCHEDULE_CS) >> (turbo ? 1 : 0);
  T1C = (1 << TCTE); //timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE)
  const uint32_t isrCycles = GetCycleCountIRQ() - isrStartCs;
  pwmIsrCalls++;
  pwmIsrTotalCycles += isrCycles;
  if (isrCycles > pwmIsrMaxCycles) {
    pwmIsrMaxCycles = isrCycles;
  }
}

#endif
