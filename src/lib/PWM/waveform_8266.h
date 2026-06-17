#pragma once

#include <stdint.h>

typedef struct {
    uint32_t sampleWindowMs;
    uint32_t isrCalls;
    uint32_t isrRate;
    uint32_t avgIsrUs;
    uint32_t maxIsrUs;
    uint32_t maxDoWhileLoops;
} pwm_isr_profile_t;

typedef struct {
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
} pwm_breadcrumbs_t;

void startWaveform8266(uint8_t gpio, uint32_t timeHighUS, uint32_t timeLowUS);
void stopWaveform8266(uint8_t gpio);
pwm_isr_profile_t getPwmIsrProfile();
pwm_breadcrumbs_t getPwmCrashBreadcrumbs();
void initPwmCrashBreadcrumbs();
const char *getPwmBreadcrumbStageName(uint32_t stage);

#define startWaveform DO_NOT_USE
#define startWaveformClockCycles DO_NOT_USE
#define stopWaveform DO_NOT_USE
#define setTimer1Callback DO_NOT_USE
#define _setPWMFreq DO_NOT_USE
#define _stopPWM DO_NOT_USE
#define _setPWM DO_NOT_USE
