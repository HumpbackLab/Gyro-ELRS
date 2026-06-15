#pragma once
#if defined(GPIO_PIN_PWM_OUTPUTS)

#include "device.h"
#include "common.h"

#if (defined(PLATFORM_ESP32))
#include "DShotRMT.h"
#endif

extern device_t ServoOut_device;
#define HAS_SERVO_OUTPUT
#define OPT_HAS_SERVO_OUTPUT (GPIO_PIN_PWM_OUTPUTS_COUNT > 0)

// Notify this unit that new channel data has arrived
void servoNewChannelsAvailable();
// WiFi debugging: set simulated RC channel value (988-2012 us)
void servoSetSimulatedChannel(uint8_t ch, uint16_t us);
uint16_t servoGetSimulatedChannel(uint8_t ch);
#else
inline void servoNewChannelsAvailable(){};
inline void servoSetSimulatedChannel(uint8_t ch, uint16_t us) {};
inline uint16_t servoGetSimulatedChannel(uint8_t ch) { return 0; };
#endif
