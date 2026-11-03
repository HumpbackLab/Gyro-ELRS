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
#if defined(PLATFORM_ESP32) && defined(TARGET_RX)
// Reserve an output pin for a peripheral such as the AM32 single-wire bus.
bool servoReserveOutputPin(uint8_t pin);
void servoRestoreOutputPin(uint8_t pin);
#endif
#else
inline void servoNewChannelsAvailable(){};
#endif
