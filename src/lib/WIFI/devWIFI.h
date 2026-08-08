#pragma once

#include "device.h"

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
#include "devButton.h"

extern device_t WIFI_device;
void setWifiUpdateMode();
#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)
void setFlightControlWifiCoexist(bool enabled);
bool isFlightControlWifiCoexist();
#endif
#define HAS_WIFI
#endif
