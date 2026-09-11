#include "targets.h"
#include "common.h"
#include "logging.h"
#include "helpers.h"
#include "device.h"

///////////////////////////////////////
// Even though we aren't using anything this keeps the PIO dependency analyzer happy!

#if defined(RADIO_SX127X)
#include "SX127xDriver.h"
#elif defined(RADIO_LR1121)
#include "LR1121Driver.h"
#elif defined(RADIO_SX128X)
#include "SX1280Driver.h"
#else
#error Invalid radio configuration!
#endif

#if defined(PLATFORM_ESP32)
#include <soc/soc_caps.h>
#define MULTICORE (SOC_CPU_CORES_NUM > 1)
#endif

///////////////////////////////////////

extern bool connectionHasModelMatch;

static device_affinity_t *uiDevices;
static uint8_t deviceCount;

static bool eventFired[2] = {false, false};
static connectionState_e lastConnectionState[2] = {disconnected, disconnected};
static bool lastModelMatch[2] = {false, false};

static unsigned long deviceTimeout[16] = {0};
static uint32_t deviceTimeoutUs[16] = {0};
static constexpr uint32_t DEVICE_TIMEOUT_US_NEVER = UINT32_MAX;

#if MULTICORE
static TaskHandle_t xDeviceTask = NULL;
static SemaphoreHandle_t taskSemaphore;
static SemaphoreHandle_t completeSemaphore;
static void deviceTask(void *pvArgs);
#define CURRENT_CORE  xPortGetCoreID()
#else
#define CURRENT_CORE -1
#endif

void devicesRegister(device_affinity_t *devices, uint8_t count)
{
    uiDevices = devices;
    deviceCount = count;

    #if MULTICORE
        taskSemaphore = xSemaphoreCreateBinary();
        completeSemaphore = xSemaphoreCreateBinary();
        disableCore0WDT();
        xTaskCreatePinnedToCore(deviceTask, "deviceTask", 32768, NULL, 0, &xDeviceTask, 0);
    #endif
}

void devicesInit()
{
    int32_t core = CURRENT_CORE;

    for(size_t i=0 ; i<deviceCount ; i++) {
        if (uiDevices[i].core == core || core == -1) {
            if (uiDevices[i].device->initialize) {
                (uiDevices[i].device->initialize)();
            }
        }
    }
    #if MULTICORE
    if (core == 1)
    {
        xSemaphoreGive(taskSemaphore);
        xSemaphoreTake(completeSemaphore, portMAX_DELAY);
    }
    #endif
}

void devicesStart()
{
    int32_t core = CURRENT_CORE;
    unsigned long now = millis();

    for(size_t i=0 ; i<deviceCount ; i++)
    {
        if (uiDevices[i].core == core || core == -1) {
            deviceTimeout[i] = 0xFFFFFFFF;
            deviceTimeoutUs[i] = DEVICE_TIMEOUT_US_NEVER;
            if (uiDevices[i].device->start)
            {
                int delay = (uiDevices[i].device->start)();
                deviceTimeout[i] = delay == DURATION_NEVER ? 0xFFFFFFFF : now + delay;
                if (uiDevices[i].device->timeoutUs && delay != DURATION_NEVER)
                {
                    deviceTimeoutUs[i] = micros() + static_cast<uint32_t>(delay) * 1000U;
                }
            }
            else if (uiDevices[i].device->timeoutUs)
            {
                deviceTimeoutUs[i] = micros();
            }
        }
    }
    #if MULTICORE
    if (core == 1)
    {
        xSemaphoreGive(taskSemaphore);
        xSemaphoreTake(completeSemaphore, portMAX_DELAY);
    }
    #endif
}

void devicesStop()
{
    #if MULTICORE
    vTaskDelete(xDeviceTask);
    #endif
}

void devicesTriggerEvent()
{
    eventFired[0] = true;
    eventFired[1] = true;
    for (size_t i = 0; i < deviceCount; ++i)
    {
        if (uiDevices[i].device->timeoutUs)
        {
            deviceTimeoutUs[i] = 0;
        }
    }
    #if MULTICORE
    // Release teh semaphore so the tasks on core 0 run now
    xSemaphoreGive(taskSemaphore);
    #endif
}

static int _devicesUpdate(unsigned long now)
{
    const int32_t core = CURRENT_CORE;
    const int32_t coreMulti = (core == -1) ? 0 : core;

    bool newModelMatch = connectionHasModelMatch && teamraceHasModelMatch;
    bool handleEvents = eventFired[coreMulti] || lastConnectionState[coreMulti] != connectionState || lastModelMatch[coreMulti] != newModelMatch;
    eventFired[coreMulti] = false;
    lastConnectionState[coreMulti] = connectionState;
    lastModelMatch[coreMulti] = newModelMatch;

    for(size_t i=0 ; i<deviceCount ; i++)
    {
        if (uiDevices[i].core == core || core == -1) {
            if (handleEvents && uiDevices[i].device->event)
            {
                int delay = (uiDevices[i].device->event)();
                if (delay != DURATION_IGNORE)
                {
                    deviceTimeout[i] = delay == DURATION_NEVER ? 0xFFFFFFFF : now + delay;
                }
            }
        }
    }

    int smallest_delay = DURATION_NEVER;
    for(size_t i=0 ; i<deviceCount ; i++)
    {
        if ((uiDevices[i].core == core || core == -1) && uiDevices[i].device->timeout)
        {
            int delay = deviceTimeout[i] == 0xFFFFFFFF ? DURATION_NEVER : (int)(deviceTimeout[i]-now);
            if (now >= deviceTimeout[i])
            {
                delay = (uiDevices[i].device->timeout)();
                deviceTimeout[i] = delay == DURATION_NEVER ? 0xFFFFFFFF : now + delay;
            }
            if (delay != DURATION_NEVER)
            {
                smallest_delay = (smallest_delay == DURATION_NEVER) ? delay : std::min(smallest_delay, delay);
            }
        }
    }
    return smallest_delay;
}

void devicesUpdate(unsigned long now)
{
    _devicesUpdate(now);
}

uint32_t devicesUpdateMicros(uint32_t nowUs)
{
    const int32_t core = CURRENT_CORE;
    uint32_t smallestDelayUs = DEVICE_TIMEOUT_US_NEVER;

    for (size_t i = 0; i < deviceCount; ++i)
    {
        if ((uiDevices[i].core != core && core != -1) || !uiDevices[i].device->timeoutUs)
        {
            continue;
        }

        const uint32_t deadline = deviceTimeoutUs[i];
        if (deadline == DEVICE_TIMEOUT_US_NEVER || (int32_t)(nowUs - deadline) < 0)
        {
            continue;
        }

        const uint32_t delayUs = uiDevices[i].device->timeoutUs();
        if (delayUs == DEVICE_TIMEOUT_US_NEVER)
        {
            deviceTimeoutUs[i] = DEVICE_TIMEOUT_US_NEVER;
        }
        else if (delayUs == 0)
        {
            deviceTimeoutUs[i] = nowUs;
        }
        else
        {
            // Keep the device's original phase instead of measuring the next
            // interval from a late callback. This preserves the requested
            // microsecond period despite the 1ms FreeRTOS task tick.
            uint32_t nextDeadline = deadline == 0 ? nowUs + delayUs : deadline + delayUs;
            if ((int32_t)(nextDeadline - nowUs) <= 0)
            {
                const uint32_t elapsedUs = nowUs - (deadline == 0 ? nowUs : deadline);
                const uint32_t missedPeriods = elapsedUs / delayUs + 1U;
                nextDeadline = (deadline == 0 ? nowUs : deadline) + missedPeriods * delayUs;
            }
            deviceTimeoutUs[i] = nextDeadline;
        }
    }

    for (size_t i = 0; i < deviceCount; ++i)
    {
        if ((uiDevices[i].core != core && core != -1) || !uiDevices[i].device->timeoutUs)
        {
            continue;
        }

        const uint32_t deadline = deviceTimeoutUs[i];
        if (deadline == DEVICE_TIMEOUT_US_NEVER)
        {
            continue;
        }

        const int32_t delayUs = (int32_t)(deadline - nowUs);
        if (delayUs <= 0)
        {
            smallestDelayUs = 0;
        }
        else if (static_cast<uint32_t>(delayUs) < smallestDelayUs)
        {
            smallestDelayUs = static_cast<uint32_t>(delayUs);
        }
    }

    return smallestDelayUs;
}

#if MULTICORE
static void deviceTask(void *pvArgs)
{
    xSemaphoreTake(taskSemaphore, portMAX_DELAY);
    devicesInit();
    xSemaphoreGive(completeSemaphore);
    xSemaphoreTake(taskSemaphore, portMAX_DELAY);
    devicesStart();
    xSemaphoreGive(completeSemaphore);
    for (;;)
    {
        int delay = _devicesUpdate(millis());
        const uint32_t delayUs = devicesUpdateMicros(micros());

        // Sleep until the earliest millisecond or microsecond callback, or wake
        // immediately when an event or an already-due callback requires it.
        TickType_t waitTicks = portMAX_DELAY;
        if (delay != DURATION_NEVER)
        {
            waitTicks = pdMS_TO_TICKS(delay);
        }
        if (delayUs != DEVICE_TIMEOUT_US_NEVER)
        {
            // FreeRTOS task scheduling is tick based. Poll at the shortest
            // available tick so microsecond deadlines are not rounded up to
            // a fixed multi-millisecond interval.
            const TickType_t microTicks = 1;
            waitTicks = waitTicks == portMAX_DELAY
                ? microTicks
                : (waitTicks < microTicks ? waitTicks : microTicks);
        }
        if (waitTicks == 0 && (delay == DURATION_IMMEDIATELY || delayUs == 0))
        {
            continue;
        }
        xSemaphoreTake(taskSemaphore, waitTicks);
    }
}
#endif
