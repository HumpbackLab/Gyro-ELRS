#pragma once

#include "devFlightControl.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

class FlightControlSensorBackend {
public:
    bool begin();
    bool read(FlightControlImuSample &sample);
};

class FlightControlEstimator {
public:
    void reset();
    void update(const FlightControlImuSample &sample, float dt);
    const FlightControlAttitude &attitude() const { return _attitude; }

private:
    FlightControlAttitude _attitude = {};
};

class FlightControlPid {
public:
    void set(float kp, float ki, float kd, float integratorLimit);
    void reset();
    float update(float target, float measurement, float dt);

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;
    float _integratorLimit = 0.0f;
    float _integrator = 0.0f;
    float _lastError = 0.0f;
    bool _hasLastError = false;
};

class FlightControlMixer {
public:
    bool begin();
    FlightControlMixerOutput mix(float throttle, float roll, float pitch, float yaw) const;

private:
    uint8_t _motorCount = 0;
    const float *_mix = nullptr;
};

class FlightControlOrientation {
public:
    bool begin();
    void apply(FlightControlImuSample &sample) const;

private:
    FlightControlVector3 rotate(const FlightControlVector3 &vector) const;

    float _matrix[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f,
    };
};

class FlightControlRuntime {
public:
    void begin();
    void reset();
    void update(uint32_t nowUs);
    bool sensorsReady() const { return _sensorsReady; }
    bool ready() const { return _sensorsReady && _mixerReady && _pidReady; }
    const FlightControlAttitude &attitude() const { return _estimator.attitude(); }
    const FlightControlMixerOutput &mixerOutput() const { return _mixerOutput; }

private:
    bool loadPidParameters();
    bool loadPidBank(FlightControlPid &rollPid, FlightControlPid &pitchPid, FlightControlPid &yawPid, const int16_t *pid, int count);
    void loadStickTargets(float &throttle, float &rollAngle, float &pitchAngle, float &yawRate);

    FlightControlSensorBackend _sensors;
    FlightControlEstimator _estimator;
    FlightControlPid _rollRatePid;
    FlightControlPid _pitchRatePid;
    FlightControlPid _yawRatePid;
    FlightControlPid _rollAnglePid;
    FlightControlPid _pitchAnglePid;
    FlightControlPid _yawAnglePid;
    FlightControlMixer _mixer;
    FlightControlOrientation _orientation;
    FlightControlMixerOutput _mixerOutput = {};
    uint32_t _lastUpdateUs = 0;
    bool _sensorsReady = false;
    bool _mixerReady = false;
    bool _pidReady = false;
    bool _angleEnabled = false;
};

#endif
