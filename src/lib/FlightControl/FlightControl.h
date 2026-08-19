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
    const FlightControlAttitude &accelAttitude() const { return _accelAttitude; }
    bool attitudeValid() const { return _attitudeValid; }

private:
    FlightControlAttitude _attitude = {};
    FlightControlAttitude _accelAttitude = {};
    bool _attitudeValid = false;
    float _quatW = 1.0f;
    float _quatX = 0.0f;
    float _quatY = 0.0f;
    float _quatZ = 0.0f;
};

class FlightControlPid {
public:
    void set(float kp, float ki, float kd, float integratorLimit, float dtermLpfHz);
    void reset();
    float update(float target, float measurement, float dt);

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;
    float _integratorLimit = 0.0f;
    float _dtermLpfHz = 0.0f;
    float _integrator = 0.0f;
    float _lastMeasurement = 0.0f;
    float _filteredDerivative = 0.0f;
    bool _hasLastMeasurement = false;
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
    bool refreshReadyState();
    void update(uint32_t nowUs);
    void updateAttitudeOnly(uint32_t nowUs);
    bool sensorsReady() const { return _sensorsReady; }
    bool ready() const { return _sensorsReady && _mixerReady && _pidReady; }
    const FlightControlAttitude &attitude() const { return _estimator.attitude(); }
    const FlightControlMixerOutput &mixerOutput() const { return _mixerOutput; }
    FlightControlMode mode() const { return _mode; }
    bool getDebugSnapshot(FlightControlDebugSnapshot &snapshot) const;

private:
    bool loadPidParameters();
    bool loadPidBank(FlightControlPid &rollPid, FlightControlPid &pitchPid, FlightControlPid &yawPid, const int16_t *pid, int count);
    void loadStickTargets(float &throttle, float &rollAngle, float &pitchAngle, float &yawRate);
    FlightControlMode readModeSwitch() const;
    void resetPidState();
    bool readTransformedImu(FlightControlImuSample &sample, float dt);
    void filterGyro(FlightControlImuSample &sample, float dt);

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
    FlightControlImuSample _lastImuSample = {};
    FlightControlVector3 _filteredGyroDps = {};
    uint32_t _lastDebugUpdateMs = 0;
    uint16_t _lastUpdateDtUs = 0;
    uint16_t _lastSampleAgeMs = 0;
    uint32_t _lastUpdateUs = 0;
    bool _sensorsReady = false;
    bool _mixerReady = false;
    bool _pidReady = false;
    bool _gyroFilterInitialized = false;
    uint8_t _gyroFilterHz = 0;
    FlightControlMode _mode = FLIGHT_CONTROL_MODE_MANUAL;
    float _rollAngleTarget = 0.0f;
    float _pitchAngleTarget = 0.0f;
    float _rollRateTarget = 0.0f;
    float _pitchRateTarget = 0.0f;
    float _yawRateTarget = 0.0f;
    bool _armed = false;
};

#endif
