#include "FlightControl.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "common.h"
#include "config.h"
#include "crsf_protocol.h"
#include "devGyro.h"
#include "helpers.h"
#include <Arduino.h>
#include <math.h>

static constexpr uint32_t FC_UPDATE_INTERVAL_US = 4000;
static constexpr float FC_MAX_ROLL_PITCH_DEG = 35.0f;
static constexpr float FC_MAX_YAW_RATE_DPS = 180.0f;
static constexpr float FC_COMPLEMENTARY_ALPHA = 0.98f;
static constexpr float FC_STANDARD_GRAVITY = 9.80665f;
static constexpr uint8_t FC_PID_COLUMNS = 4;
static constexpr uint8_t FC_PID_AXES = 3;
// RX config stores PID values in centi-units for LUA/CRSF int16 transport;
// restore them to the same floating-point units shown in the WebUI.
static constexpr float FC_PID_CONFIG_SCALE = 0.01f;
// Mixer coefficients produce an arbitrary summed control value. Convert that
// final mixer sum into a PWM pulse-width offset in microseconds.

bool FlightControlSensorBackend::begin()
{
    return GyroIsInitialized();
}

bool FlightControlSensorBackend::read(FlightControlImuSample &sample)
{
    sample = {};
    GyroSample gyroSample = {};
    if (!GyroGetSample(gyroSample))
    {
        return false;
    }

    sample.gyroDps = {
        gyroSample.gyroDps.x,
        gyroSample.gyroDps.y,
        gyroSample.gyroDps.z,
    };
    sample.accelMps2 = {
        gyroSample.accelMps2.x,
        gyroSample.accelMps2.y,
        gyroSample.accelMps2.z,
    };
    sample.gyroValid = true;
    sample.accelValid = gyroSample.accelValid;
    return true;
}

void FlightControlEstimator::reset()
{
    _attitude = {};
}

void FlightControlEstimator::update(const FlightControlImuSample &sample, float dt)
{
    if (sample.gyroValid)
    {
        _attitude.rollDeg += sample.gyroDps.x * dt;
        _attitude.pitchDeg += sample.gyroDps.y * dt;
        _attitude.yawDeg += sample.gyroDps.z * dt;
    }

    if (sample.accelValid)
    {
        const float ax = sample.accelMps2.x;
        const float ay = sample.accelMps2.y;
        const float az = sample.accelMps2.z;
        const float accelMag = sqrtf(ax * ax + ay * ay + az * az);
        if (accelMag > FC_STANDARD_GRAVITY * 0.5f && accelMag < FC_STANDARD_GRAVITY * 1.5f)
        {
            const float accelRoll = atan2f(ay, az) * 180.0f / PI;
            const float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
            _attitude.rollDeg = FC_COMPLEMENTARY_ALPHA * _attitude.rollDeg + (1.0f - FC_COMPLEMENTARY_ALPHA) * accelRoll;
            _attitude.pitchDeg = FC_COMPLEMENTARY_ALPHA * _attitude.pitchDeg + (1.0f - FC_COMPLEMENTARY_ALPHA) * accelPitch;
        }
    }
}

void FlightControlPid::set(float kp, float ki, float kd, float integratorLimit)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integratorLimit = integratorLimit;
}

void FlightControlPid::reset()
{
    _integrator = 0.0f;
    _lastError = 0.0f;
    _hasLastError = false;
}

float FlightControlPid::update(float target, float measurement, float dt)
{
    const float error = target - measurement;
    _integrator += error * dt;
    _integrator = constrain(_integrator, -_integratorLimit, _integratorLimit);

    float derivative = 0.0f;
    if (_hasLastError && dt > 0.0f)
    {
        derivative = (error - _lastError) / dt;
    }
    _lastError = error;
    _hasLastError = true;

    return _kp * error + _ki * _integrator + _kd * derivative;
}

bool FlightControlMixer::begin()
{
    const int count = config.GetFlightControlMixerCount();
    _mix = config.GetFlightControlMixer();
    if (!_mix || count <= 0 || count % FC_MIXER_COLUMNS != 0)
    {
        _motorCount = 0;
        return false;
    }

    _motorCount = constrain(count / FC_MIXER_COLUMNS, 0, FLIGHT_CONTROL_MAX_MOTORS);
    return _motorCount > 0;
}

FlightControlMixerOutput FlightControlMixer::mix(float throttle, float roll, float pitch, float yaw) const
{
    FlightControlMixerOutput output = {};
    output.motorCount = _motorCount;

    for (uint8_t i = 0; i < _motorCount; i++)
    {
        const uint8_t offset = i * FC_MIXER_COLUMNS;
        const float motor =
            throttle * _mix[offset + 0] +
            roll * _mix[offset + 1] +
            pitch * _mix[offset + 2] +
            yaw * _mix[offset + 3];
        // PWM output is absolute pulse width: base 1000us plus scaled mixer offset.
        output.motorUs[i] = (uint16_t)constrain(1000.0f + motor, 1000.0f, 2000.0f);
    }
    return output;
}

bool FlightControlOrientation::begin()
{
    const float *orientation = config.GetFlightControlOrientation();
    if (!orientation)
    {
        return true;
    }

    for (uint8_t i = 0; i < FC_ORIENTATION_VALUE_COUNT; i++)
    {
        _matrix[i] = orientation[i];
    }
    return true;
}

FlightControlVector3 FlightControlOrientation::rotate(const FlightControlVector3 &vector) const
{
    return {
        _matrix[0] * vector.x + _matrix[1] * vector.y + _matrix[2] * vector.z,
        _matrix[3] * vector.x + _matrix[4] * vector.y + _matrix[5] * vector.z,
        _matrix[6] * vector.x + _matrix[7] * vector.y + _matrix[8] * vector.z,
    };
}

void FlightControlOrientation::apply(FlightControlImuSample &sample) const
{
    if (sample.gyroValid)
    {
        sample.gyroDps = rotate(sample.gyroDps);
    }
    if (sample.accelValid)
    {
        sample.accelMps2 = rotate(sample.accelMps2);
    }
}

void FlightControlRuntime::begin()
{
    _orientation.begin();
    refreshReadyState();
    reset();
}

void FlightControlRuntime::reset()
{
    _estimator.reset();
    _rollRatePid.reset();
    _pitchRatePid.reset();
    _yawRatePid.reset();
    _rollAnglePid.reset();
    _pitchAnglePid.reset();
    _yawAnglePid.reset();
    _mixerOutput = {};
    _lastUpdateUs = 0;
}

bool FlightControlRuntime::refreshReadyState()
{
    if (!_sensorsReady)
    {
        _sensorsReady = _sensors.begin();
    }
    _mixerReady = _mixer.begin();
    _pidReady = loadPidParameters();
    return ready();
}

bool FlightControlRuntime::loadPidParameters()
{
    const bool rateReady = loadPidBank(_rollRatePid, _pitchRatePid, _yawRatePid, config.GetFlightControlRatePid(), FC_PID_TERM_COUNT);
    _angleEnabled = config.GetFlightControlAngleMode();

    if (!_angleEnabled)
    {
        return rateReady;
    }

    const bool angleReady = loadPidBank(_rollAnglePid, _pitchAnglePid, _yawAnglePid, config.GetFlightControlAnglePid(), FC_PID_TERM_COUNT);
    return rateReady && angleReady;
}

bool FlightControlRuntime::loadPidBank(FlightControlPid &rollPid, FlightControlPid &pitchPid, FlightControlPid &yawPid, const int16_t *pid, int count)
{
    if (!pid || count < FC_PID_AXES * FC_PID_COLUMNS)
    {
        return false;
    }

    // Convert centi-unit config values back to WebUI units before PID math.
    rollPid.set(pid[0] * FC_PID_CONFIG_SCALE, pid[1] * FC_PID_CONFIG_SCALE, pid[2] * FC_PID_CONFIG_SCALE, pid[3] * FC_PID_CONFIG_SCALE);
    pitchPid.set(pid[4] * FC_PID_CONFIG_SCALE, pid[5] * FC_PID_CONFIG_SCALE, pid[6] * FC_PID_CONFIG_SCALE, pid[7] * FC_PID_CONFIG_SCALE);
    yawPid.set(pid[8] * FC_PID_CONFIG_SCALE, pid[9] * FC_PID_CONFIG_SCALE, pid[10] * FC_PID_CONFIG_SCALE, pid[11] * FC_PID_CONFIG_SCALE);
    return true;
}

void FlightControlRuntime::loadStickTargets(float &throttle, float &roll, float &pitch, float &yaw)
{
    const uint16_t rollUs = CRSF_to_US(ChannelData[0]);
    const uint16_t pitchUs = CRSF_to_US(ChannelData[1]);
    const uint16_t throttleUs = CRSF_to_US(ChannelData[2]);
    const uint16_t yawUs = CRSF_to_US(ChannelData[3]);

    throttle = constrain((throttleUs - 988.0f) / (2012.0f - 988.0f), 0.0f, 1.0f) * 1000;
    roll = constrain((rollUs - 1500.0f) / 500.0f, -1.0f, 1.0f) * FC_MAX_ROLL_PITCH_DEG;
    pitch = constrain((pitchUs - 1500.0f) / 500.0f, -1.0f, 1.0f) * FC_MAX_ROLL_PITCH_DEG;
    yaw = constrain((yawUs - 1500.0f) / 500.0f, -1.0f, 1.0f) * FC_MAX_YAW_RATE_DPS;
}

void FlightControlRuntime::update(uint32_t nowUs)
{
    if (!refreshReadyState())
    {
        reset();
        return;
    }

    if (_lastUpdateUs != 0 && (uint32_t)(nowUs - _lastUpdateUs) < FC_UPDATE_INTERVAL_US)
    {
        return;
    }

    const float dt = _lastUpdateUs == 0 ? (FC_UPDATE_INTERVAL_US * 1e-6f) : (nowUs - _lastUpdateUs) * 1e-6f;
    _lastUpdateUs = nowUs;

    FlightControlImuSample sample = {};
    if (!_sensors.read(sample))
    {
        _sensorsReady = false;
        reset();
        return;
    }
    _orientation.apply(sample);

    float throttle;
    float rollAngleTarget;
    float pitchAngleTarget;
    float yawRateTarget;
    loadStickTargets(throttle, rollAngleTarget, pitchAngleTarget, yawRateTarget);

    float rollRateTarget = rollAngleTarget / FC_MAX_ROLL_PITCH_DEG * FC_MAX_YAW_RATE_DPS;
    float pitchRateTarget = pitchAngleTarget / FC_MAX_ROLL_PITCH_DEG * FC_MAX_YAW_RATE_DPS;

    if (_angleEnabled)
    {
        _estimator.update(sample, dt);
        rollRateTarget = _rollAnglePid.update(rollAngleTarget, _estimator.attitude().rollDeg, dt);
        pitchRateTarget = _pitchAnglePid.update(pitchAngleTarget, _estimator.attitude().pitchDeg, dt);
    }

    const float gyroRoll = sample.gyroValid ? sample.gyroDps.x : 0.0f;
    const float gyroPitch = sample.gyroValid ? sample.gyroDps.y : 0.0f;
    const float gyroYaw = sample.gyroValid ? sample.gyroDps.z : 0.0f;
    const float rollCorrection = _rollRatePid.update(rollRateTarget, gyroRoll, dt);
    const float pitchCorrection = _pitchRatePid.update(pitchRateTarget, gyroPitch, dt);
    const float yawCorrection = _yawRatePid.update(yawRateTarget, gyroYaw, dt);

    _mixerOutput = _mixer.mix(throttle, rollCorrection, pitchCorrection, yawCorrection);
}

#endif
