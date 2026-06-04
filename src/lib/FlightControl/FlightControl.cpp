#include "FlightControl.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "common.h"
#include "crsf_protocol.h"
#include "helpers.h"
#include <Arduino.h>
#include <math.h>

static constexpr uint32_t FC_UPDATE_INTERVAL_US = 4000;
static constexpr float FC_MAX_ROLL_PITCH_DEG = 35.0f;
static constexpr float FC_MAX_YAW_RATE_DPS = 180.0f;
static constexpr float FC_COMPLEMENTARY_ALPHA = 0.98f;
static constexpr float FC_STANDARD_GRAVITY = 9.80665f;
static constexpr uint8_t FC_MIXER_COLUMNS = 4;
static constexpr uint8_t FC_PID_COLUMNS = 4;
static constexpr uint8_t FC_PID_AXES = 3;
static constexpr uint8_t FC_ORIENTATION_VALUES = 9;

bool FlightControlSensorBackend::begin()
{
    return false;
}

bool FlightControlSensorBackend::read(FlightControlImuSample &sample)
{
    sample = {};
    return false;
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
#if defined(FC_MIXER) && defined(FC_MIXER_COUNT)
    const int count = FC_MIXER_COUNT;
    _mix = FC_MIXER;
    if (!_mix || count <= 0 || count % FC_MIXER_COLUMNS != 0)
    {
        _motorCount = 0;
        return false;
    }

    _motorCount = constrain(count / FC_MIXER_COLUMNS, 0, FLIGHT_CONTROL_MAX_MOTORS);
    return _motorCount > 0;
#else
    _motorCount = 0;
    _mix = nullptr;
    return false;
#endif
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
        output.motorUs[i] = 1000 + (uint16_t)(constrain(motor, 0.0f, 1.0f) * 1000.0f);
    }
    return output;
}

bool FlightControlOrientation::begin()
{
#if defined(FC_ORIENTATION) && defined(FC_ORIENTATION_COUNT)
    const float *orientation = FC_ORIENTATION;
    if (!orientation || FC_ORIENTATION_COUNT < FC_ORIENTATION_VALUES)
    {
        return true;
    }

    for (uint8_t i = 0; i < FC_ORIENTATION_VALUES; i++)
    {
        _matrix[i] = orientation[i];
    }
#endif
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
    _sensorsReady = _sensors.begin();
    _orientation.begin();
    _mixerReady = _mixer.begin();
    _pidReady = loadPidParameters();
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
    _newChannelsAvailable = false;
}

void FlightControlRuntime::markChannelsAvailable()
{
    _newChannelsAvailable = true;
}

bool FlightControlRuntime::loadPidParameters()
{
    bool rateReady = false;
#if defined(FC_RATE_PID) && defined(FC_RATE_PID_COUNT)
    rateReady = loadPidBank(_rollRatePid, _pitchRatePid, _yawRatePid, FC_RATE_PID, FC_RATE_PID_COUNT);
#endif
#if defined(FC_PID) && defined(FC_PID_COUNT)
    if (!rateReady)
    {
        rateReady = loadPidBank(_rollRatePid, _pitchRatePid, _yawRatePid, FC_PID, FC_PID_COUNT);
    }
#endif

#if defined(FC_ANGLE_ENABLED)
    _angleEnabled = FC_ANGLE_ENABLED;
#else
    _angleEnabled = false;
#endif

    if (!_angleEnabled)
    {
        return rateReady;
    }

#if defined(FC_ANGLE_PID) && defined(FC_ANGLE_PID_COUNT)
    const bool angleReady = loadPidBank(_rollAnglePid, _pitchAnglePid, _yawAnglePid, FC_ANGLE_PID, FC_ANGLE_PID_COUNT);
    return rateReady && angleReady;
#else
    return false;
#endif
}

bool FlightControlRuntime::loadPidBank(FlightControlPid &rollPid, FlightControlPid &pitchPid, FlightControlPid &yawPid, const float *pid, int count)
{
    if (!pid || count < FC_PID_AXES * FC_PID_COLUMNS)
    {
        return false;
    }

    rollPid.set(pid[0], pid[1], pid[2], pid[3]);
    pitchPid.set(pid[4], pid[5], pid[6], pid[7]);
    yawPid.set(pid[8], pid[9], pid[10], pid[11]);
    return true;
}

void FlightControlRuntime::loadStickTargets(float &throttle, float &roll, float &pitch, float &yaw)
{
    const uint16_t rollUs = CRSF_to_US(ChannelData[0]);
    const uint16_t pitchUs = CRSF_to_US(ChannelData[1]);
    const uint16_t throttleUs = CRSF_to_US(ChannelData[2]);
    const uint16_t yawUs = CRSF_to_US(ChannelData[3]);

    throttle = constrain((throttleUs - 988.0f) / (2012.0f - 988.0f), 0.0f, 1.0f);
    roll = constrain((rollUs - 1500.0f) / 500.0f, -1.0f, 1.0f) * FC_MAX_ROLL_PITCH_DEG;
    pitch = constrain((pitchUs - 1500.0f) / 500.0f, -1.0f, 1.0f) * FC_MAX_ROLL_PITCH_DEG;
    yaw = constrain((yawUs - 1500.0f) / 500.0f, -1.0f, 1.0f) * FC_MAX_YAW_RATE_DPS;
}

void FlightControlRuntime::update(uint32_t nowUs)
{
    if (!ready() || connectionState != connected || !connectionHasModelMatch || !teamraceHasModelMatch)
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
    _newChannelsAvailable = false;
}

#endif
