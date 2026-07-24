#include "FlightControl.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "common.h"
#include "crsf_protocol.h"
#include "devGyro.h"
#include "FlightControlConfig.h"
#include "helpers.h"
#include <Arduino.h>
#include <math.h>

static constexpr uint32_t FC_UPDATE_INTERVAL_US = 4000;
static constexpr float FC_MAX_ROLL_PITCH_DEG = 35.0f;
static constexpr float FC_MAX_YAW_RATE_DPS = 180.0f;
static constexpr float FC_COMPLEMENTARY_ALPHA = 0.98f;
static constexpr uint16_t FC_COMPLEMENTARY_ALPHA_PERMILLE = (uint16_t)(FC_COMPLEMENTARY_ALPHA * 1000.0f);
static constexpr float FC_STANDARD_GRAVITY = 9.80665f;
static constexpr uint8_t FC_PID_COLUMNS = 4;
static constexpr uint8_t FC_PID_AXES = 3;
static constexpr float FC_MANUAL_CONTROL_RANGE = 500.0f;
// fc.json stores PID values in centi-units for LUA/CRSF int16 transport;
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
    sample.timestampMs = gyroSample.timestampMs;
    sample.gyroValid = true;
    sample.accelValid = gyroSample.accelValid;
    return true;
}

static float wrapDegrees180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

void FlightControlEstimator::reset()
{
    _attitude = {};
    _accelAttitude = {};
    _attitudeValid = false;
}

void FlightControlEstimator::update(const FlightControlImuSample &sample, float dt)
{
    const bool hadAttitude = _attitudeValid;

    if (sample.gyroValid && dt > 0.0f)
    {
        _attitude.rollDeg = wrapDegrees180(_attitude.rollDeg + sample.gyroDps.x * dt);
        _attitude.pitchDeg = wrapDegrees180(_attitude.pitchDeg + sample.gyroDps.y * dt);
        _attitude.yawDeg = wrapDegrees180(_attitude.yawDeg + sample.gyroDps.z * dt);
        _attitudeValid = true;
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
            _accelAttitude.rollDeg = accelRoll;
            _accelAttitude.pitchDeg = accelPitch;
            _accelAttitude.yawDeg = 0.0f;

            if (!hadAttitude || !sample.gyroValid)
            {
                _attitude.rollDeg = accelRoll;
                _attitude.pitchDeg = accelPitch;
                _attitudeValid = true;
            }
            else
            {
                _attitude.rollDeg = wrapDegrees180(
                    FC_COMPLEMENTARY_ALPHA * _attitude.rollDeg +
                    (1.0f - FC_COMPLEMENTARY_ALPHA) * accelRoll);
                _attitude.pitchDeg = wrapDegrees180(
                    FC_COMPLEMENTARY_ALPHA * _attitude.pitchDeg +
                    (1.0f - FC_COMPLEMENTARY_ALPHA) * accelPitch);
            }
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
    const int count = flightControlConfig.GetMixerCount();
    _mix = flightControlConfig.GetMixer();
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
        // Motors start at minimum throttle. Servos use a centered neutral so
        // zero mixer input returns them to their configured center position.
        const float baseUs = flightControlConfig.GetMixerOutputServo(i) ? 1500.0f : 1000.0f;
        output.motorUs[i] = (uint16_t)constrain(baseUs + motor, 1000.0f, 2000.0f);
    }
    return output;
}

bool FlightControlOrientation::begin()
{
    const float *orientation = flightControlConfig.GetOrientation();
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
    resetPidState();
    // Preserve the configured motor count so consumers can actively drive a
    // safe minimum output instead of retaining the last control value.
    _mixerOutput = _mixer.mix(0.0f, 0.0f, 0.0f, 0.0f);
    _lastImuSample = {};
    _lastDebugUpdateMs = 0;
    _lastUpdateDtUs = 0;
    _lastSampleAgeMs = 0;
    _lastUpdateUs = 0;
}

void FlightControlRuntime::resetPidState()
{
    _rollRatePid.reset();
    _pitchRatePid.reset();
    _yawRatePid.reset();
    _rollAnglePid.reset();
    _pitchAnglePid.reset();
    _yawAnglePid.reset();
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
    const bool rateReady = loadPidBank(_rollRatePid, _pitchRatePid, _yawRatePid, flightControlConfig.GetRatePid(), FC_PID_TERM_COUNT);
    const bool angleReady = loadPidBank(_rollAnglePid, _pitchAnglePid, _yawAnglePid, flightControlConfig.GetAnglePid(), FC_PID_TERM_COUNT);
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

void FlightControlRuntime::updateAttitudeOnly(uint32_t nowUs)
{
    if (!_sensorsReady)
    {
        _sensorsReady = _sensors.begin();
    }
    if (!_sensorsReady)
    {
        reset();
        return;
    }

    if (_lastUpdateUs != 0 && (uint32_t)(nowUs - _lastUpdateUs) < FC_UPDATE_INTERVAL_US)
    {
        return;
    }

    const uint32_t dtUs = _lastUpdateUs == 0 ? FC_UPDATE_INTERVAL_US : (uint32_t)(nowUs - _lastUpdateUs);
    const float dt = dtUs * 1e-6f;
    _lastUpdateUs = nowUs;
    _lastUpdateDtUs = (uint16_t)constrain(dtUs, 0U, 65535U);

    FlightControlImuSample sample = {};
    if (!_sensors.read(sample))
    {
        _sensorsReady = false;
        reset();
        return;
    }

    _orientation.apply(sample);
    _lastImuSample = sample;
    _lastDebugUpdateMs = millis();
    _lastSampleAgeMs = sample.timestampMs == 0 ? 0 : (uint16_t)constrain((uint32_t)(_lastDebugUpdateMs - sample.timestampMs), 0U, 65535U);
    _estimator.update(sample, dt);
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
    yaw = -constrain((yawUs - 1500.0f) / 500.0f, -1.0f, 1.0f) * FC_MAX_YAW_RATE_DPS;
}

FlightControlMode FlightControlRuntime::readModeSwitch() const
{
    // Like INAV mode activation conditions, ranges are left-closed and
    // right-open. ANGLE takes priority if both optional conditions overlap;
    // MANUAL is the implicit fallback when neither condition is active.
    const uint8_t angleChannel = flightControlConfig.GetModeChannel(FLIGHT_CONTROL_MODE_ANGLE);
    if (flightControlConfig.GetModeEnabled(FLIGHT_CONTROL_MODE_ANGLE) &&
        FlightControlRangeIsActive(flightControlConfig.GetModeRange(FLIGHT_CONTROL_MODE_ANGLE), CRSF_to_US(ChannelData[angleChannel])))
    {
        return FLIGHT_CONTROL_MODE_ANGLE;
    }
    const uint8_t rateChannel = flightControlConfig.GetModeChannel(FLIGHT_CONTROL_MODE_RATE);
    if (flightControlConfig.GetModeEnabled(FLIGHT_CONTROL_MODE_RATE) &&
        FlightControlRangeIsActive(flightControlConfig.GetModeRange(FLIGHT_CONTROL_MODE_RATE), CRSF_to_US(ChannelData[rateChannel])))
    {
        return FLIGHT_CONTROL_MODE_RATE;
    }
    return FLIGHT_CONTROL_MODE_MANUAL;
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

    const uint32_t dtUs = _lastUpdateUs == 0 ? FC_UPDATE_INTERVAL_US : (uint32_t)(nowUs - _lastUpdateUs);
    const float dt = dtUs * 1e-6f;
    _lastUpdateUs = nowUs;
    _lastUpdateDtUs = (uint16_t)constrain(dtUs, 0U, 65535U);

    FlightControlImuSample sample = {};
    if (!_sensors.read(sample))
    {
        _sensorsReady = false;
        reset();
        return;
    }
    _orientation.apply(sample);
    _lastImuSample = sample;
    _lastDebugUpdateMs = millis();
    _lastSampleAgeMs = sample.timestampMs == 0 ? 0 : (uint16_t)constrain((uint32_t)(_lastDebugUpdateMs - sample.timestampMs), 0U, 65535U);
    _estimator.update(sample, dt);

    const FlightControlMode nextMode = readModeSwitch();
    if (nextMode != _mode)
    {
        resetPidState();
        _mode = nextMode;
    }

    const uint8_t armChannel = flightControlConfig.GetArmChannel();
    const bool armActive = FlightControlRangeIsActive(
        flightControlConfig.GetArmRange(), CRSF_to_US(ChannelData[armChannel]));
    if (flightControlConfig.GetArmMode() && !armActive)
    {
        resetPidState();
        return;
    }

    float throttle;
    float rollAngleTarget;
    float pitchAngleTarget;
    float yawRateTarget;
    loadStickTargets(throttle, rollAngleTarget, pitchAngleTarget, yawRateTarget);

    if (_mode == FLIGHT_CONTROL_MODE_MANUAL)
    {
        const float roll = rollAngleTarget / FC_MAX_ROLL_PITCH_DEG * FC_MANUAL_CONTROL_RANGE;
        const float pitch = pitchAngleTarget / FC_MAX_ROLL_PITCH_DEG * FC_MANUAL_CONTROL_RANGE;
        const float yaw = yawRateTarget / FC_MAX_YAW_RATE_DPS * FC_MANUAL_CONTROL_RANGE;
        _mixerOutput = _mixer.mix(throttle, roll, pitch, yaw);
        return;
    }

    float rollRateTarget = rollAngleTarget / FC_MAX_ROLL_PITCH_DEG * FC_MAX_YAW_RATE_DPS;
    float pitchRateTarget = pitchAngleTarget / FC_MAX_ROLL_PITCH_DEG * FC_MAX_YAW_RATE_DPS;

    if (_mode == FLIGHT_CONTROL_MODE_ANGLE)
    {
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

bool FlightControlRuntime::getDebugSnapshot(FlightControlDebugSnapshot &snapshot) const
{
    snapshot = {};
    snapshot.imu = _lastImuSample;
    snapshot.attitude = _estimator.attitude();
    snapshot.accelAttitude = _estimator.accelAttitude();
    snapshot.mixerOutput = _mixerOutput;
    snapshot.updateTimestampMs = _lastDebugUpdateMs;
    snapshot.updateDtUs = _lastUpdateDtUs;
    snapshot.sampleAgeMs = _lastSampleAgeMs;
    snapshot.complementaryAlphaPermille = FC_COMPLEMENTARY_ALPHA_PERMILLE;
    snapshot.sensorsReady = _sensorsReady;
    snapshot.mixerReady = _mixerReady;
    snapshot.pidReady = _pidReady;
    snapshot.mode = _mode;
    snapshot.attitudeValid = _estimator.attitudeValid();
    return _lastDebugUpdateMs != 0;
}

#endif
