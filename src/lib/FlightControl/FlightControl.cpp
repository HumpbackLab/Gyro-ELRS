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
static constexpr float FC_MAX_RATE_DPS = 180.0f;
static constexpr float FC_COMPLEMENTARY_ALPHA = 0.98f;
static constexpr uint16_t FC_COMPLEMENTARY_ALPHA_PERMILLE = (uint16_t)(FC_COMPLEMENTARY_ALPHA * 1000.0f);
// Convert the legacy per-update alpha at 250 Hz to a correction rate. This
// keeps the filter response stable if the actual update interval jitters.
static constexpr float FC_COMPLEMENTARY_GAIN_PER_SECOND = 5.05068f; // -ln(0.98) / 0.004
static constexpr float FC_STANDARD_GRAVITY = 9.80665f;
static constexpr float FC_ACCEL_FULL_TRUST_DEVIATION = 0.10f;
static constexpr float FC_ACCEL_REJECT_DEVIATION = 0.35f;
static constexpr float FC_ESTIMATOR_MAX_DT = 0.02f;
static constexpr uint8_t FC_PID_COLUMNS = 4;
static constexpr uint8_t FC_PID_AXES = 3;
static constexpr float FC_MANUAL_CONTROL_RANGE = 500.0f;
static constexpr float FC_TWO_PI = 6.28318530718f;
static constexpr uint8_t FC_ARM_GYRO_BIAS_SAMPLE_COUNT = 100;
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

static void quaternionFromEuler(float rollDeg, float pitchDeg, float yawDeg, float &w, float &x, float &y, float &z)
{
    const float roll = rollDeg * PI / 180.0f;
    const float pitch = pitchDeg * PI / 180.0f;
    const float yaw = yawDeg * PI / 180.0f;

    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
}

static void quaternionToEuler(float w, float x, float y, float z, float &rollDeg, float &pitchDeg, float &yawDeg)
{
    rollDeg = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * 180.0f / PI;
    pitchDeg = asinf(constrain(2.0f * (w * y - z * x), -1.0f, 1.0f)) * 180.0f / PI;
    yawDeg = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * 180.0f / PI;
}

static void normalizeQuaternion(float &w, float &x, float &y, float &z)
{
    const float normSquared = w * w + x * x + y * y + z * z;
    if (isfinite(normSquared) && normSquared > 1.0e-12f)
    {
        const float invNorm = 1.0f / sqrtf(normSquared);
        w *= invNorm;
        x *= invNorm;
        y *= invNorm;
        z *= invNorm;
    }
    else
    {
        w = 1.0f;
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }
}

void FlightControlEstimator::reset()
{
    _attitude = {};
    _accelAttitude = {};
    _attitudeValid = false;
    _quatW = 1.0f;
    _quatX = 0.0f;
    _quatY = 0.0f;
    _quatZ = 0.0f;
}

void FlightControlEstimator::update(const FlightControlImuSample &sample, float dt)
{
    float accelRoll = 0.0f;
    float accelPitch = 0.0f;
    float accelNormX = 0.0f;
    float accelNormY = 0.0f;
    float accelNormZ = 0.0f;
    float accelTrust = 0.0f;
    bool accelReady = false;

    if (sample.accelValid)
    {
        const float ax = sample.accelMps2.x;
        const float ay = sample.accelMps2.y;
        const float az = sample.accelMps2.z;
        const float accelMagnitudeSquared = ax * ax + ay * ay + az * az;
        if (isfinite(accelMagnitudeSquared) && accelMagnitudeSquared > 1.0e-6f)
        {
            const float accelMag = sqrtf(accelMagnitudeSquared);
            const float invMag = 1.0f / accelMag;
            accelNormX = ax * invMag;
            accelNormY = ay * invMag;
            accelNormZ = az * invMag;
            accelRoll = atan2f(accelNormY, accelNormZ) * 180.0f / PI;
            accelPitch = atan2f(-accelNormX, sqrtf(accelNormY * accelNormY + accelNormZ * accelNormZ)) * 180.0f / PI;
            _accelAttitude.rollDeg = accelRoll;
            _accelAttitude.pitchDeg = accelPitch;
            _accelAttitude.yawDeg = 0.0f;

            // Linear acceleration must not be interpreted as tilt. Use full
            // correction close to 1 g, then fade it out instead of abruptly
            // enabling/disabling the accelerometer at wide 0.5/1.5 g limits.
            const float deviation = fabsf(accelMag - FC_STANDARD_GRAVITY) / FC_STANDARD_GRAVITY;
            if (deviation < FC_ACCEL_REJECT_DEVIATION)
            {
                accelTrust = deviation <= FC_ACCEL_FULL_TRUST_DEVIATION
                    ? 1.0f
                    : 1.0f - (deviation - FC_ACCEL_FULL_TRUST_DEVIATION) /
                        (FC_ACCEL_REJECT_DEVIATION - FC_ACCEL_FULL_TRUST_DEVIATION);
                accelReady = true;
            }
        }
    }

    // Seed the actual quaternion, not just the reported Euler angles. The old
    // path initialized the display from accel while leaving the quaternion at
    // identity, causing the estimate to jump back on the following update.
    if (!_attitudeValid && accelReady)
    {
        quaternionFromEuler(accelRoll, accelPitch, 0.0f, _quatW, _quatX, _quatY, _quatZ);
        normalizeQuaternion(_quatW, _quatX, _quatY, _quatZ);
        _attitudeValid = true;
    }

    const bool gyroReady = sample.gyroValid && isfinite(dt) && dt > 0.0f && dt <= FC_ESTIMATOR_MAX_DT &&
        isfinite(sample.gyroDps.x) && isfinite(sample.gyroDps.y) && isfinite(sample.gyroDps.z);
    if (gyroReady)
    {
        float gx = sample.gyroDps.x * PI / 180.0f;
        float gy = sample.gyroDps.y * PI / 180.0f;
        float gz = sample.gyroDps.z * PI / 180.0f;

        if (accelReady)
        {
            const float vx = 2.0f * (_quatX * _quatZ - _quatW * _quatY);
            const float vy = 2.0f * (_quatW * _quatX + _quatY * _quatZ);
            const float vz = _quatW * _quatW - _quatX * _quatX - _quatY * _quatY + _quatZ * _quatZ;
            const float ex = accelNormY * vz - accelNormZ * vy;
            const float ey = accelNormZ * vx - accelNormX * vz;
            const float ez = accelNormX * vy - accelNormY * vx;

            const float correctionGain = FC_COMPLEMENTARY_GAIN_PER_SECOND * accelTrust;
            gx += correctionGain * ex;
            gy += correctionGain * ey;
            gz += correctionGain * ez;
        }

        const float halfDt = 0.5f * dt;
        float q0Tmp = _quatW + (-_quatX * gx - _quatY * gy - _quatZ * gz) * halfDt;
        float q1Tmp = _quatX + (_quatW * gx + _quatY * gz - _quatZ * gy) * halfDt;
        float q2Tmp = _quatY + (_quatW * gy - _quatX * gz + _quatZ * gx) * halfDt;
        float q3Tmp = _quatZ + (_quatW * gz + _quatX * gy - _quatY * gx) * halfDt;

        normalizeQuaternion(q0Tmp, q1Tmp, q2Tmp, q3Tmp);
        _quatW = q0Tmp;
        _quatX = q1Tmp;
        _quatY = q2Tmp;
        _quatZ = q3Tmp;
        _attitudeValid = true;
    }
    if (_attitudeValid)
    {
        float quatRoll = 0.0f;
        float quatPitch = 0.0f;
        float quatYaw = 0.0f;
        quaternionToEuler(_quatW, _quatX, _quatY, _quatZ, quatRoll, quatPitch, quatYaw);

        // Gravity correction is already applied in quaternion space. A second
        // Euler-space blend would double-count accel and can behave badly near
        // the +/-180 degree wrap boundary.
        _attitude.rollDeg = wrapDegrees180(quatRoll);
        _attitude.pitchDeg = wrapDegrees180(quatPitch);
        _attitude.yawDeg = wrapDegrees180(quatYaw);
    }
}

void FlightControlPid::set(float kp, float ki, float kd, float integratorLimit, float dtermLpfHz)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integratorLimit = integratorLimit;
    _dtermLpfHz = dtermLpfHz;
}

void FlightControlPid::reset()
{
    _integrator = 0.0f;
    _lastMeasurement = 0.0f;
    _filteredDerivative = 0.0f;
    _hasLastMeasurement = false;
}

float FlightControlPid::update(float target, float measurement, float dt)
{
    const float error = target - measurement;
    _integrator += error * dt;
    _integrator = constrain(_integrator, -_integratorLimit, _integratorLimit);

    float derivative = 0.0f;
    if (_hasLastMeasurement && dt > 0.0f)
    {
        // Derivative on measurement avoids a D kick when the stick/setpoint
        // changes. Low-pass the derivative before applying Kd, as INAV does.
        const float rawDerivative = -((measurement - _lastMeasurement) / dt);
        if (_dtermLpfHz > 0.0f)
        {
            const float rc = 1.0f / (FC_TWO_PI * _dtermLpfHz);
            const float alpha = dt / (rc + dt);
            _filteredDerivative += alpha * (rawDerivative - _filteredDerivative);
            derivative = _filteredDerivative;
        }
        else
        {
            _filteredDerivative = rawDerivative;
            derivative = rawDerivative;
        }
    }
    _lastMeasurement = measurement;
    _hasLastMeasurement = true;

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
    resetAttitudeState();
    resetControlState();
    _gyroFilterHz = 0;
    const float *configuredGyroBias = flightControlConfig.GetGyroBias();
    _armGyroBiasDps = {
        configuredGyroBias[0],
        configuredGyroBias[1],
        configuredGyroBias[2],
    };
    _armGyroBiasSumDps = {};
    _armGyroBiasSampling = false;
    _armGyroBiasSampleCount = 0;
    _armed = false;
}

void FlightControlRuntime::resetAttitudeState()
{
    _estimator.reset();
    _lastImuSample = {};
    _lastFilteredGyroDps = {};
    _filteredGyroDps = {};
    _lastDebugUpdateMs = 0;
    _lastUpdateDtUs = 0;
    _lastSampleAgeMs = 0;
    _lastUpdateUs = 0;
    _gyroFilterInitialized = false;
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

void FlightControlRuntime::resetControlState()
{
    resetPidState();
    _rollAngleTarget = _pitchAngleTarget = 0.0f;
    _rollRateTarget = _pitchRateTarget = _yawRateTarget = 0.0f;
    // Preserve the configured output count while immediately replacing every
    // previous PID/mixer command with the safe motor minimum or servo center.
    _mixerOutput = _mixer.mix(0.0f, 0.0f, 0.0f, 0.0f);
}

void FlightControlRuntime::beginArmGyroBiasSampling()
{
    // Keep applying the last configured/learned bias to attitude preview while
    // the next bias is accumulated independently from raw transformed samples.
    _armGyroBiasSumDps = {};
    _armGyroBiasSampleCount = 0;
    _armGyroBiasSampling = true;
}

void FlightControlRuntime::collectArmGyroBiasSample(const FlightControlVector3 &gyroDps)
{
    _armGyroBiasSumDps.x += gyroDps.x;
    _armGyroBiasSumDps.y += gyroDps.y;
    _armGyroBiasSumDps.z += gyroDps.z;
    ++_armGyroBiasSampleCount;
    if (_armGyroBiasSampleCount < FC_ARM_GYRO_BIAS_SAMPLE_COUNT)
    {
        return;
    }

    const float divisor = (float)_armGyroBiasSampleCount;
    const FlightControlVector3 sampledBias = {
        _armGyroBiasSumDps.x / divisor,
        _armGyroBiasSumDps.y / divisor,
        _armGyroBiasSumDps.z / divisor,
    };
    if (isfinite(sampledBias.x) && fabsf(sampledBias.x) <= 100.0f &&
        isfinite(sampledBias.y) && fabsf(sampledBias.y) <= 100.0f &&
        isfinite(sampledBias.z) && fabsf(sampledBias.z) <= 100.0f)
    {
        _armGyroBiasDps = sampledBias;
        _armGyroBiasSampling = false;
        return;
    }

    // Stay locked and retry with a fresh window rather than arming with an
    // invalid bias.
    _armGyroBiasSumDps = {};
    _armGyroBiasSampleCount = 0;
}

void FlightControlRuntime::completeArmGyroBiasSampling()
{
    // The learned bias changes the estimator/PID input. Discard all history
    // produced with the previous bias before exposing the armed state.
    resetAttitudeState();
    resetControlState();
    _armed = true;
}

void FlightControlRuntime::updateArmState()
{
    const uint8_t armChannel = flightControlConfig.GetArmChannel();
    const bool armActive = FlightControlRangeIsActive(
        flightControlConfig.GetArmRange(), CRSF_to_US(ChannelData[armChannel]));
    const bool armRequested = !flightControlConfig.GetArmMode() || armActive;

    if (!armRequested)
    {
        _armed = false;
        // Keep attitude preview running while locked, but never carry a partial
        // gyro average into the next armed session.
        _armGyroBiasSampling = false;
        _armGyroBiasSampleCount = 0;
        resetControlState();
        return;
    }

    if (_armed)
    {
        return;
    }

    if (flightControlConfig.GetGyroBiasMode() == FLIGHT_CONTROL_GYRO_BIAS_CONFIGURED)
    {
        // Preserve the established configured-bias arming behavior: only
        // controller history is reset because the estimator bias did not change.
        _armGyroBiasSampling = false;
        _armGyroBiasSampleCount = 0;
        resetControlState();
        _armed = true;
        return;
    }

    if (!_armGyroBiasSampling)
    {
        resetControlState();
        beginArmGyroBiasSampling();
    }
}

bool FlightControlRuntime::readTransformedImu(FlightControlImuSample &sample, float dt, bool collectArmBiasSample)
{
    // Keep a single, explicit IMU pipeline:
    // sensor/raw frame -> configured orientation transform -> saved
    // aircraft-frame bias/scale calibration -> estimator input.
    if (!_sensors.read(sample))
    {
        return false;
    }

    _orientation.apply(sample);

    const float *accelBias = flightControlConfig.GetAccelBias();
    const float *accelScale = flightControlConfig.GetAccelScale();
    if (sample.gyroValid)
    {
        if (collectArmBiasSample && _armGyroBiasSampling)
        {
            collectArmGyroBiasSample(sample.gyroDps);
        }

        if (flightControlConfig.GetGyroBiasMode() == FLIGHT_CONTROL_GYRO_BIAS_CONFIGURED)
        {
            // Configurator calibration stores an aircraft-frame bias in fc.json.
            const float *gyroBias = flightControlConfig.GetGyroBias();
            sample.gyroDps.x -= gyroBias[0];
            sample.gyroDps.y -= gyroBias[1];
            sample.gyroDps.z -= gyroBias[2];
        }
        else
        {
            // The samples are accumulated after orientation transformation, so
            // the runtime bias is in the aircraft frame used by estimator/PIDs.
            sample.gyroDps.x -= _armGyroBiasDps.x;
            sample.gyroDps.y -= _armGyroBiasDps.y;
            sample.gyroDps.z -= _armGyroBiasDps.z;
        }
    }
    if (sample.accelValid)
    {
        sample.accelMps2.x = (sample.accelMps2.x - accelBias[0]) * accelScale[0];
        sample.accelMps2.y = (sample.accelMps2.y - accelBias[1]) * accelScale[1];
        sample.accelMps2.z = (sample.accelMps2.z - accelBias[2]) * accelScale[2];
    }

    // Debug consumers see calibrated aircraft-frame values before gyro LPF or
    // complementary-filter corrections are applied.
    _lastImuSample = sample;

    // Gyro LPF is input conditioning for both the complementary estimator and
    // the rate controller; it intentionally does not alter the debug sample.
    filterGyro(sample, dt);
    _lastFilteredGyroDps = sample.gyroDps;
    return true;
}

void FlightControlRuntime::filterGyro(FlightControlImuSample &sample, float dt)
{
    if (!sample.gyroValid)
    {
        return;
    }

    const uint8_t cutoffHz = flightControlConfig.GetGyroLpfHz();
    if (cutoffHz == 0 || dt <= 0.0f)
    {
        _filteredGyroDps = sample.gyroDps;
        _gyroFilterInitialized = false;
        _gyroFilterHz = cutoffHz;
        return;
    }

    if (!_gyroFilterInitialized || _gyroFilterHz != cutoffHz)
    {
        _filteredGyroDps = sample.gyroDps;
        _gyroFilterInitialized = true;
        _gyroFilterHz = cutoffHz;
    }
    else
    {
        const float rc = 1.0f / (FC_TWO_PI * cutoffHz);
        const float alpha = dt / (rc + dt);
        _filteredGyroDps.x += alpha * (sample.gyroDps.x - _filteredGyroDps.x);
        _filteredGyroDps.y += alpha * (sample.gyroDps.y - _filteredGyroDps.y);
        _filteredGyroDps.z += alpha * (sample.gyroDps.z - _filteredGyroDps.z);
    }
    sample.gyroDps = _filteredGyroDps;
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
    const float dtermLpfHz = flightControlConfig.GetDtermLpfHz();
    rollPid.set(pid[0] * FC_PID_CONFIG_SCALE, pid[1] * FC_PID_CONFIG_SCALE, pid[2] * FC_PID_CONFIG_SCALE, pid[3] * FC_PID_CONFIG_SCALE, dtermLpfHz);
    pitchPid.set(pid[4] * FC_PID_CONFIG_SCALE, pid[5] * FC_PID_CONFIG_SCALE, pid[6] * FC_PID_CONFIG_SCALE, pid[7] * FC_PID_CONFIG_SCALE, dtermLpfHz);
    yawPid.set(pid[8] * FC_PID_CONFIG_SCALE, pid[9] * FC_PID_CONFIG_SCALE, pid[10] * FC_PID_CONFIG_SCALE, pid[11] * FC_PID_CONFIG_SCALE, dtermLpfHz);
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

    // FlightControl_device already schedules this function every 4 ms. Do not
    // apply a second strict microsecond gate here: millisecond scheduler jitter
    // can invoke us a few microseconds early, and rejecting that invocation
    // turns an otherwise valid 4 ms update into an 8 ms gap.
    const uint32_t dtUs = _lastUpdateUs == 0 ? FC_UPDATE_INTERVAL_US : (uint32_t)(nowUs - _lastUpdateUs);
    const float dt = dtUs * 1e-6f;
    _lastUpdateUs = nowUs;
    _lastUpdateDtUs = (uint16_t)constrain(dtUs, 0U, 65535U);

    FlightControlImuSample sample = {};
    if (!readTransformedImu(sample, dt, false))
    {
        _sensorsReady = false;
        reset();
        return;
    }

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
    yaw = -constrain((yawUs - 1500.0f) / 500.0f, -1.0f, 1.0f) * FC_MAX_RATE_DPS;
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

    updateArmState();

    // Timing is owned by FlightControl_device. A duplicate gate here caused
    // alternating 4/8 ms control updates when the device scheduler was only a
    // few microseconds early.
    const uint32_t dtUs = _lastUpdateUs == 0 ? FC_UPDATE_INTERVAL_US : (uint32_t)(nowUs - _lastUpdateUs);
    const float dt = dtUs * 1e-6f;
    _lastUpdateUs = nowUs;
    _lastUpdateDtUs = (uint16_t)constrain(dtUs, 0U, 65535U);

    const bool gyroBiasWasSampling = _armGyroBiasSampling;
    FlightControlImuSample sample = {};
    if (!readTransformedImu(sample, dt, true))
    {
        _sensorsReady = false;
        reset();
        return;
    }
    _lastDebugUpdateMs = millis();
    _lastSampleAgeMs = sample.timestampMs == 0 ? 0 : (uint16_t)constrain((uint32_t)(_lastDebugUpdateMs - sample.timestampMs), 0U, 65535U);

    if (gyroBiasWasSampling)
    {
        // Hold safe outputs for all 100 samples. After the average is ready,
        // discard all estimator/filter history produced before the new bias and
        // begin normal flight control on the following sample.
        if (!_armGyroBiasSampling)
        {
            completeArmGyroBiasSampling();
            return;
        }
        _estimator.update(sample, dt);
        return;
    }

    _estimator.update(sample, dt);
    if (!_armed)
    {
        return;
    }
    const FlightControlMode nextMode = readModeSwitch();
    if (nextMode != _mode)
    {
        resetPidState();
        _mode = nextMode;
    }

    float throttle;
    float rollAngleTarget;
    float pitchAngleTarget;
    float yawRateTarget;
    loadStickTargets(throttle, rollAngleTarget, pitchAngleTarget, yawRateTarget);

    _rollAngleTarget = rollAngleTarget;
    _pitchAngleTarget = pitchAngleTarget;
    _yawRateTarget = yawRateTarget;

    if (_mode == FLIGHT_CONTROL_MODE_MANUAL)
    {
        const float roll = rollAngleTarget / FC_MAX_ROLL_PITCH_DEG * FC_MANUAL_CONTROL_RANGE;
        const float pitch = pitchAngleTarget / FC_MAX_ROLL_PITCH_DEG * FC_MANUAL_CONTROL_RANGE;
        const float yaw = yawRateTarget / FC_MAX_RATE_DPS * FC_MANUAL_CONTROL_RANGE;
        _mixerOutput = _mixer.mix(throttle, roll, pitch, yaw);
        return;
    }

    float rollRateTarget = rollAngleTarget / FC_MAX_ROLL_PITCH_DEG * FC_MAX_RATE_DPS;
    float pitchRateTarget = pitchAngleTarget / FC_MAX_ROLL_PITCH_DEG * FC_MAX_RATE_DPS;

    if (_mode == FLIGHT_CONTROL_MODE_ANGLE)
    {
        rollRateTarget = constrain(
            _rollAnglePid.update(rollAngleTarget, _estimator.attitude().rollDeg, dt),
            -(float)flightControlConfig.GetAngleRateLimitDps(0),
            (float)flightControlConfig.GetAngleRateLimitDps(0));
        pitchRateTarget = constrain(
            _pitchAnglePid.update(pitchAngleTarget, _estimator.attitude().pitchDeg, dt),
            -(float)flightControlConfig.GetAngleRateLimitDps(1),
            (float)flightControlConfig.GetAngleRateLimitDps(1));
    }
    _rollRateTarget = rollRateTarget;
    _pitchRateTarget = pitchRateTarget;

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
    snapshot.filteredGyroDps = _lastFilteredGyroDps;
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
    snapshot.rollAngleTarget = _rollAngleTarget;
    snapshot.pitchAngleTarget = _pitchAngleTarget;
    snapshot.rollAngleState = _estimator.attitude().rollDeg;
    snapshot.pitchAngleState = _estimator.attitude().pitchDeg;
    snapshot.rollRateTarget = _rollRateTarget;
    snapshot.pitchRateTarget = _pitchRateTarget;
    snapshot.yawRateTarget = _yawRateTarget;
    snapshot.armed = _armed;
    return _lastDebugUpdateMs != 0;
}

#endif
