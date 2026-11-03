#include "PIDController.h"
#include <algorithm>
#include <cmath>

template <typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

PIDController::PIDController(const Config& config)
    : cfg_(config),
    integral_(0.0f),
    lastMeasuredFreq_(0.0f),
    filteredDerivative_(0.0f),
    isFirstRun_(true) {
}

void PIDController::reset() {
    integral_ = 0.0f;
    lastMeasuredFreq_ = 0.0f;
    filteredDerivative_ = 0.0f;
    isFirstRun_ = true;
}

void PIDController::setConfig(const Config& config) {
    cfg_ = config;
    reset();
}

PIDController::Config PIDController::getConfig() const {
    return cfg_;
}

float PIDController::update(float targetFreq, float currentFreq, float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = targetFreq - currentFreq;

    // 1. Deadband check: if within acceptable pitch tolerance, stop motor and reset accumulator
    if (std::abs(error) <= cfg_.deadbandHz) {
        integral_ = 0.0f;
        filteredDerivative_ = 0.0f;
        lastMeasuredFreq_ = currentFreq;
        return 0.0f;
    }

    // 2. Proportional term
    float pTerm = cfg_.kp * error;

    // 3. Integral term with anti-windup clamping
    integral_ += error * dt;
    integral_ = clamp(integral_, -cfg_.maxIntegral, cfg_.maxIntegral);
    float iTerm = cfg_.ki * integral_;

    // 4. Derivative on Measurement (with low-pass filter)
    // Using derivative-on-measurement prevents "derivative kick" when switching targets,
    // and the filter eliminates high-frequency noise from FFT bin jitter.
    float dTerm = 0.0f;
    if (!isFirstRun_) {
        // d(error)/dt = -d(measured)/dt when target is static
        float rawDerivative = -(currentFreq - lastMeasuredFreq_) / dt;

        filteredDerivative_ = (cfg_.derivativeFilterAlpha * filteredDerivative_) +
            ((1.0f - cfg_.derivativeFilterAlpha) * rawDerivative);

        dTerm = cfg_.kd * filteredDerivative_;
    }
    else {
        isFirstRun_ = false;
    }
    lastMeasuredFreq_ = currentFreq;

    // 5. Total Output Sum and Saturation Clamping
    float totalOutput = pTerm + iTerm + dTerm;
    return clamp(totalOutput, cfg_.minOutput, cfg_.maxOutput);
}