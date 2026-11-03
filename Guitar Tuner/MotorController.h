#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

// Compatibility clamp for C++11/C++14
template <typename T>
constexpr const T& clamp_val(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

// ============================================================================
// Abstract Motor Interface
// ============================================================================
class IMotor {
public:
    virtual ~IMotor() = default;
    // Sets effort from -100.0% (loosen) to +100.0% (tighten). 0 = brake/stop.
    virtual void setEffort(float percent) = 0;
    virtual void stop() = 0;
};

// ============================================================================
// Simulated Motor + Nylon String Physics
// ============================================================================
class SimulatedGuitarMotor : public IMotor {
public:
    /**
     * @param initialPitchHz Starting string pitch.
     * @param hzPerPercentSec Rate of pitch change per 1% motor power per second.
     *                        (Represents N20 gearbox ratio + peg spool diameter).
     */
    SimulatedGuitarMotor(float initialPitchHz, float hzPerPercentSec = 0.08f)
        : currentPitchHz_(initialPitchHz),
        hzRate_(hzPerPercentSec),
        currentEffort_(0.0f) {
    }

    void setEffort(float percent) override {
        currentEffort_ = clamp_val(percent, -100.0f, 100.0f);
    }

    void stop() override {
        currentEffort_ = 0.0f;
    }

    // Advances the physical string model by dt seconds
    void stepSimulation(float dt) {
        // Stiction threshold: Motor will not turn if PWM effort is too weak to fight string tension
        const float stictionThreshold = 4.0f; // 4% minimum power
        if (std::abs(currentEffort_) < stictionThreshold) {
            return;
        }

        // Change in pitch: df = Effort * Rate * dt
        currentPitchHz_ += currentEffort_ * hzRate_ * dt;
    }

    float getMeasuredPitch() const { return currentPitchHz_; }

private:
    float currentPitchHz_;
    float hzRate_;
    float currentEffort_;
};