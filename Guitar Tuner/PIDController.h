#pragma once

class PIDController {
public:
    struct Config {
        float kp = 2.0f;               // Proportional gain: immediate response to pitch error
        float ki = 0.08f;              // Integral gain: overcomes static friction and gear stiction
        float kd = 0.15f;              // Derivative gain: dampens overshoot caused by string elasticity
        float deadbandHz = 0.25f;      // Tolerance window (+/- Hz) where motor output is zeroed
        float minOutput = -100.0f;     // Minimum motor command (% PWM or step rate)
        float maxOutput = 100.0f;      // Maximum motor command (% PWM or step rate)
        float maxIntegral = 30.0f;     // Anti-windup limit for the accumulator
        float derivativeFilterAlpha = 0.7f; // Low-pass filter weight for D term (0.0 to 1.0)
    };

    explicit PIDController(const Config& config = Config());

    /**
     * Calculates the control output based on current frequency error.
     *
     * @param targetFreq Target pitch in Hz.
     * @param currentFreq Measured fundamental pitch from FFT in Hz.
     * @param dt Elapsed time since the last update in seconds.
     * @return Motor command: positive = tighten, negative = loosen, 0 = within tolerance.
     */
    float update(float targetFreq, float currentFreq, float dt);

    void reset();

    void setConfig(const Config& config);
    Config getConfig() const;

private:
    Config cfg_;
    float integral_;
    float lastMeasuredFreq_;
    float filteredDerivative_;
    bool isFirstRun_;
};