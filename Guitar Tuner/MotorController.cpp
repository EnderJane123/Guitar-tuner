#include "MotorController.h"
#include "PIDController.h"
#include <iostream>
#include <vector>
#include <string>

void printAsciiPlot(const std::vector<float>& history, float targetHz, float minHz, float maxHz) {
    const int plotWidth = 50;
    std::cout << "\n--- Frequency Convergence Plot ---\n";
    std::cout << "Target: " << targetHz << " Hz | Tolerance: +/- 0.25 Hz\n";

    for (size_t i = 0; i < history.size(); i += 2) { // Downsample rows for clean output
        float freq = history[i];
        int targetCol = static_cast<int>((targetHz - minHz) / (maxHz - minHz) * plotWidth);
        int freqCol = static_cast<int>((freq - minHz) / (maxHz - minHz) * plotWidth);
        targetCol = clamp_val(targetCol, 0, plotWidth);
        freqCol = clamp_val(freqCol, 0, plotWidth);

        std::string line(plotWidth + 1, ' ');
        line[targetCol] = '|'; // Target line
        line[freqCol] = '*'; // Actual pitch

        std::cout << std::fixed << std::setprecision(2)
            << "t=" << std::setw(4) << (i * 0.05f) << "s ["
            << std::setw(6) << freq << " Hz] " << line << "\n";
    }
}

int main() {
    const float targetPitch = 110.00f;   // String 5 (A2)
    const float initialPitch = 104.50f;  // Detuned by -5.5 Hz
    const float dt = 0.05f;              // 50ms control loop interval (20 Hz)
    const float simDuration = 3.0f;      // 3 seconds simulation

    PIDController::Config pidCfg;
    pidCfg.kp = 2.2f;
    pidCfg.ki = 0.12f;
    pidCfg.kd = 0.18f;
    pidCfg.deadbandHz = 0.25f;

    PIDController pid(pidCfg);
    SimulatedGuitarMotor motor(initialPitch);

    std::vector<float> pitchHistory;
    float minObserved = initialPitch - 1.0f;
    float maxObserved = targetPitch + 1.0f;

    std::cout << "Starting Simulation: " << initialPitch << " Hz -> " << targetPitch << " Hz\n";

    for (float t = 0.0f; t <= simDuration; t += dt) {
        float currentPitch = motor.getMeasuredPitch();
        pitchHistory.push_back(currentPitch);

        // 1. Calculate motor control effort using PID
        float motorEffort = pid.update(targetPitch, currentPitch, dt);

        // 2. Drive the motor interface
        motor.setEffort(motorEffort);

        // 3. Update physics
        motor.stepSimulation(dt);

        if (std::abs(targetPitch - currentPitch) <= pidCfg.deadbandHz && motorEffort == 0.0f) {
            std::cout << ">> String in tune at t = " << t << "s!\n";
            break;
        }
    }

    printAsciiPlot(pitchHistory, targetPitch, minObserved, maxObserved);
    return 0;
}