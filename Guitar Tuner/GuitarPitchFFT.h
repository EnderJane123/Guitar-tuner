#pragma once

#include <cstddef>
#include <vector>

typedef struct DetectionResult {
    float frequencyHz = 0.0f;
    float peakMagnitude = 0.0f;
    bool isSignalDetected = false;
} DetectionResult;

class GuitarPitchFFT {
public:
    GuitarPitchFFT(size_t fftSize = 2048, float sampleRate = 4096.0f, float minFreqHz = 70.0f, float maxFreqHz = 360.0f);

    DetectionResult detectPitch(const float* inputSamples);

    void setNoiseThreshold(float threshold);
    size_t getFFTSize() const;
    float getSampleRate() const;

private:
    void precomputeTables();
    void allocateBuffers();
    void computeFFT();
    float refinePeakParabolic(size_t k);

    size_t fftSize_;
    float sampleRate_;
    float minFreqHz_;
    float maxFreqHz_;
    float noiseThreshold_;

    // Lookup tables
    std::vector<float> window_;
    std::vector<size_t> bitReverseMap_;
    std::vector<float> cosTable_;
    std::vector<float> sinTable_;

    // Pre-allocated work buffers
    std::vector<float> real_;
    std::vector<float> imag_;
    std::vector<float> magnitudes_;
};