#include "GuitarPitchFFT.h"

#include <algorithm>
#include <cmath>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GuitarPitchFFT::GuitarPitchFFT(size_t fftSize, float sampleRate, float minFreqHz, float maxFreqHz)
    : fftSize_(fftSize),
    sampleRate_(sampleRate),
    minFreqHz_(minFreqHz),
    maxFreqHz_(maxFreqHz),
    noiseThreshold_(0.02f) {
    precomputeTables();
    allocateBuffers();
}

void GuitarPitchFFT::setNoiseThreshold(float threshold) {
    noiseThreshold_ = threshold;
}

size_t GuitarPitchFFT::getFFTSize() const {
    return fftSize_;
}

float GuitarPitchFFT::getSampleRate() const {
    return sampleRate_;
}

void GuitarPitchFFT::precomputeTables() {
    // Hann window: w[n] = 0.5 * (1 - cos(2*pi*n / (N - 1)))
    window_.resize(fftSize_);
    for (size_t i = 0; i < fftSize_; ++i) {
        window_[i] = 0.5f * (1.0f - std::cos(2.0f * static_cast<float>(M_PI) * i / (fftSize_ - 1)));
    }

    // Bit-reversal indices
    bitReverseMap_.resize(fftSize_);
    for (size_t i = 0; i < fftSize_; ++i) {
        size_t rev = 0;
        size_t temp = i;
        for (size_t j = 1; j < fftSize_; j <<= 1) {
            rev = (rev << 1) | (temp & 1);
            temp >>= 1;
        }
        bitReverseMap_[i] = rev;
    }

    // Twiddle factors
    cosTable_.resize(fftSize_ / 2);
    sinTable_.resize(fftSize_ / 2);
    for (size_t i = 0; i < fftSize_ / 2; ++i) {
        float angle = -2.0f * static_cast<float>(M_PI) * i / static_cast<float>(fftSize_);
        cosTable_[i] = std::cos(angle);
        sinTable_[i] = std::sin(angle);
    }
}

void GuitarPitchFFT::allocateBuffers() {
    real_.resize(fftSize_);
    imag_.resize(fftSize_);
    magnitudes_.resize(fftSize_ / 2);
}

void GuitarPitchFFT::computeFFT() {
    // Bit-reversal permutation
    for (size_t i = 0; i < fftSize_; ++i) {
        size_t j = bitReverseMap_[i];
        if (i < j) {
            std::swap(real_[i], real_[j]);
            std::swap(imag_[i], imag_[j]);
        }
    }

    // Cooley-Tukey butterfly stages
    for (size_t len = 2; len <= fftSize_; len <<= 1) {
        size_t halfLen = len >> 1;
        size_t step = fftSize_ / len;

        for (size_t i = 0; i < fftSize_; i += len) {
            for (size_t j = 0; j < halfLen; ++j) {
                size_t tableIdx = j * step;
                float wr = cosTable_[tableIdx];
                float wi = sinTable_[tableIdx];

                size_t uIdx = i + j;
                size_t vIdx = i + j + halfLen;

                float tr = wr * real_[vIdx] - wi * imag_[vIdx];
                float ti = wr * imag_[vIdx] + wi * real_[vIdx];

                real_[vIdx] = real_[uIdx] - tr;
                imag_[vIdx] = imag_[uIdx] - ti;
                real_[uIdx] += tr;
                imag_[uIdx] += ti;
            }
        }
    }
}

float GuitarPitchFFT::refinePeakParabolic(size_t k) {
    if (k == 0 || k >= (fftSize_ / 2) - 1) {
        return static_cast<float>(k);
    }

    float alpha = magnitudes_[k - 1];
    float beta = magnitudes_[k];
    float gamma = magnitudes_[k + 1];

    float denominator = alpha - 2.0f * beta + gamma;
    if (std::abs(denominator) < 1e-7f) {
        return static_cast<float>(k);
    }

    float delta = 0.5f * (alpha - gamma) / denominator;
    return static_cast<float>(k) + delta;
}

GuitarPitchFFT::DetectionResult GuitarPitchFFT::detectPitch(const float* inputSamples) {
    DetectionResult result;

    // 1. Apply Hann window into preallocated buffers
    for (size_t i = 0; i < fftSize_; ++i) {
        real_[i] = inputSamples[i] * window_[i];
        imag_[i] = 0.0f;
    }

    // 2. In-place FFT
    computeFFT();

    // 3. Magnitudes up to Nyquist limit
    const size_t halfSize = fftSize_ / 2;
    float globalMaxMag = 0.0f;

    for (size_t i = 0; i < halfSize; ++i) {
        magnitudes_[i] = std::sqrt(real_[i] * real_[i] + imag_[i] * imag_[i]);
        if (magnitudes_[i] > globalMaxMag) {
            globalMaxMag = magnitudes_[i];
        }
    }

    if (globalMaxMag < noiseThreshold_) {
        result.isSignalDetected = false;
        return result;
    }

    // 4. Band-limited peak search
    const size_t minBin = std::max(static_cast<size_t>(1),
        static_cast<size_t>((minFreqHz_ * fftSize_) / sampleRate_));
    const size_t maxBin = std::min(halfSize - 2,
        static_cast<size_t>((maxFreqHz_ * fftSize_) / sampleRate_));

    size_t candidateBin = minBin;
    float maxBandMag = 0.0f;

    for (size_t i = minBin; i <= maxBin; ++i) {
        if (magnitudes_[i] > maxBandMag) {
            maxBandMag = magnitudes_[i];
            candidateBin = i;
        }
    }

    // 5. Octave Error / Harmonic Rejection
    size_t subHarmonicBin = candidateBin / 2;
    if (subHarmonicBin >= minBin) {
        size_t bestSubBin = subHarmonicBin;
        float maxSubMag = 0.0f;
        for (int offset = -2; offset <= 2; ++offset) {
            int testBin = static_cast<int>(subHarmonicBin) + offset;
            if (testBin >= static_cast<int>(minBin) && testBin < static_cast<int>(maxBin)) {
                if (magnitudes_[testBin] > maxSubMag) {
                    maxSubMag = magnitudes_[testBin];
                    bestSubBin = testBin;
                }
            }
        }
        if (maxSubMag > 0.25f * maxBandMag) {
            candidateBin = bestSubBin;
            maxBandMag = maxSubMag;
        }
    }

    // 6. Sub-bin parabolic refinement
    float interpolatedBin = refinePeakParabolic(candidateBin);

    result.frequencyHz = interpolatedBin * (sampleRate_ / static_cast<float>(fftSize_));
    result.peakMagnitude = maxBandMag;
    result.isSignalDetected = true;

    return result;
}