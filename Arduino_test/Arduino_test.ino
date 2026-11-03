const int MIC_PIN = A0;
const int DC_BIAS = 512;
const int HYSTERESIS = 35;

struct Note {
    const char* name;
    float freq;
};

const Note STRINGS[6] = {
    {"E2 (String 6)",  82.41f},
    {"A2 (String 5)", 110.00f},
    {"D3 (String 4)", 146.83f},
    {"G3 (String 3)", 196.00f},
    {"B3 (String 2)", 246.94f},
    {"E4 (String 1)", 329.63f}
};

void setup() {
    Serial.begin(115200);

    ADCSRA = (ADCSRA & 0xF8) | 0x04;

    Serial.println(F("Arduino Uno Guitar Pitch Monitor Ready"));
    Serial.println(F("Pluck a string or play a test tone..."));
}

void loop() {
    const int NUM_PERIODS = 8; // Average over 8 complete wave cycles
    unsigned long periodDurations = 0;

    // Wait for signal to go clearly below center bias
    while (analogRead(MIC_PIN) > (DC_BIAS - HYSTERESIS));

    // Measure elapsed time over multiple full wave oscillations
    for (int cycle = 0; cycle < NUM_PERIODS; ++cycle) {
        // 1. Wait for rising edge crossing above (bias + hysteresis)
        while (analogRead(MIC_PIN) < (DC_BIAS + HYSTERESIS));
        unsigned long tStart = micros();

        // 2. Wait for falling edge crossing below (bias - hysteresis)
        while (analogRead(MIC_PIN) > (DC_BIAS - HYSTERESIS));

        // 3. Wait for the next rising edge to complete 1 full period
        while (analogRead(MIC_PIN) < (DC_BIAS + HYSTERESIS));
        unsigned long tEnd = micros();

        periodDurations += (tEnd - tStart);
    }

    // Average wave period in seconds
    float avgPeriodSeconds = (periodDurations / (float)NUM_PERIODS) / 1000000.0f;

    if (avgPeriodSeconds > 0.002f && avgPeriodSeconds < 0.02f) { // Bounds: 50 Hz to 500 Hz
        float freqHz = 1.0f / avgPeriodSeconds;

        // Find nearest string
        int bestIdx = 0;
        float minDiff = 1000.0f;
        for (int i = 0; i < 6; ++i) {
            float diff = abs(freqHz - STRINGS[i].freq);
            if (diff < minDiff) {
                minDiff = diff;
                bestIdx = i;
            }
        }

        Serial.print(F("Nearest: "));
        Serial.print(STRINGS[bestIdx].name);
        Serial.print(F(" | Measured: "));
        Serial.print(freqHz, 2);
        Serial.print(F(" Hz | Target: "));
        Serial.print(STRINGS[bestIdx].freq, 2);
        Serial.print(F(" Hz | Err: "));
        Serial.println(freqHz - STRINGS[bestIdx].freq, 2);
    }

    delay(100);
}