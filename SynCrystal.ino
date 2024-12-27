#include <avr/io.h>
#include <avr/interrupt.h>

// Pins und Variablen
constexpr int shaftPulsePin = 2;
constexpr int greenLedPin = 7;
constexpr int redLedPin = 9;
volatile unsigned long lastShaftPulseTime = 0;
constexpr unsigned long STOP_THRESHOLD = 10000; // Threshold in microseconds to detect a stop

// This is for the median approach, which finds stable freq detection after 48 impulses (4 frames). 
constexpr size_t STABILITY_WINDOW_SIZE = 12;        // Größe des Median-Fensters
constexpr size_t STABILITY_CHECKS = 36;             // Größe des Stabilitäts-Fensters
constexpr unsigned long SPEED_DETECT_TOLERANCE = 1600;           // Fester Toleranzwert in Mikrosekunden
constexpr unsigned long MIN_CHANGE = 800;           // Minimale Abweichung für eine neue Stabilität

volatile unsigned long freqMedianBuffer[STABILITY_WINDOW_SIZE];
volatile size_t freqMedianIndex = 0;
volatile unsigned long stabilityBuffer[STABILITY_CHECKS];
volatile size_t freqBufferIndex = 0;
volatile unsigned long lastStableFreqValue = 0; // Zuletzt erkannter stabiler Wert
volatile unsigned long shaftImpulseCount = 0;
volatile bool newShaftImpulseAvailable = false;
volatile bool projectorRunning = false; // true = Running, false = Stopped


// Median berechnen. A rolling average might be cehaper and good enough, esp with filtering outliers.

unsigned long calculateMedian(volatile unsigned long *buffer, size_t size)
{
    unsigned long temp[size];
    // We need a copy of the array to not get interference with the ISR. Intereference doesnt seem likely, wo maybe 100B to save here
    for (size_t i = 0; i < size; i++)
    {
        temp[i] = buffer[i];
    }

    // Einfache Sortierung (Insertion Sort)
    for (size_t i = 1; i < size; i++)
    {
        unsigned long key = temp[i];
        size_t j = i;
        while (j > 0 && temp[j - 1] > key)
        {
            temp[j] = temp[j - 1];
            j--;
        }
        temp[j] = key;
    }

    // Determine Median 
    return (size % 2 == 0) ? (temp[size / 2 - 1] + temp[size / 2]) / 2 : temp[size / 2];
}

// Stabilitätsprüfung
bool checkStability(volatile unsigned long *buffer, size_t size, unsigned long tolerance)
{
    unsigned long minVal = buffer[0];
    unsigned long maxVal = buffer[0];
    for (size_t i = 1; i < size; i++)
    {
        if (buffer[i] < minVal)
            minVal = buffer[i];
        if (buffer[i] > maxVal)
            maxVal = buffer[i];
    }
    return (maxVal - minVal) <= tolerance;
}

void setup()
{
    Serial.begin(115200);
    pinMode(shaftPulsePin, INPUT);
    pinMode(greenLedPin, OUTPUT);
    pinMode(redLedPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(shaftPulsePin), onShaftImpulse, RISING); // We only want one edge of the signal to not be duty cycle dependent
}

void loop()
{
    if (!newShaftImpulseAvailable)
        return; // Skip processing if no new data

    newShaftImpulseAvailable = false; // Reset the ISR flag

    // Calculate median and store it in the stability buffer
    unsigned long median = calculateMedian(freqMedianBuffer, STABILITY_WINDOW_SIZE);
    stabilityBuffer[freqBufferIndex] = median;
    freqBufferIndex = (freqBufferIndex + 1) % STABILITY_CHECKS;

    // Perform stability check if the buffer is full
    if (freqBufferIndex == 0 && checkStability(stabilityBuffer, STABILITY_CHECKS, SPEED_DETECT_TOLERANCE))
    {
        unsigned long newStableValue = calculateMedian(stabilityBuffer, STABILITY_CHECKS);

        // Handle projector restart or stability change
        if (!projectorRunning || abs((long)newStableValue - (long)lastStableFreqValue) > MIN_CHANGE)
        {
            lastStableFreqValue = newStableValue;
            float detectedFrequency = 1000000.0f / 12.0f / (float)lastStableFreqValue;
            projectorRunning = true; // Projector is running again
            Serial.print("[DEBUG] Projector running stable with ~");
            Serial.print(detectedFrequency, 1); // FPS
            Serial.print(" fps after ");
            Serial.print(shaftImpulseCount);
            Serial.print(" impulses. ");
            Serial.println(1000000 / STABILITY_WINDOW_SIZE / lastStableFreqValue);
            if ((detectedFrequency > 16) || (detectedFrequency < 20))
            {
                projectorSpeedSwitchPos = 18;
            }
            else if ((detectedFrequency > 22) || (detectedFrequency < 26)) 
            {
                projectorSpeedSwitchPos = 24;
            }
        }
    }
}

void onShaftImpulse()
{
    unsigned long currentTime = micros();
    unsigned long interval = currentTime - lastShaftPulseTime;
    lastShaftPulseTime = currentTime;

    shaftImpulseCount++;

    // Update the frequency buffer
    freqMedianBuffer[freqMedianIndex] = interval;
    freqMedianIndex = (freqMedianIndex + 1) % STABILITY_WINDOW_SIZE;

    // Detect if the interval exceeds the stop threshold
    if (interval > STOP_THRESHOLD && projectorRunning)
    {
        projectorRunning = false; // Projector is stopped
        Serial.println("[DEBUG] Projector stopped.");
        shaftImpulseCount = 0;
    }

    // Mark new data available
    newShaftImpulseAvailable = true;
}