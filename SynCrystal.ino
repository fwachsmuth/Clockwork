#include <avr/io.h>
#include <avr/interrupt.h>

// Pins und Variablen
constexpr int shaftPulsePin = 2;
constexpr int greenLedPin = 7;
constexpr int redLedPin = 9;
volatile unsigned long lastShaftPulseTime = 0;

// This is for the median approach, which finds stable freq detection after 48 impulses (4 frames). 
constexpr size_t STABILITY_WINDOW_SIZE = 12;        // Größe des Median-Fensters
constexpr size_t STABILITY_CHECKS = 24;             // Größe des Stabilitäts-Fensters
constexpr unsigned long TOLERANCE = 1600;           // Fester Toleranzwert in Mikrosekunden
constexpr unsigned long MIN_CHANGE = 800;           // Minimale Abweichung für eine neue Stabilität

volatile unsigned long freqMedianBuffer[STABILITY_WINDOW_SIZE];
volatile size_t freqMedianIndex = 0;
volatile unsigned long stabilityBuffer[STABILITY_CHECKS];
volatile size_t freqBufferIndex = 0;
volatile unsigned long lastStableFreqValue = 0; // Zuletzt erkannter stabiler Wert
volatile unsigned long shaftImpulseCount = 0;
volatile bool newShaftImpulseAvailable = false;
volatile int projectorRunState = 1; // 1 = Running, 0 = Stopped


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
        return; // Exit early if no new data is available

    newShaftImpulseAvailable = false; // Reset the ISR flag

    // Calculate the median for the frequency buffer
    unsigned long median = calculateMedian(freqMedianBuffer, STABILITY_WINDOW_SIZE);

    // Store the median in the stability buffer
    stabilityBuffer[freqBufferIndex] = median;
    freqBufferIndex = (freqBufferIndex + 1) % STABILITY_CHECKS;

    // Perform stability check when the stability buffer is full
    if (freqBufferIndex == 0 && checkStability(stabilityBuffer, STABILITY_CHECKS, TOLERANCE))
    {
        unsigned long newStableValue = calculateMedian(stabilityBuffer, STABILITY_CHECKS);

        // Update stability if there's a significant change or the projector was stopped
        if (abs((long)newStableValue - (long)lastStableFreqValue) > MIN_CHANGE || projectorRunState == 0)
        {
            lastStableFreqValue = newStableValue;
            projectorRunState = 1; // Projector is running again

            // Output stability detection
            Serial.print("New stability detected after ");
            Serial.print(shaftImpulseCount);
            Serial.print(" impulses. Stable Interval: ");
            Serial.print(lastStableFreqValue);
            Serial.print(" equals ");
            Serial.print(1000000.0f / 12.0f / (float)lastStableFreqValue, 2); // FPS
            Serial.println(" fps");
        }
    }
}

void onShaftImpulse()
{
    unsigned long currentTime = micros();
    unsigned long interval = currentTime - lastShaftPulseTime;
    lastShaftPulseTime = currentTime;

    shaftImpulseCount++;

    // Update the frequency buffer and index
    freqMedianBuffer[freqMedianIndex] = interval;
    freqMedianIndex = (freqMedianIndex + 1) % STABILITY_WINDOW_SIZE;

    // Detect if the interval exceeds twice the last stable interval
    if (lastStableFreqValue > 0 && interval > 2 * lastStableFreqValue)
    {
        projectorRunState = 0; // Stop the projector immediately
        Serial.println("Projector stopped due to large intervals.");
    }

    // Mark new data available for the main loop
    newShaftImpulseAvailable = true;
}