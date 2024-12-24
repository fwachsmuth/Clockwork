#include <avr/io.h>
#include <avr/interrupt.h>

// Pins und Variablen
constexpr int shaftPulsePin = 2;
constexpr int greenLedPin = 7;
constexpr int redLedPin = 9;
volatile unsigned long lastShaftPulseTime = 0;

constexpr size_t WINDOW_SIZE = 12;        // Größe des Median-Fensters
constexpr size_t STABILITY_CHECK = 24;    // Größe des Stabilitäts-Fensters
constexpr unsigned long TOLERANCE = 1600;  // Fester Toleranzwert in Mikrosekunden
constexpr unsigned long MIN_CHANGE = 800; // Minimale Abweichung für eine neue Stabilität

volatile unsigned long medianBuffer[WINDOW_SIZE];
volatile size_t medianIndex = 0;
volatile unsigned long stabilityBuffer[STABILITY_CHECK];
volatile size_t stabilityIndex = 0;
volatile unsigned long lastStableValue = 0; // Zuletzt erkannter stabiler Wert
volatile unsigned long impulseCount = 0;
volatile bool newValueAvailable = false;


// Median berechnen

unsigned long calculateMedianSmallArray(volatile unsigned long *buffer, size_t size)
{
    unsigned long temp[size];
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
    attachInterrupt(digitalPinToInterrupt(shaftPulsePin), onShaftImpulse, RISING);
}

void loop()
{
    // Prüfe, ob neue Werte verfügbar sind
    if (newValueAvailable)
    {
        newValueAvailable = false; // Zurücksetzen des Flags

        // Median berechnen
        unsigned long median = calculateMedianSmallArray(medianBuffer, WINDOW_SIZE);
        stabilityBuffer[stabilityIndex] = median;
        stabilityIndex = (stabilityIndex + 1) % STABILITY_CHECK;

        // Nur Stabilitätsprüfung durchführen, wenn der Buffer gefüllt ist
        if (stabilityIndex == 0 && checkStability(stabilityBuffer, STABILITY_CHECK, TOLERANCE))
        {
            unsigned long newStableValue = calculateMedianSmallArray(stabilityBuffer, STABILITY_CHECK);

            // Prüfe auf signifikante Änderung
            if (abs((long)newStableValue - (long)lastStableValue) > MIN_CHANGE)
            {
                lastStableValue = newStableValue;

                // Ausgabe
                Serial.print("New stability detected after ");
                Serial.print(impulseCount);
                Serial.print(" impulses. Stable Interval: ");
                Serial.print(lastStableValue);
                Serial.print(" equals ");
                Serial.print(1000000.0f / 12.0f / (float)lastStableValue, 2); // Ausgabe mit 2 Dezimalstellen
                Serial.println(" fps ");
            }
        }
    }
}

void onShaftImpulse()
{
    unsigned long currentTime = micros();
    unsigned long interval = currentTime - lastShaftPulseTime;
    lastShaftPulseTime = currentTime;

    impulseCount++;

    // Median-Berechnung im loop(), nicht hier
    medianBuffer[medianIndex] = interval;
    medianIndex = (medianIndex + 1) % WINDOW_SIZE;

    // Markiere, dass ein neuer Wert verfügbar ist
    newValueAvailable = true;
}