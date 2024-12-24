#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

// Pins und Variablen
constexpr int shaftPulsePin = 2;
constexpr int greenLedPin = 7;
constexpr int redLedPin = 9;
volatile unsigned long lastShaftPulseTime = 0;

constexpr size_t WINDOW_SIZE = 12;        // Größe des Median-Fensters
constexpr size_t STABILITY_CHECK = 24;    // Größe des Stabilitäts-Fensters
constexpr unsigned long TOLERANCE = 800; // Fester Toleranzwert in Mikrosekunden
constexpr unsigned long MIN_CHANGE = 800; // Minimale Abweichung für eine neue Stabilität

volatile unsigned long medianBuffer[WINDOW_SIZE];
volatile size_t medianIndex = 0;
volatile unsigned long stabilityBuffer[STABILITY_CHECK];
volatile size_t stabilityIndex = 0;
volatile bool isStable = false;
volatile unsigned long stableValue = 0;
volatile unsigned long lastStableValue = 0; // Zuletzt erkannter stabiler Wert
volatile unsigned long impulseCount = 0;

// Vergleichsfunktion für qsort
int compare(const void *a, const void *b)
{
    unsigned long intA = *(unsigned long *)a;
    unsigned long intB = *(unsigned long *)b;
    return (intA > intB) - (intA < intB);
}

// Median berechnen
unsigned long calculateMedian(volatile unsigned long *buffer, size_t size)
{
    unsigned long temp[size];
    for (size_t i = 0; i < size; i++)
    {
        temp[i] = buffer[i];
    }
    qsort(temp, size, sizeof(unsigned long), compare);
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
    if (isStable)
    {
        // Ausgabe nur bei einer neuen Stabilität
        Serial.print("New stability detected after ");
        Serial.print(impulseCount);
        Serial.print(" impulses. Stable Interval: ");
        Serial.println(stableValue);

        digitalWrite(greenLedPin, HIGH);
        isStable = false; // Zurücksetzen für die nächste Stabilitätsprüfung
    }
    else
    {
        digitalWrite(greenLedPin, LOW);
    }
}

void onShaftImpulse()
{
    unsigned long currentTime = micros();
    unsigned long interval = currentTime - lastShaftPulseTime;
    lastShaftPulseTime = currentTime;

    impulseCount++;

    // Median berechnen
    medianBuffer[medianIndex] = interval;
    medianIndex = (medianIndex + 1) % WINDOW_SIZE;
    unsigned long median = calculateMedian(medianBuffer, WINDOW_SIZE);

    // Stabilität prüfen
    stabilityBuffer[stabilityIndex] = median;
    stabilityIndex = (stabilityIndex + 1) % STABILITY_CHECK;

    if (stabilityIndex == 0 && checkStability(stabilityBuffer, STABILITY_CHECK, TOLERANCE))
    {
        unsigned long newStableValue = calculateMedian(stabilityBuffer, STABILITY_CHECK);

        // Prüfe, ob sich der neue Wert signifikant vom letzten unterscheidet
        if (abs((long)newStableValue - (long)lastStableValue) > MIN_CHANGE)
        {
            lastStableValue = newStableValue;
            isStable = true;
            stableValue = newStableValue;
        }
    }
}