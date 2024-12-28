#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h> // Fancy DAC for voltage control
#include <PID_v1.h>           // using 1.2.0 from https://github.com/br3ttb/Arduino-PID-Library

Adafruit_MCP4725 dac; // Instantiate the DAC

// Pins und Variablen
constexpr int shaftPulsePin = 2;
constexpr int greenLedPin = 7;
constexpr int redLedPin = 9;

const byte ledSlowerRed = 5;    //  --
const byte ledSlowerYellow = 6; //  -
const byte ledGreen = 7;        //  o
const byte ledFasterYellow = 8; //  +
const byte ledFasterRed = 9;    //  ++

uint16_t dacValue = 1200; // start somewhere, e.g. 1537. This is about 17.6 fps on my projector

// to get the approx. DAC value for any desired fps per a * x * x + b * x + c
const float a = 0.6126;
const float b = 155.44;
const float c = -1459.34;

#define FPS_18 1
#define FPS_24 2
#define FPS_25 3
#define FPS_9 4
#define FPS_16_2_3 5

int timerFactor = 0;           // this is used for the Timer1 "postscaler", since multiples of 18 and 24 Hz give better accuracy
volatile int timerModulus = 0; // For Modulo in the ISR, to compensate the timerFactor
volatile long timerFrames = 0; // This is the timer1 (frequency / timerFactor) — equalling actual desired fps (no multiples)

const byte shaftSegmentDiscDivider = 1; // Increase this if we only want to use every nth pulse
volatile int shaftModulus = 0; // For Modulo in the ISR, to compensate the multiple pulses per revolution
volatile long projectorFrames = 0; // This is the actually advanced frames (pulses / shaftSegmentDiscDivider)

// flags to assure reading only once both ISRs have done theri duty
volatile bool projectorFrameCountUpdated;
volatile bool timerFrameCountUpdated;

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
volatile byte projectorSpeedSwitchPos; // holds the (guessed) current speed switch position (18 or 24)
volatile unsigned long timer2OverflowCount = 0; // Globale Zähler-Variable für Timer2-Überläufe

// PID stuff
double Setpoint, Input, Output;
double Kp = 10, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE); // P_ON_M?

// Median berechnen. A rolling average might be cehaper and good enough, esp with filtering outliers.

unsigned long
calculateMedian(volatile unsigned long *buffer, size_t size)
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
    // Timer2 konfigurieren (replaces micros() in the ISR)
    TCCR2A = 0;            // Normaler Modus
    TCCR2B = (1 << CS22);  // Prescaler = 64 (1 Tick = 4 µs bei 16 MHz)
    TCNT2 = 0;             // Timer2 zurücksetzen
    TIMSK2 = (1 << TOIE2); // Overflow Interrupt aktivieren
    
    Serial.begin(115200);
    pinMode(shaftPulsePin, INPUT);
    pinMode(greenLedPin, OUTPUT);
    pinMode(redLedPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(shaftPulsePin), onShaftImpulseISR, RISING); // We only want one edge of the signal to not be duty cycle dependent
    dac.begin(0x60);

    dac.setVoltage(dacValue, false); // 1537 is petty much 18 fps

    Input = 0;
    Setpoint = 0;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 4095);
    myPID.SetSampleTime(26); // Nyquist: Measure twice in 1/18 Sek (= 26 msec)
}

void loop()
{
    static long lastDifference = 0; // Stores the last output difference
    static long localTimerFrames = 0;
    static long localProjectorFrames = 0;
    static long currentPulseDifference = 0;
    static long correctionSignal = 0;
    uint16_t newDacValue = 0;

    // only read a difference if both ISRs did their updates yet, otherwise we get plenty of false changes
    if (projectorFrameCountUpdated && timerFrameCountUpdated)
    {
        noInterrupts();
        localTimerFrames = timerFrames;
        localProjectorFrames = projectorFrames;
        interrupts();

        currentPulseDifference = localTimerFrames - localProjectorFrames;

        projectorFrameCountUpdated = false;
        timerFrameCountUpdated = false;

        Input = currentPulseDifference;
        myPID.Compute();
        newDacValue = dacValue + Output;
        dac.setVoltage(newDacValue, false);
    }

    // Print only if the difference has changed
    if (currentPulseDifference != lastDifference)
    {
        // Serial.print("localTimerFrames: ");
        // Serial.print(localTimerFrames);
        // Serial.print(", localProjectorFrames: ");
        // Serial.print(localProjectorFrames);
        // Serial.print(", correct to ");

        // correctionSignal = currentPulseDifference * 2;

        Serial.print("Input: ");
        Serial.print(currentPulseDifference);
        Serial.print(", Output: ");
        Serial.print(Output);
        Serial.print(", new DAC: ");

        //newDacValue = dacValue + correctionSignal;
        // Ensure newDacValue stays in [0, 4095]
        if (newDacValue < 0)
        {
            newDacValue = 0;
        }
        else if (newDacValue > 4095)
        {
            newDacValue = 4095;
        }

        Serial.println(newDacValue);

        dac.setVoltage(newDacValue, false);
        lastDifference = currentPulseDifference; // Update the lastDifference
        // dacValue = newDacValue;
    }

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
            Serial.println(" impulses. ");
            if (detectedFrequency > 16 && detectedFrequency < 20)
            {
                projectorSpeedSwitchPos = 18;
                setupTimer1forFps(FPS_18);              
            }
            else if (detectedFrequency > 22 && detectedFrequency < 26)
            {
                projectorSpeedSwitchPos = 24;
            }
        }
    }
}

void setLeds(int bargraph)
{
    switch (bargraph)
    {
    case -2:
        digitalWrite(ledSlowerRed, HIGH);
        digitalWrite(ledSlowerYellow, LOW);
        digitalWrite(ledGreen, LOW);
        digitalWrite(ledFasterYellow, LOW);
        digitalWrite(ledFasterRed, LOW);
        break;
    case -1:
        digitalWrite(ledSlowerRed, LOW);
        digitalWrite(ledSlowerYellow, HIGH);
        digitalWrite(ledGreen, LOW);
        digitalWrite(ledFasterYellow, LOW);
        digitalWrite(ledFasterRed, LOW);
        break;
    case 0:
        digitalWrite(ledSlowerRed, LOW);
        digitalWrite(ledSlowerYellow, LOW);
        digitalWrite(ledGreen, HIGH);
        digitalWrite(ledFasterYellow, LOW);
        digitalWrite(ledFasterRed, LOW);
        break;
    case 1:
        digitalWrite(ledSlowerRed, LOW);
        digitalWrite(ledSlowerYellow, LOW);
        digitalWrite(ledGreen, LOW);
        digitalWrite(ledFasterYellow, HIGH);
        digitalWrite(ledFasterRed, LOW);
        break;
    case 2:
        digitalWrite(ledSlowerRed, LOW);
        digitalWrite(ledSlowerYellow, LOW);
        digitalWrite(ledGreen, LOW);
        digitalWrite(ledFasterYellow, LOW);
        digitalWrite(ledFasterRed, HIGH);
        break;
    default:
        break;
    }
}

bool setupTimer1forFps(byte sollFpsState)
{
    // start with a new sync point, no need to catch up differences from before.
    timerFrames = 0;
    projectorFrames = 0;
    timerModulus = 0;

    if (sollFpsState >= 1 && sollFpsState <= 5)
    {
        Serial.print(F("New Timer FPS State: "));
        Serial.println(sollFpsState);

        noInterrupts();
        // Clear registers
        TCCR1A = 0;
        TCCR1B = 0;
        TCNT1 = 0;
        // CTC
        TCCR1B |= (1 << WGM12);

        switch (sollFpsState)
        {
        case FPS_9:
            OCR1A = 10100; // 198.000198000198 Hz (16000000/((10100+1)*8)),
            //              divided by 22 is 9,000009.. Hz
            //
            TCCR1B |= (1 << CS11); // Prescaler 8
            timerFactor = 22;

            break;
        case FPS_16_2_3:
            OCR1A = 14999;                       // 16 2/3 Hz (16000000/((14999+1)*64))
            TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64
            timerFactor = 1;

            break;
        case FPS_18:
            // Very acurate but not firing at 18 * 12 Hz
            // OCR1A = 10100; // 198.000198000198 Hz (16000000/((10100+1)*8)),
            // //              divided by 11 is 18.000018.. Hz
            // //              or 18 2/111,111
            // //              or 2,000,000/111,111
            // //
            // TCCR1B |= (1 << CS11); // Prescaler 8
            // timerFactor = 11;

            // Less acurate but giving (18*12=) 216 Hz
            // 1) Compare Match-Interrupt für Timer1 erlauben
            TIMSK1 = (1 << OCIE1A); // Compare A Match Interrupt Enable

            // 2) Timer1 in den CTC-Modus setzen (WGM12 = 1)
            TCCR1A = 0;            // WGM10=0, WGM11=0
            TCCR1B = (1 << WGM12); // WGM12=1 => CTC mit OCR1A
            //    + Prescaler 8 (CS11=1)
            TCCR1B |= (1 << CS11);

            // 3) OCR1A setzen
            OCR1A = 9258;
            timerFactor = 1;

            break;
        case FPS_24:
            OCR1A = 60605; // 264.000264000264 Hz (16000000/((60605+1)*1)),
            //               divided by 11 is 24.000024.. Hz
            //               or 24 8/333,333
            //               or 8,000,000 / 333,333
            //
            TCCR1B |= (1 << CS10); // Prescaler 1
            timerFactor = 11;

            break;
        case FPS_25:
            OCR1A = 624;                         // 25 Hz (16000000/((624+1)*1024))
            TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
            timerFactor = 1;

            break;
        default:
            break;
        }
        // Output Compare Match A Interrupt Enable
        TIMSK1 |= (1 << OCIE1A);
        interrupts();
    }
    else
    {
        // invalid fps requested
        Serial.println(F("Invalid FPS request"));
        return false;
    }
}

int calculateValue(float x, float a, float b, float c)
{
    return a * x * x + b * x + c; // Berechnung der quadratischen Funktion
}

ISR(TIMER2_OVF_vect)
{
    timer2OverflowCount++; // Überlaufzähler inkrementieren
}

void onShaftImpulseISR()
{
    // For stability detection in free running mode, we use timer2 with overflow instead of micros() — it's cheaper.
    static unsigned long lastTimer2Value = 0;

    // Kombiniere Überläufe und Timer-Zähler
    unsigned long currentTimer2Value = (timer2OverflowCount << 8) | TCNT2; // 8-Bit Timer2 mit Überläufen
    unsigned long elapsedTicks = currentTimer2Value - lastTimer2Value;     // Differenz der Ticks
    lastTimer2Value = currentTimer2Value;

    // Umrechnung in Mikrosekunden
    unsigned long intervalMicros = elapsedTicks * 4; // 4 µs pro Tick bei Prescaler 64
    lastShaftPulseTime += intervalMicros;            // Aktualisiere den letzten Impulszeitpunkt

    // Frequenz-Puffer aktualisieren
    freqMedianBuffer[freqMedianIndex] = intervalMicros;
    freqMedianIndex = (freqMedianIndex + 1) % STABILITY_WINDOW_SIZE;

    // Prüfe, ob das Intervall den Stopp-Schwellenwert überschreitet
    if (intervalMicros > STOP_THRESHOLD && projectorRunning)
    {
        projectorRunning = false; // Projektor gestoppt
        Serial.println("[DEBUG] Projector stopped.");
        stopTimer1();
        shaftImpulseCount = 0;
    }

    // Neue Daten verfügbar machen
    shaftImpulseCount++;
    newShaftImpulseAvailable = true;

    if (shaftModulus == 0)
    {
        projectorFrames++;
        projectorFrameCountUpdated = true;
    }
    shaftModulus++;
    shaftModulus %= (shaftSegmentDiscDivider); 
}

ISR(TIMER1_COMPA_vect)
{
    if (timerModulus == 0)
    {
        timerFrames++;
        timerFrameCountUpdated = true;
    }
    timerModulus++;
    timerModulus %= timerFactor;
}

void stopTimer1()
{ // Stops Timer1, for when we are not in craystal running mode
    // TCCR1B &= ~(1 << CS11);
    noInterrupts();
    TIMSK1 &= ~(1 << OCIE1A);
    interrupts();
}