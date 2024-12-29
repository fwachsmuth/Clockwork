/* Todo

- english comments only
- support 24 fps in dumb mode
- add an enable pin for optional "crystalization"
- save new baseline in EEPROM and read it from there
- tune the PID further
- fix that the pid never corrects below the initial dac value (dac_value never gets updated)
- dither to 216 Hz (and other freqs, where necessary)


Irgendwann
- Fernstart/stop support
- ESS support


*/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <Adafruit_MCP4725.h> // Fancy DAC for voltage control
#include <Wire.h>             // i2c to talk to the DAC
#include <PID_v1.h>           // using 1.2.0 from https://github.com/br3ttb/Arduino-PID-Library

Adafruit_MCP4725 dac; // Instantiate the DAC

// pins and consts
constexpr int shaft_pulse_pin = 2;
constexpr int greenLedPin = 7;
constexpr int redLedPin = 9;

const byte ledSlowerRed = 5;    //  --
const byte ledSlowerYellow = 6; //  -
const byte ledGreen = 7;        //  o
const byte ledFasterYellow = 8; //  +
const byte ledFasterRed = 9;    //  ++

uint16_t dac_value = 1200; // start somewhere, e.g. 1537. This is about 17.6 fps on my projector

// to get the approx. DAC value for any desired fps per a * x * x + b * x + c
const float a = 0.6126;
const float b = 155.44;
const float c = -1459.34;

#define FPS_18 1
#define FPS_24 2
#define FPS_25 3
#define FPS_9 4
#define FPS_16_2_3 5

int timer_factor = 0;           // this is used for the Timer1 "postscaler", since multiples of 18 and 24 Hz give better accuracy
volatile int timer_modulus = 0; // For Modulo in the ISR, to compensate the timer_factor
volatile long timer_frames = 0; // This is the timer1 (frequency / timer_factor) — equalling actual desired fps (no multiples)

const byte shaft_segment_disc_divider = 1; // Increase this if we only want to use every nth pulse
volatile int shaft_modulus = 0; // For Modulo in the ISR, to compensate the multiple pulses per revolution
volatile long shaft_frames = 0; // This is the actually advanced frames (pulses / shaft_segment_disc_divider)

// flags to assure reading only once both ISRs have done theri duty
volatile bool shaft_frame_count_updated;
volatile bool timer_frame_count_updated;

volatile unsigned long last_shaft_pulse_time = 0;
constexpr unsigned long STOP_THRESHOLD = 10000; // Threshold in microseconds to detect a stop

// This is for the median approach, which finds stable freq detection after 48 impulses (4 frames). 
constexpr size_t STABILITY_WINDOW_SIZE = 12;        // Größe des Median-Fensters
constexpr size_t STABILITY_CHECKS = 36;             // Größe des Stabilitäts-Fensters
constexpr unsigned long SPEED_DETECT_TOLERANCE = 1600;           // Fester Toleranzwert in Mikrosekunden
constexpr unsigned long MIN_CHANGE = 800;           // Minimale Abweichung für eine neue Stabilität

volatile unsigned long freq_median_buffer[STABILITY_WINDOW_SIZE];
volatile size_t freq_median_index = 0;
volatile unsigned long stability_buffer[STABILITY_CHECKS];
volatile size_t freq_buffer_index = 0;
volatile unsigned long last_stable_freq_value = 0; // Zuletzt erkannter stabiler Wert
volatile unsigned long shaft_impulse_count = 0;
volatile bool new_shaft_impulse_available = false;
volatile bool projector_running = false; // true = Running, false = Stopped
volatile byte projector_speed_switch_pos; // holds the (guessed) current speed switch position (18 or 24)
volatile unsigned long timer2_overflow_count = 0; // Globale Zähler-Variable für Timer2-Überläufe

// PID stuff
double pid_setpoint, pid_input, pid_output;
double pid_Kp = 10, pid_Ki = 5, pid_Kd = 1;
PID myPID(&pid_input, &pid_output, &pid_setpoint, pid_Kp, pid_Ki, pid_Kd, REVERSE); 

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
    pinMode(shaft_pulse_pin, INPUT);
    pinMode(greenLedPin, OUTPUT);
    pinMode(redLedPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(shaft_pulse_pin), onShaftImpulseISR, RISING); // We only want one edge of the signal to not be duty cycle dependent
    dac.begin(0x60);

    dac.setVoltage(dac_value, false); // 1537 is petty much 18 fps

    pid_input = 0;
    pid_setpoint = 0;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 4095);
    myPID.SetSampleTime(26); // Nyquist: Measure twice in 1/18 Sek (= 26 msec)
}

void loop()
{
    static long last_framecount_difference = 0; // Stores the last output difference
    static long local_timer_frames = 0; // for atomic reads
    static long local_shaft_frames = 0;
    static long current_pulse_difference = 0;
    uint16_t new_dac_value = 0;

    // only read a difference if both ISRs did their updates yet, otherwise we get plenty of false changes
    if (shaft_frame_count_updated && timer_frame_count_updated)
    {
        noInterrupts();
        local_timer_frames = timer_frames;
        local_shaft_frames = shaft_frames;
        interrupts();

        current_pulse_difference = local_timer_frames - local_shaft_frames;

        shaft_frame_count_updated = false;
        timer_frame_count_updated = false;

        pid_input = current_pulse_difference;
        myPID.Compute();
        new_dac_value = dac_value + pid_output;
        dac.setVoltage(new_dac_value, false);
    }

    // Print only if the difference has changed
    if (current_pulse_difference != last_framecount_difference)
    {
        Serial.print("Input: ");
        Serial.print(current_pulse_difference);
        Serial.print(", Output: ");
        Serial.print(pid_output);
        Serial.print(", new DAC: ");

        // This is probably no longer needed
        if (new_dac_value < 0)
        {
            new_dac_value = 0;
        }
        else if (new_dac_value > 4095)
        {
            new_dac_value = 4095;
        }

        Serial.println(new_dac_value);

        dac.setVoltage(new_dac_value, false);
        last_framecount_difference = current_pulse_difference; // Update the last_framecount_difference
        // dacValue = new_dac_value;
    }

        if (!new_shaft_impulse_available)
        return; // Skip processing if no new data

    new_shaft_impulse_available = false; // Reset the ISR flag

    // Calculate median and store it in the stability buffer
    unsigned long median = calculateMedian(freq_median_buffer, STABILITY_WINDOW_SIZE);
    stability_buffer[freq_buffer_index] = median;
    freq_buffer_index = (freq_buffer_index + 1) % STABILITY_CHECKS;

    // Perform stability check if the buffer is full
    if (freq_buffer_index == 0 && checkStability(stability_buffer, STABILITY_CHECKS, SPEED_DETECT_TOLERANCE))
    {
        unsigned long new_stable_value = calculateMedian(stability_buffer, STABILITY_CHECKS);

        // Handle projector restart or stability change
        if (!projector_running || abs((long)new_stable_value - (long)last_stable_freq_value) > MIN_CHANGE)
        {
            last_stable_freq_value = new_stable_value;
            float detected_frequency = 1000000.0f / 12.0f / (float)last_stable_freq_value;
            projector_running = true; // Projector is running again
            Serial.print("[DEBUG] Projector running stable with ~");
            Serial.print(detected_frequency, 1); // FPS
            Serial.print(" fps after ");
            Serial.print(shaft_impulse_count);
            Serial.println(" impulses. ");
            if (detected_frequency > 16 && detected_frequency < 20)
            {
                projector_speed_switch_pos = 18;
                setupTimer1forFps(FPS_18);              
            }
            else if (detected_frequency > 22 && detected_frequency < 26)
            {
                projector_speed_switch_pos = 24;
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
    timer_frames = 0;
    shaft_frames = 0;
    timer_modulus = 0;

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
            timer_factor = 22;

            break;
        case FPS_16_2_3:
            OCR1A = 14999;                       // 16 2/3 Hz (16000000/((14999+1)*64))
            TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64
            timer_factor = 1;

            break;
        case FPS_18:
            // Very acurate but not firing at 18 * 12 Hz
            // OCR1A = 10100; // 198.000198000198 Hz (16000000/((10100+1)*8)),
            // //              divided by 11 is 18.000018.. Hz
            // //              or 18 2/111,111
            // //              or 2,000,000/111,111
            // //
            // TCCR1B |= (1 << CS11); // Prescaler 8
            // timer_factor = 11;

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
            timer_factor = 1;

            break;
        case FPS_24:
            OCR1A = 60605; // 264.000264000264 Hz (16000000/((60605+1)*1)),
            //               divided by 11 is 24.000024.. Hz
            //               or 24 8/333,333
            //               or 8,000,000 / 333,333
            //
            TCCR1B |= (1 << CS10); // Prescaler 1
            timer_factor = 11;

            break;
        case FPS_25:
            OCR1A = 624;                         // 25 Hz (16000000/((624+1)*1024))
            TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
            timer_factor = 1;

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
    timer2_overflow_count++; // Überlaufzähler inkrementieren
}

void onShaftImpulseISR()
{
    // For stability detection in free running mode, we use timer2 with overflow instead of micros() — it's cheaper.
    static unsigned long last_timer2_value = 0;

    // Kombiniere Überläufe und Timer-Zähler
    unsigned long current_timer2_value = (timer2_overflow_count << 8) | TCNT2; // 8-Bit Timer2 mit Überläufen
    unsigned long elapsed_ticks = current_timer2_value - last_timer2_value;     // Differenz der Ticks
    last_timer2_value = current_timer2_value;

    // Umrechnung in Mikrosekunden
    unsigned long interval_micros = elapsed_ticks * 4; // 4 µs pro Tick bei Prescaler 64
    last_shaft_pulse_time += interval_micros;            // Aktualisiere den letzten Impulszeitpunkt

    // Frequenz-Puffer aktualisieren
    freq_median_buffer[freq_median_index] = interval_micros;
    freq_median_index = (freq_median_index + 1) % STABILITY_WINDOW_SIZE;

    // Prüfe, ob das Intervall den Stopp-Schwellenwert überschreitet
    if (interval_micros > STOP_THRESHOLD && projector_running)
    {
        projector_running = false; // Projektor gestoppt
        Serial.println("[DEBUG] Projector stopped.");
        stopTimer1();
        shaft_impulse_count = 0;
    }

    // Neue Daten verfügbar machen
    shaft_impulse_count++;
    new_shaft_impulse_available = true;

    if (shaft_modulus == 0)
    {
        shaft_frames++;
        shaft_frame_count_updated = true;
    }
    shaft_modulus++;
    shaft_modulus %= (shaft_segment_disc_divider); 
}

ISR(TIMER1_COMPA_vect)
{
    if (timer_modulus == 0)
    {
        timer_frames++;
        timer_frame_count_updated = true;
    }
    timer_modulus++;
    timer_modulus %= timer_factor;
}

void stopTimer1()
{ // Stops Timer1, for when we are not in craystal running mode
    // TCCR1B &= ~(1 << CS11);
    noInterrupts();
    TIMSK1 &= ~(1 << OCIE1A);
    interrupts();
}