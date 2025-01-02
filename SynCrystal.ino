/* Todo

Hardware
- add i2c display
- add IR emitter
- 74LVC2G14

Code
- consider an adaptive PID
- Add FreqMeasure
- update stop detection to a timeout (instead of period length)
- move the Serial.print out of the ISR
- Allow changing speed in manual mode at runtime
- Reset Counters? (long press both buttons?)
- Rangieren (langsam)
- "start the audio" IR

Irgendwann
- Fernstart/stop support
- ESS support
- Add Realtime Timecode


*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Adafruit_MCP4725.h> // Fancy DAC for voltage control
#include <Wire.h>             // i2c to talk to the DAC
#include <PID_v1.h>           // using 1.2.0 from https://github.com/br3ttb/Arduino-PID-Library
#include <FreqMeasure.h>
#include <Button2.h>

// pins and consts
const byte SHAFT_PULSE_PIN = 2;
const byte LED_GREEN_PIN = 7;
const byte ENABLE_PIN = 9;

const byte LEFT_BTTN_PIN = 10;
const byte RIGHT_BTTN_PIN = 13;
const byte DROP_BACK_BTTN_PIN = 11;
const byte CATCH_UP_BTTN_PIN = 12;

// const byte redLedPin = 9;

const byte ledSlowerRed = 5;    //  --
const byte ledSlowerYellow = 6; //  -
const byte ledGreen = 7;        //  o
const byte ledFasterYellow = 8; //  +
//const byte ledFasterRed = 9;    //  ++

const uint16_t DAC_INITIAL_VALUE = 1500; // This should equal a voltage that leads to approx 16-20 fps (on 18 fps) or 22-26 fps (on 24).

// Threshold in microseconds to detect a stop. (This is between shaft pulses, so too big vlaues might never happen!)
const unsigned long STOP_THRESHOLD = 15000; 

const unsigned long SAVE_THRESHOLD = 10000; // ms until a stable DAC value will be considered as new EPROM default

// to get the approx. DAC value for any desired fps per a * x * x + b * x + c
const float a = 0.6126;
const float b = 155.44;
const float c = -1459.34;


#define BTTN_NONE 0
#define BTTN_LEFT 1
#define BTTN_RIGHT 2

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

// These consts are used in the median approach, which finds stable freq detection after ~ 48 impulses (4 frames). 
constexpr size_t STABILITY_WINDOW_SIZE = 12;        // Size of the Median Window. We use 12 to capture one entire shaft revolution
constexpr size_t STABILITY_CHECKS = 36;             // Window size to determine stability
constexpr unsigned long SPEED_DETECT_TOLERANCE = 1600; // allowed tolerance between pulses in microseconds
constexpr unsigned long MIN_CHANGE = 800;           // minimum deviation to determine a new stability

volatile unsigned long freq_median_buffer[STABILITY_WINDOW_SIZE];
volatile size_t freq_median_index = 0;
volatile unsigned long stability_buffer[STABILITY_CHECKS];
volatile size_t freq_buffer_index = 0;
volatile unsigned long last_stable_freq_value = 0; // Zuletzt erkannter stabiler Wert
volatile unsigned long shaft_impulse_count = 0;
volatile bool new_shaft_impulse_available = false;
volatile bool projector_running = false; // true = Running, false = Stopped
volatile byte projector_speed_switch_pos = 0; // holds the (guessed) current speed switch position (18 or 24)
volatile unsigned long timer2_overflow_count = 0; // Globale Zähler-Variable für Timer2-Überläufe

// PID stuff
double pid_setpoint, pid_input, pid_output;
double pid_Kp = 25, pid_Ki = 35, pid_Kd = 0;
PID myPID(&pid_input, &pid_output, &pid_setpoint, pid_Kp, pid_Ki, pid_Kd, REVERSE);

// Instantiate the DAC
Adafruit_MCP4725 dac;

// Instantiate the Buttons
Button2 leftButton, rightButton, dropBackButton, catchUpButton;

// State Machine!

enum RunModes
{
    XTAL_NONE,
    XTAL_AUTO,
    XTAL_16_2_3,
    XTAL_18,
    XTAL_23_976,
    XTAL_24,
    XTAL_25,
    MODES_COUNT // Automatically equals the number of entries in the enum
};
byte current_run_mode = XTAL_AUTO;

enum SyncStates
{
    SYNC_LOCKED,
    SYNC_PROJ_TOO_FAST,
    SYNC_PROJ_TOO_SLOW
};

enum FpsSpeeds
{
    FPS_9,       // => 108 Hz
    FPS_16_2_3,  // => 200 Hz
    FPS_18,      // => 216 Hz
    FPS_23_976,  // => ~287.712 Hz
    FPS_24,      // => 288 Hz
    FPS_25,      // => 300 Hz
    SPEEDS_COUNT // Anzahl
};

// Struktur mit allen Infos, die wir für Dithering & Postscaler brauchen:
struct DitherConfig
{
    uint16_t base;   // Grundwert für OCR1A
    uint32_t frac32; // fractional part (UQ0.32), 0..(2^32-1)
    uint8_t timerFactor;
};
// Globale Dithering Variablen für die ISR:

volatile uint16_t ditherBase = 0;   // Grundwert für OCR1A
volatile uint32_t ditherFrac32 = 0; // Fraction in UQ0.32
volatile uint32_t ditherAccu32 = 0; // Akkumulator

/*
---------------------------------------------------------------------------
 Table with Base/Frac/Factor values for each target frequency
   Prescaler=8 => Timer frequency = 2 MHz
   The values are tailored for “fps * 12” = final frequency.

   Explanation:
     - base = floor( (2,000,000 / (target freq)) ) or floor( (2,000,000 / (a multiple)) )
     - frac32 = (decimal point * 2^32) (UQ0.32 fixed decimal point)
     - timerFactor => Software divider in the ISR

   Examples:
     * FPS_9 => 108 Hz = 864 / 8 => Timer-ISR=864 Hz => base=2314.8 => 2314 + frac~0.8148 => frac32~0xD0E14710
     * FPS_16_2_3 => 200 Hz => exact divider=10,000 => OCR1A=9999 => fraction=0 => no dither
     * etc.

 Note: All frac32 values here rounded to ~ <5 ppm.
---------------------------------------------------------------------------
*/

static const DitherConfig PROGMEM s_ditherTable[] = {
    // 1) FPS_9 => 108 Hz = 864 Hz ISR / 8
    //    => idealDiv = 2314.8148148..., base=2314, fraction=~0.8148148
    //    => frac32 = round(0.8148148 * 2^32) = 0xD0BE9C00
    //    => Endfreq ~ 108.000000 => 0 ppm
    {2314, 0xD0BE9C00, 8},

    // 2) FPS_16_2_3 => 200 Hz => idealDiv=10000 => fraction=0 => no dithering
    //    => base=9999, frac32=0
    //    => Endfreq=200 => 0 ppm
    {9999, 0x00000000, 1},

    // 3) FPS_18 => 216 Hz = 864 Hz ISR / 4
    //    => same as 108 Hz example but factor=4 => 0 ppm
    {2314, 0xD0BE9C00, 4},

    // 4) FPS_23_976 => ~287.712 => idealDiv ~6950.695953
    //    => fraction=0.695953..., frac32=0xB2284B17 => 0.03 ppm
    {6950, 0xB4C236F7, 1},

    // 5) FPS_24 => 288 Hz = 864 Hz ISR / 3
    //    => same base/fraction as 108 Hz, factor=3 => 0 ppm
    {2314, 0xD0BE9C00, 3},

    // 6) FPS_25 => 300 Hz => idealDiv=6666.666..., fraction=0.666..., frac32=0xAAAAAAAB => 0 ppm
    {6666, 0xAAAAAAAB, 1},
};

void setup()
{
    leftButton.begin(LEFT_BTTN_PIN);
    leftButton.setTapHandler(handleButtonTap); // react immediately when no longtap is needed
    // leftButton.setClickHandler(handleButtonClick); // needed when we need to recognize long presses in running mode
    // leftButton.setLongClickDetectedHandler(handleButtonLongClick);
    // leftButton.setLongClickTime(1000);
    leftButton.setDoubleClickTime(0); // disable double clicks
    leftButton.setDebounceTime(10);

    rightButton.begin(RIGHT_BTTN_PIN);
    rightButton.setTapHandler(handleButtonTap);
    rightButton.setDoubleClickTime(0); // disable double clicks
    rightButton.setDebounceTime(10);

    dropBackButton.begin(DROP_BACK_BTTN_PIN);
    dropBackButton.setTapHandler(handleButtonTap);
    dropBackButton.setDoubleClickTime(0); // disable double clicks
    dropBackButton.setDebounceTime(10);

    catchUpButton.begin(CATCH_UP_BTTN_PIN);
    catchUpButton.setTapHandler(handleButtonTap);
    catchUpButton.setDoubleClickTime(0); // disable double clicks
    catchUpButton.setDebounceTime(10);

    Serial.begin(115200);

    // Configure Timer2 (replaces micros() in the ISR, since it is cheaper)
    TCCR2A = 0;            // Normal Mode
    TCCR2B = (1 << CS22);  // Prescaler = 64 (1 Tick = 4 µs at 16 MHz)
    TCNT2 = 0;             // Timer2 Reset
    TIMSK2 = (1 << TOIE2); // Activate Overflow Interrupt 
    
    pinMode(SHAFT_PULSE_PIN, INPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(LEFT_BTTN_PIN, INPUT_PULLUP);
    pinMode(RIGHT_BTTN_PIN, INPUT_PULLUP);
    pinMode(DROP_BACK_BTTN_PIN, INPUT_PULLUP);
    pinMode(CATCH_UP_BTTN_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(SHAFT_PULSE_PIN), onShaftImpulseISR, RISING); // We only want one edge of the signal to not be duty cycle dependent
    dac.begin(0x60);

    dac.setVoltage(DAC_INITIAL_VALUE, false); // 1537 is petty much 18 fps and 24 fps

    // Init the PID
    pid_input = 0;
    pid_setpoint = 0;
    myPID.SetOutputLimits(0, 4095);
    myPID.SetSampleTime(50);
    myPID.SetMode(MANUAL);
    pid_output = DAC_INITIAL_VALUE; // This avoids starting with a 0-Output signal
    myPID.SetMode(AUTOMATIC);

    changeRunMode(current_run_mode);
}

void loop()
{
    static long local_timer_frames = 0; // for atomic reads
    static long local_shaft_frames = 0; // for atomic reads
    static long last_pulse_difference = 0; // Stores the last output difference
    static long current_pulse_difference = 0;
    uint16_t new_dac_value = 0;
    static uint8_t button, last_button = BTTN_NONE;
    static long last_pid_update_millis;
    static long current_pid_update_millis;

    // Poll the buttons
    leftButton.loop();
    rightButton.loop();
    dropBackButton.loop();
    catchUpButton.loop();

    // only read a pulse difference if both ISRs did their updates yet, otherwise we get plenty of false +/-1 diffs/errors
    if (shaft_frame_count_updated && timer_frame_count_updated)
    {
        noInterrupts();
        local_timer_frames = timer_frames;
        local_shaft_frames = shaft_frames;
        interrupts();

        current_pulse_difference = local_timer_frames - local_shaft_frames;

        // mark these counts as read
        shaft_frame_count_updated = false;
        timer_frame_count_updated = false;

        pid_input = current_pulse_difference;
        myPID.Compute();
        new_dac_value = pid_output;
        dac.setVoltage(new_dac_value, false);
    }

    // Print only if the difference has changed AND the projector is actually running
    if ((current_pulse_difference != last_pulse_difference) && projector_speed_switch_pos != 0)
    {
        // current_pid_update_millis = millis();
        // Serial.print(", which was stable for ");
        // Serial.print(current_pid_update_millis - last_pid_update_millis);
        // Serial.println(" ms");

        Serial.print("Target FPS: ");
        Serial.print(projector_speed_switch_pos);
        Serial.print(", Error: ");
        Serial.print(current_pulse_difference);
        Serial.print(", PID-Output: ");
        Serial.print(pid_output);
        Serial.print(", new DAC value: ");
        Serial.println(new_dac_value);

        if ((current_pulse_difference == 0) && ((current_pid_update_millis - last_pid_update_millis) > SAVE_THRESHOLD))
        {
            Serial.print(" This would be a winner: ");
            Serial.println((current_pid_update_millis - last_pid_update_millis) / 1000);
        }

        last_pulse_difference = current_pulse_difference; // Update the last_pulse_difference
        last_pid_update_millis = current_pid_update_millis;
    }

    if (!new_shaft_impulse_available)
        return; // Skip processing if no new data

    new_shaft_impulse_available = false; // Reset the ISR flag

    // Calculate median and store it in the stability buffer
    unsigned long median = calculateMedian(freq_median_buffer, STABILITY_WINDOW_SIZE);
    stability_buffer[freq_buffer_index] = median;
    freq_buffer_index = (freq_buffer_index + 1) % STABILITY_CHECKS;

    // Perform running frequency stability check if the buffer is full AND we don't know the fps-switch pos yet
    if (projector_speed_switch_pos == 0 && freq_buffer_index == 0 && checkStability(stability_buffer, STABILITY_CHECKS, SPEED_DETECT_TOLERANCE))
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
            last_pid_update_millis = millis(); // this is needed to calculate the lifetime of the first pid output

            if (detected_frequency <= 21)
            {
                projector_speed_switch_pos = 18;
                digitalWrite(ENABLE_PIN, HIGH);
                setupTimer1forFps(FPS_18);
            }
            else if (detected_frequency > 21)
            {
                projector_speed_switch_pos = 24;
                digitalWrite(ENABLE_PIN, HIGH);
                setupTimer1forFps(FPS_24);
            }       
        }
    }
}

unsigned long calculateMedian(volatile unsigned long *buffer, size_t size)
{
    // Calculates the median. A rolling average might be cehaper and good enough, esp with filtering outliers.
    unsigned long temp[size];
    // We need a copy of the array to not get interference with the ISR. Intereference doesnt seem likely, so maybe 100B could be saved here
    for (size_t i = 0; i < size; i++)
    {
        temp[i] = buffer[i];
    }

    // Simple Insertion Sort
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

bool checkStability(volatile unsigned long *buffer, size_t size, unsigned long tolerance)
{
    // Stability check
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

void changeRunMode(byte run_mode)
{
    // set the crystal LED
    digitalWrite(ENABLE_PIN, (run_mode == XTAL_NONE) ? LOW : HIGH);
        
    switch (run_mode)
    {
    case XTAL_NONE:
        Serial.println("XTAL_NONE");
        break;
    case XTAL_AUTO:
        Serial.println("XTAL_AUTO");
        break;
    case XTAL_16_2_3:
        Serial.println("XTAL_16_2_3");
        setupTimer1forFps(FPS_16_2_3);
        break;
    case XTAL_18:
        Serial.println("XTAL_18");
        setupTimer1forFps(FPS_18);
        break;
    case XTAL_23_976:
        Serial.println("XTAL_23_976");
        setupTimer1forFps(FPS_23_976);
        break;
    case XTAL_24:
        Serial.println("XTAL_24");
        setupTimer1forFps(FPS_24);
        break;
    case XTAL_25:
        Serial.println("XTAL_25");
        setupTimer1forFps(FPS_25);
        break;
    default:
        Serial.println("Unknown Mode");
        break;
    }
}

void handleButtonTap(Button2 &btn)
{
    if (btn == leftButton)
    {
        selectNextMode(btn);
    }
    else if (btn == rightButton)
    {
        selectNextMode(btn);
    }
    // Frame up/down is only allowed while the projector is running
    if (projector_running)
    {
        if (btn == dropBackButton)
        {
            shaft_frames ++;
        }
        else if (btn == catchUpButton)
        {
            shaft_frames--;
        }
    }
}

void selectNextMode(Button2 &btn)
{
    // Determine the change based on which button was pressed
    int8_t change = (btn == leftButton) ? -1 : 1;

    // Update the run mode
    current_run_mode = (current_run_mode + change + MODES_COUNT) % MODES_COUNT;
    changeRunMode(current_run_mode);
}


bool setupTimer1forFps(byte desiredFps)
{
    // start with a new sync point, no need to catch up differences from before.
    timer_frames = 0;
    shaft_frames = 0;
    timer_modulus = 0;

    if (desiredFps >= SPEEDS_COUNT)
    {
        Serial.println(F("Invalid FPS index!"));
        return false;
    }

    // --- Tabellenwerte aus PROGMEM lesen
    DitherConfig cfg;
    memcpy_P(&cfg, &s_ditherTable[desiredFps], sizeof(DitherConfig));

    // Übernehmen in globale (ISR-)Variablen
    noInterrupts();
    // Timer stoppen/Reset
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
    TCNT1 = 0;

    // Globale Dither-Variablen belegen
    ditherBase = cfg.base;
    ditherFrac32 = cfg.frac32;
    ditherAccu32 = 0;

    // Deinen timerFactor & -Divider init
    timer_factor = cfg.timerFactor;
    timer_modulus = 0;
    timer_frames = 0;

    // 3) OCR1A Startwert
    OCR1A = ditherBase;

    // 4) CTC-Mode (WGM12=1), Prescaler=8 => CS11=1
    TCCR1B |= (1 << WGM12) | (1 << CS11);

    // Compare-A-Interrupt an
    TIMSK1 |= (1 << OCIE1A);

    interrupts();

    Serial.print(F("Timer1 configured for Fps index="));
    Serial.print(desiredFps);
    Serial.print(F("; base="));
    Serial.print(ditherBase);
    Serial.print(F(", frac32=0x"));
    Serial.print(ditherFrac32, HEX);
    Serial.print(F(", timerFactor="));
    Serial.println(timer_factor);

    /*    if (desiredFps >= SPEEDS_COUNT)
        {
            Serial.print(F("New Timer FPS State: "));
            Serial.println(desiredFps);

            noInterrupts();
            // Clear registers
            TCCR1A = 0;
            TCCR1B = 0;
            TCNT1 = 0;
            // CTC
            TCCR1B |= (1 << WGM12);

            switch (desiredFps)
            {
            case FPS_9:
            // Goal: (9 * 12 =) 108 Hz
            // OCR1A = 2314
            // timer_factor = 8;
            // 3 ppm?

                break;
            case FPS_16_2_3:
                // Goal: (16 2/3 * 12 =) 200 Hz
                // This should be preciseley 200 Hz
                // (AI: but it is not. It is 200.00000000000006 Hz, so ~0.00000000000003 ppm off)
                TIMSK1 = (1 << OCIE1A); // Compare A Match Interrupt Enable
                TCCR1A = 0;             // WGM10=0, WGM11=0
                TCCR1B = (1 << WGM12) | (1 << CS11);
                OCR1A = 9999;
                timer_factor = 1;

            break;
            case FPS_18:
                // Goal: (18 * 12 =) 216 Hz
                // this config gives approx 216,00605 Hz, so ~28 ppm off
                // Prescaoer 1 oder Dithering würden helfen
                TIMSK1 = (1 << OCIE1A); // Compare A Match Interrupt Enable
                TCCR1A = 0;            // WGM10=0, WGM11=0
                TCCR1B = (1 << WGM12) | (1 << CS11);
                OCR1A = 9258;
                timer_factor = 1;
                // OCR1A = 2314
                // timer_factor = 4;
                // 3 ppm?

                break;
            case FPS_23_976:
                // Goal: ((24000 /1001) * 12 = 288000 / 1001) 287.712'287712 Hz
                break;
            case FPS_24:
                // Goal: (24*12=) 288 Hz
                // this config gives approx (3x288) 864.02 Hz, so ~23 ppm off
                TIMSK1 = (1 << OCIE1A);
                TCCR1A = 0;
                TCCR1B = (1 << WGM12) | (1 << CS11);
                OCR1A = 2314;
                timer_factor = 3;
                // OCR1A = 2314
                // timer_factor = 3;
                // 3 ppm?

                break;
            case FPS_25:
                // Goal: (25 * 12 =) 300 Hz

                break;
            default:
                break;
            }
            // Output Compare Match A Interrupt Enable
            TIMSK1 |= (1 << OCIE1A);
            interrupts();

        } */
    return true;
}


ISR(TIMER2_OVF_vect)
{
    timer2_overflow_count++; // increment overflow counter
}

void onShaftImpulseISR()
{
    // For stability detection in free running mode, we use timer2 with overflow instead of micros() — it's cheaper.
    static unsigned long last_timer2_value = 0;

    // ombine overflow and timer counters
    unsigned long current_timer2_value = (timer2_overflow_count << 8) | TCNT2; // 8-Bit Timer2 plus overflows
    unsigned long elapsed_ticks = current_timer2_value - last_timer2_value;     // tick difference
    last_timer2_value = current_timer2_value;

    // convert to microseonds
    unsigned long interval_micros = elapsed_ticks * 4; // 4 µs per tick with prescaler 64
    last_shaft_pulse_time += interval_micros;            // update last pulse timestamp

    // update the frequencies buffer
    freq_median_buffer[freq_median_index] = interval_micros;
    freq_median_index = (freq_median_index + 1) % STABILITY_WINDOW_SIZE;

    // Check if the interval length qualifies to recognize a stopped projector
    // Todo: This should probably better use a timeout
    if (interval_micros > STOP_THRESHOLD && projector_running)
    {
        projector_running = false; // Projector stopped
        Serial.println("[DEBUG] Projector stopped.");
        projector_speed_switch_pos = 0; // forget the previously determined switch pos, it might be changed
        stopTimer1();
        timer_frame_count_updated = 0; // just in case the ISR fired again AND the shaft was still breaking. This could cause false PID computations.
        shaft_impulse_count = 0;
        // Reset DAC and PID
        dac.setVoltage(DAC_INITIAL_VALUE, false); // reset the DAc to compensate for wound-up break corrections
        myPID.SetMode(MANUAL);
        pid_output = DAC_INITIAL_VALUE;
        pid_input = 0;
        myPID.Compute();
        Serial.print("PID Reset to initial DAC value: ");
        Serial.println(pid_output);
        myPID.SetMode(AUTOMATIC);
        digitalWrite(ENABLE_PIN, LOW);
    }

    // Expose the news
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

    // 2) Dither-Logik (Festkomma-Akkumulator)
    ditherAccu32 += ditherFrac32;
    // Wenn Überlauf => ditherAccu32 < ditherFrac32
    if (ditherAccu32 < ditherFrac32)
    {
        OCR1A = ditherBase + 1;
    }
    else
    {
        OCR1A = ditherBase;
    }
}

void stopTimer1()
{ // Stops Timer1, for when we are not in craystal running run_mode
    // TCCR1B &= ~(1 << CS11);
    noInterrupts();
    TIMSK1 &= ~(1 << OCIE1A);
    interrupts();
}
