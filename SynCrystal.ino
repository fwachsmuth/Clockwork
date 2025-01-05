/* Todo

Test controlling pulse count, not framecount!
Fix projector stop detecttion to use micros again

Move controller and stop detection from loop intofunctions

Pending Bugs for Sepmag Sync:
- Estimated fps so far (for previous_freq) seems wrong (low)
- Changing form 25 fps back to Auto does not retain a previous 18fps-Auto
- rundetection does not kick in when starting after manual mode selection

Hardware
- add IR emitter
- 74LVC2G14
- Strobe Output
- Reconfigure OpAmp offset to allow tha DAC to halt the motor

Code
- Implement 3 catch-up modes for speed changes while running:
    - A:  Convert timer_pulses to new speed: Projector will catch up until sync is reached again.
          Stopping the projector resets the counters.
        + allows speed corrections without finding the start mark again
        + good for sepmag
        - projector might "run" of "crawl" for quite some time
    Timecode conversion is possible.

    Todo:
    √ Keep prevvious speed_mode around
    - capture millis when projector started running (for Auto -> Fixed transition)


    - B:  Forget any previous errors and start fresh
        + allws for quick speed changes
        + ideal mode for use as "Peaceman's Box" (ESS out mode)
        + good for telecine (which could also just restartm though)
        - loses sync with any sepmag audio
    Timecode would restart at 0:00:00.00

    - C: keep old errors around and correct them too
         Shaft and Pulse totals will always be in sync, speed changes cause no pulse loss
        • currently implemented already
        • not very meaningful though?
    Timecode conversion is possible, but pointless (since not in sync with virtual sepmag time)

Bug:
- Changing form 25 fps back to Auto does not retain a previous 18fps-Auto
- Decrement Shaft Impulses after detected shaft noise!
- Cannot start in a manual selected mode
- Clamp the error to not switch sign

- Use the display
- Review volatile vars (only for variables modified inside ISRs and used outside)
- Reset Counters? (long press both buttons?)
- consider an adaptive PID
- Rangieren (langsam)
- "start the audio" IR
- Clamp the PID output to avoid halting the motor

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
#include <U8x8lib.h> // Display Driver

// projector specific constants
const uint16_t DAC_INITIAL_VALUE = 1500; // This should equal a voltage that leads to approx 16-20 fps (on 18 fps) or 22-26 fps (on 24).
const unsigned long STOP_THRESHOLD = 15000; // microseconds until a stop will be detected
const unsigned long SAVE_THRESHOLD = 10000; // ms until a stable DAC value will be considered as new EPROM default
constexpr size_t SHAFT_SEGMENT_COUNT = 12;             // Size of the Median Window. We use 12 to capture one entire shaft revolution


// pins and consts
const byte SHAFT_PULSE_PIN = 2;
const byte LED_RED_PIN = 5;
const byte LED_GREEN_PIN = 7;
const byte ENABLE_PIN = 9;

const byte LEFT_BTTN_PIN = 10;
const byte RIGHT_BTTN_PIN = 13;
const byte DROP_BACK_BTTN_PIN = 11;
const byte CATCH_UP_BTTN_PIN = 12;

#define BTTN_NONE 0
#define BTTN_LEFT 1
#define BTTN_RIGHT 2

const byte ledRed = 5;          //  Out of Sync
const byte ledGreen = 7;        //  Crystal enabled
const byte ledSlowerYellow = 6; //  -
const byte ledFasterYellow = 8; //  +

// Timer Variables
volatile uint32_t timer_pulses = 0;
volatile uint32_t timer_frames = 0; // This is the timer1 (frequency / timer_factor) — equalling actual desired fps (no multiples)
volatile uint8_t timer_modulus = 0; // For Modulo in the ISR, to compensate the timer_factor
int timer_factor = 0;           // this is used for the Timer1 "postscaler", since multiples of 18 and 24 Hz give better accuracy
volatile uint32_t last_pulse_timestamp; // Timestamp of the last pulse, used to detect a stop


// Shaft Encoder Variables
volatile uint32_t shaft_pulses = 0;
volatile uint32_t shaft_frames = 0;   // This is the actually advanced frames (pulses / shaft_segment_disc_divider)
volatile uint8_t shaft_modulus = 0;            // For Modulo in the ISR, to compensate the multiple pulses per revolution
volatile bool new_shaft_impulse_available = false;
uint32_t projector_start_millis = 0; // To track the start time of the projector running
volatile uint32_t last_frame_timestamp;      // Timestamp of the last pulse, used to detect a stop


uint8_t projector_speed_switch = 0;  // To track the detected position of the projector's speed switch (18 or 24)

// flags to assure reading only once both ISRs have done their duty
volatile bool shaft_frame_count_updated;
volatile bool timer_frame_count_updated;



// PID stuff
double pid_setpoint, pid_input, pid_output;
double pid_Kp = 25, pid_Ki = 35, pid_Kd = 0;
PID myPID(&pid_input, &pid_output, &pid_setpoint, pid_Kp, pid_Ki, pid_Kd, REVERSE);

// Instantiate the DAC
Adafruit_MCP4725 dac;

// Instantiate the Buttons
Button2 leftButton, rightButton, dropBackButton, catchUpButton;

// Instantiate the Display
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

enum SpeedModes
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
byte speed_mode = XTAL_AUTO;
float previous_freq = 1.00; // to allow recalculation of timer_frames and timecode

enum ProjectorStates
{
    PROJ_IDLE,
    PROJ_RUNNING
};
byte projector_state = PROJ_IDLE;

enum FpsSpeeds
{
    /* FPS_9,       // => 108 Hz */
    FPS_16_2_3,  // => 200 Hz
    FPS_18,      // => 216 Hz
    FPS_23_976,  // => ~287.712 Hz
    FPS_24,      // => 288 Hz
    FPS_25,      // => 300 Hz
    SPEEDS_COUNT // Total Count of speeds supported
};

enum BlendModes
{
    BLEND_SEPMAG,  // Option A: Convert timer_pulses to new speed: Projector will catch up until sync is reached again.
    BLEND_RESET,   // Option B: Forget any previous pulses and start fresh
    BLEND_EQUAL    // Option C: keep old errors around and correct them too
};
byte blend_mode = BLEND_SEPMAG;

// Global dithering variables for the ISR:
volatile uint16_t ditherBase = 0;   // base for OCR1A
volatile uint32_t ditherFrac32 = 0; // fraction in UQ0.32
volatile uint32_t ditherAccu32 = 0; // Accumulator
volatile float ditherEndFreq = 0;   // Actual frequency of the dithered timer / SHAFT_SEGMENT_COUNT (proj. freq)

// Struct with all the information we need for dithering & postscaler:
struct DitherConfig
{
    uint16_t base;   // Grundwert für OCR1A
    uint32_t frac32; // fractional part (UQ0.32), 0..(2^32-1)
    uint8_t timerFactor; // a frequency multiplier for each timer config (some multiples give better accuracy)
    float endFreq; // the actual frequency we get with this config
};

static const DitherConfig PROGMEM s_ditherTable[] = {

    /*
    ---------------------------------------------------------------------------
     Table with Base/Frac/Factor values for each target frequency
       Prescaler=8 => Timer frequency = 2 MHz
       The values are tailored for “fps * 12” = final frequency.

       Explanation:
         - base = floor( (2000000 / (target freq)) ) or floor( (2000000 / (a multiple)) )
         - frac32 = (decimal point * 2^32) (UQ0.32 fixed decimal point)
         - timerFactor => Software divider in the ISR

       Examples:
         * FPS_9 => 108 Hz = 864 / 8 => Timer-ISR=864 Hz => base=2314.8 => 2314 + frac~0.8148 => frac32~0xD0E14710
         * FPS_16_2_3 => 200 Hz => exact divider=10,000 => OCR1A=9999 => fraction=0 => no dither
         * etc.

     Note: All frac32 values here rounded to ~ <5 ppm.
    ---------------------------------------------------------------------------
    */

    // 1) FPS_9 => 108 Hz = 864 Hz ISR / 8
    //    => idealDiv = 2314.8148148..., base=2314, fraction=~0.8148148
    //    => frac32 = round(0.8148148 * 2^32) = 0xD0BE9C00
    //    => Endfreq ~ 108.000000 => 0 ppm
    // {2314, 0xD0BE9C00, 8, 9.000000},

    // 2) FPS_16_2_3 => 200 Hz => idealDiv=10000 => fraction=0 => no dithering
    //    => base=9999, frac32=0
    //    => Endfreq=200 => 0 ppm
    {10000, 0x00000000, 1, 16.666666},

    // 3) FPS_18 => 216 Hz = 864 Hz ISR / 4
    //    => same as 108 Hz example but factor=4 => 0 ppm
    {2314, 0xD0BE9C00, 4, 18.000000},

    // 4) FPS_23_976
    /*  => fraction = 288000/1001 ~ 287.712287712288 Hz
    timer_factor= 1
    best base   = 6951
    best frac32 = 0x638E38E4  (decimal 1670265060)
    freqActual  = 2147483648000000/7463996984889 ~ 287.712287712283 Hz
    error Hz    = -0.000000000004
    ppm error   = -0.000000
    */
    {6951, 0x638E38E4, 1, 23.976024},

    // 5) FPS_24 => 288 Hz = 864 Hz ISR / 3
    //    => same base/fraction as 108 Hz, factor=3 => 0 ppm
    {2314, 0xD0BE9C00, 3, 24.000000},

    // 6) FPS_25 => 300 Hz => idealDiv=6666.666..., fraction=0.666..., frac32=0xAAAAAAAB => 0 ppm
    {6666, 0xAAAAAAAB, 1, 25.000000},
};

void setup()
{
    // Initialize buttons using the helper function
    initializeButton(leftButton, LEFT_BTTN_PIN);
    initializeButton(rightButton, RIGHT_BTTN_PIN);
    initializeButton(dropBackButton, DROP_BACK_BTTN_PIN);
    initializeButton(catchUpButton, CATCH_UP_BTTN_PIN);

    Serial.begin(115200);

    pinMode(SHAFT_PULSE_PIN, INPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
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

    changeSpeedMode(speed_mode);

    u8x8.begin();
    u8x8.setFont(u8x8_font_profont29_2x3_n); // https://github.com/olikraus/u8g2/wiki/fntlist8x8

    // FreqMeasure.begin();
}

void loop()
{
    static long local_timer_frames = 0;    // for atomic reads
    static long local_shaft_frames = 0;    // for atomic reads
    static long local_timer_pulses = 0;    // for atomic reads
    static long local_shaft_pulses = 0;    // for atomic reads
    static long last_pulse_difference = 0; // Stores the last output difference. )Just used to limit the printf output)
    static long current_frame_difference = 0;
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

    if (projector_state == PROJ_IDLE)
    {
        checkProjectorRunning();
    }
    else if (projector_state == PROJ_RUNNING)
    {
        // leave out the if to check if both ISRs updated yet. Maybe it doesn't matter.
    if (shaft_frame_count_updated && timer_frame_count_updated)
    {

        noInterrupts();
        local_timer_pulses = timer_pulses;
        local_shaft_pulses = shaft_pulses;
        interrupts();

        shaft_frame_count_updated = false;
        timer_frame_count_updated = false;

        current_pulse_difference = local_timer_pulses / timer_factor - local_shaft_pulses;

        unsigned long current_time = millis(); // might make this micros()?
        last_pulse_timestamp = current_time;

        pid_input = current_pulse_difference;
        myPID.Compute();
        new_dac_value = pid_output;
        dac.setVoltage(new_dac_value, false);
    }

        // Compute Error and feed PID and DAC
        // only consume pulse counters if both ISRs did their updates yet, otherwise we get plenty of false +/-1 diffs

        // if (shaft_frame_count_updated && timer_frame_count_updated)
        // {
        //     // read the counters atomically
        //     noInterrupts();
        //     local_timer_frames = timer_frames; // To Do: Could directly use current_frame_difference here?
        //     local_shaft_frames = shaft_frames;
        //     interrupts();


        //     // mark these counts as read
        //     shaft_frame_count_updated = false;
        //     timer_frame_count_updated = false;

        //     current_frame_difference = local_timer_frames - local_shaft_frames;
        //     // Serial.print(F("local_timer_frames: "));
        //     // Serial.print(local_timer_frames);
        //     // Serial.print(F(" - "));
        //     // Serial.print(F("local_shaft_frames: "));
        //     // Serial.println(local_shaft_frames);

        //     //temp debug code
        //     unsigned long current_time = millis();
        //     // unsigned long time_since_last = current_time - last_frame_timestamp;
        //     // Serial.print(F("Time since last frame (ms): "));
        //     // Serial.println(time_since_last);
        //     last_frame_timestamp = current_time;

        //     // should this be further down in the if block? Doesn't seem so
        //     pid_input = current_frame_difference;
        //     myPID.Compute();
        //     new_dac_value = pid_output;
        //     dac.setVoltage(new_dac_value, false);
        // }

        // Detect if we have are > half a frame off and light the red LED
        if (current_frame_difference < -6 || current_frame_difference > 6)
        {
            digitalWrite(LED_RED_PIN, HIGH);
        }
        else
        {
            digitalWrite(LED_RED_PIN, LOW);
        }

        // Debug output
        if (current_pulse_difference != last_pulse_difference)
        {
            if (millis() % 1000 < 10)
            {
                Serial.print(F("Mode: "));
                Serial.print(speedModeToString(speed_mode));
                Serial.print(F(", Error: "));
                Serial.print(current_pulse_difference);
                Serial.print(F(", DAC: "));
                Serial.println(new_dac_value);

                // Serial.print(F(" Current Pulse Difference: "));
                // Serial.print(current_pulse_difference);
                // Serial.print(F(" - Last Pulse Difference: "));
                // Serial.println(last_pulse_difference);
                }

            last_pulse_difference = current_pulse_difference; // Update the last_pulse_difference

            Serial.print(" Timer-Pulses: ");
            Serial.print(local_timer_pulses / timer_factor);
            Serial.print(", Shaft-Pulses: ");
            Serial.println(local_shaft_pulses);
        }

        // Stop detection
        if (hasStoppedSince(last_pulse_timestamp, 1000))
        {
            projector_state = PROJ_IDLE; // Projector is stopped
            Serial.println(F("[DEBUG] Projector stopped."));
            stopTimer1();
            timer_frame_count_updated = 0; // just in case the ISR fired again AND the shaft was still breaking. This could cause false PID computations.
            // timer_pulse_count_updated = 0; // just in case the ISR fired again AND the shaft was still breaking. This could cause false PID computations.
            shaft_pulses = 0;
            timer_pulses = 0;
            // Reset DAC and PID
            dac.setVoltage(DAC_INITIAL_VALUE, false); // reset the DAc to compensate for wound-up break corrections
            myPID.SetMode(MANUAL);
            pid_output = DAC_INITIAL_VALUE;
            pid_input = 0;
            myPID.Compute();
            Serial.print(F("PID Reset to initial DAC value: "));
            Serial.println(pid_output);
            myPID.SetMode(AUTOMATIC);
            digitalWrite(ENABLE_PIN, LOW);
            digitalWrite(LED_RED_PIN, LOW);

            FreqMeasure.end();
        }
    }
}

void measureFrequency()
{
    // ignore first revolution
    // average 3 revolutions
    // OR: 
    // average 2 revolutions
    // take 2nd result

    static double sum = 0;
    static int count = 48;
    
    if (FreqMeasure.available())
    {
        // average several reading together
        sum += FreqMeasure.read();
        count++;
        if (count > 48) // 48 impulses = 4 frames
        {
            float frequency = FreqMeasure.countToFrequency(sum / count);

            // Buffer to hold the formatted string
            char buffer[20];
            // Convert float to string with 5 decimal places
            dtostrf(frequency / 12, 8, 5, buffer); // (value, width, precision, buffer)

            Serial.println(buffer);
            u8x8.drawString(0, 1, buffer);
            sum = 0;
            count = 0;
        }
    }
    // FreqMeasure.end();
}

void initializeButton(Button2 &button, byte pin)
{
    button.begin(pin);
    button.setTapHandler(handleButtonTap);
    button.setDoubleClickTime(0); // disable double clicks
    button.setDebounceTime(10);
}

bool hasStoppedSince(unsigned long start, unsigned long duration)
{
    // Serial.print(millis());
    // Serial.print(" - ");
    // Serial.print(start);
    // Serial.print(" = ");
    // Serial.println(millis() - start);

    return (millis() - start) > duration;
}

void checkProjectorRunning()
{
    if (!new_shaft_impulse_available)
        return; // Skip processing if no new data

    new_shaft_impulse_available = false; // Reset the ISR's "new data available" flag

    projector_start_millis = millis(); // Track the start time of the projector running

    if (speed_mode == XTAL_AUTO)
    {
        double freq_sum = 0;
        int freq_count = 0;
        unsigned long last_available_freqMeasure_time = micros(); // Track the last time data was available

        FreqMeasure.begin();

        while (freq_count <= 47)    // collect 48 samples
        {
            if (FreqMeasure.available())
            {
                // Average several readings together
                freq_sum += FreqMeasure.read();
                freq_count++;
                /*
                Serial.print(freq_count);
                Serial.print(": ");
                Serial.println(freq_sum);
                */

                last_available_freqMeasure_time = micros(); // Update the last available time
            }

            // Check for timeout
            if (micros() - last_available_freqMeasure_time > STOP_THRESHOLD)
            {
                Serial.println(F("Shaft Noise ignored."));
                break; // Exit the loop
            }
        }

        FreqMeasure.end();

        // Only process if a full set of data was collected
        if (freq_count >= 48)
        {
            float detected_frequency = FreqMeasure.countToFrequency(freq_sum / freq_count);
            Serial.print(F("Detected frequency: "));
            Serial.println(detected_frequency / SHAFT_SEGMENT_COUNT);

            // Determine the mode based on the detected frequency
            changeSpeedMode(detected_frequency <= 21 * SHAFT_SEGMENT_COUNT ? XTAL_18 : XTAL_24);
            projector_speed_switch = (detected_frequency <= 21 * SHAFT_SEGMENT_COUNT ? 18 : 24);

            // Update projector state
            projector_state = PROJ_RUNNING;
        }

        // Reset variables
        freq_sum = 0;
        freq_count = 0;
    }
}


void changeSpeedMode(byte run_mode)
{
    // connect or disconnect the DAC
    digitalWrite(ENABLE_PIN, (run_mode == XTAL_NONE) ? LOW : HIGH);

    switch (run_mode)
    {
    case XTAL_NONE:
        Serial.println(F("XTAL_NONE"));
        break;
    case XTAL_AUTO:
        // reset the PID Output
        myPID.SetMode(MANUAL);
        pid_output = DAC_INITIAL_VALUE;
        myPID.Compute();
        myPID.SetMode(AUTOMATIC);
        // set DAC to initial value
        dac.setVoltage(DAC_INITIAL_VALUE, false);
        Serial.println(F("XTAL_AUTO"));
        break;
    case XTAL_16_2_3:
        Serial.println(F("XTAL_16_2_3"));
        setupTimer1forFps(FPS_16_2_3);
        break;
    case XTAL_18:
        Serial.println(F("XTAL_18"));
        setupTimer1forFps(FPS_18);
        break;
    case XTAL_23_976:
        Serial.println(F("XTAL_23_976"));
        setupTimer1forFps(FPS_23_976);
        break;
    case XTAL_24:
        Serial.println(F("XTAL_24"));
        setupTimer1forFps(FPS_24);
        break;
    case XTAL_25:
        Serial.println(F("XTAL_25"));
        setupTimer1forFps(FPS_25);
        break;
    default:
        Serial.println(F("Unknown Mode"));
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
    if (projector_state == PROJ_RUNNING)
    {
        if (btn == dropBackButton)
        {
            noInterrupts();
            shaft_pulses += SHAFT_SEGMENT_COUNT;
            interrupts();
        }
        else if (btn == catchUpButton)
        {
            noInterrupts();
            shaft_pulses -= SHAFT_SEGMENT_COUNT;
            interrupts();
        }
    }
}

void selectNextMode(Button2 &btn)
{
    // Determine the change based on which button was pressed
    int8_t change = (btn == leftButton) ? -1 : 1;

    // Update the run mode
    // previous_freq = ditherEndFreq;  // the struct's float with the actual fps. Could probably just use ditherEndFreq here.

    // Change the speed mode to an adjacent enum value ("change")
    speed_mode = (speed_mode + change + MODES_COUNT) % MODES_COUNT;
    
    // Prepare for re-calculating the ISR's timer_frames in sepmag mode
    float next_freq;

    noInterrupts();
    long local_shaft_impulse_count = shaft_pulses;
    interrupts();

    Serial.print(F("Average fps since start: "));
    // Serial.print((float)local_shaft_impulse_count / SHAFT_SEGMENT_COUNT);
    // Serial.print(F(" / "));
    // Serial.print((float)(millis() - projector_start_millis) / 1000);
    // Serial.print(F(" = "));
    Serial.println(((float)local_shaft_impulse_count / SHAFT_SEGMENT_COUNT) / (((float)millis() - projector_start_millis) / 1000), 2);

    // switch (speed_mode) // This is the new speed_mode we are about to switch TO
    // {
    //     case XTAL_NONE:
    //         break;
    //     case XTAL_AUTO:
    //         // Since
    //         Serial.print(F("Estimated fps so far (for previous_freq):"));
    //         Serial.println((millis() - projector_start_millis) / shaft_frames, 2);
    //         Serial.print(F("Proj. Speed Switch (to): "));
    //         Serial.println(projector_speed_switch);
    //         break;
    //     case XTAL_16_2_3:
    //     case XTAL_18:
    //     case XTAL_23_976:
    //     case XTAL_24:
    //     case XTAL_25:
    //         // Serial.println(F("Multi-Case!"));
    //         // copy the endFreq of the struct wuth speed_mode index - 2 (0 and 1 are NONE and AUTO)
    //         memcpy_P(&next_freq, &s_ditherTable[((speed_mode - 2))].endFreq, sizeof(float));
    //         break;
    // }

    // memcpy_P(&next_freq, &s_ditherTable[((speed_mode - 2))].endFreq, sizeof(float));

    // Serial.print(F("Speed change: "));
    // Serial.print(previous_freq, 4); // Print the float with 4 decimal places
    // Serial.print(F(" -> "));
    // Serial.println(next_freq, 4 );

    // Serial.print(timer_frames);
    // Serial.print(" / ");

    // if (speed_mode != XTAL_AUTO && speed_mode != XTAL_NONE)
    // {
    //     Serial.print(timer_frames);
    //     Serial.print(F(" / "));
    //     Serial.print(previous_freq);
    //     Serial.print(F(" * "));
    //     Serial.print(next_freq);
    //     Serial.print(F(" = "));
    //     Serial.println(static_cast<uint32_t>((timer_frames / previous_freq) * next_freq));

    //     noInterrupts();
    //     timer_frames = static_cast<uint32_t>((timer_frames / previous_freq) * next_freq);
    //     interrupts();
    // }

    changeSpeedMode(speed_mode);
}

bool setupTimer1forFps(byte desiredFps)
{
    // start with a new sync point, no need to catch up differences from before.
    if (projector_state == PROJ_IDLE)
    {
        noInterrupts();
        timer_frames = 0;
        // shaft_frames = 0;
        timer_modulus = 0;
        shaft_modulus = 0; 
        interrupts();
    }

    // Read the required dither config from PROGMEM, this saves RAM
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

    timer_factor = cfg.timerFactor;
    ditherEndFreq = cfg.endFreq;

    OCR1A = ditherBase;                   // OCR1A start value
    TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC-Mode (WGM12=1), Prescaler=8 => CS11=1
    TIMSK1 |= (1 << OCIE1A);              // enable Compare-A-Interrupt
    interrupts();

    return true;
}

void onShaftImpulseISR()
{
    // Expose the news
    shaft_pulses++;
    new_shaft_impulse_available = true;

    if (shaft_modulus == 0)
    {
        shaft_frames++;
        shaft_frame_count_updated = true;
    }
    shaft_modulus++;
    shaft_modulus %= (SHAFT_SEGMENT_COUNT);
}

ISR(TIMER1_COMPA_vect)
{   
    timer_pulses++;

    if (timer_modulus == 0)
    {
        timer_frames++;
        timer_frame_count_updated = true;
    }
    timer_modulus++;
    timer_modulus %= SHAFT_SEGMENT_COUNT * timer_factor;

    // Dither logic (fixed-point accumulator)
    ditherAccu32 += ditherFrac32;

    // If overflow => ditherAccu32 < ditherFrac32
    OCR1A = (ditherAccu32 < ditherFrac32) ? (ditherBase + 1) : ditherBase;
}

void stopTimer1()
{ // Stops Timer1, for when we are not in craystal running run_mode
    // TCCR1B &= ~(1 << CS11);
    noInterrupts();
    TIMSK1 &= ~(1 << OCIE1A);
    interrupts();
}

const char *speedModeToString(byte run_mode)
{
    switch (run_mode)
    {
    case XTAL_NONE:
        return "NONE";
    case XTAL_AUTO:
        return "AUTO";
    case XTAL_16_2_3:
        return "16 2/3";
    case XTAL_18:
        return "18";
    case XTAL_23_976:
        return "23.976";
    case XTAL_24:
        return "24";
    case XTAL_25:
        return "25";
    }
}
