#include <avr/io.h>
#include <avr/interrupt.h>

// Pins and variables
const int shaftImpulse = 2;                 // Pin for the optical sensor
volatile byte validCount = 0;               // Counter for consecutive valid intervals
volatile bool projectorRunning = false;     // Flag indicating if the projector is running
volatile unsigned long lastImpulseTime = 0; // Timestamp of the last ISR trigger

// Timer-related variables
volatile uint8_t isr_counter = 0;  // Counter for cycles (stored as uint8_t)
volatile uint8_t multiplier = 1;   // Multiplier for the base frequency (stored as uint8_t)
volatile uint32_t crystalImps = 0; // Global variable to count timer events

// Timer frequency enum
typedef enum
{
    FREQ_16_66_HZ, // 16.666... Hz: Prescaler = 1, OCR = 63999, Multiplier = 15, Deviation ~0.000 ppm
    FREQ_18_HZ,    // 18 Hz: Prescaler = 1, OCR = 13266, Multiplier = 67, Deviation ~0.125 ppm
    FREQ_24_HZ,    // 24 Hz: Prescaler = 1, OCR = 333332, Multiplier = 2, Deviation ~1.000 ppm
    FREQ_25_HZ     // 25 Hz: Prescaler = 1, OCR = 319999, Multiplier = 2, Deviation ~0.000 ppm
} frequency_t;

// Multiplier constants for accuracy
const uint8_t multipliers[] = {15, 67, 2, 2};                 // Corresponding to the frequencies
const uint16_t ocr_values[] = {63999, 13266, 333332, 319999}; // OCR values for the timer frequencies

// FPS range settings (in microseconds per change)
const unsigned long MIN_INTERVAL_17FPS = 2448; // (1 / 17 fps) / 24 changes
const unsigned long MAX_INTERVAL_19FPS = 2193; // (1 / 19 fps) / 24 changes
const unsigned long MIN_INTERVAL_23FPS = 1812; // (1 / 23 fps) / 24 changes
const unsigned long MAX_INTERVAL_25FPS = 1666; // (1 / 25 fps) / 24 changes
const byte REQUIRED_CONSECUTIVE = 6;           // Required number of consecutive valid intervals

void configure_timer1(frequency_t freq_option)
{
    if (freq_option < FREQ_16_66_HZ || freq_option > FREQ_25_HZ)
    {
        return; // Invalid option
    }

    // Disable interrupts during configuration
    noInterrupts();

    // Configure the timer
    TCCR1B = (1 << WGM12);           // CTC mode
    OCR1A = ocr_values[freq_option]; // Set the appropriate OCR value
    TIMSK1 = (1 << OCIE1A);          // Enable compare match interrupt

    // Fixed prescaler of 1
    TCCR1B |= (1 << CS10);

    // Initialize the counter and multiplier
    isr_counter = 0;
    multiplier = multipliers[freq_option];

    // Re-enable interrupts after configuration
    interrupts();
}

ISR(TIMER1_COMPA_vect)
{
    isr_counter++;
    if (isr_counter >= multiplier)
    {
        isr_counter = 0;

        // Increment the global crystalImps variable
        crystalImps++;
    }
}

void setup()
{
    pinMode(shaftImpulse, INPUT);    // Set pin for the optical sensor
    pinMode(13, OUTPUT);             // LED pin as output
    configure_timer1(FREQ_16_66_HZ); // Example frequency configuration
    attachInterrupt(digitalPinToInterrupt(shaftImpulse), onShaftImpulse, CHANGE);
}

void loop()
{
    static bool previousProjectorRunning = false;

    // Detect a change in the projectorRunning flag
    if (projectorRunning != previousProjectorRunning)
    {
        digitalWrite(13, projectorRunning ? HIGH : LOW);
    }

    previousProjectorRunning = projectorRunning;

    // Handle timer-based crystalImps logic
    noInterrupts();                   // Temporarily disable interrupts
    uint32_t localImps = crystalImps; // Safely copy shared variable
    interrupts();                     // Re-enable interrupts

    if (localImps >= 1000)
    {
        PORTB ^= (1 << PORTB5); // Toggle the LED for every 1000 events

        noInterrupts();
        crystalImps = 0; // Reset counter safely
        interrupts();
    }
}

void onShaftImpulse()
{
    unsigned long currentTime = micros();
    unsigned long interval = currentTime - lastImpulseTime;
    lastImpulseTime = currentTime;

    // Check if the interval is within valid FPS ranges
    if ((interval >= MIN_INTERVAL_17FPS && interval <= MAX_INTERVAL_19FPS) ||
        (interval >= MIN_INTERVAL_23FPS && interval <= MAX_INTERVAL_25FPS))
    {
        if (validCount < REQUIRED_CONSECUTIVE)
        {
            validCount++;
        }
    }
    else
    {
        validCount = 0; // Reset the counter on invalid interval
    }

    // Update the projectorRunning flag
    projectorRunning = (validCount >= REQUIRED_CONSECUTIVE);
}