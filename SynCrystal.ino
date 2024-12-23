#include <avr/io.h>
#include <avr/interrupt.h>

// Pin and variables
constexpr int shaftPulsePin = 2;            // Pin for the projector's optical sensor signal (Pin 4 on TCA955)
constexpr int greenLedPin = 7;            
volatile byte validCount = 0;               // Counter for consecutive valid intervals
volatile bool projectorRunning = false;     // Flag indicating if the projector is running
volatile unsigned long lastShaftPulseTime = 0; // Timestamp of the last ISR trigger

// Timer-related variables
volatile uint8_t isr_counter = 0;  // Counter for cycles (stored as uint8_t)
volatile uint8_t multiplier = 1;   // Multiplier for the base frequency (stored as uint8_t)
volatile uint32_t crystalImps = 0; // Global variable to count crystal events
volatile uint32_t projectorImps = 0; // Global variable to count projector pulses

// Global variable to reflect the determined position of the projector's original fps switch
volatile uint8_t detectedFpsSwitchPos = 0; // 0 = No valid fps switch pos detected, 18 = 18fps, 24 = 24fps

// Timer frequency enum

enum class Frequency : uint8_t
{
    FREQ_16_66_HZ, // 16.666... Hz: Prescaler = 1, OCR = 63999, Multiplier = 15, Deviation ~0.000 ppm
    FREQ_18_HZ,    // 18 Hz: Prescaler = 1, OCR = 13266, Multiplier = 67, Deviation ~0.125 ppm
    FREQ_24_HZ,    // 24 Hz: Prescaler = 1, OCR = 333332, Multiplier = 2, Deviation ~1.000 ppm
    FREQ_25_HZ     // 25 Hz: Prescaler = 1, OCR = 319999, Multiplier = 2, Deviation ~0.000 ppm
};

// Multiplier constants for accuracy
constexpr uint8_t multipliers[] = {15, 67, 2, 2};                 // Corresponding to the frequencies
constexpr uint16_t ocr_values[] = {63999, 13266, 333332, 319999}; // OCR values for the timer frequencies

// FPS range settings (in microseconds per change)
constexpr unsigned long MIN_INTERVAL_16FPS = 2604; // (1 / 16 fps) / 24 changes = 2604 uS
constexpr unsigned long MIN_INTERVAL_17FPS = 2448; // (1 / 17 fps) / 24 changes = 2448 uS
constexpr unsigned long MAX_INTERVAL_19FPS = 2193; // (1 / 19 fps) / 24 changes = 2193 uS
constexpr unsigned long MAX_INTERVAL_20FPS = 2083; // (1 / 20 fps) / 24 changes = 2083 uS
constexpr unsigned long MIN_INTERVAL_22FPS = 1894; // (1 / 22 fps) / 24 changes = 1894 uS
constexpr unsigned long MIN_INTERVAL_23FPS = 1812; // (1 / 23 fps) / 24 changes = 1812 uS
constexpr unsigned long MAX_INTERVAL_25FPS = 1666; // (1 / 25 fps) / 24 changes = 1666 uS
constexpr unsigned long MAX_INTERVAL_26FPS = 1603; // (1 / 26 fps) / 24 changes = 1603 uS
constexpr byte REQUIRED_CONSECUTIVE = 6;           // Required number of consecutive valid intervals

void configure_timer1(Frequency freq_option)
{
    if (static_cast<uint8_t>(freq_option) >= sizeof(ocr_values) / sizeof(ocr_values[0]))
    {
        return; // Invalid option
    }

    // Disable interrupts during configuration
    noInterrupts();

    // Configure the timer
    TCCR1B = (1 << WGM12);                                 // CTC mode
    OCR1A = ocr_values[static_cast<uint8_t>(freq_option)]; // Set the appropriate OCR value
    TIMSK1 = (1 << OCIE1A);                                // Enable compare match interrupt

    // Set fixed prescaler of 1
    TCCR1B |= (1 << CS10);

    // Initialize the counter and multiplier
    isr_counter = 0;
    multiplier = multipliers[static_cast<uint8_t>(freq_option)];

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
    Serial.begin(115200);
    pinMode(shaftPulsePin, INPUT);              // Set pin for the optical sensor
    pinMode(13, OUTPUT);                        // LED pin as output
    pinMode(greenLedPin, OUTPUT);
    configure_timer1(Frequency::FREQ_16_66_HZ); // Example frequency configuration
    attachInterrupt(digitalPinToInterrupt(shaftPulsePin), onShaftImpulse, CHANGE);
}

void loop()
{
    static bool previousProjectorRunning = false;

    // Detect a change in the projectorRunning flag
    if (projectorRunning != previousProjectorRunning)
    {
        digitalWrite(greenLedPin, projectorRunning ? HIGH : LOW);
    }

    previousProjectorRunning = projectorRunning;

    // Handle timer-based crystalImps logic
    noInterrupts();                   // Temporarily disable interrupts
    uint32_t localImps = crystalImps; // Safely copy shared variable
    interrupts();                     // Re-enable interrupts

    if (localImps >= 1000)
    {
        //PORTB ^= (1 << PORTB5); // Toggle the LED for every 1000 events

        noInterrupts();
        crystalImps = 0; // Reset counter safely
        interrupts();
    }
}

void onShaftImpulse()
{
    unsigned long currentTime = micros();
    unsigned long interval = currentTime - lastShaftPulseTime;

    lastShaftPulseTime = currentTime;
    Serial.println(interval);

    // Check if the interval is within valid FPS ranges
    if (interval <= MIN_INTERVAL_16FPS && interval >= MAX_INTERVAL_20FPS)
    {
        detectedFpsSwitchPos = 18; // 18 fps detected
        if (validCount < REQUIRED_CONSECUTIVE)
        {
            validCount++;
        }
    }
    else if (interval <= MIN_INTERVAL_22FPS && interval >= MAX_INTERVAL_26FPS)
    {
        detectedFpsSwitchPos = 24; // 24 fps detected
        if (validCount < REQUIRED_CONSECUTIVE)
        {
            validCount++;
        }
    }
    else
    {
        detectedFpsSwitchPos = 0; // No valid frequency detected
        validCount = 0;           // Reset the valid count on an invalid interval
    }

    // Update the projectorRunning flag
    projectorRunning = (validCount >= REQUIRED_CONSECUTIVE);
}