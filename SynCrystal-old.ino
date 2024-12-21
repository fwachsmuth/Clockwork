/*

   18/24 Hz Reference and Controller
   for Projectors with TCA 955-driven DC motors

   Friedemann Wachsmuth 2 Nov 2019
    Todo:
    - Wait for the first projector impulse to set offset to 0
    - Allow speed changes
    - Build Menu
    - Auto-Measure correction metrics with poor-mans freq measure
    - Store them
    - Correct Divider Errors
    - Add SSR to decouple the Controller
    - Measure the 678 voltage
    - Consider an ePot
    - Move to 3.3V Logic (OLED!)
    - Wago Leiterplattenklemmen
    - Verpolungsschutz
    - slow pwm
    - Testen ob 2. Display erkannt wird
    
    


*/

#include <Arduino.h>
// #include <U8x8lib.h>          // Cheap Display Driver
#include <U8g2lib.h>             // Display Driver

#include <Adafruit_MCP4725.h> // Fancy DAC for voltage control
#include <SwitchManager.h>    // Button Handling and Debouncing

// U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);  // Instatiate the cheapo I2C OLED Display
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

Adafruit_MCP4725 dac;                                   // Instantiate the DAC
SwitchManager leftButton;                               // Instantiate the Button Handlers  
SwitchManager midButton;
SwitchManager rightButton;

#define impDetectorPin    3  // Here goes the projector impulses

#define ledSlowerRed      4  //  --
#define ledSlowerYellow   5  //  -
#define ledGreen          6  //  o
#define ledFasterYellow   7  //  +
#define ledFasterRed      8  //  ++

#define leftButtonPin        9
#define BLOCKED_BY_TIMER1    10 // So do not use it! 
#define midButtonPin         11
#define rightButtonPin       12


// Timer Variables

int timerFactor = 0;      // this is used for the Timer1 postscaler, since multiples of 18 and 24 Hz give better accuracy
volatile int timerDivider = 0; // For Modulo in the ISR
volatile unsigned long timerFrames = 0;


// Projector Variables

byte segmentCount = 4;    // What kind of Shutter Blade do we have?
volatile int projectorDivider = 0; // For Modulo in the ISR
volatile unsigned long projectorFrames = 0;


// Other variables
byte selectedSpeed;       // Takes 16, 16 2/3, 18, 24 or 25 fps right now
unsigned long millisNow = 0;
unsigned long lastMillis = 0;
long frameDifference = 0;

int lastCorrection = 0;


void setup() {

  selectedSpeed = 24;

  Serial.begin(115200);
  pinMode(impDetectorPin, INPUT);
  //  pinMode(ctrlOutPin, OUTPUT);

  pinMode(ledSlowerRed, OUTPUT);     //  --
  pinMode(ledSlowerYellow, OUTPUT);  //  -
  pinMode(ledGreen, OUTPUT);         //  o
  pinMode(ledFasterYellow, OUTPUT);  //  +
  pinMode(ledFasterRed, OUTPUT);     //  ++

  leftButton.begin(leftButtonPin, onLeftPressed);
  midButton.begin(midButtonPin, onMidPressed);
  rightButton.begin(rightButtonPin, onRightPressed);

  setupTimer1forFps(selectedSpeed); // takes 16, 1666, 18, 24 or 25 only

  dac.begin(0x60);

  attachInterrupt(digitalPinToInterrupt(impDetectorPin), projectorCountISR, CHANGE);

//  u8x8.begin();
//  u8x8.setPowerSave(0);
//  helloWorld();
  u8g2.begin();  
  

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvR18_tr);
    u8g2.drawStr(0,20,"17.5924 fps");
    
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.drawStr(0,64,"18");
    u8g2.setFont(u8g2_font_helvR14_tr);
    u8g2.drawStr(57,64,"24");
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.drawStr(110,64,"25");
  } while ( u8g2.nextPage() );

}

void loop() {
  leftButton.check();  // check for presses  
  midButton.check();
  rightButton.check();
  
  millisNow = millis();

  frameDifference = timerFrames - projectorFrames;
  controlProjector(frameDifference);

  if ((millisNow % 10000 == 0) && (lastMillis != millisNow)) {

    Serial.print("Timer: ");
    Serial.print(timerFrames);
    Serial.print(", Projektor: ");
    Serial.print(projectorFrames);
    Serial.print(", Differenz ***: ");
    Serial.println(frameDifference);
    lastMillis = millisNow;

  }
}


ISR(TIMER1_COMPA_vect) {
  if (timerDivider == 0) {
    //    digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
    //    digitalWrite(ledPin, HIGH);
    //    digitalWrite(ledPin, LOW);
    timerFrames++;
  }
  timerDivider++;
  timerDivider %= timerFactor;

}

void handleSwitchPress (const byte newState, const unsigned long interval, const byte whichPin) {
  if (interval >= 1000)
    return;
}

  
void onLeftPressed(const byte newState, const unsigned long interval, const byte whichPin) {
  if (newState == LOW)
    Serial.println("Left has been pressed!");
}

void onMidPressed(const byte newState, const unsigned long interval, const byte whichPin) {
  if (newState == LOW)
    Serial.println("Middle has been pressed!");
}

void onRightPressed(const byte newState, const unsigned long interval, const byte whichPin) {
  if (newState == LOW)
    Serial.println("Right has been pressed!");
}



void projectorCountISR() {
  if (projectorDivider == 0) {
    projectorFrames++;
  }
  projectorDivider++;
  projectorDivider %= (segmentCount * 2); // we are triggering on CHANGE
}

void helloWorld() {

}

void setLeds(int bargraph) {
  if (bargraph >= -2 && bargraph <= 2) {
    digitalWrite(ledSlowerRed, LOW);
    digitalWrite(ledSlowerYellow, LOW);
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledFasterYellow, LOW);
    digitalWrite(ledFasterRed, LOW);

    digitalWrite(bargraph + 6, HIGH);   // LDs are on PIN 4-8, so mapping to -2...+2 here
  }
}

void controlProjector(int correction) {
  if (correction != lastCorrection) {
    if (correction <= -2) {
      setLeds(-2);
      dac.setVoltage(1710, false);
    } else if (correction == -1) {
      setLeds(-1);
      dac.setVoltage(1605, false);
    } else if (correction == 0) {
      setLeds(0);
      dac.setVoltage(1555, false);
    } else if (correction == 1) {
      setLeds(1);
      dac.setVoltage(1500, false);  //1500
    } else if (correction >= 2) {
      setLeds(2);
      dac.setVoltage(1395, false);
    }
    lastCorrection = correction;
  }
}


bool setupTimer1forFps(byte sollfps) {

  if (sollfps == 16 || sollfps == 1666 || sollfps ==  18 || sollfps == 24 || sollfps == 25) {
    noInterrupts();
    // Clear registers
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    // CTC
    TCCR1B |= (1 << WGM12);

    switch (sollfps) {
      case 16:
        OCR1A = 15624;    // 16 Hz (16000000/((15624+1)*64))
        TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64
        timerFactor = 1;

        break;
      case 1666:
        OCR1A = 14999;    // 16 2/3 Hz (16000000/((14999+1)*64))
        TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64
        timerFactor = 1;

        break;
      case 18:
        OCR1A = 10100;    // 198.000198000198 Hz (16000000/((10100+1)*8)),
        // divided by 3 is 18.000018.. Hz
        //              or 18 2/111,111
        //              or 2,000,000/111,111
        //
        TCCR1B |= (1 << CS11);  // Prescaler 8
        timerFactor = 11;

        break;
      case 24:
        OCR1A = 60605;    // 264.000264000264 Hz (16000000/((60605+1)*1)),
        // divided by 11 is 24.000024.. Hz
        //               or 24 8/333,333
        //               or 8,000,000 / 333,333
        //
        TCCR1B |= (1 << CS10);  // Prescaler 1
        timerFactor = 11;

        break;
      case 25:
        OCR1A = 624;      // 25 Hz (16000000/((624+1)*1024))
        TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
        timerFactor = 1;

        break;
      default:
        break;
    }
    // Output Compare Match A Interrupt Enable
    TIMSK1 |= (1 << OCIE1A);
    interrupts();
  } else {
    // invalid fps requested
    return false;
  }

}
