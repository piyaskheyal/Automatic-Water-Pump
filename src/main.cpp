#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include<Preferences.h>


/*
==========================================================================================
||                                Motor Variables                                       ||
==========================================================================================
*/
Preferences mem;

// Motor control with persistent storage
bool motorState = false;
const char* prefsNamespace = "motorCtrl";
const char* stateKey = "motorState";
const int motorPin = 18;

// Motor status text coordinates (right portion of the display)
const int motorStatusX = 65;
const int motorStatusY = 40;


// Display Settings
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

/*
==========================================================================================
||                                 Container Spec                                       ||
==========================================================================================
*/
const int lowerThreshold = 5;
const int upperThreshold = 90;
const int containerOffset = 10;
const int containerHeight = 120;  // Maximum measurable distance in cm

/*
==========================================================================================
||                               Button Variables                                       ||
==========================================================================================
*/
const int buttonPin = 32;           // GPIO connected to the button
volatile bool buttonReleased = false; // Flag set by the ISR
unsigned long lastDebounceTime = 0; // Timestamp of last valid event
const unsigned long debounceDelay = 50; // Debounce time in ms

/*
==========================================================================================
||                            Ultrasonic Sensor Variable                                ||
==========================================================================================
*/

const int trig_pin = 33;
const int echo_pin = 25;

/*
==========================================================================================
||                                      Title                                           ||
==========================================================================================
*/

const char* title = "Water Level";
const int titleX = 20;   // X position for the title text
const int titleY = 12;   // Y position for the title text

/*
==========================================================================================
||                                   Vertical Bar                                       ||
==========================================================================================
*/
const int barX = 10;
const int barY = 20;         // Start the bar below the title
const int barMaxHeight = 35; // Maximum height in pixels for the bar
const int barWidth = 10;     // Bar width in pixels


/*
==========================================================================================
||                                  ISR (Button)                                        ||
==========================================================================================
*/
void IRAM_ATTR handleButtonRelease() {
    buttonReleased = true; // Set flag when button is released
    lastDebounceTime = millis();
}


/*
==========================================================================================
||                               Function Declaration                                   ||
==========================================================================================
*/
float getDistance();
bool getEvent();


/*
==========================================================================================
||                                      Setup                                           ||
==========================================================================================
*/
void setup() {
    Serial.begin(115200);

    mem.begin(prefsNamespace, false);  // RW mode

    // Load stored motor state
    motorState = mem.getBool(stateKey, false);  // Default to false if not found
    Serial.printf("Loaded motor state: %s\n", motorState ? "ON" : "OFF");

    pinMode(motorPin, OUTPUT);
    digitalWrite(motorPin, motorState);

    pinMode(echo_pin, INPUT);
    pinMode(trig_pin, OUTPUT);
    digitalWrite(trig_pin, LOW);

    pinMode(buttonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonRelease, RISING);

    u8g2.begin();
}


/*
==========================================================================================
||                                      Loop                                           ||
==========================================================================================
*/
void loop() {
    float d = getDistance();
    Serial.print("Distance: ");
    Serial.print(d);
    Serial.print(" cm | ");
    Serial.print(d / 2.54);
    Serial.println(" in");

    int waterLevelPercentage = map((int)d, containerHeight, containerOffset, 0, 100);
    waterLevelPercentage = constrain(waterLevelPercentage, 0, 100); // Clamp to 0-100

    bool previousMotorState = mem.getBool(stateKey);

    if(getEvent() && (waterLevelPercentage >= lowerThreshold && waterLevelPercentage <= upperThreshold)){
        motorState = !motorState;

        if(previousMotorState != motorState){
            mem.putBool(stateKey, motorState);  // Save new state
            Serial.println("State saved");
        }
        Serial.println("Manual Toggle!");
    }else{
        if(waterLevelPercentage < lowerThreshold){
            if(!motorState) {
                motorState = true;
                if(previousMotorState != motorState){
                    mem.putBool(stateKey, motorState);  // Save auto-enable
                }
            }
        } else if(waterLevelPercentage > upperThreshold){
            if(motorState) {
                motorState = false;
                if(previousMotorState != motorState){
                    mem.putBool(stateKey, motorState);  // Save auto-disable
                }
            }
        }
    }

    if(motorState) digitalWrite(motorPin,HIGH);
    else digitalWrite(motorPin, LOW);
    
    // Calculate the fill height for the bar:
    // When waterLevelPercentage is 100, fill height equals barMaxHeight (full bar);
    // When waterLevelPercentage is 0, fill height is 0.
    int fillHeight = (waterLevelPercentage * barMaxHeight / 100.0);
    // Compute the top Y coordinate of the filled area (to fill from the bottom up)
    int fillY = barY + (barMaxHeight - fillHeight);

    u8g2.clearBuffer();

    // Title
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(titleX, titleY, title);

    // Vertical Bar
    u8g2.drawFrame(barX, barY, barWidth, barMaxHeight);
    u8g2.drawBox(barX, fillY, barWidth, fillHeight);

    // Water Level Percentage
    char buf[20];
    sprintf(buf, "%d%%", waterLevelPercentage);
    u8g2.drawStr(barX + barWidth + 5, barY + barMaxHeight / 2, buf);

    if (motorState) {
        u8g2.drawStr(motorStatusX, motorStatusY, "Motor ON");
    } else{
        u8g2.drawStr(motorStatusX, motorStatusY, "Motor OFF");
    }

    u8g2.sendBuffer();

    delay(10);
}

/*
==========================================================================================
||                                 Button Event                                         ||
==========================================================================================
*/
bool getEvent() {
    if (buttonReleased) {
      unsigned long currentTime = millis();
      
      // Check if debounce delay has passed
      if (currentTime - lastDebounceTime >= debounceDelay) {
        bool buttonState = digitalRead(buttonPin);
        buttonReleased = false; // Reset flag FIRST to prevent re-trigger
        lastDebounceTime = currentTime;
        
        // Return true only if button is still released (HIGH)
        return (buttonState == HIGH);
      }
    }
    return false;
  }


/*
==========================================================================================
||                            Ultrasonic Distance Detection                             ||
==========================================================================================
*/
float getDistance() {
    const int numSamples = 2;
    float sum = 0.0;

    for (int i = 0; i < numSamples; i++) {
        digitalWrite(trig_pin, LOW);
        delayMicroseconds(2);

        digitalWrite(trig_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig_pin, LOW);

        // Measure the duration of the HIGH pulse on the echo pin with a timeout of 30ms
        long duration = pulseIn(echo_pin, HIGH, 30000);
        float distance = (duration * 0.034) / 2;
        sum += distance;

        delay(5);  // Short delay between samples
    }
    
    return sum / numSamples;
}