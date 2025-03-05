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
const unsigned long toggleCooldown = 2000; // 2 seconds
unsigned long lastAutoToggleTime = 0;

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
const int hysteresis = 2; // 2% buffer

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

// Pin assignments
const int trigPin = 33;
const int echoPin = 25;

// Timeout in microseconds (30ms)
const unsigned int timeout = 30000; 

// State machine for ultrasonic measurement
enum UltrasonicState { READY, TRIGGERED, WAITING_FOR_FALLING, DONE };
volatile UltrasonicState usState = READY;

volatile unsigned long echoStartTime = 0;
volatile unsigned long echoEndTime = 0;
unsigned long triggerTime = 0;
float lastDistance = 0.0;
int sensorMaxError = 10;
int errorCount = 0;

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
||                                 ISR (Ultrasonic Sensor)                              ||
==========================================================================================
*/

void IRAM_ATTR echoISR() {
    static unsigned long lastPulseTime = 0;
    unsigned long now = micros();

    if (digitalRead(echoPin) == HIGH) {
        echoStartTime = now;
        usState = WAITING_FOR_FALLING;
    } else {
        // Ensure pulse duration is valid (>0)
        if (usState == WAITING_FOR_FALLING && now > echoStartTime) {
            echoEndTime = now;
            usState = DONE;
        }
    }
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

    // Set up sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT); // Try using INPUT, or if needed, INPUT_PULLUP
    delay(500);

    pinMode(buttonPin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonRelease, RISING);
    attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);

    echoStartTime = micros();
    echoEndTime = micros();

    u8g2.begin();
}


/*
==========================================================================================
||                                       Loop                                           ||
==========================================================================================
*/
void loop() {
    bool previousMotorState = mem.getBool(stateKey);

    float d = getDistance();

    Serial.print("Distance: ");
    Serial.print(d);
    Serial.println(" cm");

    if(d == -1 || d > containerHeight){
        Serial.println("Error!!!");
        ++errorCount;
        if(errorCount >= sensorMaxError){
            motorState = false;
            if(motorState != previousMotorState){
                mem.putBool(stateKey, motorState);
            }
            digitalWrite(motorPin, LOW);
        }
        return;
    }
    errorCount = 0;

    int waterLevelPercentage = map((int)d, containerHeight, containerOffset, 0, 100);
    waterLevelPercentage = constrain(waterLevelPercentage, 0, 100); // Clamp to 0-100

    /*
    if (getEvent() && waterLevelPercentage >= lowerThreshold && waterLevelPercentage <= upperThreshold) {
        motorState = !motorState;  // Toggle state manually
    } else if (waterLevelPercentage < (lowerThreshold - hysteresis)) {
        motorState = true;  // Auto-start motor
    } else if (waterLevelPercentage > (upperThreshold + hysteresis)) {
        motorState = false;  // Auto-stop motor
    }
    */

    if (getEvent() && waterLevelPercentage >= lowerThreshold && waterLevelPercentage <= upperThreshold) {
        motorState = !motorState;  // Toggle state manually (no cooldown)
        Serial.println("Manual Toggle!");
    } 
    else {
        // Auto-toggle with cooldown protection
        if (waterLevelPercentage < (lowerThreshold - hysteresis)) {
            if (!motorState && (millis() - lastAutoToggleTime >= toggleCooldown)) {
                motorState = true;
                lastAutoToggleTime = millis();
                Serial.println("Auto-ON (Low Level)");
            }
        } 
        else if (waterLevelPercentage > (upperThreshold + hysteresis)) {
            if (motorState && (millis() - lastAutoToggleTime >= toggleCooldown)) {
                motorState = false;
                lastAutoToggleTime = millis();
                Serial.println("Auto-OFF (High Level)");
            }
        }
    }
    
    // Save state if changed
    if (previousMotorState != motorState) {
        mem.putBool(stateKey, motorState);
        Serial.println("Motor state changed & saved!");
    }
    digitalWrite(motorPin, motorState ? HIGH : LOW);

    
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

    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= 10) {  // Non-blocking delay
        lastUpdate = millis();
        u8g2.sendBuffer();
    }
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
    // Start a new measurement if the sensor is ready
    if (usState == READY) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        triggerTime = micros();
        usState = TRIGGERED;
    }

    // Check for timeout in the WAITING_FOR_FALLING state
    if (usState == WAITING_FOR_FALLING && (micros() - triggerTime > timeout)) {
        // If no falling edge within timeout, force completion using timeout duration
        usState = READY;  // Reset state for the next measurement
        lastDistance = -1;
        return -1;  // Return error indicator
    }

    // Once the measurement is complete, calculate the distance
    if (usState == DONE) {
        unsigned long duration = echoEndTime - echoStartTime;

        // Check for invalid duration (e.g., if both times are 0)
        if (duration == 0) {
            usState = READY;
            lastDistance = -1;
            return -1;
        }

        lastDistance = (duration * 0.034) / 2.0;  // Convert duration to cm
        usState = READY;  // Reset state for the next measurement
    }

    return lastDistance;
}