#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include<Preferences.h>
#include <WiFi.h>
#include "time.h"


const unsigned char sensorError [] PROGMEM = {
	// 'sensorError, 128x64px
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x02, 0x8e, 0x87, 0xc3, 0x91, 0x81, 0x40, 0x26, 0xe3, 0xc8, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x02, 0x91, 0x48, 0x24, 0x52, 0x80, 0x40, 0xa1, 0x10, 0x29, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x1c, 0x91, 0x48, 0x20, 0x32, 0x80, 0xc3, 0x60, 0x10, 0x19, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x20, 0x9f, 0x88, 0x23, 0x12, 0x80, 0x40, 0x20, 0x10, 0x09, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x20, 0x81, 0x08, 0x24, 0x12, 0x80, 0x40, 0x20, 0x10, 0x09, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x22, 0x91, 0x48, 0x24, 0x12, 0x80, 0x40, 0x20, 0x10, 0x09, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x1c, 0x8e, 0x88, 0xc3, 0x11, 0x80, 0x4f, 0x20, 0xe0, 0x08, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*
==========================================================================================
||                                  Wi-Fi & Timer                                       ||
==========================================================================================
*/
const char* ssid     = "Router 1";
const char* password = "kheyal2g";

// NTP settings
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 6 * 3600;  // For UTC+6
const int   daylightOffset_sec = 0;    // No DST
const unsigned int ntpTimeout = 5000; // 5 sec

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
const unsigned long toggleCooldown = 2000; // 2 sec
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
const int lowerThreshold = 30;
const int upperThreshold = 85;
const int extremeUpperThreshold = 95;
const int containerOffset = 15;
const int containerHeight = 115;  // Maximum measurable distance in cm
const int hysteresis = 2; // 2% buffer

/*
==========================================================================================
||                               Button Variables                                       ||
==========================================================================================
*/

const int buttonPin = 32;           // GPIO connected to the button
volatile bool buttonPressed = false; // Flag set by the ISR
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
const int titleX = 30;   // X position for the title text
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
void IRAM_ATTR handleButtonPress() {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = micros();
    
    // Software debounce
    if (interruptTime - lastInterruptTime > debounceDelay*1000) {
        buttonPressed = true;
    }
    lastInterruptTime = interruptTime;
}


/*
==========================================================================================
||                                 ISR (Ultrasonic Sensor)                              ||
==========================================================================================
*/

void IRAM_ATTR echoISR() {
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
void checkWiFi();


/*
==========================================================================================
||                                      Setup                                           ||
==========================================================================================
*/
void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    // Connect to Wi-Fi
    Serial.println("Connecting to WiFi");
        while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print("#");
    }
    Serial.println("\nConnected to WiFi");

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
    attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);

    attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);

    echoStartTime = micros();
    echoEndTime = micros();

    u8g2.begin();

    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Synchronize time with NTP server
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    unsigned long start = millis();
    bool timeObtained = false;

    while (!timeObtained && (millis() - start < ntpTimeout)) { // try for up to 5 seconds
        if (getLocalTime(&timeinfo)) {
            timeObtained = true;
        } else {
            delay(100);
            Serial.print(".");
        }
    }

    if (!timeObtained) {
        Serial.println("Failed to obtain time");
        return;
    }


    Serial.print("Current time: ");
    Serial.println(asctime(&timeinfo));
}


/*
==========================================================================================
||                                       Loop                                           ||
==========================================================================================
*/
void loop() {
    bool previousMotorState = mem.getBool(stateKey);
    struct tm timeinfo;
    int hr = -1, min = -1;

    checkWiFi();

    if (getLocalTime(&timeinfo)){
        hr = timeinfo.tm_hour;
        min = timeinfo.tm_min;
    }else{
        Serial.println("Time unavilable.  Skipping auto Toggle");
    }
    if(hr == 1 && min == 30){
        // Visual feedback
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(10, 32, "Scheduled Restart");
        u8g2.sendBuffer();

        delay(60*1000);  // Ensure display updates

        ESP.restart();
    }
    bool withinAllowedHours = 
    (hr >= 7 && hr < 12) || 
    (hr == 13 && min >= 30) ||  // 1:30 PM to 1:59 PM
    (hr >= 14 && hr < 18);      // 2:00 PM to 5:59 PM

    float d = getDistance();

    Serial.print("Distance: ");
    Serial.print(d);
    Serial.println(" cm");

    if((d == -1) || (d > containerHeight)){
        Serial.println("Error!!!");
        ++errorCount;
        if(errorCount >= sensorMaxError){
            motorState = false;
            if(motorState != previousMotorState){
                mem.putBool(stateKey, motorState);
            }
            digitalWrite(motorPin, LOW);
        }
        u8g2.clearBuffer();
        u8g2.drawXBMP(0, 0, 128, 64, sensorError);
        u8g2.sendBuffer();
        delay(10);
    }else{
        errorCount = 0;

        int waterLevelPercentage = map((int)d, containerHeight, containerOffset, 0, 100);
        waterLevelPercentage = constrain(waterLevelPercentage, 0, 100); // Clamp to 0-100

        if (getEvent()) {
            motorState = !motorState;  // Toggle state manually (no cooldown)
            Serial.println("Manual Toggle!");
        }else {
            // Auto-toggle with cooldown protection
            if ((waterLevelPercentage < (lowerThreshold - hysteresis)) && withinAllowedHours) {
                // Auto-on: if water level is too low, turn on the pump
                if (!motorState && (millis() - lastAutoToggleTime >= toggleCooldown)) {
                    motorState = true;
                    lastAutoToggleTime = millis();
                    Serial.println("Auto-ON (Low Level)");
                }
            }else if (waterLevelPercentage > extremeUpperThreshold) {// Extreme level check (MUST COME BEFORE REGULAR UPPER CHECK)
                if (motorState) {
                    motorState = false;
                    lastAutoToggleTime = millis();  // Reset cooldown
                    Serial.println("Emergency OFF (Extreme Level)");
                }
            }else if (waterLevelPercentage > (upperThreshold + hysteresis)) {
                // Immediate shutdown when upper threshold is crossed
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
        u8g2.drawStr(barX + barWidth + 5, barY + (barMaxHeight / 2) + 3, buf);

        if (motorState) {
            u8g2.drawStr(motorStatusX, motorStatusY, "Pump ON");
        } else{
            u8g2.drawStr(motorStatusX, motorStatusY, "Pump OFF");
        }

        static unsigned long lastUpdate = 0;
        if (millis() - lastUpdate >= 10) {
            lastUpdate = millis();
            u8g2.sendBuffer();
        }
    }
}

/*
==========================================================================================
||                                 Button Event                                         ||
==========================================================================================
*/

bool getEvent() {
    static bool waitingForRelease = false;
    static unsigned long pressTime = 0;
    
    if (buttonPressed) {
        buttonPressed = false;
        waitingForRelease = true;
        pressTime = millis();
    }
    
    if (waitingForRelease) {
        // Check if button is released after minimum press time
        if (digitalRead(buttonPin) == HIGH) { // Button released
            waitingForRelease = false;
            return true;
        }
        
        // Timeout for button being held too long
        if (millis() - pressTime > 1000) { // 1 second timeout
            waitingForRelease = false;
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

    // Check for timeout in Triggered state
    if (usState == TRIGGERED) {
        if (micros() - triggerTime > timeout) {
            usState = READY;
            lastDistance = -1;
            return -1;
        }
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

void checkWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(ssid, password);
        Serial.println("Reconnecting to WiFi...");
        delay(1000); // Adjust as needed
    }
}