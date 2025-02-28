#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


const int lowerThreshold = 5;
const int upperThreshold = 90;

// Button Variables
const int buttonPin = 16;           // GPIO connected to the button
volatile bool buttonReleased = false; // Flag set by the ISR
unsigned long lastDebounceTime = 0; // Timestamp of last valid event
const unsigned long debounceDelay = 50; // Debounce time in ms

const int motorPin = 4;
bool motorState = false;

const int trig_pin = 19;
const int echo_pin = 18;

const float maxDistance = 30;  // Maximum measurable distance in cm

// Title settings
const char* title = "Water Level";
const int titleX = 20;   // X position for the title text
const int titleY = 12;   // Y position for the title text

// Vertical bar settings (positioned below the title)
const int barX = 10;
const int barY = 20;         // Start the bar below the title
const int barMaxHeight = 35; // Maximum height in pixels for the bar
const int barWidth = 10;     // Bar width in pixels

// Motor status text coordinates (right portion of the display)
const int motorStatusX = 65;
const int motorStatusY = 40;

// Interrupt Service Routine (ISR) for button release
void IRAM_ATTR handleButtonRelease() {
    buttonReleased = true; // Set flag when button is released
    lastDebounceTime = millis();
}

float getDistance();
bool getEvent();

void setup() {
    Serial.begin(115200);
    pinMode(motorPin, OUTPUT);
    pinMode(echo_pin, INPUT);
    pinMode(trig_pin, OUTPUT);
    digitalWrite(trig_pin, LOW);
    digitalWrite(motorPin, LOW);
    pinMode(buttonPin, INPUT_PULLUP); // Enable internal pull-up
    attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonRelease, RISING);

    // Initialize the OLED display
    u8g2.begin();
}

void loop() {
    float d = getDistance();
    Serial.print("Distance: ");
    Serial.print(d);
    Serial.print(" cm | ");
    Serial.print(d / 2.54);
    Serial.println(" in");

    // Calculate percentage based on the measured distance.
    // p represents the empty percentage.
    int p = (int)((d / maxDistance) * 100);
    // The water level percentage (full when distance is 0)
    int waterLevelPercentage = 100 - p;


    // Clamp waterLevelPercentage between 0 and 100.
    if(waterLevelPercentage < 0) waterLevelPercentage = 0;
    else if(waterLevelPercentage > 100) waterLevelPercentage = 100;

    if(getEvent() && (waterLevelPercentage >= lowerThreshold && waterLevelPercentage <= upperThreshold)){
        motorState = !motorState;
    }else{
        if(waterLevelPercentage < lowerThreshold){
            motorState = true;
        }else if(waterLevelPercentage > upperThreshold){
            motorState = false;
        }
    }

    if(motorState) digitalWrite(motorPin,HIGH);
    else digitalWrite(motorPin, LOW);
    
    // Calculate the fill height for the bar:
    // When waterLevelPercentage is 100, fill height equals barMaxHeight (full bar);
    // When waterLevelPercentage is 0, fill height is 0.
    int fillHeight = (int)(waterLevelPercentage * barMaxHeight / 100.0);
    // Compute the top Y coordinate of the filled area (to fill from the bottom up)
    int fillY = barY + (barMaxHeight - fillHeight);

    // Prepare the display
    u8g2.clearBuffer();

    // Draw the title
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(titleX, titleY, title);

    // Draw the vertical bar frame
    u8g2.drawFrame(barX, barY, barWidth, barMaxHeight);
    // Draw the filled portion of the bar from the bottom up
    u8g2.drawBox(barX, fillY, barWidth, fillHeight);

    // Display the water level percentage next to the bar
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

// Returns true only if a valid button release is detected
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

float getDistance() {
    const int numSamples = 15;
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

        delay(8);  // Short delay between samples
    }
    
    return sum / numSamples;
}