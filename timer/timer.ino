// Adafruit GFX Library - Version: 1.11.3 
#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

// Adafruit LED Backpack Library - Version: 1.3.2
#include <Adafruit_LEDBackpack.h>


/***************************************************
  This is a library for our I2C LED Backpacks

  Designed specifically to work with the Adafruit LED 7-Segment backpacks
  ----> http://www.adafruit.com/products/881
  ----> http://www.adafruit.com/products/880
  ----> http://www.adafruit.com/products/879
  ----> http://www.adafruit.com/products/878

  These displays use I2C to communicate, 2 pins are required to
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
  with 3 Address Select pins: 0x70 thru 0x77

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_7segment matrix = Adafruit_7segment();
Adafruit_7segment matrix2 = Adafruit_7segment();

unsigned long startMillis;
unsigned long elapsedMillis = 0;

int hallSensor = 7; // Hall effect sensor signals start of race
int led = 13; // LED displays when hall effect sensor is triggered

bool started = false;
bool racer1Finished = false;
bool racer2Finished = false;

void setup() {
  Serial.begin(9600);
  Serial.println("setup timer");
  matrix.begin(0x70);
  matrix2.begin(0x72);

  pinMode(led, OUTPUT); //define LED as a output port
  pinMode(hallSensor, INPUT); //define switch as a output port
}

void loop() {
  // Check serial port for finished racers
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    switch (inByte) {
      case '1':
        racer1Finished = true;
        break;
      case '2':
        racer2Finished = true;
        break;
      case '0':
        started = false; // reset for the next race
        elapsedMillis = 0;
        racer1Finished = false;
        racer2Finished = false;
    }
  }

  // Hall effect sensor on switch starts race
  if (digitalRead(hallSensor) == LOW) {
    if (!started) {
      Serial.println("go"); // Send message to race control system
      started = true;
      startMillis = millis();
    }
    digitalWrite(led, HIGH);
  } else {
    digitalWrite(led, LOW);
  }

  // use internal time once race has started
  if (started) {
    elapsedMillis = (millis() - startMillis) / 10; // display hundredths of a second
  }

  // Control LED displays
  if (!racer1Finished) {
    matrix.println(elapsedMillis);
    matrix.drawColon(true);
    matrix.writeDisplay();
  }
  if (!racer2Finished) {
    matrix2.println(elapsedMillis);
    matrix2.drawColon(true);
    matrix2.writeDisplay();
  }

  delay(10);

}
