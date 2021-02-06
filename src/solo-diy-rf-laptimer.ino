/**
 * DIY RF Laptimer by Andrey Voroshkov (bshep)
 * SPI driver based on fs_skyrf_58g-main.c by Simon Chambers
 * fast ADC reading code is by "jmknapp" from Arduino forum
 * fast port I/O code from http://masteringarduino.blogspot.com.by/2013/10/fastest-and-smallest-digitalread-and.html

The MIT License (MIT)

Copyright (c) 2016 by Andrey Voroshkov (bshep)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// #define DEBUG

#ifdef DEBUG
    #define DEBUG_CODE(x) do { x } while(0)
#else
    #define DEBUG_CODE(x) do { } while(0)
#endif

#include <avr/pgmspace.h>
#include "fastReadWrite.h"
#include "fastADC.h"
#include "pinAssignments.h"
#include "channels.h"
#include "sendSerialHex.h"
#include "rx5808spi.h"
#include "sounds.h"

#define BAUDRATE 115200

const uint16_t musicNotes[] PROGMEM = { 523, 587, 659, 698, 784, 880, 988, 1046 };

// rx5808 module needs >20ms to tune.
#define MIN_TUNE_TIME 25


// number of analog rssi reads to average for the current check.
// single analog read with FASTADC defined (see below) takes ~20us on 16MHz arduino
// so e.g. 10 reads will take 200 ms, which gives resolution of 5 RSSI reads per ms,
// this means that we can theoretically have 1ms timing accuracy :)
#define RSSI_READS 5 // 5 should give about 10 000 readings per second

// input control byte constants
#define CONTROL_START_RACE 1
#define CONTROL_END_RACE 2
#define CONTROL_DEC_MIN_LAP 3
#define CONTROL_INC_MIN_LAP 4
#define CONTROL_DEC_CHANNEL 5
#define CONTROL_INC_CHANNEL 6
#define CONTROL_DEC_THRESHOLD 7
#define CONTROL_INC_THRESHOLD 8
#define CONTROL_SET_THRESHOLD 9
#define CONTROL_DATA_REQUEST 255

//----- RSSI --------------------------------------
#define FILTER_ITERATIONS 1
uint16_t rssiArr[FILTER_ITERATIONS + 1];
uint16_t rssiThreshold = 0;
uint16_t rssi;
int valrssi=0;

#define RSSI_MAX 1024
#define RSSI_MIN 0
#define MAGIC_THRESHOLD_REDUCE_CONSTANT 2
#define THRESHOLD_ARRAY_SIZE  100
uint16_t rssiThresholdArray[THRESHOLD_ARRAY_SIZE];

//----- Lap timings--------------------------------
uint32_t lastMilliseconds = 0;
#define MIN_MIN_LAP_TIME 1 //seconds
#define MAX_MIN_LAP_TIME 60 //seconds
#define MIN_LAP_TIME 12000//millis
uint8_t minLapTime = 5; //seconds
#define MAX_LAPS 50
uint32_t lapTimes[MAX_LAPS];

int lap;
unsigned long firstMillis;
unsigned long startMillis;
unsigned long currentMillis;
unsigned long elapsedMillis;
float laptime,totaltime;

//----- other globals------------------------------
uint8_t allowEdgeGeneration = 1;
uint8_t channelIndex = 2;
uint8_t isRaceStarted = 1;
uint8_t newLapIndex = 0;
uint8_t sendData = 0;
uint8_t sendStage = 0;
uint8_t sendLapTimesIndex = 0;

// ----------------------------------------------------------------------------
void setup() {
    //set up rssiThreshold
    rssiThreshold=380;
    // initialize digital pin 13 LED as an output.
    pinMode(led, OUTPUT);
    digitalHigh(led);

    // init buzzer pin
    pinMode(buzzerPin, OUTPUT);

    //init raspberrypi interrupt generator pin
    pinMode(pinRaspiInt, OUTPUT);
    digitalLow(pinRaspiInt);

    // SPI pins for RX control
    setupSPIpins();

    // set the channel as soon as we can
    // faster boot up times :)
    channelIndex = 2;
    setChannelModule(channelIndex);
    wait_rssi_ready();
    Serial.begin(BAUDRATE);

    allowEdgeGeneration = 1;

    initFastADC();

    // Setup Done - Turn Status LED off.
    digitalLow(led);

    DEBUG_CODE(
        pinMode(serialTimerPin, OUTPUT);
        pinMode(loopTimerPin, OUTPUT);
    );
}
// ----------------------------------------------------------------------------
void loop() {
    DEBUG_CODE(
        digitalToggle(loopTimerPin);
    );
       long lasttime;
   // delay(100);
    rssi = getFilteredRSSI();
   // rssi =  analogRead(rssiPinA);
     // Serial.println(rssi); 
    


if (rssi>rssiThreshold){//if drone pass the rssiThreshold
   //get current time
   //  Serial.println(rssi);     
    currentMillis = millis();
    elapsedMillis = (currentMillis - startMillis);// calcuate lap time in million seconds
    laptime=elapsedMillis ; // calcuate 
    laptime=laptime/1000 ; // convert to seconds
       if (lap<=0) {  // ignore first lap stat to record time
          firstMillis=millis();
          Serial.print("<=================Start=================>");
          Serial.println();;
       }
    totaltime=(currentMillis-firstMillis);
    totaltime=totaltime/1000;
    //laptime=microsecondsToseconds(mtime);
   if(lap>0){
    Serial.print("lap:");
    Serial.print(lap);
    Serial.print(" Laptime: ");
    Serial.print(laptime,2);
    Serial.print("s, total:");
    Serial.print(totaltime);
    Serial.print("s ");
    Serial.println();
   }
   startMillis=millis(); //save this lap
   lasttime=laptime;
   lap=lap+1;
  tone(buzzerPin, 2000); // Send 2KHz sound signal...
  delay(90);        // ...for 0.1 sec
  noTone(buzzerPin);     // Stop sound...
    delay(20);   
   tone(buzzerPin, 1000); // Send 1KHz sound signal...
  delay(90);        // ...for 0.1 sec
  noTone(buzzerPin);     // Stop sound...
   delay(MIN_LAP_TIME);
    }
    
   
}
// ----------------------------------------------------------------------------
void decMinLap() {
    if (minLapTime > MIN_MIN_LAP_TIME) {
        minLapTime--;
    }
}
// ----------------------------------------------------------------------------
void incMinLap() {
    if (minLapTime < MAX_MIN_LAP_TIME) {
        minLapTime++;
    }
}
// ----------------------------------------------------------------------------
void incThreshold() {
    if (rssiThreshold < RSSI_MAX) {
        rssiThreshold++;
    }
}
// ----------------------------------------------------------------------------
void decThreshold() {
    if (rssiThreshold > RSSI_MIN) {
        rssiThreshold--;
    }
}
// ----------------------------------------------------------------------------
void setThreshold() {
    if (rssiThreshold == 0) {
        uint16_t median;
        for(uint8_t i=0; i < THRESHOLD_ARRAY_SIZE; i++) {
            rssiThresholdArray[i] = getFilteredRSSI();
        }
        sortArray(rssiThresholdArray, THRESHOLD_ARRAY_SIZE);
        median = getMedian(rssiThresholdArray, THRESHOLD_ARRAY_SIZE);
        if (median > MAGIC_THRESHOLD_REDUCE_CONSTANT) {
            rssiThreshold = median - MAGIC_THRESHOLD_REDUCE_CONSTANT;
            playSetThresholdTones();
        }
    }
    else {
        rssiThreshold = 0;
        playClearThresholdTones();
    }
}
// ----------------------------------------------------------------------------
uint16_t getFilteredRSSI() {
    rssiArr[0] = readRSSI();

    // several-pass filter (need several passes because of integer artithmetics)
    // it reduces possible max value by 1 with each iteration.
    // e.g. if max rssi is 300, then after 5 filter stages it won't be greater than 295
    for(uint8_t i=1; i<=FILTER_ITERATIONS; i++) {
        rssiArr[i] = (rssiArr[i-1] + rssiArr[i]) >> 1;
    }

    return rssiArr[FILTER_ITERATIONS];
}
// ----------------------------------------------------------------------------
void sortArray(uint16_t a[], uint16_t size) {
    for(uint16_t i=0; i<(size-1); i++) {
        for(uint16_t j=0; j<(size-(i+1)); j++) {
                if(a[j] > a[j+1]) {
                    uint16_t t = a[j];
                    a[j] = a[j+1];
                    a[j+1] = t;
                }
        }
    }
}
// ----------------------------------------------------------------------------
uint16_t getMedian(uint16_t a[], uint16_t size) {
    return a[size/2];
}
// ----------------------------------------------------------------------------
void gen_rising_edge(int pin) {
    digitalHigh(pin); //this will open mosfet and pull the RasPi pin to GND
    delayMicroseconds(10);
    digitalLow(pin); // this will close mosfet and pull the RasPi pin to 3v3 -> Rising Edge
}
// ----------------------------------------------------------------------------
void wait_rssi_ready() {
    delay(MIN_TUNE_TIME);
}
// ----------------------------------------------------------------------------
uint16_t readRSSI() {
    int rssi = 0;
    int rssiA = 0;

    for (uint8_t i = 0; i < RSSI_READS; i++) {
        rssiA += analogRead(rssiPinA);
    }

    rssiA = rssiA/RSSI_READS; // average of RSSI_READS readings
    return rssiA;
}
