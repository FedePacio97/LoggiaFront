#include <Arduino.h>

#define REQUEST 0 //For type of mex in sendMex
#define RESPONSE 1
const unsigned int GATE_PIR_PIN = D1;
const unsigned int NEIGHBORHOOD_PIR_PIN = D2; 
const unsigned int SWIMMINGPOOL_PIR_PIN = D6;
const unsigned int DOOR_PIR_PIN = D5;

const unsigned int GATE_SWITCH_NODE_DETECTED_LED = D3;
const unsigned int SWIMMINGPOOL_SWITCH_NODE_DETECTED_LED = D4;

uint8_t MAX_CONNECTION_PER_NODE = 10;

#define GATE_PIR 0
#define DOOR_PIR 1
#define SWIMMINGPOOL_PIR 2
#define NEIGHBORHOOD_PIR 3

const unsigned int DETECTION_THRESHOLD = 1023;
const unsigned int MESSAGE_PER_PIR_DETECTED = 3;

/**************************************/
//For DHT sensor
#include "DHT.h"
#define DHTPIN D8     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);
/**************************************/
