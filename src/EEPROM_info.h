#include<Arduino.h>
#include<EEPROM.h>
// define the number of bytes you want to access
#define EEPROM_SIZE 28

//Saved in EEPROM
#define BASE 0
const unsigned int ROOT_NODE_FLASH_ADDRESS = (BASE)%512;
const unsigned int MQTTBRIDGE_NODE_FLASH_ADDRESS = (ROOT_NODE_FLASH_ADDRESS + sizeof(uint32_t))%512;
const unsigned int GATE_NODE_FLASH_ADDRESS = (MQTTBRIDGE_NODE_FLASH_ADDRESS + sizeof(uint32_t))%512;
const unsigned int SWIMMINGPOOL_NODE_FLASH_ADDRESS = (GATE_NODE_FLASH_ADDRESS + sizeof(uint32_t))%512;
const unsigned int NEIGHBORHOOD_NODE_FLASH_ADDRESS = (SWIMMINGPOOL_NODE_FLASH_ADDRESS + sizeof(uint32_t))%512;

const unsigned int HUMIDITY_TEMPERATURE_SAMPLING_PERIOD_FLASH_ADDRESS = (NEIGHBORHOOD_NODE_FLASH_ADDRESS + sizeof(uint32_t))%512;

const int next_FLASH_ADDRESS = (HUMIDITY_TEMPERATURE_SAMPLING_PERIOD_FLASH_ADDRESS + sizeof(uint32_t))%512;