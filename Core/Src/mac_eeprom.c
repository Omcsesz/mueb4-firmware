/// @file

#include "mac_eeprom.h"

#include "main.h"

extern I2C_HandleTypeDef hi2c2;

/// EEPROM I2C device address
const uint8_t eeprom_address = 0b10100001;

/// EUI-48 value's start address
const uint16_t eui48_start_addr = 0xFA;

void getMAC(uint8_t* dest) {
  if (HAL_I2C_Mem_Read(&hi2c2, eeprom_address, eui48_start_addr,
                       I2C_MEMADD_SIZE_8BIT, dest, 6, HAL_MAX_DELAY) !=
      HAL_OK) {  // TODO set max_dalay from the eeprom datasheet
    // Error handling
  }
}
