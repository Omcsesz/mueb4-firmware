/**
 * Contains MAC address getter function.
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 * @note The device uses Microchip 24AA02E48T-I/OT EEPROM
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * Gets MAC address from EEPROM.
 * @param[out] dest The memory area to save MAC address to.
 */
void getMAC(uint8_t* dest);

#ifdef __cplusplus
}
#endif
