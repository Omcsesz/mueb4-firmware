/**
 * Contains DHCP buffer definition.
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/// @note 1 kb should be enough for DHCP RX buffer
#define DATA_BUF_SIZE 1024

/// DHCP RX buffer.
extern uint8_t gDATABUF[DATA_BUF_SIZE];

#ifdef __cplusplus
}
#endif
