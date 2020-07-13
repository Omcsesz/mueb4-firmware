/*
 * firm_update.hpp
 *
 *  Created on: Dec 25, 2018
 *      Author: kisada
 */

#pragma once

#include <stdint.h>

#include "stddef.h"

namespace firmware_update {
uint64_t checksum_of_new_fw();
void refurbish [[noreturn]] ();
}  // namespace firmware_update
