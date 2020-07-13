/*
 * firm_update.hpp
 *
 *  Created on: Dec 25, 2018
 *      Author: kisada
 */

#pragma once

#include <cstdint>

namespace firmware_update {
std::uint64_t checksum_of_new_fw();
void refurbish [[noreturn]] ();
}  // namespace firmware_update
