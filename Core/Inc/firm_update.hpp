/*
 * firm_update.hpp
 *
 *  Created on: Dec 25, 2018
 *      Author: kisada
 */

#ifndef FIRM_UPDATE_HPP_
#define FIRM_UPDATE_HPP_

#include <stdint.h>

#include "stddef.h"

namespace firmware_update {
uint64_t checksum_of_new_fw();
void refurbish [[noreturn]] ();
}  // namespace firmware_update

#endif /* FIRM_UPDATE_HPP_ */
