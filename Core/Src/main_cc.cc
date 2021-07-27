/**
 * This file contains the c++ main loop
 * @file
 */

#include "network.h"
#include "panel.h"

/**
 * C++ main loop.
 * Can be called from C.
 */
extern "C" [[noreturn]] void Main() {
  auto& network{Network::Instance()};
  auto& left_panel{Panel::left_panel()};
  auto& right_panel{Panel::right_panel()};

  while (true) {
    network.Step();
    left_panel.Step();
    right_panel.Step();
  }
}
