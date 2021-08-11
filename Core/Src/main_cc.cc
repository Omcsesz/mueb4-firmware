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

  while (true) {
    network.Step();
    Panel::StepAll();
  }
}
