#include "status.hpp"

namespace {
bool windows_swapped = false;
}

namespace status {
bool if_internal_animation_is_on = false;
std::uint8_t emelet_szam = 0;
std::uint8_t szoba_szam = 0;

void swap_windows() { windows_swapped = not windows_swapped; }

windows::window& getWindow(window_from_outside w) {
  const bool target = w xor windows_swapped;

  if (target == LEFT)
    return *windows::right_window;
  else
    return *windows::left_window;
}

}  // namespace status
