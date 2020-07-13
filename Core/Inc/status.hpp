#pragma once

#include <cstdint>

#include "window.hpp"

namespace status {
enum window_from_outside : bool { LEFT = false, RIGHT = true };

extern bool if_internal_animation_is_on;
extern std::uint8_t emelet_szam;
extern std::uint8_t szoba_szam;

inline void turn_internal_anim_on() { if_internal_animation_is_on = true; }
inline void turn_internal_anim_off() { if_internal_animation_is_on = false; }

void swap_windows();
windows::window& getWindow(window_from_outside);
}  // namespace status
