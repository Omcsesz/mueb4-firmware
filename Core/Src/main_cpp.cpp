/**
 * This file contains the c++ main loop
 * @file
 */

#include "main.h"
#include "network.hpp"
#include "window.hpp"

extern TIM_HandleTypeDef htim17;

/**
 * C++ main loop.
 * Can be called from C
 */
extern "C" void main_cpp() {
  auto& net{network::instance()};
  auto& left_window{window::get_left_window()};
  auto& right_window{window::get_right_window()};

  HAL_TIM_Base_Start_IT(&htim17);

  while (1) {
    net.step_network();

    if (window::internal_animation_on) window::step_anim();

    left_window.step_state();
    right_window.step_state();
  }
}
