/**
 * This file contains the c++ main loop
 * @file
 */

#include "main.h"
#include "network.h"
#include "panel.h"

extern TIM_HandleTypeDef htim17;

/**
 * C++ main loop.
 * Can be called from C
 */
extern "C" void Main() {
  auto& network{Network::Instance()};
  auto& left_panel{Panel::LeftPanel()};
  auto& right_panel{Panel::RightPanel()};

  HAL_TIM_Base_Start_IT(&htim17);

  while (1) {
    network.Step();

    if (Panel::internal_animation_enabled_) Panel::StepAnim();

    left_panel.Step();
    right_panel.Step();
  }
}
