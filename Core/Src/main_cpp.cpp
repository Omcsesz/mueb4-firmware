#include "main.h"
#include "network.hpp"
#include "window.hpp"

extern TIM_HandleTypeDef htim17;

extern "C" void main_cpp() {
  network inetwork;
  auto& left_window{window::get_left_window()};
  auto& right_window{window::get_right_window()};

  HAL_TIM_Base_Start_IT(&htim17);

  while (1) {
    inetwork.step_network();

    if (window::internal_animation_on) window::step_anim();

    left_window.step_state();
    right_window.step_state();

    if (LL_GPIO_IsInputPinSet(USER_INPUT_BUTTON_GPIO_Port,
                              USER_INPUT_BUTTON_Pin)) {
      // TODO do sg
    }
  }
}
