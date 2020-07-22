#include "main.h"
#include "network.hpp"
#include "window.hpp"

window* windows::left_window;
window* windows::right_window;

extern TIM_HandleTypeDef htim17;

extern "C" void main_cpp() {
  window left_window(WINDOW_3V3_LEFT_GPIO_Port, WINDOW_3V3_LEFT_Pin,
                     WINDOW_POWER_LEFT_GPIO_Port, WINDOW_POWER_LEFT_Pin,
                     WINDOW_TX_LEFT_GPIO_Port, WINDOW_TX_LEFT_Pin, USART2, DMA1,
                     LL_DMA_CHANNEL_4);
  window right_window(WINDOW_3V3_RIGHT_GPIO_Port, WINDOW_3V3_RIGHT_Pin,
                      WINDOW_POWER_RIGHT_GPIO_Port, WINDOW_POWER_RIGHT_Pin,
                      WINDOW_TX_RIGHT_GPIO_Port, WINDOW_TX_RIGHT_Pin, USART1,
                      DMA1, LL_DMA_CHANNEL_2);
  windows::left_window = &left_window;
  windows::right_window = &right_window;
  network inetwork;
  HAL_TIM_Base_Start_IT(&htim17);

  while (1) {
    inetwork.step_network();

    if (window::is_internal_animation_on()) window::step_anim();

    left_window.step_state();
    right_window.step_state();

    if (LL_GPIO_IsInputPinSet(USER_INPUT_BUTTON_GPIO_Port,
                              USER_INPUT_BUTTON_Pin)) {
      // TODO do sg
    }
  }
}
