#include "internal_anim.hpp"
#include "main.h"
#include "network.hpp"
#include "status.hpp"
#include "window.hpp"

windows::window* windows::left_window;
windows::window* windows::right_window;

extern "C" void main_cpp() {
  windows::window left_window(WINDOW_3V3_LEFT_GPIO_Port, WINDOW_3V3_LEFT_Pin,
                              WINDOW_POWER_LEFT_GPIO_Port,
                              WINDOW_POWER_LEFT_Pin, WINDOW_TX_LEFT_GPIO_Port,
                              WINDOW_TX_LEFT_Pin, USART2, DMA1,
                              LL_DMA_CHANNEL_4);
  windows::window right_window(WINDOW_3V3_RIGHT_GPIO_Port, WINDOW_3V3_RIGHT_Pin,
                               WINDOW_POWER_RIGHT_GPIO_Port,
                               WINDOW_POWER_RIGHT_Pin,
                               WINDOW_TX_RIGHT_GPIO_Port, WINDOW_TX_RIGHT_Pin,
                               USART1, DMA1, LL_DMA_CHANNEL_2);
  windows::left_window = &left_window;
  windows::right_window = &right_window;
  network inetwork;

  status::turn_internal_anim_on();

  while (1) {
    inetwork.step_network();

    if (status::if_internal_animation_is_on) internal_animation::step_anim();

    left_window.step_state();
    right_window.step_state();

    left_window.update_image();
    right_window.update_image();

    if (LL_GPIO_IsInputPinSet(USER_INPUT_BUTTON_GPIO_Port,
                              USER_INPUT_BUTTON_Pin)) {
      // TODO do sg
    }
  }
}
