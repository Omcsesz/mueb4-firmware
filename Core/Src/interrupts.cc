#include <dhcp.h>

#include <cstdint>

#include "main.h"
#include "panel.h"
#include "tim.h"
#include "usart.h"

extern "C" void HAL_GPIO_EXTI_Callback(std::uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_2) {
    Panel::ToggleInternalAnimation();
  }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    HAL_GPIO_TogglePin(LED_HEART_GPIO_Port, LED_HEART_Pin);
    DHCP_time_handler();
    Panel::TimeHandler();
  } else if (htim == &htim16) {
    HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, GPIO_PIN_RESET);
  }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    Panel::right_panel().Heartbeat();
  } else if (huart == &huart2) {
    Panel::left_panel().Heartbeat();
  }
}
