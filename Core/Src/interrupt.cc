#include <dhcp.h>

#include <cstdint>

#include "main.h"
#include "panel.h"

extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

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
    Panel::StepInternalAnimation();
  }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    Panel::right_panel().SetActive(true);
    HAL_UART_Receive_IT(huart, &Panel::right_panel().heartbeat, 1u);
  } else if (huart == &huart2) {
    Panel::left_panel().SetActive(true);
    HAL_UART_Receive_IT(huart, &Panel::left_panel().heartbeat, 1u);
  }
}
