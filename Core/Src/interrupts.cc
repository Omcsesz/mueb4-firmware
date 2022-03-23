#include <dhcp.h>

#include <cstdint>

#include "main.h"
#include "network.h"
#include "panel.h"
#include "tim.h"
#include "usart.h"

extern "C" void HAL_GPIO_EXTI_Callback(std::uint16_t GPIO_Pin) {
  // User button
  if (GPIO_Pin == GPIO_PIN_2) {
    Panel::ToggleInternalAnimation();
  }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // 1s timer
  if (htim == &htim6) {
    HAL_GPIO_TogglePin(LED_HEART_GPIO_Port, LED_HEART_Pin);
    HAL_GPIO_WritePin(LED_SERVER_GPIO_Port, LED_SERVER_Pin, GPIO_PIN_RESET);
    DHCP_time_handler();
    Panel::TimeHandler();
  } else if (htim == &htim16) {
    HAL_TIM_Base_Stop_IT(&htim16);
    Network::Instance().E131NetworkDataLossTimeout();
  }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  Panel::GetPanel(huart).Heartbeat();
}
