/**
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#include "panel.h"

#include <algorithm>

#include "adc.h"
#include "usart.h"

/**
 * Used for window#step_anim
 * @see ::TIM17_IRQHandler
 */
bool Panel::internal_animation_enabled_{false};
bool Panel::swapped_{false};
std::array<std::uint32_t, 2u> Panel::adc_{};

void Panel::TimeHandler() {
  Panel::left_panel().tick_1s_++;
  Panel::right_panel().tick_1s_++;
  Panel::StepInternalAnimation();
}

void Panel::ToggleInternalAnimation() {
  Panel::SetInternalAnimation(!Panel::internal_animation_enabled_);
}

void Panel::SetInternalAnimation(bool value) {
  if (!value && internal_animation_enabled_) {
    Panel::BlankAll();
  }

  internal_animation_enabled_ = value;
}

bool Panel::internal_animation_enabled() { return internal_animation_enabled_; }

Panel::Panel(GPIO_TypeDef* const gpio_port_3v3,
             const std::uint16_t gpio_pin_3v3,
             GPIO_TypeDef* const gpio_port_power,
             const std::uint16_t gpio_pin_power,
             UART_HandleTypeDef* const huartx)
    : gpio_port_3v3_(gpio_port_3v3),
      gpio_port_power_(gpio_port_power),
      huartx_(huartx),
      gpio_pin_3v3_(gpio_pin_3v3),
      gpio_pin_power_(gpio_pin_power) {}

void Panel::SwapPanels() { swapped_ = !swapped_; }

Panel& Panel::left_panel() {
  static Panel instance(PANEL_3V3_LEFT_GPIO_Port, PANEL_3V3_LEFT_Pin,
                        PANEL_POWER_LEFT_GPIO_Port, PANEL_POWER_LEFT_Pin,
                        &huart2);

  return instance;
}

Panel& Panel::right_panel() {
  static Panel instance(PANEL_3V3_RIGHT_GPIO_Port, PANEL_3V3_RIGHT_Pin,
                        PANEL_POWER_RIGHT_GPIO_Port, PANEL_POWER_RIGHT_Pin,
                        &huart1);

  return instance;
}

Panel& Panel::GetPanel(Side side) {
  const Side target{static_cast<Side>(side ^ swapped_)};

  if (target == LEFT) {
    return Panel::right_panel();
  } else {
    return Panel::left_panel();
  }
}

void Panel::StepInternalAnimation() {
  if (!internal_animation_enabled_) {
    return;
  }

  static std::uint8_t color{0u};
  static std::uint8_t phase{0u};

  Panel::PanelColorData colors{};
  for (auto i{colors.begin()}; i != colors.end(); i++) {
    // red
    if (phase == 0u) {
      *i = color;
    }

    // green
    i++;
    if (phase == 1u) {
      *i = color;
    }

    // blue
    i++;
    if (phase == 2u) {
      *i = color;
    }
  }

  Panel::right_panel().SendPixels(colors);
  Panel::left_panel().SendPixels(colors);

  color++;
  if (color == 8u) {
    color = 0u;

    phase++;
    if (phase == 3u) {
      phase = 0u;
    }
  }
}

void Panel::BlankAll() {
  Panel::left_panel().Blank();
  Panel::right_panel().Blank();
}

void Panel::Step() {
  switch (status_) {
    case kPowerOff: {
      HAL_ADC_Start(&hadc);
      HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
      adc_[0] = HAL_ADC_GetValue(&hadc);
      HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
      adc_[1] = HAL_ADC_GetValue(&hadc);
      HAL_ADC_Stop(&hadc);

      if ((huartx_ == &huart2 && adc_[0] < 100u) ||
          (huartx_ == &huart1 && adc_[1] < 100u)) {
        SetStatus(kVcc3v3On);
      }
      break;
    }

    case kVcc3v3On: {
      if (tick_1s_ > 1u) {
        if (active_) {
          SetStatus(kVcc12vOn);
        } else {
          SetStatus(kDisabled);
        }
      }
      break;
    }
    default:
      break;
  }
}

void Panel::SetStatus(enum Status status) {
  switch (status) {
    case kDisabled:
    case kPowerOff: {
      active_ = false;

      HAL_GPIO_WritePin(gpio_port_3v3_, gpio_pin_3v3_, GPIO_PIN_SET);
      HAL_GPIO_WritePin(gpio_port_power_, gpio_pin_power_, GPIO_PIN_RESET);

      // Turn UART off
      if (huartx_ == &huart1) {
        HAL_UART_DeInit(&huart1);
      } else {
        HAL_UART_DeInit(&huart2);
      }

      tick_1s_ = 0u;
      break;
    }
    case kVcc3v3On: {
      HAL_GPIO_WritePin(gpio_port_3v3_, gpio_pin_3v3_, GPIO_PIN_RESET);

      // Turn UART on
      if (huartx_ == &huart1) {
        if (!mx_uart_initialized_) {
          MX_USART1_UART_Init();
          mx_uart_initialized_ = true;
        } else if (HAL_UART_Init(&huart1) != HAL_OK) {
          Error_Handler();
        }
      } else if (huartx_ == &huart2) {
        if (!mx_uart_initialized_) {
          MX_USART2_UART_Init();
          mx_uart_initialized_ = true;
        } else if (HAL_UART_Init(&huart2) != HAL_OK) {
          Error_Handler();
        }
      }

      tick_1s_ = 0u;
      HAL_UART_Receive_IT(huartx_, &heartbeat_, 1u);
      break;
    }
    case kVcc12vOn:
      HAL_GPIO_WritePin(gpio_port_power_, gpio_pin_power_, GPIO_PIN_SET);
      break;
    default:
      return;
  }

  status_ = status;
}

void Panel::Blank() { SendPixels(Panel::PanelColorData{}); }

void Panel::SendPixels(const PanelColorData& pixels) {
  if (status_ < kVcc12vOn) {
    return;
  }

  // NOLINTNEXTLINE
  for (std::uint8_t i{0u}; i < pixels.size(); i++) {
    dma_tx_buffer_[i + 1u] =
        static_cast<std::uint8_t>(i << 4u | (pixels[i] & 0x07u));
  }

  HAL_UART_Transmit_DMA(huartx_, dma_tx_buffer_.data(), dma_tx_buffer_.size());
}

void Panel::SendWhiteBalance(const WhiteBalanceData& white_balance) {
  if (status_ < kVcc3v3On) {
    return;
  }

  std::copy_n(white_balance.begin(), white_balance.size(),
              white_balance_.begin() + 1u);
  HAL_UART_Transmit_DMA(huartx_, white_balance_.data(), white_balance_.size());
}

void Panel::Heartbeat() {
  HAL_UART_Receive_IT(huartx_, &heartbeat_, 1u);
  if (status_ < kVcc3v3On || (heartbeat_ & 0xF0u) != 0x80u) {
    return;
  }

  active_ = true;
}
