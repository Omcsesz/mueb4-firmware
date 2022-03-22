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

Panel& Panel::GetPanel(Side side) {
  const Side target{static_cast<Side>(static_cast<bool>(side) ^ swapped_)};

  if (target == Side::RIGHT) {
    return Panel::left_panel();
  } else {
    return Panel::right_panel();
  }
}

Panel& Panel::GetPanel(UART_HandleTypeDef* huartx) {
  if (huartx == &huart1) {
    return Panel::right_panel();
  } else if (huartx == &huart2) {
    return Panel::left_panel();
  }

  return Panel::left_panel();
}

void Panel::TimeHandler() {
  Panel::left_panel().tick_1s_++;
  Panel::right_panel().tick_1s_++;
  Panel::StepInternalAnimation();
}

void Panel::ToggleInternalAnimation() {
  Panel::SetInternalAnimation(!Panel::internal_animation_enabled_);
}

void Panel::SetInternalAnimation(bool on) {
  if (!on && internal_animation_enabled_) {
    Panel::BlankAll();
  }

  internal_animation_enabled_ = on;
}

bool Panel::internal_animation_enabled() { return internal_animation_enabled_; }

void Panel::Swap() { swapped_ = !swapped_; }

void Panel::BlankAll() {
  Panel::left_panel().Blank();
  Panel::right_panel().Blank();
}

void Panel::StepAll() {
  Panel::left_panel().Step();
  Panel::right_panel().Step();
}

void Panel::DisableAll() {
  Panel::left_panel().Disable();
  Panel::right_panel().Disable();
}

void Panel::SendColorDataAll() {
  Panel::left_panel().SendColorData();
  Panel::right_panel().SendColorData();
}

void Panel::SendWhiteBalanceToAll(
    const Panel::WhiteBalanceData& white_balance) {
  Panel::left_panel().SendWhiteBalance(white_balance);
  Panel::right_panel().SendWhiteBalance(white_balance);
}

void Panel::SendWhiteBalance(const WhiteBalanceData& white_balance) {
  if (state_ < State::kVcc3v3On) {
    return;
  }

  std::copy_n(white_balance.begin(), white_balance.size(),
              white_balance_data_.begin() + 1u);
  HAL_UART_Transmit_DMA(huartx_, white_balance_data_.data(),
                        white_balance_data_.size());
}

Panel Panel::SetColorData(const ColorData& colorData) {
  for (auto i{0u}; i < colorData.size(); i++) {
    color_data_[i + 1u] =
        static_cast<std::uint8_t>(i << 4u | (colorData[i] & 0x0Fu));
  }
}

void Panel::Heartbeat() {
  if (state_ >= State::kVcc3v3On) {
    if ((heartbeat_ & 0xF0u) == 0x80u) {
      HAL_UART_Receive_IT(huartx_, &heartbeat_, 1u);
      active_ = true;
    } else {  // invalid state
      active_ = false;
    }
  }
}

Panel::State Panel::state() { return state_; }

std::array<std::uint8_t, 2> Panel::GetPanelStates() {
  return {static_cast<std::uint8_t>(left_panel().state_),
          static_cast<std::uint8_t>(right_panel().state_)};
}

Panel::Panel(Side side)
    :  // NOLINTNEXTLINE
      gpio_3v3_port_(side == Side::LEFT ? PANEL_3V3_LEFT_GPIO_Port
                                        : PANEL_3V3_RIGHT_GPIO_Port),
      gpio_12v_port_(side == Side::LEFT ? PANEL_12v_LEFT_GPIO_Port
                                        : PANEL_12v_RIGHT_GPIO_Port),
      huartx_(side == Side::LEFT ? &huart2 : &huart1),
      side_(side),
      gpio_3v3_pin_(side == Side::LEFT ? PANEL_3V3_LEFT_Pin
                                       : PANEL_3V3_RIGHT_Pin),
      gpio_12v_pin_(side == Side::LEFT ? PANEL_12v_LEFT_Pin
                                       : PANEL_12v_RIGHT_Pin) {}

Panel& Panel::left_panel() {
  static Panel instance(Side::LEFT);

  return instance;
}

Panel& Panel::right_panel() {
  static Panel instance(Side::RIGHT);

  return instance;
}

void Panel::StepInternalAnimation() {
  if (!internal_animation_enabled_) {
    return;
  }

  static std::uint8_t color{0u};
  static std::uint8_t phase{0u};

  Panel::ColorData colors{};
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

  Panel::left_panel().SetColorData(colors).SendColorData();
  Panel::right_panel().SetColorData(colors).SendColorData();

  color++;
  if (color == 16u) {
    color = 0u;

    phase++;
    if (phase == 3u) {
      phase = 0u;
    }
  }
}

void Panel::Step() {
  switch (state_) {
    case State::kPowerOff: {
      // -- No debug --
      HAL_ADC_Start(&hadc);
      // PA0/IN0/LEFT
      HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
      adc_[0] = HAL_ADC_GetValue(&hadc);

      // PA1/IN1/RIGHT
      HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
      adc_[1] = HAL_ADC_GetValue(&hadc);
      HAL_ADC_Stop(&hadc);
      // -- No debug --

      // Check if the capacitors are discharged
      if ((side_ == Side::LEFT && adc_[0] < 100u) ||
          (side_ == Side::RIGHT && adc_[1] < 100u)) {
        // Turn 3v3 on
        HAL_GPIO_WritePin(gpio_3v3_port_, gpio_3v3_pin_, GPIO_PIN_RESET);
        state_ = State::kVcc3v3On;

        // Turn UART on
        if (side_ == Side::LEFT) {
          MX_USART2_UART_Init();
        } else {
          MX_USART1_UART_Init();
        }

        // Start heartbeat detection
        HAL_UART_Receive_IT(huartx_, &heartbeat_, 1u);
        tick_1s_ = 0u;
      }
      break;
    }
    case State::kVcc3v3On: {
      // Check PWM panel heartbeat after 1s
      if (tick_1s_ > 1u) {
        // Check if panel is active
        if (active_) {
          HAL_GPIO_WritePin(gpio_12v_port_, gpio_12v_pin_, GPIO_PIN_SET);
          state_ = State::kVcc12vOn;
          tick_1s_ = 0u;
        } else {
          // Panel is inactive or disconnected
          // Turn 3v3 off
          HAL_GPIO_WritePin(gpio_3v3_port_, gpio_3v3_pin_, GPIO_PIN_SET);

          // Turn UART off
          HAL_UART_DeInit(huartx_);

          state_ = State::kPowerOff;
          tick_1s_ = 0u;
        }

        active_ = false;
      }
      break;
    }
    case State::kVcc12vOn: {
      // Check if panel is active after 2s, 1s heartbeat + 1s PWM processing
      if (tick_1s_ > 2u) {
        if (active_) {
          tick_1s_ = 0u;
        } else {
          // Panel is inactive or disconnected
          // Power off order: 12v, 3v3
          HAL_GPIO_WritePin(gpio_12v_port_, gpio_12v_pin_, GPIO_PIN_RESET);

          // Turn 3v3 off
          HAL_GPIO_WritePin(gpio_3v3_port_, gpio_3v3_pin_, GPIO_PIN_SET);

          // Turn UART off
          HAL_UART_DeInit(huartx_);

          state_ = State::kPowerOff;
          tick_1s_ = 0u;
        }

        active_ = false;
      }
      break;
    }
    default:
      break;
  }
}

void Panel::Blank() { SetColorData(Panel::ColorData{}).SendColorData(); }

void Panel::Disable() {
  HAL_GPIO_WritePin(gpio_12v_port_, gpio_12v_pin_, GPIO_PIN_RESET);
  // Turn 3v3 off
  HAL_GPIO_WritePin(gpio_3v3_port_, gpio_3v3_pin_, GPIO_PIN_SET);

  // Turn UART off
  HAL_UART_DeInit(huartx_);

  state_ = State::kDisabled;
  tick_1s_ = 0u;
  active_ = false;
}

void Panel::SendColorData() {
  if (state_ < State::kVcc12vOn) {
    return;
  }

  HAL_UART_Transmit_DMA(huartx_, color_data_.data(), color_data_.size());
}
