/**
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#include "panel.h"

#include <algorithm>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/**
 * Used for window#step_anim
 * @see ::TIM17_IRQHandler
 */
std::uint8_t time_to_next_frame{0u};
bool Panel::internal_animation_enabled_{false};
bool Panel::swapped_{false};

/**
 * Calls window#time_handler
 * @see ::TIM17_IRQHandler
 */
extern "C" void PanelTimeHandler() {
  Panel::LeftPanel().TimeHandler();
  Panel::RightPanel().TimeHandler();
}

/**
 * Used for GPIO USER_INPUT_BUTTON
 * @see ::EXTI2_3_IRQHandler
 */
extern "C" void PanelInternalAnimationToggle() {
  Panel::SetInternalAnimation(!Panel::internal_animation_enabled());
}

void Panel::SetInternalAnimation(bool value) {
  if (Panel::internal_animation_enabled_) {
    Panel::LeftPanel().Blank();
    Panel::RightPanel().Blank();
  }

  internal_animation_enabled_ = value;
}

bool Panel::internal_animation_enabled() { return internal_animation_enabled_; }

Panel::Panel(GPIO_TypeDef* const gpio_port_3v3,
             const std::uint16_t gpio_pin_3v3,
             GPIO_TypeDef* const gpio_port_power,
             const std::uint16_t gpio_pin_power,
             GPIO_TypeDef* const gpio_port_tx, const std::uint16_t gpio_pin_tx,
             UART_HandleTypeDef* const huartx)
    : gpio_port_3v3_(gpio_port_3v3),
      gpio_port_tx_(gpio_port_tx),
      gpio_port_power_(gpio_port_power),
      gpio_pin_3v3_(gpio_pin_3v3),
      gpio_pin_tx_(gpio_pin_tx),
      gpio_pin_power_(gpio_pin_power),
      huartx_(huartx) {
  SetStatus(kDischargeCaps);
}

void Panel::SwapPanels() { swapped_ = !swapped_; }

Panel& Panel::LeftPanel() {
  static Panel instance(WINDOW_3V3_LEFT_GPIO_Port, WINDOW_3V3_LEFT_Pin,
                        WINDOW_POWER_LEFT_GPIO_Port, WINDOW_POWER_LEFT_Pin,
                        WINDOW_TX_LEFT_GPIO_Port, WINDOW_TX_LEFT_Pin, &huart2);

  return instance;
}

Panel& Panel::RightPanel() {
  static Panel instance(WINDOW_3V3_RIGHT_GPIO_Port, WINDOW_3V3_RIGHT_Pin,
                        WINDOW_POWER_RIGHT_GPIO_Port, WINDOW_POWER_RIGHT_Pin,
                        WINDOW_TX_RIGHT_GPIO_Port, WINDOW_TX_RIGHT_Pin,
                        &huart1);

  return instance;
}

Panel& Panel::GetPanel(Side side) {
  const bool target{side ^ swapped_};

  if (target == LEFT) {
    return Panel::RightPanel();
  } else {
    return Panel::LeftPanel();
  }
}

void Panel::StepAnim() {
  if (!time_to_next_frame || !internal_animation_enabled_) {
    return;
  }
  time_to_next_frame = 0u;

  static std::uint8_t color{0u};
  static std::uint8_t phase{0u};

  auto pixels{kPixels};
  for (auto& pixel : pixels) {
    pixel.red = (phase == 0u) ? color : 0u;
    pixel.green = (phase == 1u) ? color : 0u;
    pixel.blue = (phase == 2u) ? color : 0u;
  }

  Panel::RightPanel().SendPixels(pixels);
  Panel::LeftPanel().SendPixels(pixels);

  if (++color == 8u) {
    color = 0u;

    if (++phase == 3u) {
      phase = 0u;
    }
  }
}

void Panel::Step() {
  switch (status_) {
    case kDischargeCaps:
      if (tick_1s_ > 10u) SetStatus(kVcc3v3On);
      break;
    case kVcc3v3Off:
      if (tick_1s_ > 5u) SetStatus(kVcc3v3On);
      break;
    case kVcc3v3On:
      if (tick_1s_ > 1u) {
        if (active_) {
          SetStatus(kVcc12vOn);
        } else {
          SetStatus(kVcc3v3Off);
        }
      }
      break;
    case kVcc12vOff:
      break;
    case kVcc12vOn:
      break;
  }
}

void Panel::SetStatus(enum Status status) {
  switch (status) {
    case kDischargeCaps:
      tick_1s_ = 0u;
      HAL_GPIO_WritePin(gpio_port_3v3_, gpio_pin_3v3_, GPIO_PIN_SET);
      HAL_GPIO_WritePin(gpio_port_power_, gpio_pin_power_, GPIO_PIN_RESET);
      break;
    case kVcc3v3Off:
      tick_1s_ = 0u;
      HAL_GPIO_WritePin(gpio_port_3v3_, gpio_pin_3v3_, GPIO_PIN_SET);
      break;
    case kVcc3v3On:
      tick_1s_ = 0u;
      HAL_GPIO_WritePin(gpio_port_3v3_, gpio_pin_3v3_, GPIO_PIN_RESET);
      break;
    case kVcc12vOff:
      HAL_GPIO_WritePin(gpio_port_power_, gpio_pin_power_, GPIO_PIN_RESET);
      break;
    case kVcc12vOn:
      HAL_GPIO_WritePin(gpio_port_power_, gpio_pin_power_, GPIO_PIN_SET);
      Blank();
      break;
    default:
      return;
  }

  status_ = status;
}

void Panel::Blank() { SendPixels(kPixels); }

void Panel::TimeHandler() { tick_1s_++; }

void Panel::SendPixels(const std::array<Pixel, kPixelCount>& pixels) {
  if (status_ < kVcc3v3On) {
    return;
  }

  // Besides the first F0 byte
  std::size_t transfer_size{1u};
  for (const auto& pixel : pixels) {
    std::uint8_t base = pixel.position * 3u;

    dma_tx_buffer_[transfer_size++] = (base + 0u) << 4u | (pixel.red & 0x07u);
    dma_tx_buffer_[transfer_size++] = (base + 1u) << 4u | (pixel.green & 0x07u);
    dma_tx_buffer_[transfer_size++] = (base + 2u) << 4u | (pixel.blue & 0x07u);
  }

  HAL_UART_Transmit_DMA(huartx_, dma_tx_buffer_.data(), transfer_size);
}

void Panel::SetWhitebalance(
    const std::array<std::uint8_t, kWhiteBalanceDataSize>& white_balance) {
  if (status_ < kVcc3v3On) {
    return;
  }

  std::copy_n(white_balance.begin(), white_balance.size(),
              white_balance_with_header_.begin() + 1);
  HAL_UART_Transmit_DMA(huartx_, white_balance_with_header_.data(),
                        white_balance_with_header_.size());
}
