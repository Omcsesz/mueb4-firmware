/**
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#include "panel.h"

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
  if (Panel::internal_animation_enabled_) {
    Panel::LeftPanel().Blank();
    Panel::RightPanel().Blank();
  }

  Panel::internal_animation_enabled_ = !Panel::internal_animation_enabled_;
}

Panel::Panel(GPIO_TypeDef* gpio_port_3v3, std::uint16_t gpio_pin_3v3,
             GPIO_TypeDef* gpio_port_power, std::uint16_t gpio_pin_power,
             GPIO_TypeDef* gpio_port_tx, std::uint16_t gpio_pin_tx,
             USART_TypeDef* USARTx, DMA_TypeDef* DMAx,
             std::uint32_t dma_tx_channel)
    : gpio_port_3v3_(gpio_port_3v3),
      gpio_port_tx_(gpio_port_tx),
      gpio_port_power_(gpio_port_power),
      DMAx_(DMAx),
      USARTx_(USARTx),
      dma_tx_channel_(dma_tx_channel),
      gpio_pin_3v3_(gpio_pin_3v3),
      gpio_pin_tx_(gpio_pin_tx),
      gpio_pin_power_(gpio_pin_power) {
  LL_DMA_SetPeriphAddress(DMAx, dma_tx_channel, (uint32_t)&USARTx->TDR);
  LL_DMA_SetMemoryAddress(DMAx, dma_tx_channel,
                          (std::uint32_t)dma_tx_buffer_.data());
  LL_DMA_SetDataLength(DMAx, dma_tx_channel, 13u);
  LL_USART_EnableDMAReq_TX(USARTx);

  SetStatus(kDischargeCaps);
}

void Panel::SwapPanels() { swapped_ = !swapped_; }

Panel& Panel::LeftPanel() {
  static Panel instance(WINDOW_3V3_LEFT_GPIO_Port, WINDOW_3V3_LEFT_Pin,
                        WINDOW_POWER_LEFT_GPIO_Port, WINDOW_POWER_LEFT_Pin,
                        WINDOW_TX_LEFT_GPIO_Port, WINDOW_TX_LEFT_Pin, USART2,
                        DMA1, LL_DMA_CHANNEL_4);

  return instance;
}

Panel& Panel::RightPanel() {
  static Panel instance(WINDOW_3V3_RIGHT_GPIO_Port, WINDOW_3V3_RIGHT_Pin,
                        WINDOW_POWER_RIGHT_GPIO_Port, WINDOW_POWER_RIGHT_Pin,
                        WINDOW_TX_RIGHT_GPIO_Port, WINDOW_TX_RIGHT_Pin, USART1,
                        DMA1, LL_DMA_CHANNEL_2);

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
  if (!time_to_next_frame) {
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
    default:
      SetStatus(kDischargeCaps);
  }
}

void Panel::SetStatus(enum Status status) {
  switch (status) {
    case kDischargeCaps:
      tick_1s_ = 0u;
      LL_GPIO_SetOutputPin(gpio_port_3v3_, gpio_pin_3v3_);
      LL_GPIO_ResetOutputPin(gpio_port_power_, gpio_pin_power_);
      break;
    case kVcc3v3Off:
      tick_1s_ = 0u;
      LL_GPIO_SetOutputPin(gpio_port_3v3_, gpio_pin_3v3_);
      break;
    case kVcc3v3On:
      tick_1s_ = 0u;
      LL_GPIO_ResetOutputPin(gpio_port_3v3_, gpio_pin_3v3_);
      break;
    case kVcc12vOff:
      LL_GPIO_ResetOutputPin(gpio_port_power_, gpio_pin_power_);
      break;
    case kVcc12vOn:
      LL_GPIO_SetOutputPin(gpio_port_power_, gpio_pin_power_);
      break;
    default:
      return;
  }

  status_ = status;
}

void Panel::Blank() { SendPixels(kPixels); }

void Panel::TimeHandler() { tick_1s_++; }

void Panel::SendPixels(const std::array<Pixel, kPixelCount>& pixels) {
  if (!LL_USART_IsActiveFlag_TC(USARTx_) || status_ < kVcc3v3On ||
      pixels.size() > kPixelCount) {
    return;
  }

  // besides the first F0 byte
  std::size_t transfer_size{0u};
  for (const auto& pixel : pixels) {
    std::uint8_t base = pixel.position * 3u;

    dma_tx_buffer_[++transfer_size] = (base + 0u) << 4u | (pixel.red & 0x07u);
    dma_tx_buffer_[++transfer_size] = (base + 1u) << 4u | (pixel.green & 0x07u);
    dma_tx_buffer_[++transfer_size] = (base + 2u) << 4u | (pixel.blue & 0x07u);
  }

  LL_DMA_DisableChannel(DMAx_, dma_tx_channel_);
  LL_DMA_SetDataLength(DMAx_, dma_tx_channel_, transfer_size + 1);
  LL_USART_ClearFlag_TC(USARTx_);
  LL_DMA_EnableChannel(DMAx_, dma_tx_channel_);
}

void Panel::SetWhitebalance(
    const std::array<std::uint8_t, kWhiteBalanceDataSize>& white_balance) {
  if (!LL_USART_IsActiveFlag_TC(USARTx_) || status_ < kVcc3v3On) {
    return;
  }

  LL_DMA_DisableChannel(DMAx_, dma_tx_channel_);
  LL_DMA_SetMemoryAddress(DMAx_, dma_tx_channel_,
                          (std::uint32_t)white_balance.data());
  LL_DMA_SetDataLength(DMAx_, dma_tx_channel_, 22u);
  LL_USART_ClearFlag_TC(USARTx_);
  LL_DMA_EnableChannel(DMAx_, dma_tx_channel_);

  while (!LL_USART_IsActiveFlag_TC(USARTx_)) {
  }

  LL_DMA_DisableChannel(DMAx_, dma_tx_channel_);
  LL_DMA_SetMemoryAddress(DMAx_, dma_tx_channel_,
                          (std::uint32_t)dma_tx_buffer_.data());
}
