#include "window.hpp"

/*****************************
 *  Instance of static members
 *****************************/

std::uint8_t time_to_next_frame = 0;
bool window::internal_animation_on{true};
bool window::windows_swapped{false};

extern "C" void window_time_handler() {
  window::get_left_window().time_handler();
  window::get_right_window().time_handler();
}

/*****************************
 *    Class pixel_data
 *****************************/

void pixel_data::set(std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
  m_red = red;
  m_green = green;
  m_blue = blue;
  m_stat = pixel_data::buffer_full;
}

void pixel_data::flush() { m_stat = pixel_data::buffer_free; }

bool pixel_data::isFull() { return m_stat == pixel_data::buffer_full; }

std::uint8_t pixel_data::red() { return m_red; }

std::uint8_t pixel_data::green() { return m_green; }

std::uint8_t pixel_data::blue() { return m_blue; }

/*****************************
 *    Class window
 *****************************/

void window::step_state() {
  switch (status) {
    case discharge_caps:
      if (tick_1s > 10) set_state(vcc_3v3_on);
      break;
    case vcc_3v3_off:
      if (tick_1s > 5) set_state(vcc_3v3_on);
      break;
    case vcc_3v3_on:
      if (tick_1s > 1) {
        if (usart_active) {
          set_state(vcc_12v_on);
        } else
          set_state(vcc_3v3_off);
      }
      break;
    case vcc_12v_off:
      break;
    case vcc_12v_on:
      if (whitebalance_flag) update_whitebalance();
      update_image();
      break;
    default:
      set_state(discharge_caps);
  }
}

window::twindow_status window::get_state() const { return status; }

void window::set_state(enum twindow_status state) {
  switch (state) {
    default:
      state = discharge_caps;
    case discharge_caps:
      tick_1s = 0;
      LL_GPIO_SetOutputPin(gpio_port_3v3, gpio_pin_3v3);
      LL_GPIO_ResetOutputPin(gpio_port_power, gpio_pin_power);
      break;
    case vcc_3v3_off:
      tick_1s = 0;
      LL_GPIO_SetOutputPin(gpio_port_3v3, gpio_pin_3v3);
      break;
    case vcc_3v3_on:
      tick_1s = 0;
      LL_GPIO_ResetOutputPin(gpio_port_3v3, gpio_pin_3v3);
      break;
    case vcc_12v_off:
      LL_GPIO_ResetOutputPin(gpio_port_power, gpio_pin_power);
      break;
    case vcc_12v_on:
      LL_GPIO_SetOutputPin(gpio_port_power, gpio_pin_power);
      break;
  }
  status = state;
}

void window::blank() {
  for (auto&& pixel : pixels) {
    pixel.set(0, 0, 0);
  }
}

void window::time_handler() { tick_1s++; }

void window::set_usart_active(bool value) { usart_active = value; }

void window::swap_windows() { windows_swapped = not windows_swapped; }

window& window::get_left_window() {
  static window left(WINDOW_3V3_LEFT_GPIO_Port, WINDOW_3V3_LEFT_Pin,
                     WINDOW_POWER_LEFT_GPIO_Port, WINDOW_POWER_LEFT_Pin,
                     WINDOW_TX_LEFT_GPIO_Port, WINDOW_TX_LEFT_Pin, USART2, DMA1,
                     LL_DMA_CHANNEL_4);

  return left;
}

window& window::get_right_window() {
  static window right(WINDOW_3V3_RIGHT_GPIO_Port, WINDOW_3V3_RIGHT_Pin,
                      WINDOW_POWER_RIGHT_GPIO_Port, WINDOW_POWER_RIGHT_Pin,
                      WINDOW_TX_RIGHT_GPIO_Port, WINDOW_TX_RIGHT_Pin, USART1,
                      DMA1, LL_DMA_CHANNEL_2);

  return right;
}

window& window::get_window(window_from_outside w) {
  const bool target = w xor windows_swapped;

  if (target == LEFT)
    return window::get_right_window();
  else
    return window::get_left_window();
}

void window::step_anim() {
  static std::uint32_t i = 0;
  static char szin = 0;

  if (!time_to_next_frame) return;

  time_to_next_frame = 0;

  for (std::size_t k = 0; k < window::num_of_pixels; k++) {
    std::uint8_t j = i << 5;
    window::get_right_window().pixels[k].set(
        szin == 0 ? j : 0, szin == 1 ? j : 0, szin == 2 ? j : 0);
    window::get_left_window().pixels[k].set(
        szin == 0 ? j : 0, szin == 1 ? j : 0, szin == 2 ? j : 0);
  }

  i++;
  if (i == 0x8) i = 0;
  if (i == 0) szin++;
  if (szin == 3) szin = 0;
}

window::window(GPIO_TypeDef* gpio_port_3v3, std::uint16_t gpio_pin_3v3,
               GPIO_TypeDef* gpio_port_power, std::uint16_t gpio_pin_power,
               GPIO_TypeDef* gpio_port_tx, std::uint16_t gpio_pin_tx,
               USART_TypeDef* USARTx, DMA_TypeDef* DMAx,
               std::uint32_t dma_tx_channel)
    : gpio_port_3v3(gpio_port_3v3),
      gpio_port_tx(gpio_port_tx),
      gpio_port_power(gpio_port_power),
      DMAx(DMAx),
      USARTx(USARTx),
      dma_tx_channel(dma_tx_channel),
      gpio_pin_3v3(gpio_pin_3v3),
      gpio_pin_tx(gpio_pin_tx),
      gpio_pin_power(gpio_pin_power) {
  LL_DMA_SetPeriphAddress(DMAx, dma_tx_channel, (uint32_t)&USARTx->TDR);
  LL_DMA_SetMemoryAddress(DMAx, dma_tx_channel, (std::uint32_t)dma_tx_buffer);
  LL_DMA_SetDataLength(DMAx, dma_tx_channel, 13);
  LL_USART_EnableDMAReq_TX(USARTx);

  set_state(discharge_caps);
};

void window::update_whitebalance() {
  LL_DMA_DisableChannel(DMAx, dma_tx_channel);

  LL_DMA_SetMemoryAddress(DMAx, dma_tx_channel,
                          (std::uint32_t)whitebalance_data);
  LL_DMA_SetDataLength(DMAx, dma_tx_channel, 22);

  LL_USART_ClearFlag_TC(USARTx);
  LL_DMA_EnableChannel(DMAx, dma_tx_channel);

  while (!LL_USART_IsActiveFlag_TC(USARTx))
    ;

  LL_DMA_DisableChannel(DMAx, dma_tx_channel);
  LL_DMA_SetMemoryAddress(DMAx, dma_tx_channel, (std::uint32_t)dma_tx_buffer);

  whitebalance_flag = false;
}

void window::update_image() {
  if (!LL_USART_IsActiveFlag_TC(USARTx) || status < vcc_3v3_on) return;

  std::size_t transfer_size{0};  // besides the first F0 byte
  std::size_t i{0};

  for (auto&& pixel : pixels) {
    if (pixel.isFull()) {
      pixel.flush();
      std::uint8_t base = i++ * 3;

      dma_tx_buffer[++transfer_size] =
          (std::uint8_t)(base + 0) << 4 | (pixel.red() & 0xE0) >> 5;
      dma_tx_buffer[++transfer_size] =
          (std::uint8_t)(base + 1) << 4 | (pixel.green() & 0xE0) >> 5;
      dma_tx_buffer[++transfer_size] =
          (std::uint8_t)(base + 2) << 4 | (pixel.blue() & 0xE0) >> 5;
    }
  }

  if (transfer_size) {
    LL_DMA_DisableChannel(DMAx, dma_tx_channel);
    LL_DMA_SetDataLength(DMAx, dma_tx_channel, transfer_size + 1);
    LL_USART_ClearFlag_TC(USARTx);
    LL_DMA_EnableChannel(DMAx, dma_tx_channel);
  }
}
