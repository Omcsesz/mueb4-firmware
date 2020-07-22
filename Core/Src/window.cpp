#include "window.hpp"

#include "gpios.h"

/*****************************
 *  Instance of static members
 *****************************/

std::uint8_t time_to_next_frame = 0;
bool window::internal_animation_on{true};
bool window::windows_swapped{false};

extern "C" void window_time_handler() {
  windows::left_window->time_handler();
  windows::right_window->time_handler();
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
  switch (this->status) {
    case vcc_12v_on:
      if (whitebalance_flag) update_whitebalance();
      this->update_image();
      break;
    case vcc_12v_off:
      break;
    case vcc_3v3_off:
      if (tick_1s > 4) this->set_state(vcc_3v3_on);
      break;
    case vcc_3v3_on:
      if (tick_1s > 1) {  // TODO reference time point
        if (check_uart_welcome_message())
          this->set_state(vcc_12v_on);
        else
          this->set_state(vcc_3v3_off);
      }
      break;
    case discharge_caps:
      if (tick_1s > 10) this->set_state(vcc_3v3_off);
      break;
    default:
      this->set_state(discharge_caps);
  }
}

window::window(GPIO_TypeDef* gpio_port_3v3, std::uint16_t gpio_pin_3v3,
               GPIO_TypeDef* gpio_port_power, std::uint16_t gpio_pin_power,
               GPIO_TypeDef* gpio_port_tx, std::uint16_t gpio_pin_tx,
               USART_TypeDef* USARTx, DMA_TypeDef* DMAx,
               std::uint32_t DMA_Channel)
    : status(vcc_3v3_off),
      gpio_port_3v3(gpio_port_3v3),
      gpio_port_tx(gpio_port_tx),
      gpio_port_power(gpio_port_power),
      gpio_pin_3v3(gpio_pin_3v3),
      gpio_pin_tx(gpio_pin_tx),
      gpio_pin_power(gpio_pin_power),
      DMAx(DMAx),
      DMA_Channel(DMA_Channel),
      usart(USARTx),
      transmitted_before(false),
      whitebalance_flag(false) {
  LL_DMA_SetPeriphAddress(DMAx, DMA_Channel, (uint32_t)&usart->TDR);
  LL_DMA_SetMemoryAddress(DMAx, DMA_Channel, (std::uint32_t)DMA_buffer);
  LL_DMA_SetDataLength(DMAx, DMA_Channel, 13);
  LL_USART_EnableDMAReq_TX(usart);

  this->set_state(discharge_caps);
};

window::twindow_status window::get_state() { return this->status; }

void window::set_state(enum twindow_status new_stat) {
  switch (new_stat) {
    default:
      new_stat = discharge_caps;
    case discharge_caps:
      tick_1s = 0;
      LL_GPIO_SetOutputPin(gpio_port_3v3, gpio_pin_3v3);
      break;
    case vcc_3v3_off:
      tick_1s = 0;
      LL_GPIO_SetOutputPin(gpio_port_3v3, gpio_pin_3v3);
      LL_GPIO_ResetOutputPin(gpio_port_power, gpio_pin_power);
      break;
    case vcc_3v3_on:
      tick_1s = 0;
      // EMPTY DMA BUFFER
      LL_GPIO_ResetOutputPin(gpio_port_3v3, gpio_pin_3v3);
      LL_GPIO_ResetOutputPin(gpio_port_power, gpio_pin_power);
      break;
    case vcc_12v_on:
      LL_GPIO_SetOutputPin(gpio_port_power, gpio_pin_power);
      break;
    case vcc_12v_off:
      LL_GPIO_ResetOutputPin(gpio_port_power, gpio_pin_power);
      break;
  }
  this->status = new_stat;
}

bool window::check_uart_welcome_message() { return true; }

void window::update_image() {
  if (!LL_USART_IsActiveFlag_TC(usart)) return;

  std::size_t transfer_size{0};  // besides the first F0 byte
  std::size_t i{0};

  for (auto&& pixel : pixels) {
    if (pixel.isFull()) {
      pixel.flush();
      std::uint8_t base = i++ * 3;

      DMA_buffer[++transfer_size] =
          (std::uint8_t)(base + 0) << 4 | (pixel.red() & 0xE0) >> 5;
      DMA_buffer[++transfer_size] =
          (std::uint8_t)(base + 1) << 4 | (pixel.green() & 0xE0) >> 5;
      DMA_buffer[++transfer_size] =
          (std::uint8_t)(base + 2) << 4 | (pixel.blue() & 0xE0) >> 5;
    }
  }

  if (transfer_size) {
    LL_DMA_DisableChannel(DMAx, DMA_Channel);
    LL_DMA_SetDataLength(DMAx, DMA_Channel, transfer_size + 1);
    LL_USART_ClearFlag_TC(usart);
    LL_DMA_EnableChannel(DMAx, DMA_Channel);
  }
}

void window::blank() {
  for (auto&& pixel : pixels) {
    pixel.set(0, 0, 0);
  }
}

void window::set_whitebalance_flag(bool value) {
  this->whitebalance_flag = value;
}

bool window::get_whitebalance_flag() { return this->whitebalance_flag; }

void window::time_handler() { tick_1s++; }

void window::swap_windows() { windows_swapped = not windows_swapped; }

window& window::get_window(window_from_outside w) {
  const bool target = w xor windows_swapped;

  if (target == LEFT)
    return *windows::right_window;
  else
    return *windows::left_window;
}

void window::turn_internal_anim_on() { internal_animation_on = true; }

void window::turn_internal_anim_off() { internal_animation_on = false; }

bool window::is_internal_animation_on() { return internal_animation_on; }

void window::step_anim() {
  static std::uint32_t i = 0;
  static char szin = 0;

  if (!time_to_next_frame) return;

  time_to_next_frame = 0;

  for (std::size_t k = 0; k < window::num_of_pixels; k++) {
    std::uint8_t j = i << 5;
    windows::right_window->pixels[k].set(szin == 0 ? j : 0, szin == 1 ? j : 0,
                                         szin == 2 ? j : 0);
    windows::left_window->pixels[k].set(szin == 0 ? j : 0, szin == 1 ? j : 0,
                                        szin == 2 ? j : 0);
  }

  i++;
  if (i == 0x8) i = 0;
  if (i == 0) szin++;
  if (szin == 3) szin = 0;
}

void window::update_whitebalance() {
  LL_DMA_DisableChannel(DMAx, DMA_Channel);

  LL_DMA_SetMemoryAddress(DMAx, DMA_Channel, (std::uint32_t)whitebalance_data);
  LL_DMA_SetDataLength(DMAx, DMA_Channel, 22);

  LL_USART_ClearFlag_TC(usart);
  LL_DMA_EnableChannel(DMAx, DMA_Channel);

  while (!LL_USART_IsActiveFlag_TC(usart))
    ;

  LL_DMA_DisableChannel(DMAx, DMA_Channel);
  LL_DMA_SetMemoryAddress(DMAx, DMA_Channel, (std::uint32_t)DMA_buffer);

  whitebalance_flag = false;
}
