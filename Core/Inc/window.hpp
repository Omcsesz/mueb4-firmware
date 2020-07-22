/*
 * window.hpp
 *
 *  Created on: May 4, 2018
 *      Author: kisada
 */

/******************************************************************************
 *
 *     Window management subsystem for SCH MATRIX4
 *
 *     Kiss Ádám 2018
 *
 *****************************************************************************/

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "main.h"

class pixel_data {
 public:
  /*!
   * \brief Type created for storing the state of the internal buffer
   * The buffer stores the next differential frame to be sent
   */
  enum tframe_status { buffer_free, buffer_full } m_stat{buffer_full};

  pixel_data() = default;
  pixel_data(const pixel_data&) = delete;
  pixel_data& operator=(const pixel_data&) = delete;

  /*
   * Sets a pixel for the given color.
   */
  void set(std::uint8_t red, std::uint8_t green, std::uint8_t blue);

  /*
   * Sets the stat to free
   */
  void flush();

  bool isFull();

  std::uint8_t red();
  std::uint8_t green();
  std::uint8_t blue();

 private:
  std::uint8_t m_red{0};
  std::uint8_t m_green{200};
  std::uint8_t m_blue{0};
};

class window {
 public:
  /*!
   * \brief Type created for storing states for each window.
   *
   * see documentation for further information
   */
  enum twindow_status {
    discharge_caps,
    vcc_3v3_off,  // waiting for plug
    vcc_12v_off,  // panel turned off remotely
    vcc_3v3_on,   // waiting for comm
    vcc_12v_on    // comm ok
  };

  enum window_from_outside : bool { LEFT = false, RIGHT = true };

  static constexpr std::size_t num_of_pixels{4};

  window() = delete;
  window(GPIO_TypeDef* gpio_port_3v3, std::uint16_t gpio_pin_3v3,
         GPIO_TypeDef* gpio_port_power, std::uint16_t gpio_pin_power,
         GPIO_TypeDef* gpio_port_tx, std::uint16_t gpio_pin_tx,
         USART_TypeDef* USARTx, DMA_TypeDef* DMAx, std::uint32_t DMA_Channel);

  window(const window&) = delete;
  window& operator=(const window&) = delete;

  /*!
   * \brief the the jobs defined by the state of the window
   *
   * @param w The window which status is to be changed
   */
  void step_state();

  /*!
   * \brief get the status of a window
   *
   * @param w which window's status to be returned
   * @return the status of the window asked in the argument line
   */
  twindow_status get_state();

  /*!
   * \brief changes the state of the window passed on the argument line and do
   * the necessary modifications on the board
   *
   * @param w The window which status is to be changed
   */

  void set_state(twindow_status);

  /*!
   * \brief stores the status of each window
   */
  std::array<pixel_data, num_of_pixels> pixels;

  volatile std::uint8_t whitebalance_data[22]{0xE0};

  /*!
   * \brief Blanks every panel connected to the window.
   */
  void blank();

  void set_whitebalance_flag(bool value);
  bool get_whitebalance_flag();
  void time_handler();
  void set_usart_active(bool value);

  static void swap_windows();
  static window& get_window(window_from_outside);

  static void turn_internal_anim_on();
  static void turn_internal_anim_off();
  static bool is_internal_animation_on();

  static void step_anim();

 private:
  twindow_status status;

  GPIO_TypeDef* gpio_port_3v3;
  GPIO_TypeDef* gpio_port_tx;
  GPIO_TypeDef* gpio_port_power;  // TODO add const keyword
  std::uint16_t gpio_pin_3v3, gpio_pin_tx,
      gpio_pin_power;  // TODO add const keyword

  DMA_TypeDef* DMAx;
  std::uint32_t DMA_Channel;
  USART_TypeDef* usart;

  std::uint8_t DMA_buffer[13]{0xF0};

  bool transmitted_before;

  volatile bool whitebalance_flag;

  /*!
   * \brief counts the seconds
   * used to countdown the timeout in panel probing
   */
  std::uint8_t tick_1s{0};
  bool usart_active{true};

  static bool internal_animation_on;
  static bool windows_swapped;

  void update_whitebalance();
  void update_image();
};

namespace windows {
extern window* left_window;
extern window* right_window;
};  // namespace windows
