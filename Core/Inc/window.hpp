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

  static constexpr std::size_t num_of_pixels{4};

  enum window_from_outside : bool { LEFT = false, RIGHT = true };

  std::uint8_t whitebalance_data[22]{0xE0};
  bool whitebalance_flag{false};
  bool usart_active{true};

  static bool internal_animation_on;

  /*!
   * \brief stores the status of each window
   */
  std::array<pixel_data, num_of_pixels> pixels;

  window() = delete;
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
  twindow_status get_state() const;

  /*!
   * \brief changes the state of the window passed on the argument line and do
   * the necessary modifications on the board
   *
   * @param w The window which status is to be changed
   */

  void set_state(twindow_status state);

  /*!
   * \brief Blanks every panel connected to the window.
   */
  void blank();

  void time_handler();
  void set_usart_active(bool value);

  static void swap_windows();

  static window& get_left_window();
  static window& get_right_window();
  static window& get_window(window_from_outside);

  static void step_anim();

 private:
  twindow_status status;

  GPIO_TypeDef* const gpio_port_3v3;
  GPIO_TypeDef* const gpio_port_tx;
  GPIO_TypeDef* const gpio_port_power;
  DMA_TypeDef* const DMAx;
  USART_TypeDef* const USARTx;
  const std::uint32_t dma_tx_channel;
  const std::uint16_t gpio_pin_3v3, gpio_pin_tx, gpio_pin_power;

  std::uint8_t dma_tx_buffer[13]{0xF0};

  /*!
   * \brief counts the seconds
   * used to countdown the timeout in panel probing
   */
  std::uint8_t tick_1s{0};

  static bool windows_swapped;

  window(GPIO_TypeDef* const gpio_port_3v3, const std::uint16_t gpio_pin_3v3,
         GPIO_TypeDef* const gpio_port_power,
         const std::uint16_t gpio_pin_power, GPIO_TypeDef* const gpio_port_tx,
         const std::uint16_t gpio_pin_tx, USART_TypeDef* const USARTx,
         DMA_TypeDef* const DMAx, const std::uint32_t dma_tx_channel);
  void update_whitebalance();
  void update_image();
};
