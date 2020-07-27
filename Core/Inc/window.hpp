/**
 * Window management subsystem for Schönherz Mátrix 4.
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "main.h"

/// Stores RGB color data.
class pixel_data {
 public:
  /**
   * Type created for storing the state of the internal buffer.
   * The buffer stores the next differential frame to be sent
   * @note This can be used for skipping unchanged pixels
   */
  enum tframe_status {
    buffer_free,  ///< Pixel data unchanged
    buffer_full   ///< Pixel data changed
  };

  tframe_status m_stat{buffer_full};  ///< Stores pixel status

  pixel_data() = default;
  pixel_data(const pixel_data&) = delete;
  pixel_data& operator=(const pixel_data&) = delete;

  /// Sets the pixel for the given color.
  void set(std::uint8_t red, std::uint8_t green, std::uint8_t blue);

  /// Sets the #m_stat to #buffer_free
  void flush();

  /// Checks if #m_stat status is #buffer_full
  bool isFull();

  std::uint8_t red();    ///< Gets red color value
  std::uint8_t green();  ///< Gets green color value
  std::uint8_t blue();   ///< Gets blue color value

 private:
  std::uint8_t m_red{0};      ///< Stores red color value
  std::uint8_t m_green{200};  ///< Stores green color value
  std::uint8_t m_blue{0};     ///< Stores blue color value
};

/// Manages all window, panel related functionality
class window {
 public:
  /// Type created for storing states for each window.
  enum twindow_status {
    discharge_caps,  //!< Discharge capacitors on panel.
    vcc_3v3_off,     //!< Turn 3v3 off to panel.
    vcc_12v_off,     //!< Turn 12v off to panel.
    vcc_3v3_on,      //!< Turn 3v3 on to panel.
    vcc_12v_on       //!< Turn 12v on to panel.
  };

  /// Stores pixel amount on one panel.
  static constexpr std::size_t num_of_pixels{4};

  /**
   * Stores which side the window is on viewed from outside.
   * @see #swap_windows
   */
  enum window_from_outside : bool { LEFT = false, RIGHT = true };

  /// Stores white balance configuration data.
  std::uint8_t whitebalance_data[22]{0xE0};

  /// Stores if white balance needs to be updated.
  bool whitebalance_flag{false};

  /// Stores if we can communicate with the panel.
  bool usart_active{true};

  /// Stores if the internal animation is enabled.
  static bool internal_animation_on;

  /// Stores #pixel_data for each pixel.
  std::array<pixel_data, num_of_pixels> pixels;

  window() = delete;
  window(const window&) = delete;
  window& operator=(const window&) = delete;

  /// Window class' loop
  void step_state();

  /**
   * Get window status.
   * @return The status of the window
   * @see twindow_status
   */
  twindow_status get_state() const;

  /**
   * Set window state.
   * Changes the state of the window and the necessary modifications on the
   * board
   * @see #twindow_status
   */
  void set_state(twindow_status state);

  /**
   * Blanks the window.
   * @see network#blank
   */
  void blank();

  /**
   * Increase #tick_1s by 1.
   * @see ::TIM17_IRQHandler
   */
  void time_handler();

  /**
   * Swap windows.
   * @see #windows_swapped network#swap_windows
   */
  static void swap_windows();

  /**
   * Returns left #window instance independently of #windows_swapped.
   * @see #get_left_window
   * @return Left #window instance
   */
  static window& get_left_window();

  /**
   * Returns right #window instance independently of #windows_swapped.
   * @see #get_right_window
   * @return Right #window instance
   */
  static window& get_right_window();

  /**
   * This function returns a #window instance using #windows_swapped.
   * Can be used to get the correct window even if the cables are swapped
   * @warning You should always use this function if the position of the window
   * matters
   * @see #window_from_outside
   * @see network#swap_windows
   * @param w Which window to get viewed from the outside
   * @return Window instance viewed from the outside
   */
  static window& get_window(window_from_outside w);

  /// Internal animation's loop
  static void step_anim();

 private:
  twindow_status status;  ///< Stores state of window @see #twindow_status

  ///@{
  /// Defined in main.h
  GPIO_TypeDef* const gpio_port_3v3;
  GPIO_TypeDef* const gpio_port_tx;
  GPIO_TypeDef* const gpio_port_power;
  DMA_TypeDef* const DMAx;
  USART_TypeDef* const USARTx;
  const std::uint32_t dma_tx_channel;
  const std::uint16_t gpio_pin_3v3, gpio_pin_tx, gpio_pin_power;
  ///@}

  /// DMA TX buffer.
  std::uint8_t dma_tx_buffer[13]{0xF0};

  /**
   * Counts the seconds.
   * Used to countdown the timeout in panel probing
   */
  std::uint8_t tick_1s{0};

  /// Stores if left and right windows are swapped
  static bool windows_swapped;

  window(GPIO_TypeDef* const gpio_port_3v3, const std::uint16_t gpio_pin_3v3,
         GPIO_TypeDef* const gpio_port_power,
         const std::uint16_t gpio_pin_power, GPIO_TypeDef* const gpio_port_tx,
         const std::uint16_t gpio_pin_tx, USART_TypeDef* const USARTx,
         DMA_TypeDef* const DMAx, const std::uint32_t dma_tx_channel);

  /// Send white balance data to panel
  void update_whitebalance();

  /// Send pixel data to panel
  void update_image();
};
