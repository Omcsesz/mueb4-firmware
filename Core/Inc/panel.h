/**
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#ifndef MATRIX4_MUEB_FW_INC_PANEL_H_
#define MATRIX4_MUEB_FW_INC_PANEL_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "main.h"

/// Manages all panel related functionality.
class Panel final {
 public:
  /// #Panel status.
  enum Status {
    kDischargeCaps,  ///< Discharge capacitors
    kVcc3v3Off,      ///< Turn 3v3 off
    kVcc12vOff,      ///< Turn 12v off
    kVcc3v3On,       ///< Turn 3v3 on
    kVcc12vOn        ///< Turn 12v on
  };

  /**
   * Stores which side the panel is on viewed from outside.
   * @see #SwapWindows
   */
  enum Side { LEFT, RIGHT };

  /// Stores pixel amount on one panel.
  static constexpr std::size_t kPixelCount{4u};

  /// Stores pixels size in bytes.
  static constexpr std::size_t kPanelColorDataSize{kPixelCount * 3u};

  /// Stores the color values of the panel pixels.
  using PanelColorData = std::array<std::uint8_t, kPanelColorDataSize>;

  /// Stores white balance data size.
  static constexpr std::size_t kWhiteBalanceDataSize{21u};

  /// Stores white balance configuration data.
  using WhiteBalanceData = std::array<std::uint8_t, kWhiteBalanceDataSize>;

  static constexpr std::uint8_t kInitCommand{0xF0u};

  static constexpr std::uint8_t kConfigCommand{0xE0u};

  Panel(const Panel&) = delete;
  Panel& operator=(const Panel&) = delete;

  /**
   * Increase #tick_1s_ by 1.
   * @see ::TIM17_IRQHandler
   */
  static void TimeHandler();

  /**
   * Used for GPIO USER_INPUT_BUTTON
   * @see ::EXTI2_3_IRQHandler
   */
  static void ToggleInternalAnimation();

  static void SetInternalAnimation(bool value);

  static bool internal_animation_enabled();

  /**
   * Swap panels.
   * @see #swapped_ network#kSwapPanels
   */
  static void SwapPanels();

  /**
   * Returns left #Panel instance independently of #swapped_.
   * @see #LeftPanel
   * @return Left #Panel instance
   */
  static Panel& left_panel();

  /**
   * Returns right #Panel instance independently of #swapped_.
   * @see #RightPanel
   * @return Right #Panel instance
   */
  static Panel& right_panel();

  /**
   * This function returns a #Panel instance using #swapped_.
   * Can be used to get the correct panel even if the cables are swapped.
   * @warning You should always use this function if the position of the panel.
   * matters
   * @see #Side
   * @see network#SwapWindows
   * @param side Which panel to get viewed from the outside
   * @return Panel instance viewed from the outside
   */
  static Panel& GetPanel(Side side);

  /// Internal animation's loop
  static void StepInternalAnimation();

  static void BlankAll();

  /// Panel class' loop
  void Step();

  /**
   * Set status.
   * Changes the status of the panel and makes the necessary modifications on
   * the MCU.
   * @see #Status
   */
  void SetStatus(Status status);

  /**
   * Blanks the panel.
   * @see Network#kBlank
   */
  void Blank();

  /// Send pixel data to panel.
  void SendPixels(const PanelColorData& pixels);

  /// Send white balance data to panel.
  void SendWhitebalance(const WhiteBalanceData& white_balance);

  void Heartbeat();

 private:
  Panel(GPIO_TypeDef* const gpio_port_3v3, const std::uint16_t gpio_pin_3v3,
        GPIO_TypeDef* const gpio_port_power, const std::uint16_t gpio_pin_power,
        GPIO_TypeDef* const gpio_port_tx, const std::uint16_t gpio_pin_tx,
        UART_HandleTypeDef* const huartx);

  /// DMA TX buffer.
  std::array<std::uint8_t, kPanelColorDataSize + 1u> dma_tx_buffer_{
      kInitCommand};

  std::array<std::uint8_t, kWhiteBalanceDataSize + 1u> white_balance_{
      kConfigCommand};

  /// Stores state of panel @see #Status.
  Status status_{kDischargeCaps};

  ///@{
  /// Defined in main.h
  GPIO_TypeDef* const gpio_port_3v3_;
  GPIO_TypeDef* const gpio_port_tx_;
  GPIO_TypeDef* const gpio_port_power_;
  UART_HandleTypeDef* const huartx_;
  const std::uint16_t gpio_pin_3v3_, gpio_pin_tx_, gpio_pin_power_;
  ///@}

  /**
   * Counts the seconds.
   * Used to countdown the timeout in panel probing.
   */
  std::uint8_t tick_1s_{0u};

  /// Stores if the internal animation is enabled.
  static bool internal_animation_enabled_;

  /// Stores if left and right panels are swapped.
  static bool swapped_;

  std::uint8_t heartbeat_{0u};

  /// Stores if we can communicate with the panel.
  bool active_{false};

  bool uart_initialized_{false};
};

#endif  // MATRIX4_MUEB_FW_INC_PANEL_H_
