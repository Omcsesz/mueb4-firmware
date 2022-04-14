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
#include <span>

#include "main.h"

/// Manages all panel related functionality.
class Panel final {
 public:
  /// #Panel state.
  enum class State {
    kDisabled = 0x00u,  ///< Panel is disabled
    kPowerOff = 0x01u,  ///< 12v, 3v3 are off and capacitors are discharging
    kVcc3v3On = 0x02u,  ///< 3v3 is on
    kVcc12vOn = 0x03u   ///< 12v is on
  };

  /**
   * Stores which side the panel is on viewed from outside.
   * @see #SwapWindows
   */
  enum class Side { LEFT, RIGHT };

  /// Stores pixel amount on one panel.
  static constexpr std::uint8_t kPixelCount{4u};

  /// Stores pixels size in bytes.
  static constexpr std::uint8_t kColorDataSize{kPixelCount * 3u};

  /// Stores the color values of the panel pixels.
  using ColorData = std::array<std::uint8_t, kColorDataSize>;

  /// Stores white balance data size.
  static constexpr std::uint8_t kWhiteBalanceDataSize{45u};

  /// Stores white balance configuration data.
  using WhiteBalanceData = std::array<std::uint8_t, kWhiteBalanceDataSize>;

  static constexpr std::uint8_t kInitCommand{0xF0u};

  static constexpr std::uint8_t kConfigCommand{0xE0u};

  Panel(const Panel&) = delete;
  Panel& operator=(const Panel&) = delete;

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

  static Panel& GetPanel(UART_HandleTypeDef* huartx);

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

  static void SetInternalAnimation(bool on);

  static bool internal_animation_enabled();

  /**
   * Swap panels.
   * @see #swapped_ network#kSwapPanels
   */
  static void Swap();

  static void BlankAll();

  static void StepAll();

  static void DisableAll();

  static void SendColorDataAll();

  static void SendWhiteBalanceToAll(const WhiteBalanceData& white_balance);

  /// Send white balance data to panel.
  void SendWhiteBalance(const WhiteBalanceData& white_balance);

  Panel& SetColorData(std::span<const std::uint8_t> data, bool _8bit);

  void Heartbeat();

  State state();

  static std::array<std::uint8_t, 2> GetPanelStates();

 private:
  explicit Panel(Side side);

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

  /// Internal animation's loop
  static void StepInternalAnimation();

  /// Panel class' loop
  void Step();

  /**
   * Blanks the panel.
   * @see Network#kBlank
   */
  void Blank();

  void Disable();

  void SendColorData();

  static std::array<std::uint32_t, 2u> adc_;

  /// Stores if the internal animation is enabled.
  static bool internal_animation_enabled_;

  /// Stores if left and right panels are swapped.
  static bool swapped_;

  std::array<std::uint8_t, kWhiteBalanceDataSize + 1u> white_balance_data_{
      kConfigCommand};

  /// DMA TX buffer.
  std::array<std::uint8_t, kColorDataSize + 1u> color_data_{kInitCommand};

  GPIO_TypeDef* const gpio_3v3_port_;
  GPIO_TypeDef* const gpio_12v_port_;
  UART_HandleTypeDef* const huartx_;

  /// Stores state of panel @see #State.
  State state_{State::kPowerOff};

  const Side side_;

  const std::uint16_t gpio_3v3_pin_;

  const std::uint16_t gpio_12v_pin_;

  /**
   * Counts the seconds.
   * Used to countdown the timeout in panel probing.
   */
  std::uint8_t tick_1s_{0u};

  std::uint8_t heartbeat_{0u};

  /// Stores if we can communicate with the panel.
  bool active_{false};
};

#endif  // MATRIX4_MUEB_FW_INC_PANEL_H_
