/**
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#ifndef MATRIX4_MUEB_FW_INC_NETWORK_H_
#define MATRIX4_MUEB_FW_INC_NETWORK_H_

#include <cstdint>

/**
 * Manages all network related functionality.
 */
class Network final {
 public:
  /// Byte code for network commands.
  enum Command {
    kTurn12vOffLeft,       ///< Turn left panel's 12v off
    kTurn12vOffRight,      ///< Turn right panel's 12v off
    kResetLeftPanel,       ///< Reset left panel state
    kResetRightPanel,      ///< Reset right panel state
    kReboot,               ///< Reboot device
    kGetStatus,            ///< Get device's status
    kGetMac,               ///< Get device's MAC address
    kUseInternalAnim,      ///< Use internal animation
    kUseExternalAnim,      ///< Use external animation
    kBlank,                ///< Blank both panels
    kFlushSocketBuffers,   ///< Flush socket buffers
    kPing,                 ///< Send back 'pong' response
    kStartFirmwareUpdate,  ///< Start firmware update process
    kGetFirmwareChecksum,  ///< Return main program checksum
    kSwapPanels,           ///< Swap left and right panels
    kSetWhitebalance       ///< Set white balance
  };

  Network(const Network&) = delete;
  Network& operator=(const Network&) = delete;

  static Network& Instance();

  /// Network class' loop.
  void Step();

 private:
  /// Command socket port number.
  static constexpr std::uint16_t kCommandSocketPort{2000u};

  /// Broadcast socket port number.
  static constexpr std::uint16_t kBroadcastSocketPort{10000u};

  /// EUI-48 value's start address.
  static constexpr std::uint16_t kEui48StartAddress{0xFAu};

  /// EEPROM I2C device address.
  static constexpr std::uint8_t kEepromAddress{0b10100001u};

  /// Socket number for DHCP.
  static constexpr std::uint8_t kDhcpSocket{0u};

  /// Socket number for remote command handling.
  static constexpr std::uint8_t kCommandSocket{1u};

  /// Socket number for broadcast protocol.
  static constexpr std::uint8_t kBroadcastSocket{2u};

  Network();

  /// Handles broadcast protocol.
  void FetchFrameBroadcastProtocol();

  /**
   * Handles remote command @see #Command.
   * Can be used without IP address.
   */
  void FetchRemoteCommand();

  /// Flushes socket buffers.
  void FlushBuffers();

  /**
   * DHCP RX buffer.
   * @note 1 kb should be enough for DHCP RX buffer.
   */
  std::uint8_t dhcp_rx_buffer_[1024u]{};
};

#endif  // MATRIX4_MUEB_FW_INC_NETWORK_H_
