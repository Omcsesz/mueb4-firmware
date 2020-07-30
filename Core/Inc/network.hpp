/**
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#pragma once

#include <w5500.h>

#include <cstddef>
#include <cstdint>

/**
 * Manages all network related functionality.
 */
class network {
 public:
  /// Byte code for network commands.
  enum commands {
    turn_12v_off_left = 0x00,           //!< Turn left panel's 12v off
    turn_12v_off_right = 0x01,          //!< Turn right panel's 12v off
    reset_left_panel = 0x02,            //!< Reset left panel state
    reset_right_panel = 0x03,           //!< Reset right panel state
    reboot = 0x04,                      //!< Reboot device
    get_status = 0x05,                  //!< Get device's status
    get_mac = 0x07,                     //!< Get device's MAC address
    use_internal_anim = 0x10,           //!< Use internal animation
    use_external_anim = 0x20,           //!< Use external animation
    blank = 0x30,                       //!< Blank both panels
    delete_anim_network_buffer = 0x06,  //!< Unused
    ping = 0x40,                        //!< Send back 'pong' response
    start_firmware_update = 0x50,       //!< Start firmware update process
    get_firmware_checksum = 0x51,       //!< Return main program checksum
    swap_windows = 0x70,                //!< Swap left and right windows
    set_whitebalance = 0x80             //!< Set white balance
  };

  network(const network&) = delete;
  network& operator=(const network&) = delete;

  /// Network class' loop
  void step_network();

  static network& instance();

 private:
  /// Socket number for remote command handling
  static constexpr std::uint8_t command_socket{0};
  /// Socket number for unicast protocol
  static constexpr std::uint8_t unicast_socket{1};
  /// Socket number for broadcast protocol
  static constexpr std::uint8_t broadcast_socket{2};
  /// Socket number for DHCP
  static constexpr std::uint8_t dhcp_socket{7};

  /// Stores device's status string
  char status_string[512]{};

  network();

  /// Generates #status_string.
  std::size_t create_status_string();

  /**
   * Calls #fetch_frame_broadcast_proto, #fetch_frame_unicast_proto in this
   * order
   */
  void fetch_frame();

  /// Handles unicast protocol
  void fetch_frame_unicast_proto();

  /// Handles broadcast protocol
  void fetch_frame_broadcast_proto();

  /// Handles remote command @see #commands
  void do_remote_command();
};
