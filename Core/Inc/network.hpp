/*
 * network.h
 *
 *  Created on: Apr 7, 2018
 *      Author: kisada
 */

#pragma once

#include <w5500.h>

#include <cstddef>
#include <cstdint>

class network {
 public:
  enum commands {
    turn_12v_off_left = 0x00,
    turn_12v_off_right = 0x01,
    reset_left_panel = 0x02,
    reset_right_panel = 0x03,
    reboot = 0x04,
    get_status = 0x05,
    get_mac = 0x07,
    use_internal_anim = 0x10,
    use_external_anim = 0x20,
    blank = 0x30,
    delete_anim_network_buffer = 0x06,
    ping = 0x40,
    enable_update = 0x50,
    get_new_fw_chksum = 0x51,
    refurbish = 0x60,
    swap_windows = 0x70,
    set_whitebalance = 0x80
  };

  network();
  network(const network&) = delete;
  network& operator=(const network&) = delete;

  void step_network();

 private:
  static constexpr std::uint8_t command_socket{0};
  static constexpr std::uint8_t unicast_socket{1};
  static constexpr std::uint8_t multicast_socket{2};
  static constexpr std::uint8_t fw_update_socket{3};
  static constexpr std::uint8_t dhcp_socket{7};
  wiz_NetInfo netInfo;

  std::uint8_t emelet_szam{0};
  std::uint8_t szoba_szam{0};

  char status_string[512];
  bool is_update_enabled = false;

  std::size_t create_status_string();
  void enable_update_scoket();
  void step_update();
  std::size_t calc_new_fw_chksum();

  void fetch_frame();
  void fetch_frame_unicast_proto();
  void fetch_frame_multicast_proto();
  void do_remote_command();
};
