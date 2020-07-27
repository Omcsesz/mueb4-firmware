/**
 * Helper functions for LL GPIO driver.
 * @file
 * @author Ádám Kiss
 * @author Zsombor Bodnár
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/// Output

enum output_gpioes {
  WINDOW_POWER_LEFT,
  SPI1_NSS,
  W5500_RESET,
  LED_JOKER,
  LED_COMM,
  LED_DHCP,
  LED_HEART,
  WINDOW_POWER_RIGHT,
  WINDOW_3V3_RIGHT,
  WINDOW_3V3_LEFT
};

void reset_gpio(enum output_gpioes);
#define reset_gpio(x) LL_GPIO_ResetOutputPin(x##_GPIO_Port, x##_Pin);

void set_gpio(enum output_gpioes);
#define set_gpio(x) LL_GPIO_SetOutputPin(x##_GPIO_Port, x##_Pin);

void toogle_gpio(enum output_gpioes);
#define toogle_gpio(x) LL_GPIO_TogglePin(x##_GPIO_Port, x##_Pin);

/// Input

enum input_gpioes { USER_INPUT_BUTTON };

uint32_t read_gpio(enum input_gpioes);
#define read_gpio(x) LL_GPIO_IsInputPinSet(x##_GPIO_Port, x##_Pin);

#ifdef __cplusplus
}
#endif
