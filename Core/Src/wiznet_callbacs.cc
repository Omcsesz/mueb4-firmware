#include <socket.h>
#include <wizchip_conf.h>

#include <cstdint>

#include "main.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi1;

void CrisEn() { __disable_irq(); }

void CrisEx() { __enable_irq(); }

void CsSel() {
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
}

void CsDesel() {
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}

std::uint8_t SpiRb() {
  std::uint8_t pData;
  HAL_SPI_Receive(&hspi1, &pData, 1u, HAL_MAX_DELAY);

  return pData;
}

void SpiWb(std::uint8_t pData) {
  HAL_SPI_Transmit(&hspi1, &pData, 1u, HAL_MAX_DELAY);
}

void SpiRBurst(std::uint8_t *pData, std::uint16_t Size) {
  HAL_SPI_Receive(&hspi1, pData, Size, HAL_MAX_DELAY);
}

void SpiWBurst(std::uint8_t *pData, std::uint16_t Size) {
  HAL_SPI_Transmit(&hspi1, pData, Size, HAL_MAX_DELAY);
}
