/// WIZnet critical enter.
void CrisEn();

/// WIZnet critical exit.
void CrisEx();

/**
 * WIZnet chip select.
 * Sets SCSn to low.
 */
void CsSel();

/**
 * WIZnet chip deselect.
 * Sets SCSn to high.
 */
void CsDesel();

/// Read byte from WIZnet chip through SPI.
std::uint8_t SpiRb();

/// Write byte to WIZnet chip through SPI.
void SpiWb(std::uint8_t pData);

/// Read burst from WIZnet chip through SPI.
void SpiRBurst(std::uint8_t *pData, std::uint16_t Size);

/// Write burst to WIZnet chip through SPI.
void SpiWBurst(std::uint8_t *pData, std::uint16_t Size);
