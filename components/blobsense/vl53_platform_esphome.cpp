// Implémentation de la couche plateforme des drivers ST VL53L5CX / VL53L8CX ULD
// au-dessus du bus I2C d'ESPHome.

#include "platform.h"
#include "blobsense.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"

using esphome::blobsense::BlobSenseComponent;

static inline BlobSenseComponent *parent_of(VL53LX_Platform *p) {
  return static_cast<BlobSenseComponent *>(p->handle);
}

extern "C" {

uint8_t RdByte(VL53LX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {
  return parent_of(p_platform)->platform_read(RegisterAdress, p_value, 1);
}

uint8_t WrByte(VL53LX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {
  return parent_of(p_platform)->platform_write(RegisterAdress, &value, 1);
}

uint8_t RdMulti(VL53LX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
  return parent_of(p_platform)->platform_read(RegisterAdress, p_values, size);
}

uint8_t WrMulti(VL53LX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
  return parent_of(p_platform)->platform_write(RegisterAdress, p_values, size);
}

uint8_t Reset_Sensor(VL53LX_Platform *p_platform) { return parent_of(p_platform)->platform_reset(); }

void SwapBuffer(uint8_t *buffer, uint16_t size) {
  for (uint32_t i = 0; i < size; i += 4) {
    uint32_t tmp = (static_cast<uint32_t>(buffer[i]) << 24) | (static_cast<uint32_t>(buffer[i + 1]) << 16) |
                   (static_cast<uint32_t>(buffer[i + 2]) << 8) | static_cast<uint32_t>(buffer[i + 3]);
    memcpy(&buffer[i], &tmp, 4);
  }
}

uint8_t WaitMs(VL53LX_Platform * /*p_platform*/, uint32_t TimeMs) {
  // Attente découpée pour nourrir le watchdog pendant l'initialisation.
  while (TimeMs > 0) {
    uint32_t step = TimeMs > 10 ? 10 : TimeMs;
    esphome::delay(step);
    esphome::App.feed_wdt();
    TimeMs -= step;
  }
  return 0;
}

}  // extern "C"
