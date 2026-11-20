/*
 * Couche plateforme du driver ST VL53L5CX ULD pour BlobSense (ESPHome).
 *
 * Les prototypes suivent ceux du fichier platform.h fourni par ST avec le
 * driver ULD (BSD 3-clause). L'implémentation est dans
 * vl53l5cx_platform_esphome.cpp.
 */
#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  /* Adresse I2C 8 bits utilisée par le driver ST (non utilisée ici). */
  uint16_t address;
  /* Pointeur opaque vers l'objet ESPHome qui gère le bus I2C. */
  void *handle;
} VL53L5CX_Platform;

/* Une seule cible par zone : la plus proche (voir set_target_order). */
#define VL53L5CX_NB_TARGET_PER_ZONE 1U

/* Sorties inutiles désactivées pour réduire le volume lu sur l'I2C. */
#define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM
#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
#define VL53L5CX_DISABLE_MOTION_INDICATOR

uint8_t RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);
uint8_t WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);
uint8_t RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t Reset_Sensor(VL53L5CX_Platform *p_platform);
void SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs);

#ifdef __cplusplus
}
#endif

#endif /* _PLATFORM_H_ */
