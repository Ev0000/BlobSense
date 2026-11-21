#pragma once

// Couche d'adaptation entre BlobSense et le driver ST du capteur choisi
// (option `model` du YAML). Un seul driver est compilé : les deux drivers ST
// définissent des macros internes de même nom et ne peuvent pas cohabiter.

#include "esphome/core/defines.h"

extern "C" {
#ifdef USE_BLOBSENSE_VL53L8CX
#include "vl53l8cx_api.h"
#include "vl53l8cx_plugin_xtalk.h"
#else
#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_xtalk.h"
#endif
}

namespace esphome {
namespace blobsense {

#ifdef USE_BLOBSENSE_VL53L8CX
#define VLX(fn) vl53l8cx_##fn
#define VLX_C(name) VL53L8CX_##name
using VLX_Configuration = VL53L8CX_Configuration;
using VLX_ResultsData = VL53L8CX_ResultsData;
static constexpr const char *VLX_MODEL = "VL53L8CX";
static constexpr uint8_t VLX_REVISION_ID = 0x0C;
#else
#define VLX(fn) vl53l5cx_##fn
#define VLX_C(name) VL53L5CX_##name
using VLX_Configuration = VL53L5CX_Configuration;
using VLX_ResultsData = VL53L5CX_ResultsData;
static constexpr const char *VLX_MODEL = "VL53L5CX";
static constexpr uint8_t VLX_REVISION_ID = 0x02;
#endif

// Identifiants des deux modèles, pour un diagnostic clair en cas d'erreur de `model`.
static constexpr uint8_t VLX_DEVICE_ID = 0xF0;
static constexpr uint8_t VL53L5CX_REVISION = 0x02;
static constexpr uint8_t VL53L8CX_REVISION = 0x0C;

}  // namespace blobsense
}  // namespace esphome
