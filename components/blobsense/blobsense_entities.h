#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/macros.h"
#include "esphome/core/version.h"
#include "esphome/core/preferences.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"

#include "blobsense.h"

namespace esphome {
namespace blobsense {

// --- Switches : lecture active, debug de la matrice ---

enum class SwitchType : uint8_t { ACTIVE, DEBUG_MATRIX };

class BlobSenseSwitch : public switch_::Switch, public Component, public Parented<BlobSenseComponent> {
 public:
  explicit BlobSenseSwitch(SwitchType type) : type_(type) {}

  void setup() override {
    auto initial = this->get_initial_state_with_restore_mode();
    this->write_state(initial.has_value() ? *initial : (this->type_ == SwitchType::ACTIVE));
  }
  // Doit être configuré avant le premier traitement d'image, mais après le capteur.
  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

 protected:
  void write_state(bool state) override {
    if (this->type_ == SwitchType::ACTIVE) {
      this->parent_->set_active(state);
    } else {
      this->parent_->set_debug(state);
    }
    this->publish_state(state);
  }

  SwitchType type_;
};

// --- Numbers : hauteur min / max (cm), valeur conservée après redémarrage ---

enum class NumberType : uint8_t { MIN_HEIGHT, MAX_HEIGHT };

class BlobSenseNumber : public number::Number, public Component, public Parented<BlobSenseComponent> {
 public:
  explicit BlobSenseNumber(NumberType type) : type_(type) {}
  void set_initial_value(float value) { this->initial_value_ = value; }
  void set_restore_value(bool restore) { this->restore_value_ = restore; }

  void setup() override {
    float value = this->initial_value_;
    if (this->restore_value_) {
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 2, 0)
      this->pref_ = this->make_entity_preference<float>();
#else
      this->pref_ = global_preferences->make_preference<float>(this->get_preference_hash());
#endif
      float stored;
      if (this->pref_.load(&stored))
        value = stored;
    }
    this->apply_(value);
    this->publish_state(value);
  }
  float get_setup_priority() const override { return setup_priority::DATA + 1.0f; }

 protected:
  void control(float value) override {
    this->apply_(value);
    this->publish_state(value);
    if (this->restore_value_)
      this->pref_.save(&value);  // écrit en flash seulement quand la valeur change
  }

  void apply_(float value) {
    if (this->type_ == NumberType::MIN_HEIGHT) {
      this->parent_->set_min_height(value);
    } else {
      this->parent_->set_max_height(value);
    }
  }

  NumberType type_;
  float initial_value_{0};
  bool restore_value_{true};
  ESPPreferenceObject pref_;
};

// --- Buttons ---

enum class ButtonType : uint8_t {
  RECALIBRATE,
  RESET_A,
  RESET_B,
  CALIBRATE_DIRECTION,
  CALIBRATE_COVER,
  CLEAR_COVER_CALIBRATION,
};

class BlobSenseButton : public button::Button, public Parented<BlobSenseComponent> {
 public:
  explicit BlobSenseButton(ButtonType type) : type_(type) {}

 protected:
  void press_action() override {
    switch (this->type_) {
      case ButtonType::RECALIBRATE:
        this->parent_->recalibrate();
        break;
      case ButtonType::RESET_A:
        this->parent_->reset_counter(false);
        break;
      case ButtonType::RESET_B:
        this->parent_->reset_counter(true);
        break;
      case ButtonType::CALIBRATE_DIRECTION:
        this->parent_->calibrate_direction();
        break;
      case ButtonType::CALIBRATE_COVER:
        this->parent_->calibrate_cover();
        break;
      case ButtonType::CLEAR_COVER_CALIBRATION:
        this->parent_->clear_cover_calibration();
        break;
    }
  }

  ButtonType type_;
};

}  // namespace blobsense
}  // namespace esphome
