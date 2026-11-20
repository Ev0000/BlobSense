#pragma once

#include <string>

#include "esphome/core/automation.h"
#include "esphome/core/log.h"

#include "blobsense.h"

namespace esphome {
namespace blobsense {

// Action blobsense.subtract : retire une valeur au compteur A ou B.
template<typename... Ts> class SubtractAction : public Action<Ts...>, public Parented<BlobSenseComponent> {
 public:
  TEMPLATABLE_VALUE(std::string, counter)
  TEMPLATABLE_VALUE(int32_t, value)

  void play(const Ts &...x) override {
    const std::string counter = this->counter_.value(x...);
    if (counter == "a" || counter == "A") {
      this->parent_->subtract(false, this->value_.value(x...));
    } else if (counter == "b" || counter == "B") {
      this->parent_->subtract(true, this->value_.value(x...));
    } else {
      ESP_LOGW("blobsense", "subtract : compteur inconnu \"%s\" (attendu \"a\" ou \"b\")", counter.c_str());
    }
  }
};

}  // namespace blobsense
}  // namespace esphome
