#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/sensor/sensor.h"

#ifdef USE_ESP32
#include <esp_gattc_api.h>

namespace esphome {
namespace ble_client {

namespace espbt = esphome::esp32_ble_tracker;

using data_to_value_t = std::function<float(std::vector<uint8_t>)>;

class BLESensor : public sensor::Sensor, public PollingComponent, public BLEClientNode {
 public:
  void loop() override;
  void update() override;
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void set_service_uuid16(uint16_t uuid) { this->service_uuid_ = espbt::ESPBTUUID::from_uint16(uuid); }
  void set_service_uuid32(uint32_t uuid) { this->service_uuid_ = espbt::ESPBTUUID::from_uint32(uuid); }
  void set_service_uuid128(uint8_t *uuid) { this->service_uuid_ = espbt::ESPBTUUID::from_raw(uuid); }
  void set_char_uuid16(uint16_t uuid) { this->char_uuid_ = espbt::ESPBTUUID::from_uint16(uuid); }
  void set_char_uuid32(uint32_t uuid) { this->char_uuid_ = espbt::ESPBTUUID::from_uint32(uuid); }
  void set_char_uuid128(uint8_t *uuid) { this->char_uuid_ = espbt::ESPBTUUID::from_raw(uuid); }
  void set_descr_uuid16(uint16_t uuid) { this->descr_uuid_ = espbt::ESPBTUUID::from_uint16(uuid); }
  void set_descr_uuid32(uint32_t uuid) { this->descr_uuid_ = espbt::ESPBTUUID::from_uint32(uuid); }
  void set_descr_uuid128(uint8_t *uuid) { this->descr_uuid_ = espbt::ESPBTUUID::from_raw(uuid); }
  void set_data_to_value(data_to_value_t &&lambda) { this->data_to_value_func_ = lambda; }
  void set_enable_notify(bool notify) { this->notify_ = notify; }
  uint16_t handle;

 protected:
  float parse_data_(uint8_t *value, uint16_t value_len);
  optional<data_to_value_t> data_to_value_func_{};
  bool notify_;
  espbt::ESPBTUUID service_uuid_;
  espbt::ESPBTUUID char_uuid_;
  espbt::ESPBTUUID descr_uuid_;
};

}  // namespace ble_client
}  // namespace esphome
#endif
