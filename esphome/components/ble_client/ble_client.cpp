#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "ble_client.h"


#ifdef USE_ESP32

namespace esphome {
namespace ble_client {

static const char *const TAG = "ble_client";

float BLEClient::get_setup_priority() const { return setup_priority::AFTER_BLUETOOTH; }

void BLEClient::setup() {
  auto ret = esp_ble_gattc_app_register(this->app_id);
  if (ret) {
    ESP_LOGE(TAG, "gattc app register failed. app_id=%d code=%d", this->app_id, ret);
    this->mark_failed();
  }
  this->set_states_(espbt::ClientState::IDLE);
  this->enabled = true;
}

void BLEClient::loop() {
  if (this->state() == espbt::ClientState::DISCOVERED) {
    this->connect();
  }
  for (auto *node : this->nodes_)
    node->loop();
}

void BLEClient::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE Client:");
  ESP_LOGCONFIG(TAG, "  Address: %s", this->address_str().c_str());
}

bool BLEClient::parse_device(const espbt::ESPBTDevice &device) {
  if (!this->enabled)
    return false;
  if (device.address_uint64() != this->address)
    return false;
  if (this->state() != espbt::ClientState::IDLE)
    return false;

  ESP_LOGD(TAG, "Found device at MAC address [%s]", device.address_str().c_str());
  this->set_states_(espbt::ClientState::DISCOVERED);

  auto addr = device.address_uint64();
  this->remote_bda[0] = (addr >> 40) & 0xFF;
  this->remote_bda[1] = (addr >> 32) & 0xFF;
  this->remote_bda[2] = (addr >> 24) & 0xFF;
  this->remote_bda[3] = (addr >> 16) & 0xFF;
  this->remote_bda[4] = (addr >> 8) & 0xFF;
  this->remote_bda[5] = (addr >> 0) & 0xFF;
  this->remote_addr_type = device.get_address_type();
  return true;
}

std::string BLEClient::address_str() const {
  char buf[20];
  sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", (uint8_t)(this->address >> 40) & 0xff,
          (uint8_t)(this->address >> 32) & 0xff, (uint8_t)(this->address >> 24) & 0xff,
          (uint8_t)(this->address >> 16) & 0xff, (uint8_t)(this->address >> 8) & 0xff,
          (uint8_t)(this->address >> 0) & 0xff);
  std::string ret;
  ret = buf;
  return ret;
}

void BLEClient::set_enabled(bool enabled) {
  if (enabled == this->enabled)
    return;
  if (!enabled && this->state() != espbt::ClientState::IDLE) {
    ESP_LOGI(TAG, "[%s] Disabling BLE client.", this->address_str().c_str());
    auto ret = esp_ble_gattc_close(this->gattc_if, this->conn_id);
    if (ret) {
      ESP_LOGW(TAG, "esp_ble_gattc_close error, address=%s status=%d", this->address_str().c_str(), ret);
    }
  }
  this->enabled = enabled;
}

void BLEClient::connect() {
  ESP_LOGI(TAG, "Attempting BLE connection to %s", this->address_str().c_str());
  auto ret = esp_ble_gattc_open(this->gattc_if, this->remote_bda, this->remote_addr_type, true);
  if (ret) {
    ESP_LOGW(TAG, "esp_ble_gattc_open error, address=%s status=%d", this->address_str().c_str(), ret);
    this->set_states_(espbt::ClientState::IDLE);
  } else {
    this->set_states_(espbt::ClientState::CONNECTING);
  }
}

const static char* get_gattc_string(esp_gattc_cb_event_t event) {
  switch(event) {
    case 0: return "ESP_GATTC_REG_EVT";               /*!< When GATT client is registered, the event comes */
    case 1: return "ESP_GATTC_UNREG_EVT";             /*!< When GATT client is unregistered, the event comes */
    case 2: return "ESP_GATTC_OPEN_EVT";              /*!< When GATT virtual connection is set up, the event comes */
    case 3: return "ESP_GATTC_READ_CHAR_EVT";         /*!< When GATT characteristic is read, the event comes */
    case 4: return "ESP_GATTC_WRITE_CHAR_EVT";        /*!< When GATT characteristic write operation completes, the event comes */
    case 5: return "ESP_GATTC_CLOSE_EVT";             /*!< When GATT virtual connection is closed, the event comes */
    case 6: return "ESP_GATTC_SEARCH_CMPL_EVT";       /*!< When GATT service discovery is completed, the event comes */
    case 7: return "ESP_GATTC_SEARCH_RES_EVT";        /*!< When GATT service discovery result is got, the event comes */
    case 8: return "ESP_GATTC_READ_DESCR_EVT";        /*!< When GATT characteristic descriptor read completes, the event comes */
    case 9: return "ESP_GATTC_WRITE_DESCR_EVT";       /*!< When GATT characteristic descriptor write completes, the event comes */
    case 10: return "ESP_GATTC_NOTIFY_EVT";           /*!< When GATT notification or indication arrives, the event comes */
    case 11: return "ESP_GATTC_PREP_WRITE_EVT";       /*!< When GATT prepare-write operation completes, the event comes */
    case 12: return "ESP_GATTC_EXEC_EVT";             /*!< When write execution completes, the event comes */
    case 13: return "ESP_GATTC_ACL_EVT";              /*!< When ACL connection is up, the event comes */
    case 14: return "ESP_GATTC_CANCEL_OPEN_EVT";      /*!< When GATT client ongoing connection is cancelled, the event comes */
    case 15: return "ESP_GATTC_SRVC_CHG_EVT";         /*!< When "service changed" occurs, the event comes */
    case 17: return "ESP_GATTC_ENC_CMPL_CB_EVT";      /*!< When encryption procedure completes, the event comes */
    case 18: return "ESP_GATTC_CFG_MTU_EVT";          /*!< When configuration of MTU completes, the event comes */
    case 19: return "ESP_GATTC_ADV_DATA_EVT";         /*!< When advertising of data, the event comes */
    case 20: return "ESP_GATTC_MULT_ADV_ENB_EVT";     /*!< When multi-advertising is enabled, the event comes */
    case 21: return "ESP_GATTC_MULT_ADV_UPD_EVT";     /*!< When multi-advertising parameters are updated, the event comes */
    case 22: return "ESP_GATTC_MULT_ADV_DATA_EVT";    /*!< When multi-advertising data arrives, the event comes */
    case 23: return "ESP_GATTC_MULT_ADV_DIS_EVT";     /*!< When multi-advertising is disabled, the event comes */
    case 24: return "ESP_GATTC_CONGEST_EVT";          /*!< When GATT connection congestion comes, the event comes */
    case 25: return "ESP_GATTC_BTH_SCAN_ENB_EVT";     /*!< When batch scan is enabled, the event comes */
    case 26: return "ESP_GATTC_BTH_SCAN_CFG_EVT";     /*!< When batch scan storage is configured, the event comes */
    case 27: return "ESP_GATTC_BTH_SCAN_RD_EVT";      /*!< When Batch scan read event is reported, the event comes */
    case 28: return "ESP_GATTC_BTH_SCAN_THR_EVT";     /*!< When Batch scan threshold is set, the event comes */
    case 29: return "ESP_GATTC_BTH_SCAN_PARAM_EVT";   /*!< When Batch scan parameters are set, the event comes */
    case 30: return "ESP_GATTC_BTH_SCAN_DIS_EVT";     /*!< When Batch scan is disabled, the event comes */
    case 31: return "ESP_GATTC_SCAN_FLT_CFG_EVT";     /*!< When Scan filter configuration completes, the event comes */
    case 32: return "ESP_GATTC_SCAN_FLT_PARAM_EVT";   /*!< When Scan filter parameters are set, the event comes */
    case 33: return "ESP_GATTC_SCAN_FLT_STATUS_EVT";  /*!< When Scan filter status is reported, the event comes */
    case 34: return "ESP_GATTC_ADV_VSC_EVT";          /*!< When advertising vendor spec content event is reported, the event comes */
    case 38: return "ESP_GATTC_REG_FOR_NOTIFY_EVT";   /*!< When register for notification of a service completes, the event comes */
    case 39: return "ESP_GATTC_UNREG_FOR_NOTIFY_EVT"; /*!< When unregister for notification of a service completes, the event comes */
    case 40: return "ESP_GATTC_CONNECT_EVT";          /*!< When the ble physical connection is set up, the event comes */
    case 41: return "ESP_GATTC_DISCONNECT_EVT";       /*!< When the ble physical connection disconnected, the event comes */
    case 42: return "ESP_GATTC_READ_MULTIPLE_EVT";    /*!< When the ble characteristic or descriptor multiple complete, the event comes */
    case 43: return "ESP_GATTC_QUEUE_FULL_EVT";       /*!< When the gattc command queue full, the event comes */
    case 44: return "ESP_GATTC_SET_ASSOC_EVT";        /*!< When the ble gattc set the associated address complete, the event comes */
    case 45: return "ESP_GATTC_GET_ADDR_LIST_EVT";    /*!< When the ble get gattc address list in cache finish, the event comes */
    case 46: return "ESP_GATTC_DIS_SRVC_CMPL_EVT";    /*!< When the ble discover service complete, the event comes */
    default: return "UNKNOWN GATTC EVENT";
  }
}

void BLEClient::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t esp_gattc_if,
                                    esp_ble_gattc_cb_param_t *param) {
  ESP_LOGI(TAG, "GOT EVENT %s", get_gattc_string(event));

  if (event == ESP_GATTC_REG_EVT && this->app_id != param->reg.app_id)
    return;
  if (event != ESP_GATTC_REG_EVT && esp_gattc_if != ESP_GATT_IF_NONE && esp_gattc_if != this->gattc_if)
    return;
//
//  ESP_LOGW(TAG, ":: 1 :: Warning  gattc_event_handler esp_gattc_cb_event_t = %d", event);
//  ESP_LOGD(TAG, ":: 2 :: Debug    gattc_event_handler esp_gattc_cb_event_t = %d", event);
//  ESP_LOGI(TAG, ":: 3 :: Info     gattc_event_handler esp_gattc_cb_event_t = %d", event);

  bool all_established = this->all_nodes_established_();

  switch (event) {
    case ESP_GATTC_REG_EVT: {
      if (param->reg.status == ESP_GATT_OK) {
        ESP_LOGV(TAG, "gattc registered app id %d", this->app_id);
        this->gattc_if = esp_gattc_if;
      } else {
        ESP_LOGE(TAG, "gattc app registration failed id=%d code=%d", param->reg.app_id, param->reg.status);
      }
      break;
    }
    case ESP_GATTC_OPEN_EVT: {
      ESP_LOGV(TAG, "[%s] ESP_GATTC_OPEN_EVT", this->address_str().c_str());
      this->conn_id = param->open.conn_id;
      if (param->open.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "connect to %s failed, status=%d", this->address_str().c_str(), param->open.status);
        this->set_states_(espbt::ClientState::IDLE);
        break;
      }
      break;
    }
    case ESP_GATTC_CONNECT_EVT: {
      ESP_LOGV(TAG, "[%s] ESP_GATTC_CONNECT_EVT", this->address_str().c_str());
      if (this->conn_id != param->connect.conn_id) {
        ESP_LOGD(TAG, "[%s] Unexpected conn_id in CONNECT_EVT: param conn=%d, open conn=%d",
                 this->address_str().c_str(), param->connect.conn_id, this->conn_id);
      }
      auto ret = esp_ble_gattc_send_mtu_req(this->gattc_if, param->connect.conn_id);
      if (ret) {
        ESP_LOGW(TAG, "esp_ble_gattc_send_mtu_req failed, status=%x", ret);
      }
      break;
    }
    case ESP_GATTC_CFG_MTU_EVT: {
      if (param->cfg_mtu.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "cfg_mtu to %s failed, mtu %d, status %d", this->address_str().c_str(), param->cfg_mtu.mtu,
                 param->cfg_mtu.status);
        this->set_states_(espbt::ClientState::IDLE);
        break;
      }
      ESP_LOGV(TAG, "cfg_mtu status %d, mtu %d", param->cfg_mtu.status, param->cfg_mtu.mtu);
      esp_ble_gattc_search_service(esp_gattc_if, param->cfg_mtu.conn_id, nullptr);
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      if (memcmp(param->disconnect.remote_bda, this->remote_bda, 6) != 0) {
        return;
      }
      ESP_LOGV(TAG, "[%s] ESP_GATTC_DISCONNECT_EVT, reason %d", this->address_str().c_str(), param->disconnect.reason);
      for (auto &svc : this->services_)
        delete svc;  // NOLINT(cppcoreguidelines-owning-memory)
      this->services_.clear();
      this->set_states_(espbt::ClientState::IDLE);
      break;
    }
    case ESP_GATTC_SEARCH_RES_EVT: {
      BLEService *ble_service = new BLEService();  // NOLINT(cppcoreguidelines-owning-memory)
      ble_service->uuid = espbt::ESPBTUUID::from_uuid(param->search_res.srvc_id.uuid);
      ble_service->start_handle = param->search_res.start_handle;
      ble_service->end_handle = param->search_res.end_handle;
      ble_service->client = this;
      this->services_.push_back(ble_service);
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      ESP_LOGV(TAG, "[%s] ESP_GATTC_SEARCH_CMPL_EVT", this->address_str().c_str());
      for (auto &svc : this->services_) {
        ESP_LOGI(TAG, "Service UUID: %s", svc->uuid.to_string().c_str());
        ESP_LOGI(TAG, "  start_handle: 0x%x  end_handle: 0x%x", svc->start_handle, svc->end_handle);
        svc->parse_characteristics();
      }
      this->set_states_(espbt::ClientState::CONNECTED);
      this->set_state(espbt::ClientState::ESTABLISHED);
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      auto *descr = this->get_config_descriptor(param->reg_for_notify.handle);
      if (descr == nullptr) {
        ESP_LOGW(TAG, "No descriptor found for notify of handle 0x%x", param->reg_for_notify.handle);
        break;
      }
      if (descr->uuid.get_uuid().len != ESP_UUID_LEN_16 ||
          descr->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
        ESP_LOGW(TAG, "Handle 0x%x (uuid %s) is not a client config char uuid", param->reg_for_notify.handle,
                 descr->uuid.to_string().c_str());
        break;
      }
      uint16_t notify_en = 1;
      auto status =
          esp_ble_gattc_write_char_descr(this->gattc_if, this->conn_id, descr->handle, sizeof(notify_en),
                                         (uint8_t *) &notify_en, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
      if (status) {
        ESP_LOGW(TAG, "esp_ble_gattc_write_char_descr error, status=%d", status);
      }
      break;
    }

    default:
      break;
  }
  for (auto *node : this->nodes_)
    node->gattc_event_handler(event, esp_gattc_if, param);

  // Delete characteristics after clients have used them to save RAM.
  if (!all_established && this->all_nodes_established_()) {
    for (auto &svc : this->services_)
      delete svc;  // NOLINT(cppcoreguidelines-owning-memory)
    this->services_.clear();
  }
}

const static char* get_gap_event_type_string(esp_gap_ble_cb_event_t event) {
  switch (event) {
#if (BLE_42_FEATURE_SUPPORT == TRUE)
    case 0: return "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT";
    case 1: return "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT";
    case 2: return "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT";
    case 3: return "ESP_GAP_BLE_SCAN_RESULT_EVT";
    case 4: return "ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT";
    case 5: return "ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT";
    case 6: return "ESP_GAP_BLE_ADV_START_COMPLETE_EVT";
    case 7: return "ESP_GAP_BLE_SCAN_START_COMPLETE_EVT";
#endif  // #if (BLE_42_FEATURE_SUPPORT == TRUE)
    case 8: return "ESP_GAP_BLE_AUTH_CMPL_EVT";
    case 9: return "ESP_GAP_BLE_KEY_EVT";
    case 10: return "ESP_GAP_BLE_SEC_REQ_EVT";
    case 11: return "ESP_GAP_BLE_PASSKEY_NOTIF_EVT";
    case 12: return "ESP_GAP_BLE_PASSKEY_REQ_EVT";
    case 13: return "ESP_GAP_BLE_OOB_REQ_EVT";
    case 14: return "ESP_GAP_BLE_LOCAL_IR_EVT";
    case 15: return "ESP_GAP_BLE_LOCAL_ER_EVT";
    case 16: //ESP_GAP_BLE_NC_REQ_EVT:
      return "ESP_GAP_BLE_NC_REQ_EVT";
#if (BLE_42_FEATURE_SUPPORT == TRUE)
    case 17: //ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
      return "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT";
    case 18: //ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
      return "ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT";
#endif  // #if (BLE_42_FEATURE_SUPPORT == TRUE)
    case 19: //ESP_GAP_BLE_SET_STATIC_RAND_ADDR_EVT:
      return "ESP_GAP_BLE_SET_STATIC_RAND_ADDR_EVT";
    case 20: //ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
      return "ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT";
    case 21: //ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
      return "ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT";
    case 22: //ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
      return "ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT";
    case 23: //ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
      return "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT";
    case 24: //ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT:
      return "ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT";
    case 25: //ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT:
      return "ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT";
    case 26: //ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
      return "ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT";
    case 27: //ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT:
      return "ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT";
#if (BLE_42_FEATURE_SUPPORT == TRUE)
    case 28: //ESP_GAP_BLE_UPDATE_DUPLICATE_EXCEPTIONAL_LIST_COMPLETE_EVT:
      return "ESP_GAP_BLE_UPDATE_DUPLICATE_EXCEPTIONAL_LIST_COMPLETE_EVT";
#endif  // #if (BLE_42_FEATURE_SUPPORT == TRUE)
    case 29: //ESP_GAP_BLE_SET_CHANNELS_EVT:
      return "ESP_GAP_BLE_SET_CHANNELS_EVT";
#if (BLE_50_FEATURE_SUPPORT == TRUE)
    case 30: //ESP_GAP_BLE_READ_PHY_COMPLETE_EVT:
      return "ESP_GAP_BLE_READ_PHY_COMPLETE_EVT";
    case 31: //ESP_GAP_BLE_SET_PREFERRED_DEFAULT_PHY_COMPLETE_EVT:
      return "ESP_GAP_BLE_SET_PREFERRED_DEFAULT_PHY_COMPLETE_EVT";
    case 32: //ESP_GAP_BLE_SET_PREFERRED_PHY_COMPLETE_EVT:
      return "ESP_GAP_BLE_SET_PREFERRED_PHY_COMPLETE_EVT";
    case 33: //ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT:
      return "ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT";
    case 34: //ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
      return "ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT";
    case 35: //ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
      return "ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT";
    case 36: //ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT:
      return "ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT";
    case 37: //ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
      return "ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT";
    case 38: //ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
      return "ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT";
    case 39: //ESP_GAP_BLE_EXT_ADV_SET_REMOVE_COMPLETE_EVT:
      return "ESP_GAP_BLE_EXT_ADV_SET_REMOVE_COMPLETE_EVT";
    case 40: //ESP_GAP_BLE_EXT_ADV_SET_CLEAR_COMPLETE_EVT:
      return "ESP_GAP_BLE_EXT_ADV_SET_CLEAR_COMPLETE_EVT";
    case 41: //ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT:
      return "ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT";
    case 42: //ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT:
      return "ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT";
    case 43: //ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT:
      return "ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT";
    case 44: //ESP_GAP_BLE_PERIODIC_ADV_STOP_COMPLETE_EVT:
      return "ESP_GAP_BLE_PERIODIC_ADV_STOP_COMPLETE_EVT";
    case 45: //ESP_GAP_BLE_PERIODIC_ADV_CREATE_SYNC_COMPLETE_EVT:
      return "ESP_GAP_BLE_PERIODIC_ADV_CREATE_SYNC_COMPLETE_EVT";
    case 46: //ESP_GAP_BLE_PERIODIC_ADV_SYNC_CANCEL_COMPLETE_EVT:
      return "ESP_GAP_BLE_PERIODIC_ADV_SYNC_CANCEL_COMPLETE_EVT";
    case 47: //ESP_GAP_BLE_PERIODIC_ADV_SYNC_TERMINATE_COMPLETE_EVT:
      return "ESP_GAP_BLE_PERIODIC_ADV_SYNC_TERMINATE_COMPLETE_EVT";
    case 48: //ESP_GAP_BLE_PERIODIC_ADV_ADD_DEV_COMPLETE_EVT:
      return "ESP_GAP_BLE_PERIODIC_ADV_ADD_DEV_COMPLETE_EVT";
    case 49: //ESP_GAP_BLE_PERIODIC_ADV_REMOVE_DEV_COMPLETE_EVT:
      return "ESP_GAP_BLE_PERIODIC_ADV_REMOVE_DEV_COMPLETE_EVT";
    case 50: //ESP_GAP_BLE_PERIODIC_ADV_CLEAR_DEV_COMPLETE_EVT:
      return "ESP_GAP_BLE_PERIODIC_ADV_CLEAR_DEV_COMPLETE_EVT";
//    case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
//      return "ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT";
//    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
//      return "ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT";
//    case ESP_GAP_BLE_EXT_SCAN_STOP_COMPLETE_EVT:
//      return "ESP_GAP_BLE_EXT_SCAN_STOP_COMPLETE_EVT";
//    case ESP_GAP_BLE_PREFER_EXT_CONN_PARAMS_SET_COMPLETE_EVT:
//      return "ESP_GAP_BLE_PREFER_EXT_CONN_PARAMS_SET_COMPLETE_EVT";
//    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
//      return "ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT";
//    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT:
//      return "ESP_GAP_BLE_EXT_ADV_REPORT_EVT";
//    case ESP_GAP_BLE_SCAN_TIMEOUT_EVT:
//      return "ESP_GAP_BLE_SCAN_TIMEOUT_EVT";
//    case ESP_GAP_BLE_ADV_TERMINATED_EVT:
//      return "ESP_GAP_BLE_ADV_TERMINATED_EVT";
//    case ESP_GAP_BLE_SCAN_REQ_RECEIVED_EVT:
//      return "ESP_GAP_BLE_SCAN_REQ_RECEIVED_EVT";
//    case ESP_GAP_BLE_CHANNEL_SELECT_ALGORITHM_EVT:
//      return "ESP_GAP_BLE_CHANNEL_SELECT_ALGORITHM_EVT";
//    case ESP_GAP_BLE_PERIODIC_ADV_REPORT_EVT:
//      return "ESP_GAP_BLE_PERIODIC_ADV_REPORT_EVT";
//    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_LOST_EVT:
//      return "ESP_GAP_BLE_PERIODIC_ADV_SYNC_LOST_EVT";
//    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_ESTAB_EVT:
//      return "ESP_GAP_BLE_PERIODIC_ADV_SYNC_ESTAB_EVT";
#endif  // #if (BLE_50_FEATURE_SUPPORT == TRUE)
    default:
      return "event unknown";
  };
}


void BLEClient::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  ESP_LOGI(TAG, "GOT EVENT %s", get_gap_event_type_string(event));

  switch (event) {
    // This event is sent by the server when it requests security
    case ESP_GAP_BLE_SEC_REQ_EVT:
      ESP_LOGV(TAG, "ESP_GAP_BLE_SEC_REQ_EVT %x", event);
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;
    // This event is sent once authentication has completed
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      esp_bd_addr_t bd_addr;
      memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
      ESP_LOGI(TAG, "auth complete. remote BD_ADDR: %s", format_hex(bd_addr, 6).c_str());
      if (!param->ble_security.auth_cmpl.success) {
        ESP_LOGE(TAG, "auth fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
      } else {
        ESP_LOGI(TAG, "auth success. address type = %d auth mode = %d", param->ble_security.auth_cmpl.addr_type,
                 param->ble_security.auth_cmpl.auth_mode);
      }
      break;
    // There are other events we'll want to implement at some point to support things like pass key
    // https://github.com/espressif/esp-idf/blob/cba69dd088344ed9d26739f04736ae7a37541b3a/examples/bluetooth/bluedroid/ble/gatt_security_client/tutorial/Gatt_Security_Client_Example_Walkthrough.md
    default:
      break;
  }
  for (auto *node : this->nodes_)
    node->gap_event_handler(event, param);
}

// Parse GATT values into a float for a sensor.
// Ref: https://www.bluetooth.com/specifications/assigned-numbers/format-types/
float BLEClient::parse_char_value(uint8_t *value, uint16_t length) {
  // A length of one means a single octet value.
  if (length == 0)
    return 0;
  if (length == 1)
    return (float) ((uint8_t) value[0]);

  switch (value[0]) {
    case 0x1:  // boolean.
    case 0x2:  // 2bit.
    case 0x3:  // nibble.
    case 0x4:  // uint8.
      return (float) ((uint8_t) value[1]);
    case 0x5:  // uint12.
    case 0x6:  // uint16.
      if (length > 2) {
        return (float) ((uint16_t)(value[1] << 8) + (uint16_t) value[2]);
      }
    case 0x7:  // uint24.
      if (length > 3) {
        return (float) ((uint32_t)(value[1] << 16) + (uint32_t)(value[2] << 8) + (uint32_t)(value[3]));
      }
    case 0x8:  // uint32.
      if (length > 4) {
        return (float) ((uint32_t)(value[1] << 24) + (uint32_t)(value[2] << 16) + (uint32_t)(value[3] << 8) +
                        (uint32_t)(value[4]));
      }
    case 0xC:  // int8.
      return (float) ((int8_t) value[1]);
    case 0xD:  // int12.
    case 0xE:  // int16.
      if (length > 2) {
        return (float) ((int16_t)(value[1] << 8) + (int16_t) value[2]);
      }
    case 0xF:  // int24.
      if (length > 3) {
        return (float) ((int32_t)(value[1] << 16) + (int32_t)(value[2] << 8) + (int32_t)(value[3]));
      }
    case 0x10:  // int32.
      if (length > 4) {
        return (float) ((int32_t)(value[1] << 24) + (int32_t)(value[2] << 16) + (int32_t)(value[3] << 8) +
                        (int32_t)(value[4]));
      }
  }
  ESP_LOGW(TAG, "Cannot parse characteristic value of type 0x%x length %d", value[0], length);
  return NAN;
}

BLEService *BLEClient::get_service(espbt::ESPBTUUID uuid) {
  for (auto *svc : this->services_) {
    if (svc->uuid == uuid)
      return svc;
  }
  return nullptr;
}

BLEService *BLEClient::get_service(uint16_t uuid) { return this->get_service(espbt::ESPBTUUID::from_uint16(uuid)); }

BLECharacteristic *BLEClient::get_characteristic(espbt::ESPBTUUID service, espbt::ESPBTUUID chr) {
  auto *svc = this->get_service(service);
  if (svc == nullptr)
    return nullptr;
  return svc->get_characteristic(chr);
}

BLECharacteristic *BLEClient::get_characteristic(uint16_t service, uint16_t chr) {
  return this->get_characteristic(espbt::ESPBTUUID::from_uint16(service), espbt::ESPBTUUID::from_uint16(chr));
}

BLEDescriptor *BLEClient::get_config_descriptor(uint16_t handle) {
  for (auto &svc : this->services_) {
    for (auto &chr : svc->characteristics) {
      if (chr->handle == handle) {
        for (auto &desc : chr->descriptors) {
          if (desc->uuid == espbt::ESPBTUUID::from_uint16(0x2902))
            return desc;
        }
      }
    }
  }
  return nullptr;
}

BLECharacteristic *BLEService::get_characteristic(espbt::ESPBTUUID uuid) {
  for (auto &chr : this->characteristics) {
    if (chr->uuid == uuid)
      return chr;
  }
  return nullptr;
}

BLECharacteristic *BLEService::get_characteristic(uint16_t uuid) {
  return this->get_characteristic(espbt::ESPBTUUID::from_uint16(uuid));
}

BLEDescriptor *BLEClient::get_descriptor(espbt::ESPBTUUID service, espbt::ESPBTUUID chr, espbt::ESPBTUUID descr) {
  auto *svc = this->get_service(service);
  if (svc == nullptr)
    return nullptr;
  auto *ch = svc->get_characteristic(chr);
  if (ch == nullptr)
    return nullptr;
  return ch->get_descriptor(descr);
}

BLEDescriptor *BLEClient::get_descriptor(uint16_t service, uint16_t chr, uint16_t descr) {
  return this->get_descriptor(espbt::ESPBTUUID::from_uint16(service), espbt::ESPBTUUID::from_uint16(chr),
                              espbt::ESPBTUUID::from_uint16(descr));
}

BLEService::~BLEService() {
  for (auto &chr : this->characteristics)
    delete chr;  // NOLINT(cppcoreguidelines-owning-memory)
}

void BLEService::parse_characteristics() {
  uint16_t offset = 0;
  esp_gattc_char_elem_t result;

  while (true) {
    uint16_t count = 1;
    esp_gatt_status_t status = esp_ble_gattc_get_all_char(
        this->client->gattc_if, this->client->conn_id, this->start_handle, this->end_handle, &result, &count, offset);
    if (status == ESP_GATT_INVALID_OFFSET || status == ESP_GATT_NOT_FOUND) {
      break;
    }
    if (status != ESP_GATT_OK) {
      ESP_LOGW(TAG, "esp_ble_gattc_get_all_char error, status=%d", status);
      break;
    }
    if (count == 0) {
      break;
    }

    BLECharacteristic *characteristic = new BLECharacteristic();  // NOLINT(cppcoreguidelines-owning-memory)
    characteristic->uuid = espbt::ESPBTUUID::from_uuid(result.uuid);
    characteristic->properties = result.properties;
    characteristic->handle = result.char_handle;
    characteristic->service = this;
    this->characteristics.push_back(characteristic);
    ESP_LOGI(TAG, " characteristic %s, handle 0x%x, properties 0x%x", characteristic->uuid.to_string().c_str(),
             characteristic->handle, characteristic->properties);
    characteristic->parse_descriptors();
    offset++;
  }
}

BLECharacteristic::~BLECharacteristic() {
  for (auto &desc : this->descriptors)
    delete desc;  // NOLINT(cppcoreguidelines-owning-memory)
}

void BLECharacteristic::parse_descriptors() {
  uint16_t offset = 0;
  esp_gattc_descr_elem_t result;

  while (true) {
    uint16_t count = 1;
    esp_gatt_status_t status = esp_ble_gattc_get_all_descr(
        this->service->client->gattc_if, this->service->client->conn_id, this->handle, &result, &count, offset);
    if (status == ESP_GATT_INVALID_OFFSET || status == ESP_GATT_NOT_FOUND) {
      break;
    }
    if (status != ESP_GATT_OK) {
      ESP_LOGW(TAG, "esp_ble_gattc_get_all_descr error, status=%d", status);
      break;
    }
    if (count == 0) {
      break;
    }

    BLEDescriptor *desc = new BLEDescriptor();  // NOLINT(cppcoreguidelines-owning-memory)
    desc->uuid = espbt::ESPBTUUID::from_uuid(result.uuid);
    desc->handle = result.handle;
    desc->characteristic = this;
    this->descriptors.push_back(desc);
    ESP_LOGV(TAG, "   descriptor %s, handle 0x%x", desc->uuid.to_string().c_str(), desc->handle);
    offset++;
  }
}

BLEDescriptor *BLECharacteristic::get_descriptor(espbt::ESPBTUUID uuid) {
  for (auto &desc : this->descriptors) {
    if (desc->uuid == uuid)
      return desc;
  }
  return nullptr;
}
BLEDescriptor *BLECharacteristic::get_descriptor(uint16_t uuid) {
  return this->get_descriptor(espbt::ESPBTUUID::from_uint16(uuid));
}

void BLECharacteristic::write_value(uint8_t *new_val, int16_t new_val_size, esp_gatt_write_type_t write_type) {
  auto *client = this->service->client;
  auto status = esp_ble_gattc_write_char(client->gattc_if, client->conn_id, this->handle, new_val_size, new_val,
                                         write_type, ESP_GATT_AUTH_REQ_NONE);
  if (status) {
    ESP_LOGW(TAG, "Error sending write value to BLE gattc server, status=%d", status);
  }
}

void BLECharacteristic::write_value(uint8_t *new_val, int16_t new_val_size) {
  write_value(new_val, new_val_size, ESP_GATT_WRITE_TYPE_NO_RSP);
}

}  // namespace ble_client
}  // namespace esphome

#endif
