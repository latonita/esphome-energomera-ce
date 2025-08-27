#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "energomera_ce.h"

#define STATE_BOOTUP_WAIT "Waiting to boot up"
#define STATE_METER_NOT_FOUND "Meter not found"
#define STATE_METER_FOUND "Meter found"
#define STATE_OK "OK"
#define STATE_PARTIAL_OK "Read partial data"
#define STATE_DATA_FAIL "Unable to read data"

#define MASK_GOT_PING 0b001
#define MASK_GOT_SERIAL 0b010
#define MASK_GOT_TARIFF 0b100
#define MASK_GOT_DATETIME 0b1000

namespace esphome {
namespace energomera_ce {

static const char *TAG = "Energomera-CE";

static constexpr uint32_t NO_GOOD_READS_TIMEOUT_MS = 5 * 60 * 1000;  // 5 minutes
static constexpr uint32_t RESPONSE_TIMEOUT_MS = 2 * 1000;            // 2 seconds
static constexpr uint32_t SETUP_DELAY_MS = 1 * 1000;                 // 1 second

// CE Protocol Constants
constexpr uint8_t CE_END = 0xC0;
constexpr uint8_t CE_ESC = 0xDB;
constexpr uint8_t CE_ESC_END = 0xDC;
constexpr uint8_t CE_ESC_ESC = 0xDD;

constexpr uint8_t CE_OPT = 0x48;
constexpr uint8_t CE_DIRECT_REQ = 0b10000000;
constexpr uint8_t CE_CLASS_ACCESS = 0b01010000;

// now for each model we need to have: for each request: its command code, bytes in, bytes out. array.
uint16_t CECommands[(size_t) CEMeterModel::MODEL_COUNT][(size_t) CECmd::CMD_COUNT][3] = {
    // PING          SERIAL           DATE_TIME      ENERGY_BY_TARIFF  POWER
    {{0x0001, 0, 2}, {0x011A, 1, 8}, {0x0000, 0, 0}, {0x0000, 0, 0}, {0x0000, 0, 0}},  // MODEL_UNKNOWN
    {{0x0001, 0, 2}, {0x011A, 1, 8}, {0x0120, 0, 7}, {0x0130, 2, 7}, {0x0132, 0, 3}},  // MODEL_CE102
    {{0x0001, 0, 2}, {0x011A, 1, 8}, {0x0120, 0, 7}, {0x0130, 2, 7}, {0x0000, 0, 0}},  // MODEL_CE307_R33
};

// CRC-8 table for CE protocol (from the library)
const uint8_t CEComponent::crc8_table_[256] = {
    0x0,  0xb5, 0xdf, 0x6a, 0xb,  0xbe, 0xd4, 0x61, 0x16, 0xa3, 0xc9, 0x7c, 0x1d, 0xa8, 0xc2, 0x77, 0x2c, 0x99, 0xf3,
    0x46, 0x27, 0x92, 0xf8, 0x4d, 0x3a, 0x8f, 0xe5, 0x50, 0x31, 0x84, 0xee, 0x5b, 0x58, 0xed, 0x87, 0x32, 0x53, 0xe6,
    0x8c, 0x39, 0x4e, 0xfb, 0x91, 0x24, 0x45, 0xf0, 0x9a, 0x2f, 0x74, 0xc1, 0xab, 0x1e, 0x7f, 0xca, 0xa0, 0x15, 0x62,
    0xd7, 0xbd, 0x8,  0x69, 0xdc, 0xb6, 0x3,  0xb0, 0x5,  0x6f, 0xda, 0xbb, 0xe,  0x64, 0xd1, 0xa6, 0x13, 0x79, 0xcc,
    0xad, 0x18, 0x72, 0xc7, 0x9c, 0x29, 0x43, 0xf6, 0x97, 0x22, 0x48, 0xfd, 0x8a, 0x3f, 0x55, 0xe0, 0x81, 0x34, 0x5e,
    0xeb, 0xe8, 0x5d, 0x37, 0x82, 0xe3, 0x56, 0x3c, 0x89, 0xfe, 0x4b, 0x21, 0x94, 0xf5, 0x40, 0x2a, 0x9f, 0xc4, 0x71,
    0x1b, 0xae, 0xcf, 0x7a, 0x10, 0xa5, 0xd2, 0x67, 0xd,  0xb8, 0xd9, 0x6c, 0x6,  0xb3, 0xd5, 0x60, 0xa,  0xbf, 0xde,
    0x6b, 0x1,  0xb4, 0xc3, 0x76, 0x1c, 0xa9, 0xc8, 0x7d, 0x17, 0xa2, 0xf9, 0x4c, 0x26, 0x93, 0xf2, 0x47, 0x2d, 0x98,
    0xef, 0x5a, 0x30, 0x85, 0xe4, 0x51, 0x3b, 0x8e, 0x8d, 0x38, 0x52, 0xe7, 0x86, 0x33, 0x59, 0xec, 0x9b, 0x2e, 0x44,
    0xf1, 0x90, 0x25, 0x4f, 0xfa, 0xa1, 0x14, 0x7e, 0xcb, 0xaa, 0x1f, 0x75, 0xc0, 0xb7, 0x2,  0x68, 0xdd, 0xbc, 0x9,
    0x63, 0xd6, 0x65, 0xd0, 0xba, 0xf,  0x6e, 0xdb, 0xb1, 0x4,  0x73, 0xc6, 0xac, 0x19, 0x78, 0xcd, 0xa7, 0x12, 0x49,
    0xfc, 0x96, 0x23, 0x42, 0xf7, 0x9d, 0x28, 0x5f, 0xea, 0x80, 0x35, 0x54, 0xe1, 0x8b, 0x3e, 0x3d, 0x88, 0xe2, 0x57,
    0x36, 0x83, 0xe9, 0x5c, 0x2b, 0x9e, 0xf4, 0x41, 0x20, 0x95, 0xff, 0x4a, 0x11, 0xa4, 0xce, 0x7b, 0x1a, 0xaf, 0xc5,
    0x70, 0x7,  0xb2, 0xd8, 0x6d, 0xc,  0xb9, 0xd3, 0x66};

float CEComponent::get_setup_priority() const { return setup_priority::AFTER_WIFI; }

void CEComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Energomera CE:");
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Meter address requested: %u", this->requested_meter_address_);
  ESP_LOGCONFIG(TAG, "  Receive timeout: %.1fs", this->receive_timeout_ / 1e3f);
  ESP_LOGCONFIG(TAG, "  Update interval: %.1fs", this->update_interval_ / 1e3f);
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  Password: 0x%08X", this->password_);
  ESP_LOGCONFIG(TAG, "Data errors %d, proper reads %d", this->data_.read_errors, this->data_.proper_reads);
}

void CEComponent::set_tariff_sensor(uint8_t tariff, sensor::Sensor *sensor) {
  if (tariff >= TARIFF_COUNT) {
    ESP_LOGE(TAG, "Invalid tariff %d", tariff);
    return;
  }
  this->tariff_consumption_[tariff] = sensor;
}

void CEComponent::setup() {
  if (this->reading_state_text_sensor_ != nullptr) {
    this->reading_state_text_sensor_->publish_state(STATE_BOOTUP_WAIT);
  }

  // Configure flow control pin if specified
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
    this->flow_control_pin_->digital_write(false);  // Set to receive mode
  }

  this->set_timeout(SETUP_DELAY_MS, [this]() { this->state_ = State::IDLE; });
}

uint16_t CEComponent::get_command_for_meter(CECmd cmd) {
  return CECommands[(size_t) this->meter_model_][(size_t) cmd][0];
}

uint16_t CEComponent::get_request_size_for_meter(CECmd cmd) {
  return CECommands[(size_t) this->meter_model_][(size_t) cmd][1];
}

uint16_t CEComponent::get_response_size_for_meter(CECmd cmd) {
  return CECommands[(size_t) this->meter_model_][(size_t) cmd][2];
}

void CEComponent::loop() {
  if (!this->is_ready())
    return;

  switch (this->state_) {
    case State::NOT_INITIALIZED: {
      this->log_state_();
    } break;

    case State::IDLE: {
      this->log_state_();
      uint32_t now = millis();
      if (now - this->data_.last_good_read_ms > NO_GOOD_READS_TIMEOUT_MS) {
        ESP_LOGE(TAG, "Rebooting due to no good reads from the meter for 5 minutes...");
        delay(1000);
        yield();  // Give time to log the message
        App.reboot();
      }
    } break;

    case State::WAITING_FOR_RESPONSE: {
      this->log_state_(&this->next_state_);
      if (!process_response()) {
        // Check timeout
        uint32_t now = millis();
        if (now - this->request_tracker_.start_time > RESPONSE_TIMEOUT_MS) {
          ESP_LOGW(TAG, "Response timeout for command 0x%04X",
                   static_cast<uint16_t>(this->request_tracker_.current_cmd));
          this->data_.read_errors++;
          this->state_ = this->next_state_;  // State::IDLE;
          this->request_tracker_.reset();
        }
      }
    } break;

    case State::PING_METER: {
      this->log_state_();
      uint8_t ping_data = 0;

      auto ping_processor = [this](const uint8_t *payload, size_t payload_len) -> bool {
        ESP_LOGI(TAG, "Ping response received");
        this->data_.meter_found = true;

        size_t offset = 8;
        if (payload_len >= offset + 2) {
          this->data_.meter_info.network_address = payload[offset] | (payload[offset + 1] << 8);
        }

        this->data_.got |= MASK_GOT_PING;
        return true;
      };

      prepare_and_send_command(get_command_for_meter(CECmd::PING), &ping_data, 0,
                               get_request_size_for_meter(CECmd::PING), State::GET_SERIAL_NR, ping_processor);
    } break;

    case State::GET_SERIAL_NR: {
      this->log_state_();
      // Prepare data for SERIAL_NR command
      uint8_t serial_data = 0;

      auto serial_processor = [this](const uint8_t *payload, size_t payload_len) -> bool {
        ESP_LOGI(TAG, "Serial number response received");

        // TODO: Process serial number data when format is known
        // size_t PAL = 6;
        // if (payload_len >= PAL + 8) {
        //   // Process serial number from payload[PAL] to payload[PAL+7]
        // }

        this->data_.got |= MASK_GOT_SERIAL;
        return true;
      };

      prepare_and_send_command(get_command_for_meter(CECmd::SERIAL_NR), &serial_data, 1,
                               get_request_size_for_meter(CECmd::SERIAL_NR), State::GET_DATETIME, serial_processor);
    } break;

    case State::GET_DATETIME: {
      this->log_state_();
      // Prepare data for DATE_TIME command

      auto datetime_processor = [this](const uint8_t *payload, size_t payload_len) -> bool {
        ESP_LOGI(TAG, "DateTime response received");

        // TODO: Process datetime data when format is known
        // size_t PAL = 6;
        // if (payload_len >= PAL + 7) {
        //   // Process datetime from payload[PAL] to payload[PAL+6]
        // }

        this->data_.got |= MASK_GOT_DATETIME;
        return true;
      };

      prepare_and_send_command(get_command_for_meter(CECmd::DATE_TIME), nullptr, 0,
                               get_request_size_for_meter(CECmd::DATE_TIME), State::GET_ENERGY_TARIFFS,
                               datetime_processor);
    } break;

    case State::GET_ENERGY_TARIFFS: {
      this->log_state_();

      // Check if we've read all tariffs
      if (this->current_tariff_ >= TARIFF_COUNT) {
        this->state_ = State::PUBLISH_INFO;
        break;
      }

      // Prepare data for ENERGY_BY_TARIFF command for current tariff
      size_t data_len = 2;
      uint8_t energy_data[2];
      switch (this->meter_model_) {
        case CEMeterModel::MODEL_CE102:
          energy_data[0] = 0;
          energy_data[1] = this->current_tariff_ + 1;  // 1-based tariff number
          break;
        case CEMeterModel::MODEL_CE307:
          energy_data[0] = this->current_tariff_;  // CE307 uses 0-based indexing
          energy_data[1] = 0;
          break;
        default:
          ESP_LOGW(TAG, "Unsupported meter model for tariff reading");
          this->state_ = State::PUBLISH_INFO;
          break;
      }

      uint8_t current_tariff_for_lambda = this->current_tariff_;  // Capture for lambda

      auto energy_processor = [this, current_tariff_for_lambda](const uint8_t *payload, size_t payload_len) -> bool {
        ESP_LOGI(TAG, "Energy T%d response received, p 0x%p, len %d, expected data block %d",
                 current_tariff_for_lambda + 1, payload, payload_len, this->request_tracker_.expected_data_size);

        size_t offset = 8;
        size_t resp_len = this->request_tracker_.expected_data_size;
        if (payload_len >= 9 + resp_len) {
          uint32_t value = 0;

          if (resp_len == 7) {
            offset += 3;
          }

          value =
              payload[offset] | (payload[offset + 1] << 8) | (payload[offset + 2] << 16) | (payload[offset + 3] << 24);
          ESP_LOGI(TAG, "Raw value: %u", value);
          this->data_.consumption[current_tariff_for_lambda] = value * 10;  // Convert to kWh
          if (current_tariff_for_lambda == 0) {
            this->data_.got |= MASK_GOT_TARIFF;  // Set flag only once for all tariffs
          }

          return true;
        } else {
          ESP_LOGW(TAG, "Tariff T%d response too short: %d bytes", current_tariff_for_lambda + 1, payload_len);
          return false;
        }
      };

      // Increment tariff counter for next iteration
      this->current_tariff_++;

      // Determine next state - stay in same state to read next tariff, or move to publish
      State next_state = (this->current_tariff_ < TARIFF_COUNT) ? State::GET_ENERGY_TARIFFS : State::PUBLISH_INFO;

      prepare_and_send_command(get_command_for_meter(CECmd::ENERGY_BY_TARIFF), energy_data, data_len,
                               get_response_size_for_meter(CECmd::ENERGY_BY_TARIFF), next_state, energy_processor);
    } break;

    case State::PUBLISH_INFO: {
      this->log_state_();
      this->state_ = State::IDLE;
      ESP_LOGD(TAG, "Data errors %d, proper reads %d", this->data_.read_errors, this->data_.proper_reads);

      if (!this->data_.meter_found) {
        if (this->reading_state_text_sensor_ != nullptr) {
          this->reading_state_text_sensor_->publish_state(STATE_METER_NOT_FOUND);
        }
        return;
      }

      uint8_t expected_mask = MASK_GOT_PING | MASK_GOT_SERIAL | MASK_GOT_TARIFF;
      if (this->data_.got == expected_mask) {
        if (this->reading_state_text_sensor_ != nullptr) {
          this->reading_state_text_sensor_->publish_state(STATE_OK);
        }
      } else {
        ESP_LOGW(TAG, "Got no or partial data %o", this->data_.got);
        if (this->reading_state_text_sensor_ != nullptr) {
          this->reading_state_text_sensor_->publish_state((this->data_.got == 0) ? STATE_DATA_FAIL : STATE_PARTIAL_OK);
        }
      }

      if (this->data_.got) {
        this->data_.last_good_read_ms = millis();
      }

      // Publish meter info
      if (this->network_address_text_sensor_ != nullptr) {
        this->network_address_text_sensor_->publish_state(this->data_.meter_info.network_address_str);
      }
      if (this->serial_nr_text_sensor_ != nullptr) {
        this->serial_nr_text_sensor_->publish_state(this->data_.meter_info.serial_str);
      }

      // Publish tariff consumption data
      if (this->data_.got & MASK_GOT_TARIFF) {
        for (uint8_t tariff = 0; tariff < TARIFF_COUNT; ++tariff) {
          if (this->tariff_consumption_[tariff] != nullptr) {
            this->tariff_consumption_[tariff]->publish_state(this->data_.consumption[tariff]);
          }
        }
      }
      if (this->data_.got & MASK_GOT_DATETIME) {
        if (this->date_text_sensor_ != nullptr) {
          this->date_text_sensor_->publish_state(this->data_.date_str);
        }
        if (this->time_text_sensor_ != nullptr) {
          this->time_text_sensor_->publish_state(this->data_.time_str);
        }
        if (this->datetime_text_sensor_ != nullptr) {
          this->datetime_text_sensor_->publish_state(this->data_.datetime_str);
        }
      }
    } break;

  }  // switch
}

void CEComponent::update() {
  if (this->state_ == State::IDLE) {
    ESP_LOGV(TAG, "Starting meter communication cycle");
    this->data_.got = 0;  // Reset data collection flags
    this->current_tariff_ = 0;
    this->state_ = State::PING_METER;

  } else {
    ESP_LOGW(TAG, "Skipping update - component busy in state %s", this->state_to_string(this->state_));
  }
}

void CEComponent::prepare_and_send_command(uint16_t cmd, const uint8_t *data, size_t data_len,
                                           size_t expected_data_size, State next_state, ResponseProcessor processor) {
  ESP_LOGV(TAG, "Starting async request for command %d", static_cast<uint8_t>(cmd));

  if (cmd == 0) {
    ESP_LOGV(TAG, " - No command to send");
    this->state_ = next_state;
    return;
  }

  // Fill request tracker
  this->request_tracker_.current_cmd = cmd;
  this->request_tracker_.start_time = millis();
  this->request_tracker_.bytes_read = 0;
  this->request_tracker_.waiting_for_end = false;
  this->request_tracker_.expected_data_size = expected_data_size;
  this->request_tracker_.response_processor = processor;
  this->rx_buffer_pos_ = 0;

  // Set next state and current state
  this->next_state_ = next_state;
  this->state_ = State::WAITING_FOR_RESPONSE;

  // Send the command with prepared data
  send_ce_command(this->requested_meter_address_, cmd, data, data_len);
}

bool CEComponent::process_response() {
  // Read available data
  while (this->available()) {
    uint8_t byte = this->read();

    if (!this->request_tracker_.waiting_for_end) {
      // Looking for start byte
      if (byte == CE_END) {
        this->request_tracker_.waiting_for_end = true;
        this->rx_buffer_pos_ = 0;
        continue;
      }
    } else {
      // Collecting data until end byte
      if (byte == CE_END) {
        // End of packet found
        ESP_LOGV(TAG, "Received complete CE packet, size: %d", this->rx_buffer_pos_);
        if (this->rx_buffer_pos_ > 0) {
          if (process_received_data()) {
            this->state_ = this->next_state_;
            return true;
          }
        }
        this->data_.read_errors++;
        this->state_ = State::IDLE;
        this->request_tracker_.reset();
        return false;
      } else {
        if (this->rx_buffer_pos_ < MAX_RX_PACKET_SIZE - 1) {
          this->rx_buffer_[this->rx_buffer_pos_++] = byte;
        } else {
          ESP_LOGW(TAG, "RX buffer overflow");
          this->data_.read_errors++;
          this->state_ = State::IDLE;
          this->request_tracker_.reset();
          return false;
        }
      }
    }
  }

  return false;  // Still waiting for more data
}

bool CEComponent::process_received_data() {
  if (this->rx_buffer_pos_ == 0) {
    ESP_LOGW(TAG, "Empty response received");
    return false;
  }
  ESP_LOGVV(TAG, "Raw RX data (SLIPPED): %s", format_hex_pretty(this->rx_buffer_, this->rx_buffer_pos_).c_str());

  // Decode SLIP-like encoding
  if (!decode_ce_packet(this->rx_buffer_, this->rx_buffer_pos_)) {
    ESP_LOGE(TAG, "Failed to decode CE packet");
    return false;
  }

  ESP_LOGV(TAG, "Processing command response: 0x%04X", static_cast<uint16_t>(this->request_tracker_.current_cmd));
  ESP_LOGVV(TAG, "Decoded payload: %s", format_hex_pretty(this->rx_buffer_, this->rx_buffer_pos_).c_str());

  // expected length
  size_t expected_len =
      9 + this->request_tracker_.expected_data_size;  // OPT(1)+ADDR(2)+SRC(2)+PASSWD(4)+SERV(1)+CMD(2)+DATA+CRC(1)
  if (expected_len != this->rx_buffer_pos_) {
    ESP_LOGE(TAG, "Wrong expected length: %d, Received length: %d", expected_len, this->rx_buffer_pos_);
    return false;
  }

  ESP_LOGV(TAG, "Expected length: %d, Received length: %d", expected_len, this->rx_buffer_pos_);

  this->data_.proper_reads++;

  // Use the lambda function to process the response
  if (this->request_tracker_.response_processor) {
    return this->request_tracker_.response_processor(this->rx_buffer_, this->rx_buffer_pos_);
  } else {
    ESP_LOGE(TAG, "No response processor available for command 0x%04X",
             static_cast<uint16_t>(this->request_tracker_.current_cmd));
    return false;
  }
}

void CEComponent::send_ce_command(uint16_t addr, uint16_t cmd, const uint8_t *data, size_t data_len) {
  ESP_LOGV(TAG, "Sending CE command 0x%04X to address %u, data_len %d", cmd, addr, data_len);

  // Build the complete packet in the static buffer
  size_t packet_len = build_ce_packet(this->tx_buffer_, addr, cmd, data, data_len);

  if (packet_len == 0) {
    ESP_LOGE(TAG, "Failed to build CE packet");
    return;
  }

  ESP_LOGVV(TAG, "TX packet (SLIPPED): %s", format_hex_pretty(this->tx_buffer_, packet_len).c_str());

  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(true);
  }

  this->write_array(this->tx_buffer_, packet_len);
  this->flush();

  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(false);
  }
}

size_t CEComponent::build_ce_packet(uint8_t *buffer, uint16_t addr, uint16_t cmd, const uint8_t *data,
                                    size_t data_len) {
  if (buffer == nullptr) {
    return 0;
  }

  // ============================================================================
  // BLOCK 1: Construct the raw payload (without CRC)
  // ============================================================================
  uint8_t payload[MAX_TX_PACKET_SIZE];
  size_t payload_len = 0;

  // Add OPT byte
  payload[payload_len++] = CE_OPT;

  // Add destination address (low byte first)
  uint8_t addr_l = addr & 0xFF;
  uint8_t addr_h = (addr >> 8) & 0xFF;
  payload[payload_len++] = addr_l;
  payload[payload_len++] = addr_h;

  // Add source address (0)
  payload[payload_len++] = 253;
  payload[payload_len++] = 0;

  // Add password (4 bytes)
  for (int i = 0; i < 4; i++) {
    uint8_t passwd_byte = (this->password_ >> (i * 8)) & 0xFF;
    payload[payload_len++] = passwd_byte;
  }

  // Add service byte
  uint8_t serv = CE_DIRECT_REQ | CE_CLASS_ACCESS | data_len;
  payload[payload_len++] = serv;

  // Add command (high byte first)
  uint16_t cmd_val = static_cast<uint16_t>(cmd);
  uint8_t cmd_h = (cmd_val >> 8) & 0xFF;
  uint8_t cmd_l = cmd_val & 0xFF;
  payload[payload_len++] = cmd_h;
  payload[payload_len++] = cmd_l;

  // Add data if any
  if (data != nullptr && data_len > 0) {
    for (size_t i = 0; i < data_len; i++) {
      payload[payload_len++] = data[i];
    }
  }

  // Calculate and add CRC for the entire payload
  uint8_t crc8 = crc8_ce(payload, payload_len);
  payload[payload_len++] = crc8;
  ESP_LOGVV(TAG, "TX payload: %s", format_hex_pretty(payload, payload_len).c_str());
  // ============================================================================
  // BLOCK 2: Construct the final output buffer with SLIP encoding
  // ============================================================================
  size_t out_pos = 0;

  // Start with END byte (frame boundary)
  buffer[out_pos++] = CE_END;

  // Encode the payload with escape sequences
  for (size_t payload_idx = 0; payload_idx < payload_len; payload_idx++) {
    uint8_t byte = payload[payload_idx];

    if (byte == CE_END) {
      // Escape the END character
      if (out_pos + 1 >= MAX_TX_PACKET_SIZE)
        return 0;  // Buffer overflow check
      buffer[out_pos++] = CE_ESC;
      buffer[out_pos++] = CE_ESC_END;
    } else if (byte == CE_ESC) {
      // Escape the ESC character
      if (out_pos + 1 >= MAX_TX_PACKET_SIZE)
        return 0;  // Buffer overflow check
      buffer[out_pos++] = CE_ESC;
      buffer[out_pos++] = CE_ESC_ESC;
    } else {
      // Regular byte - copy as is
      if (out_pos >= MAX_TX_PACKET_SIZE)
        return 0;  // Buffer overflow check
      buffer[out_pos++] = byte;
    }
  }

  // End with END byte (frame boundary)
  if (out_pos >= MAX_TX_PACKET_SIZE)
    return 0;  // Buffer overflow check
  buffer[out_pos++] = CE_END;

  return out_pos;
}

bool CEComponent::decode_ce_packet(uint8_t *buffer, size_t &buffer_len) {
  size_t write_pos = 0;

  for (size_t read_pos = 0; read_pos < buffer_len; read_pos++) {
    if (buffer[read_pos] == CE_ESC && read_pos + 1 < buffer_len) {
      if (buffer[read_pos + 1] == CE_ESC_END) {
        buffer[write_pos++] = CE_END;
        read_pos++;  // Skip next byte
      } else if (buffer[read_pos + 1] == CE_ESC_ESC) {
        buffer[write_pos++] = CE_ESC;
        read_pos++;  // Skip next byte
      } else {
        buffer[write_pos++] = buffer[read_pos];
      }
    } else {
      buffer[write_pos++] = buffer[read_pos];
    }
  }

  buffer_len = write_pos;
  return true;
}

uint8_t CEComponent::crc8_ce(const uint8_t *buffer, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc = crc8_table_[crc ^ buffer[i]];
  }
  return crc;
}

const char *CEComponent::state_to_string(State state) {
  switch (state) {
    case State::NOT_INITIALIZED:
      return "NOT_INITIALIZED";
    case State::IDLE:
      return "IDLE";
    case State::WAITING_FOR_RESPONSE:
      return "WAITING_FOR_RESPONSE";
    case State::PING_METER:
      return "PING_METER";
    case State::GET_SERIAL_NR:
      return "GET_SERIAL_NR";
    case State::GET_DATETIME:
      return "GET_DATETIME";
    case State::GET_ENERGY_TARIFFS:
      return "GET_ENERGY_TARIFFS";
    case State::PUBLISH_INFO:
      return "PUBLISH_INFO";
    default:
      return "UNKNOWN_STATE";
  }
}

void CEComponent::log_state_(State *next_state) {
  if (this->state_ != this->last_reported_state_) {
    if (next_state == nullptr) {
      ESP_LOGV(TAG, "State::%s", this->state_to_string(this->state_));
    } else {
      ESP_LOGV(TAG, "State::%s -> %s", this->state_to_string(this->state_), this->state_to_string(*next_state));
    }
    this->last_reported_state_ = this->state_;
  }
}

}  // namespace energomera_ce
}  // namespace esphome