#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <functional>

namespace esphome {
namespace energomera_ce {

constexpr size_t TARIFF_COUNT = 4;  // Number of tariffs
constexpr size_t MAX_RX_PACKET_SIZE = 128;
constexpr size_t MAX_TX_PACKET_SIZE = 64;  // Maximum size of a CE command packet

// Function type for processing received data
using ResponseProcessor = std::function<bool(const uint8_t *payload, size_t payload_len, uint8_t *msg, size_t msg_len)>;

struct InternalDataState {
  struct MeterInfo {
    uint32_t serial_number{0};
    uint16_t network_address{0};
    char serial_str[32]{0};
    bool ping_successful{false};
  } meter_info;

  float consumption[TARIFF_COUNT];
  float power;
  char time_str[9]{0};       // "23:59:59"
  char date_str[11]{0};      // "30/08/2023"
  char datetime_str[25]{0};  // "30/08/2023 23:59:59"

  uint32_t proper_reads{0};
  uint32_t read_errors{0};
  bool meter_found{false};
  uint8_t got{0};
  uint32_t last_good_read_ms{0};
};

enum class CEMeterModel : uint8_t {
  MODEL_UNKNOWN = 0,
  MODEL_CE102,
  MODEL_CE102_R51,
  MODEL_CE307_R33,
  MODEL_COUNT,
};

enum class CECmd : uint8_t {
  PING = 0,
  VERSION,
  SERIAL_NR,
  DATE_TIME,
  ENERGY_BY_TARIFF,
  POWER,
  CMD_COUNT,
};

class CEComponent : public PollingComponent, public uart::UARTDevice {
#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(network_address)
  SUB_TEXT_SENSOR(date)
  SUB_TEXT_SENSOR(time)
  SUB_TEXT_SENSOR(datetime)
  SUB_TEXT_SENSOR(serial_nr)
  SUB_TEXT_SENSOR(reading_state)
  SUB_TEXT_SENSOR(error_code)
  SUB_TEXT_SENSOR(about)
#endif
#ifdef USE_SENSOR
  sensor::Sensor *tariff_consumption_[TARIFF_COUNT] = {{nullptr}};
#endif

 public:
  CEComponent() = default;

  void set_tariff_sensor(uint8_t tariff, sensor::Sensor *sensor);

  void set_meter_model(CEMeterModel model) { this->meter_model_ = model; }
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  void set_receive_timeout(uint32_t receive_timeout) { this->receive_timeout_ = receive_timeout; }
  void set_requested_meter_address(uint16_t address) { this->requested_meter_address_ = address; }
  void set_password(uint32_t password) { this->password_ = password; }

  float get_setup_priority() const override;

  void dump_config() override;
  void setup() override;

  void loop() override;
  void update() override;

 protected:
  CEMeterModel meter_model_{CEMeterModel::MODEL_CE102_R51};
  GPIOPin *flow_control_pin_{nullptr};
  uint32_t receive_timeout_{2000};
  uint16_t requested_meter_address_{0xFFFF};
  uint32_t password_{0};

  InternalDataState data_{};

  // CE Protocol specific
  uint8_t tx_buffer_[MAX_TX_PACKET_SIZE];
  uint8_t rx_buffer_[MAX_RX_PACKET_SIZE];
  size_t rx_buffer_pos_{0};

  // Tariff reading counter (0-3 for tariffs 1-4)
  uint8_t current_tariff_{0};

  // CRC-8 table for CE protocol
  static const uint8_t crc8_table_[256];

  // Tracker for the current command and response
  struct {
    uint16_t current_cmd{};
    uint16_t expected_message_len{0};
    uint32_t start_time{0};
    uint16_t bytes_read{0};
    bool waiting_for_end{false};
    ResponseProcessor response_processor{};
    void reset() {
      current_cmd = 0;
      expected_message_len = 0;
      start_time = 0;
      bytes_read = 0;
      waiting_for_end = false;
      response_processor = nullptr;
    }
  } request_tracker_{};

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    WAITING_FOR_RESPONSE,
    PING_METER,
    GET_VERSION,
    GET_SERIAL_NR_0,
    GET_SERIAL_NR_1,
    GET_DATETIME,
    GET_ENERGY_TARIFFS,
    PUBLISH_INFO,
  } state_{State::NOT_INITIALIZED};

  State next_state_{State::IDLE};
  State last_reported_state_{State::NOT_INITIALIZED};

  void prepare_and_send_command(uint16_t cmd, const uint8_t *data, size_t data_len, size_t expected_message_len,
                                State next_state, ResponseProcessor processor);
  void send_ce_command(uint16_t addr, uint16_t cmd, const uint8_t *data = nullptr, size_t data_len = 0);
  bool process_response();
  bool process_received_data();

  const char *state_to_string(State state);
  void log_state_(State *next_state = nullptr);

  uint8_t crc8_ce(const uint8_t *buffer, size_t len);
  size_t build_ce_packet(uint8_t *buffer, uint16_t addr, uint16_t cmd, const uint8_t *data, size_t data_len);
  bool decode_ce_packet(uint8_t *buffer, size_t &buffer_len);
  void reverse_string_inplace(char *str);

  inline uint16_t get_command_for_meter(CECmd cmd);
  inline uint16_t get_request_size_for_meter(CECmd cmd);
  inline uint16_t get_response_size_for_meter(CECmd cmd);

  // Response processor functions
  ResponseProcessor get_ping_processor();
  ResponseProcessor get_version_processor();
  ResponseProcessor get_serial_processor();
  ResponseProcessor get_datetime_processor();
  ResponseProcessor get_energy_processor(uint8_t tariff_zero_based);
};

}  // namespace energomera_ce
}  // namespace esphome