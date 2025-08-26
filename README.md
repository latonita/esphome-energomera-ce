Example config:


```<yaml>

esphome:
  name: energomera-ce-307

esp32:
  board: esp32dev
  framework:
    type: arduino

external_components:
  - source: github://latonita/esphome-energomera-ce
    refresh: 30s
    components: [energomera_ce]

wifi:
  ssid: "TestWiFi"
  password: "testpassword"

logger:
  level: DEBUG

api:

ota:
  platform: esphome

uart:
  id: uart_bus
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600
  parity: EVEN
  stop_bits: 1

energomera_ce:
  id: ce_meter
  model: CE307
  flow_control_pin: GPIO4
  address: 1
  password: 1234
  update_interval: 30s

sensor:
  - platform: energomera_ce
    tariff_1: "Energy T1"
    tariff_2: "Energy T2"

text_sensor:
  - platform: energomera_ce
    serial_nr: "Meter Serial Number"
    network_address: "Meter Network Address"
    reading_state: "Reading State"
    date: "Meter Date"
    time: "Meter Time"
    datetime: "Meter DateTime"

```
