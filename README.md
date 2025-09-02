# Считывание данных с электросчетчиков Энергомера по протоколу СЕ
Подключение через Оптопорт или RS-485. Настройки порта: 9600,8n1.

>В процессе разработки. Необходимо тестирование на приборах.

# Получаемые данные
- Энергия в Вт*ч по 4 тарифам
- Сетевой адрес
- Серийный номер
- Дата/Время

# Модели ПУ

| Модель     | Версия |  Какой тип надо указать в конфиге | |
|------------|--------|--------------------------|--|
| CE102 R5   | V1-V5  | CE102 |
| СЕ102 R5.1 |        | CE102_R51 |
| CE102 R8   | V1-V6  | CE102 |
| CE102 R8Q  | V1-V2  | CE102 |
| CE102 S6   | V1-V4  | CE102 |
| CE102 S7   | V1-V6  | CE102 |
| CE102 S7J  | V5-V6  | CE102 |
| CE102 S7J  | V10    | CE102 |
| CE301M R33 | V1     | CE102 |
| CE301M S31 | V1     | CE102 |
| CE306 R33  | V10    | CE102 |
| CE306 S31  | V10    | CE102 |
| CE307 R33  | V1-V3  | CE307_R33 |
| CE307 R34  |        | нет | см (esphome-dlms-cosem)[http://github.com/latonita/esphome-dlms-cosem] или (esphome-energomera-iec)[http://github.com/latonita/esphome-energomera-iec] |


# Пример конфигурации

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

uart:
  id: uart_bus
  tx_pin: GPIO12
  rx_pin: GPIO14
  baud_rate: 9600
  data_bits: 8
  parity: NONE
  stop_bits: 1

energomera_ce:
  id: ce_meter
  model: CE307_R33    # UNKNOWN / CE102 / CE102_R51 / CE307_R33
  address: 1234       # последние 5 цифр серийного номера, но если число получается 
                      # больше 65535, то - последние 4 цифры
  password: 777777
  update_interval: 30s
  #flow_control_pin: GPIO4  # for rs485 with DE/RE

sensor:
  - platform: energomera_ce
    energy_total: "Energy Total"
    energy_t1: "Energy T1"
    energy_t2: "Energy T2"

    voltage_a: "Voltage A"
    voltage_b: "Voltage B"
    voltage_c: "Voltage C"

    current_a: "Irms A"
    current_b: "Irms B"
    current_c: "Irms C"

    power_a: "Power A"
    power_b: "Power B"
    power_c: "Power C"
    

text_sensor:
  - platform: energomera_ce
    serial_nr: "Meter Serial Number"
    network_address: "Meter Network Address"
    reading_state: "Reading State"
    date: "Meter Date"    
    time: "Meter Time"
    datetime: "Meter DateTime"


wifi:
  ssid: "TestWiFi"
  password: "testpassword"

logger:
  level: DEBUG

api:

ota:
  platform: esphome

    time: "Meter Time"
    datetime: "Meter DateTime"

```

Для однофазных
```<yaml>


sensor:
  - platform: energomera_ce
    energy_total: "Energy Total"
    energy_t1: "Energy T1"
    energy_t2: "Energy T2"

    voltage: "Voltage"
    current: "Irms"
    power: "Power"

    ...

```
