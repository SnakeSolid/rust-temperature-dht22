## Bluetooth Temperature and Humidity Sensor

Once per minute read temperature and humidity from DHT22 sensor and send values to Bluetooth channel. Sensor uses CSV format with `;` as field separator and `\n\` as row separator. First row contains header `CPU;Humidity;Temperature`.

* `CPU` - internal temperature sensor, only for reference;
* `Humidity` - DHT22 humidity value with one decimal sign;
* `Temperature` - DHT22 temperature value with one decimal sign;

If DHT22 data contains incorrect checksum columns `Humidity` and `Temperature` will be empty.

### Cargo Configuration

By default cargo target set tot `thumbv7m-none-eabi` in local configuration file, see `.cargo/config`.

### Software Requirements

* `cargo-binutils` - cargo subcommand to create firmware;
* `stlink-tools` - flashing tools to write firmware to microcontroller;

### Hardware Requirements

* Blue pill development board (use STM32F103C8T6 microcontroller), can be replaced with aly STM microcontroller, probably with changing pins and clocks;
* DHT22 temperature and humidity sensor (DHT11 also can be used);
* HC-06 Bluetooth module (can be replaced with any bluetooth serial module);

### Build and Write flash

Use following commands to build firmware and write it to blue pill:

```
cargo objcopy --release -- -O binary "temperature-dht22.bin" # Build firmware
st-flash write "temperature-dht22.bin" 0x08000000 # Write firmware to blue pill
```

### Connections

PA0 - DHT22 data pin;
PB10 - HC-06 module Tx pin;
PB11 - HC-06 module Rx pin;

Pin PC13 (green LED) will be set to high on start to turned off LED. LED will blink after each measurement (once per minute).
