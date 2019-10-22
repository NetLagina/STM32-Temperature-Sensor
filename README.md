# STM32-Temperature-Sensor
Temperature sensor with display

## Devices
* STM32F103C8T6
* Led Display 1602a
* Remote 8-bit I/O expander for I2C-bus with interrupt (Serial I2C LCD Display Adapter) PCF8574 or similiar
* Digital Humidity and Temperature Sensor DHT22 (AM2302)
* Noname button
* Breadbord
* 2 resistors 5-10k Ohms
* Resistor ~1k Ohms

## Connection
* 5V and Ground pins connected to Breadbord/DHT22/I2C LCD adapter
* 5V pin connected to Button's pin
* PA0 pin connected to another Button's pin from the same side
* PB6/PB7 (I2C SCL/SDA) pins connected to the corresponding I2C LCD adapter pins
* PC14 connected to DHT22 Data pin (2nd pin)
* 10k Ohms resistors connects SCL/SDA pins with 5V
* 1k Ohms resistor connects DHT22 Data pin with 5V
