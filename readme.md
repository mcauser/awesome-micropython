# Awesome MicroPython [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

<a href="http://www.micropython.org/"><img src="https://raw.githubusercontent.com/mcauser/awesome-micropython/master/logo.svg?sanitize=true" alt="Awesome MicroPython" width="150" align="left"></a>
<br>

> [MicroPython](http://micropython.org/) is a lean and efficient implementation of the Python 3 programming language that includes a small subset of the Python standard library and is optimised to run on microcontrollers and in constrained environments.

<br>

A curated list of awesome MicroPython libraries, frameworks, software and resources.

Inspired by the [Awesome lists](https://github.com/sindresorhus/awesome).

## Contents

- [Libraries](#libraries)
	- [Audio](#audio)
	- [Communications](#communications)
		- [Bluetooth](#bluetooth)
		- [Ethernet](#ethernet)
		- [GPS](#gps)
		- [IR](#ir)
		- [OneWire](#onewire)
		- [Radio](#radio)
		- [RFID](#rfid)
		- [RTC](#rtc)
		- [WiFi](#wifi)
	- [Display](#display)
		- [E-Paper](#e-paper)
		- [LCD Character](#lcd-character)
		- [LCD Graphic](#lcd-graphic)
		- [LCD TFT](#lcd-tft)
		- [LED Matrix](#led-matrix)
		- [LED Segment](#led-segment)
		- [LEDs](#leds)
		- [OLED](#oled)
	- [IO](#io)
		- [ADC](#adc)
		- [DAC](#dac)
		- [IO-Expander](#io-expander)
		- [Joystick](#joystick)
	- [Motion](#motion)
		- [DC Motor](#dc-motor)
		- [Servo](#servo)
		- [Stepper](#stepper)
	- [Sensors](#sensors)
		- [Accelerometer Digital](#accelerometer-digital)
		- [Air Quality](#air-quality)
		- [Barometer](#barometer)
		- [Compass](#compass)
		- [Current](#current)
		- [Distance IR](#distance-ir)
		- [Distance Ultrasonic](#distance-ultrasonic)
		- [Light](#light)
		- [Motion Inertial](#motion-inertial)
		- [Temperature Analog](#temperature-analog)
		- [Temperature Digital](#temperature-digital)
		- [Temperature IR](#temperature-ir)
		- [Touch Capacitive](#touch-capacitive)
		- [Touch Resistive](#touch-resistive)
- [Community](#community)
- [Books](#books)
- [Resources](#resources)
- [Miscellaneous](#miscellaneous)
- [Contributing](#contributing)

## Libraries

### Audio

* [JQ6500](https://github.com/rdagger/micropython-jq6500) - Driver for JQ6500 UART MP3 modules.
* [KT403A-MP3](https://github.com/jczic/KT403A-MP3) - Driver for KT403A, used by DFPlayer Mini and Grove MP3 v2.0.
* [micropython-buzzer](https://github.com/fruch/micropython-buzzer) - Play nokia compose and mid files on buzzers.
* [micropython-dfplayer](https://github.com/ShrimpingIt/micropython-dfplayer) - Driver for DFPlayer Mini using UART.

### Communications

#### Bluetooth

* [PyBoard-HC05-Android](https://github.com/KipCrossing/PyBoard-HC05-Android) - Pyboard HC05 Bluetooth adaptor example application.

#### Ethernet

* [Official wiznet5k](https://github.com/micropython/micropython/tree/master/drivers/wiznet5k) - Official driver for the WIZnet5x00 series of Ethernet controllers.

#### GPS

* [micropyGPS](https://github.com/inmcm/micropyGPS) - Full featured GPS NMEA sentence parser.

#### IR

* [micropython-necir](https://github.com/MattMatic/micropython-necir) - NEC infrared capture for TL1838 IR receiver LEDs.
* [Micropython-IR](https://github.com/designerPing/Micropython-IR) - Pyboard infrared remote sniff and replay.

#### OneWire

* [Official OneWire](https://github.com/micropython/micropython/tree/master/drivers/onewire) - For devices using the OneWire bus, eg Dallas ds18x20.

#### Radio

* [micropython-radio](https://github.com/peterhinch/micropython-radio) - Protocols for nRF24L01 2.4Ghz radio modules.
* [Official nRF24L01](https://github.com/micropython/micropython/tree/master/drivers/nrf24l01) - Official driver for nRF24L01 2.4Ghz radio modules.

#### RFID

* [micropython-mfrc522](https://github.com/wendlers/micropython-mfrc522) - Driver for NXP MFRC522 RFID reader/writer.

#### RTC

* [micropython-tinyrtc-i2c](https://github.com/mcauser/micropython-tinyrtc-i2c) - Driver for DS1307 RTC and AT24C32N EEPROM.
* [Micropython_TinyRTC](https://github.com/AnthonyKNorman/Micropython_TinyRTC) - Driver for DS1307 RTC.

#### WiFi

* [HueBridge](https://github.com/FRC4564/HueBridge) - Philips Hue Bridge.

### Display

#### E-Paper

* [micropython-epaper](https://github.com/peterhinch/micropython-epaper) - Pyboard driver for Embedded Artists 2.7 inch e-paper display.

#### LCD Character

* [Grove_RGB_LCD](https://github.com/dda/MicroPython/blob/master/Grove_RGB_LCD.py) - Driver for SeeedStudio's Grove RGB LCD.
* [micropython-i2c-lcd](https://github.com/Bucknalla/micropython-i2c-lcd) - Driver for I2C 2x16 LCD Screens.
* [python_lcd](https://github.com/dhylands/python_lcd) - Driver for HD44780 compatible dot matrix LCDs.

#### LCD Graphic

* [micropython-lcd-AQM1248A](https://github.com/forester3/micropython-lcd-AQM1248A) - ESP8266 driver for AQM1248A graphic LCD.
* [micropython-pcd8544](https://github.com/mcauser/micropython-pcd8544) - Driver for Nokia 5110 PCD8544 84x48 LCD modules.
* [micropython-st7920](https://github.com/ShrimpingIt/micropython-st7920) - Library for simple graphic primitives on ST7920 128x64 monochrome LCD panel using ESP8266 and SPI.
* [MicroPython_PCD8544](https://github.com/AnthonyKNorman/MicroPython_PCD8544) - ESP8266 driver for Nokia 5110 PCD8544.
* [Official LCD160CR](https://github.com/micropython/micropython/tree/master/drivers/display) - Driver for official MicroPython LCD160CR display with resistive touch sensor.

#### LCD TFT

* [micropython-ili9341](https://bitbucket.org/thesheep/micropython-ili9341) - Collection of drivers for TFT displays, ILI9341, SH1106, SSD1606, ST7735.
* [MicroPython-ST7735](https://github.com/boochow/MicroPython-ST7735) - ESP32 version of GuyCarvers's ST7735 TFT LCD driver.
* [micropython-st7735](https://github.com/hosaka/micropython-st7735) - Driver for ST7735 TFT LCDs.
* [MicroPython_ST7735](https://github.com/AnthonyKNorman/MicroPython_ST7735) - Driver for ST7735 128x128 TFT.
* [ST7735](https://github.com/GuyCarver/MicroPython/blob/master/lib/ST7735.py) - Driver for ST7735 TFT LCDs.

#### LED Matrix

* [micropython-max7219](https://github.com/mcauser/micropython-max7219) - Driver for MAX7219 8x8 LED matrix modules.

#### LED Segment

* [LKM1638](https://github.com/arikb/LKM1638) - Driver for JY-LKM1638 displays based on TM1638 controller.
* [micropython-max7219](https://github.com/JulienBacquart/micropython-max7219) - Driver for MAX7219 8-digit 7-segment LED modules.
* [micropython-my9221](https://github.com/mcauser/micropython-my9221) - Driver for MY9221 10-segment LED bar graph modules.
* [micropython-tm1637](https://github.com/mcauser/micropython-tm1637) - Driver for TM1637 quad 7-segment LED modules.
* [micropython-tm1638](https://github.com/mcauser/micropython-tm1638) - Driver for TM1638 dual quad 7-segment LED modules with switches.
* [micropython-tm1640](https://github.com/mcauser/micropython-tm1640) - Driver for TM1740 8x8 LED matrix modules.

#### LEDs

* [micropython-p9813](https://github.com/mcauser/micropython-p9813) - Driver for P9813 RGB LED used in SeeedStudio's Grove Chainable RGB LED.
* [micropython-ws2812-7seg](https://github.com/HubertD/micropython-ws2812-7seg) - 7-segment display using WS2812 RGB LEDs.
* [Official APA102](http://docs.micropython.org/en/latest/esp8266/quickref.html#apa102-driver) - ESP8266 APA102/DotStar RGB LED driver.
* [Official WS2811](http://docs.micropython.org/en/latest/esp8266/quickref.html#neopixel-driver) - ESP8266 WS2811/NeoPixel RGB LED driver.
* [tlc5940-micropython](https://github.com/oysols/tlc5940-micropython) - Driver for TLC5940 16 channel LED driver.

#### OLED

* [Grove_OLED](https://github.com/dda/MicroPython/blob/master/Grove_OLED.py) - Driver for SSD1327 used by SeeedStudio's Grove OLED Display 1.12" v1.0.
* [micropython-oled](https://bitbucket.org/thesheep/micropython-oled) - Collection of drivers for monochrome OLED displays, PCD8544, SH1106, SSD1306, UC1701X.
* [micropython-ssd1327](https://github.com/mcauser/micropython-ssd1327) - Driver for SSD1327 128x128 4-bit greyscale OLED displays.
* [micropython-ssd1351](https://github.com/rdagger/micropython-ssd1351) - Driver for SSD1351 OLED displays.
* [MicroPython_SSD1306](https://github.com/AnthonyKNorman/MicroPython_SSD1306) - ESP8266 driver for SSD1306 OLED 128x64 displays.
* [Official SSD1306](https://github.com/micropython/micropython/tree/master/drivers/display) - Driver for SSD1306 128x64 OLED displays.

### IO

#### ADC

* [micropython-ads1015](https://bitbucket.org/thesheep/micropython-ads1015) - ADS1015 12-Bit and ADS1115 16-bit ADC, 4 channels with programmable gain, I2C interface.
* [Micropython_ADS1115](https://github.com/AnthonyKNorman/Micropython_ADS1115) - ADS1115 16-bit ADC, 4 channels with programmable gain, I2C interface.

#### DAC

* [micropython-mcp4725](https://github.com/wayoda/micropython-mcp4725) - Driver for the MCP4725 I2C DAC.

#### IO-Expander

* [MCP23017-ESP8266-Miniature-Driver](https://github.com/forkachild/MCP23017-ESP8266-Miniature-Driver) - Driver for MCP23017 16-bit I/O Expander.

#### Joystick

* [micropython-nunchuck](https://github.com/kfricke/micropython-nunchuck) - Driver for Nunchuk game controller, I2C interface.

### Motion

#### DC Motor

* [L298N](https://github.com/GuyCarver/MicroPython/blob/master/lib/L298N.py) - Driver for the L298N dual h-bridge motor controller.

#### Servo

* [micropython-pca9685](https://bitbucket.org/thesheep/micropython-pca9685) - 16-channel 12-bit PWM/servo driver.

#### Stepper

* [micropython-upybbot](https://github.com/jeffmer/micropython-upybbot) - A4988 driver for bipolar stepper motors.
* [uln2003](https://github.com/IDWizard/uln2003) - Driver for 5V 28BYJ-48 stepper motors

### Sensors

#### Accelerometer Digital

* [ADXL345-with-Pyboard](https://github.com/AbhinayBandaru/ADXL345-with-Pyboard) - Driver for ADXL345 16g 3-axis accelerometer.
* [adxl345_micropython](https://github.com/fanday/adxl345_micropython) - Driver for ADXL345 16g 3-axis accelerometer.
* [MMA7660](https://github.com/Bucknalla/MicroPython-3-Axis-Accelerometer/blob/master/MMA7660.py) - Driver for MMA7660 1.5g 3-axis accelerometer.

#### Air Quality

* [upython-aq-monitor](https://github.com/ayoy/upython-aq-monitor) - Air Quality monitor using PMS5003 sensor and WiPy.

#### Barometer

* [mpy_bme280_esp8266](https://github.com/catdog2/mpy_bme280_esp8266) - Bosch BME280 temperature/pressure/humidity sensor.

#### Compass

* [micropython-esp8266-hmc5883l](https://github.com/gvalkov/micropython-esp8266-hmc5883l) - 3-axis digital compass on the ESP8266.

#### Current

* [micropythonINA219](https://github.com/kabel42/micropythonINA219) - Driver for INA219 current sensor.
* [pyb_ina219](https://github.com/chrisb2/pyb_ina219) - Driver for INA219 current sensor.

#### Distance IR

* [micropython-gp2y0e03](https://bitbucket.org/thesheep/micropython-gp2y0e03) - IR-LED distance measuring sensor using Sharp GP2Y0E03.
* [micropython-vl53l0x](https://bitbucket.org/thesheep/micropython-vl53l0x) - Time-of-Flight laser-ranging sensor.
* [micropython-vl6180](https://bitbucket.org/thesheep/micropython-vl6180) - Time-of-Flight sensor, ambient light sensor & IR emitter.

#### Distance Ultrasonic

* [micropython-hcsr04](https://github.com/rsc1975/micropython-hcsr04) - Driver for HC-SR04 ultrasonic distance sensors.

#### Light

* [micropython-tsl2561](https://github.com/kfricke/micropython-tsl2561) - Driver for the TSL2561 illumination sensor from TAOS / ams.
* [mpy_bh1750fvi_esp8266](https://github.com/catdog2/mpy_bh1750fvi_esp8266) - ESP8266 driver for BH1750FVI sensor.

#### Motion Inertial

* [micropython-bno055](https://github.com/deshipu/micropython-bno055) - Bosch Sensortec BNO055 9DOF IMU sensor, I2C interface.
* [MPU6050-ESP8266-MicroPython](https://github.com/adamjezek98/MPU6050-ESP8266-MicroPython) - Driver for MPU6050 Accelerometer/Gyroscope on ESP8266.

#### Temperature Analog

* [micropython-max31855](https://bitbucket.org/thesheep/micropython-max31855) - Thermocouple amplifier, SPI interface.
* [max31856](https://github.com/alinbaltaru/max31856) - Precision thermocouple to digital converter with linearization, SPI interface.

#### Temperature Digital

* [bme680-mqtt-micropython](https://github.com/robmarkcole/bme680-mqtt-micropython) - Driver for BME680 gas, pressure, temperature and humidity sensor.
* [micropython-am2320](https://github.com/mcauser/micropython-am2320) - Aosong AM2320 temperature and humidity sensor, I2C interface.
* [micropython-dht12](https://github.com/mcauser/micropython-dht12) - Aosong DHT12 temperature and humidity sensor, I2C interface.
* [micropython-hdc1008](https://github.com/kfricke/micropython-hdc1008) - Driver for the Texas Instruments HDC1008 humidity and temperature sensor.
* [micropython-mcp9808](https://github.com/kfricke/micropython-mcp9808) - Driver for the Microchip MCP9808 temperature sensor.
* [micropython-mpl115a2](https://github.com/khoulihan/micropython-mpl115a2) - Pyboard driver for the MPL115A2 barometric pressure sensor.
* [micropython-sht31](https://github.com/kfricke/micropython-sht31) - Driver for the SHT31 temperature and humidity sensor.
* [micropython-si7021](https://bitbucket.org/thesheep/micropython-si7021) - SI7021 Temperature and humidity sensor, I2C interface.
* [micropython-si7021](https://github.com/chrisbalmer/micropython-si7021) - SI7021 Temperature and humidity sensor, I2C interface.
* [micropython-Si705x](https://github.com/billyrayvalentine/micropython-Si705x) - Silicon Labs Si705x series of temperature sensors, I2C interface.
* [micropython-Si70xx](https://github.com/billyrayvalentine/micropython-Si70xx) - Silicon Labs Si70xx series of relative humidity and temperature sensors, I2C interface.
* [micropython-tmp102](https://github.com/khoulihan/micropython-tmp102) - Driver for TMP102 digital temperature sensor.
* [Official DHT11+DHT12](https://github.com/micropython/micropython/blob/master/drivers/dht/dht.py) - ESP8266 driver for DHT11 and DHT12 temperature and humidity sensor.

#### Temperature IR

* [micropython-mlx90614](https://github.com/mcauser/micropython-mlx90614) - Driver for Melexis MLX90614 IR temperature sensor.

#### Touch Capacitive

* [micropython-mpr121](https://github.com/mcauser/micropython-mpr121) - Driver for MPR121 capacitive touch keypads and breakout boards.
* [micropython-ttp223](https://github.com/mcauser/micropython-ttp223) - Examples using TTP223 capacitive touch module.

#### Touch Resistive

* [XPT2046-touch-pad-driver-for-PyBoard](https://github.com/robert-hh/XPT2046-touch-pad-driver-for-PyBoard) - Driver for XPT2046 touch pad controller used in many TFT modules.

## Community

* [MicroPython Forum](https://forum.micropython.org/) - Online community of over 3400 users discussing all things related to MicroPython.
* [MicroPython on Twitter](https://twitter.com/micropython?lang=en) - Follow MicroPython on Twitter for latest news and updates.

## Books

* [Programming with MicroPython: Embedded Programming with Microcontrollers and Python](http://shop.oreilly.com/product/0636920056515.do) - by Nicholas H. Tollervey.

## Resources

* [MicroPython](http://micropython.org) - Project website. Test drive the pyboard. Try MicroPython online with unicorn.
* [MicroPython on GitHub](https://github.com/micropython/micropython) - Submit bug reports, follow and join in development on GitHub.
* [MicroPython Official Documentation](http://docs.micropython.org/) - For various ports, including quick reference, general information, examples and tutorials.
* [MicroPython Wiki](http://wiki.micropython.org/Home) - Community generated documentation and examples of the features of MicroPython and the pyboard.
* [MicroPython Newsletter](http://micropython.org/newsletter) - Subscribe to the MicroPython newsletter for news and announcements including new features and new products.
* [MicroPython Store](https://store.micropython.org/) - Where you can buy the pyboard, housings, skins, books, connectors and peripherals.
* [MicroPython on Wikipedia](https://en.wikipedia.org/wiki/MicroPython)

## Miscellaneous

* [MicroPython Kickstarter](https://www.kickstarter.com/projects/214379695/micro-python-python-for-microcontrollers) - 1,931 backers pledged £97,803 to help bring this project to life.
* [MicroPython on the ESP8266 Kickstarter](https://www.kickstarter.com/projects/214379695/micropython-on-the-esp8266-beautifully-easy-iot) - 1,399 backers pledged £28,534 to help bring this project to life.

## Contributing

Contributions and suggestions are always welcome! Please take a look at the [contribution guidelines](https://github.com/mcauser/awesome-micropython/blob/master/contributing.md) first.

## License & Trademarks

[![CC0](http://mirrors.creativecommons.org/presskit/buttons/88x31/svg/cc-zero.svg)](https://creativecommons.org/publicdomain/zero/1.0/)

To the extent possible under law, the authors have waived all copyright and related or neighbouring rights to this work.
