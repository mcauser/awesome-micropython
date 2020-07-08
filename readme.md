<p align="center">
  <a href="https://awesome-micropython.com/" style="display:block"><img src="https://raw.githubusercontent.com/mcauser/awesome-micropython/master/docs/img/logo.svg"></a>
</p>
<p align="center">
  <a href="https://awesome.re">
    <img alt="Awesome" src="https://awesome.re/badge-flat.svg">
  </a>
</p>
<hr>

A curated list of awesome MicroPython libraries, frameworks, software and resources.

[MicroPython](http://micropython.org/) is a lean and efficient implementation of the Python 3 programming language that includes a small subset of the Python standard library and is optimised to run on microcontrollers and in constrained environments.

## Contents

* [Libraries](#libraries)
  * [AI](#ai)
  * [Analytics](#analytics)
  * [Audio](#audio)
  * [Communications](#communications)
  * [Display](#display)
  * [IO](#io)
  * [Motion](#motion)
  * [Sensors](#sensors)
  * [Scheduling](#scheduling)
  * [Storage](#storage)
* [Community](#community)
* [Books](#books)
* [Resources](#resources)
* [Development](#development)
  * [Code Generation](#code-generation)
  * [Debugging](#debugging)
  * [IDEs](#ides)
  * [Shells](#Shells)
* [Miscellaneous](#miscellaneous)

## Libraries

Other places you can look for MicroPython Libraries:

* [PyPi](https://pypi.org/search/?c=Programming+Language+%3A%3A+Python+%3A%3A+Implementation+%3A%3A+MicroPython) - This filter shows just the MicroPython libraries on PyPi. Note: You cannot pip install micropython libraries. See [MicroPython docs](https://docs.micropython.org/en/latest/reference/packages.html?highlight=upip) for more information on upip.
* [GitHub Search](https://github.com/search?q=micropython) - Search GitHub for repositories containing MicroPython.
* [GitHub Topic - MicroPython](https://github.com/topics/micropython) - Browse GitHub Topics for projects tagged with MicroPython.
* [Libraries.io](https://libraries.io/search?q=micropython) - Libraries.io query for MicroPython.
* [GitLab Explore](https://gitlab.com/explore?sort=latest_activity_desc&utf8=%E2%9C%93&name=micropython&sort=latest_activity_desc) - Explore repositories on GitLab.

### AI

* [MicroMLP](https://github.com/jczic/MicroMLP) - A micro neural network multilayer perceptron for MicroPython (used on ESP32 and Pycom modules).

### Analytics

* [uMath](https://github.com/AaronKel/uMath) - Computer Algebra for microcontrollers.
* [micropython-ulab](https://github.com/v923z/micropython-ulab) - A numpy-like fast vector module for MicroPython.
* [micropython-fourier](https://github.com/peterhinch/micropython-fourier) - Fast Fourier transform in MicroPython's inline ARM assembler.
* [ulinalg](https://github.com/jalawson/ulinalg) - Small size matrix handling module with a few linear algebra operations specifically for MicroPython (Python3).
* [micropython-mtx](https://gitlab.com/nickoala/micropython-mtx) - Fast Matrix Multiplication and Linear Solver on MicroPython.
* [micropython-vec](https://gitlab.com/nickoala/micropython-vec) - Vector Operations on MicroPython.

### Audio

* [JQ6500](https://github.com/rdagger/micropython-jq6500) - Driver for JQ6500 UART MP3 modules.
* [KT403A-MP3](https://github.com/jczic/KT403A-MP3) - Driver for KT403A, used by DFPlayer Mini and Grove MP3 v2.0.
* [micropython-buzzer](https://github.com/fruch/micropython-buzzer) - Play nokia compose and mid files on buzzers.
* [micropython-dfplayer](https://github.com/ShrimpingIt/micropython-dfplayer) - Driver for DFPlayer Mini using UART.
* [micropython-longwave](https://github.com/MattMatic/micropython-longwave) - WAV player for MicroPython board.
* [micropython-vs1053](https://github.com/peterhinch/micropython-vs1053) - Asynchronous driver for VS1053b MP3 player.
* [micropython-midi](https://github.com/cjbarnes18/micropython-midi) - A midi implementation example for MicroPython.
* [upy-rtttl](https://github.com/dhylands/upy-rtttl) - Python Parser for Ring Tone Text Transfer Language (RTTTL).

### Communications

#### Bluetooth

* [PyBoard-HC05-Android](https://github.com/KipCrossing/PyBoard-HC05-Android) - Pyboard HC05 Bluetooth adaptor example application.

#### CAN

* [micropython-spacecan](https://gitlab.com/alphaaomega/micropython-spacecan) - Spacecan is a MicroPython implementation of the SpaceCAN protocol for embedded systems.

#### DNS

* [ICantBelieveItsNotDNS](https://github.com/yschaeff/ICantBelieveItsNotDNS) - "I Can't Believe It's Not DNS!" (ICBIND) is an authoritative DNS server for the ESP8266 written in MicroPython.
* [MicroDNSSrv](https://github.com/jczic/MicroDNSSrv) - A micro DNS server for MicroPython to simply respond to A queries on multi-domains with or without wildcards (used on Pycom modules & ESP32).

#### Ethernet

* [Official wiznet5k](https://github.com/micropython/micropython/tree/master/drivers/wiznet5k) - Official driver for the WIZnet5x00 series of Ethernet controllers.

#### FTP

* [micropython-ftplib](https://github.com/SpotlightKid/micropython-ftplib) - An FTP client library for MicroPython.
* [FTP-Server-for-ESP8266-ESP32-and-PYBD](https://github.com/robert-hh/FTP-Server-for-ESP8266-ESP32-and-PYBD) - Small FTP server for ESP8266/ESP32/PYBD on the MicroPython platform.
* [MicroFTPServer](https://github.com/cpopp/MicroFTPServer) - Minimal FTP Server that can run on an ESP8266 with MicroPython.

#### GPS

* [micropyGPS](https://github.com/inmcm/micropyGPS) - Full featured GPS NMEA sentence parser.
* [micropython-gnssl76l](https://github.com/tuupola/micropython-gnssl76l) - MicroPython I2C driver for Quectel GNSS L76-L (GPS).
* [mpy-agps](https://github.com/pulkin/mpy-agps) - MicroPython implementation of assisted location services (AGPS).

#### GSM

* [micropython-upyphone](https://github.com/jeffmer/micropython-upyphone) - A gsm phone using pyboard and sim800l.

#### IoT

* [microhomie](https://github.com/microhomie/microhomie) - MicroPython implementation of the Homie MQTT convention for IoT.
* [uPyEcho](https://github.com/lemariva/uPyEcho) - Emulated Belkin WeMo device that works with Amazon Echo (Alexa) using MicroPython on an ESP32.
* [SonosRemote](https://github.com/foosel/SonosRemote) - A remote for Sonos installations running on an ESP8266 and using Sonos HTTP API.
* [micropython-home-assistant](https://gitlab.com/aapjeisbaas/micropython-home-assistant) - MicroPython based scripts to extend you home assistant driven home automation projects.

#### IR

* [micropython-necir](https://github.com/MattMatic/micropython-necir) - NEC infrared capture for TL1838 IR receiver LEDs.
* [Micropython-IR](https://github.com/designerPing/Micropython-IR) - Pyboard infrared remote sniff and replay.
* [micropython_ir](https://github.com/peterhinch/micropython_ir) - Nonblocking device drivers to receive from IR remotes and for IR "blaster" apps.
* [micropython-amg88xx](https://github.com/peterhinch/micropython-amg88xx) - Driver for Grid-EYE thermal infra red array sensor (Adafruit 3538).
* [micropython-ys-irtm](https://github.com/mcauser/micropython-ys-irtm) - MicroPython examples for YS-IRTM 5V NEC Infrared UART transceivers.
* [esp8266_ir](https://github.com/ruoyu0088/esp8266_ir) - Control IR signal by websocket.

#### LoRaWAN

* [uPyLoRaWAN](https://github.com/lemariva/uPyLoRaWAN) - ESP32 using MicroPython meets LoRa and LoRaWAN.

#### MQTT

* [micropython-mqtt](https://github.com/peterhinch/micropython-mqtt) - A 'resilient' asynchronous MQTT driver. Plus a means of using an ESP8266 to bring MQTT to non-networked targets.
* [MQBoard](https://github.com/tve/mqboard) - A micro-framework for using MQTT with asyncio on MicroPython boards, primarily on the ESP32.
* [micropython-mqtt](https://github.com/tve/micropython-mqtt) - MQTT libraries and tools for MicroPython.
* [pysmartnode](https://github.com/kevinkk525/pysmartnode) -  MicroPython Smarthome framework.
* [umqtt_aws_iot](https://github.com/juwul/umqtt_aws_iot) - Publish UMQTT messages with MicroPython to AWS IoT.
* [sonoff-mqtt by davea](https://github.com/davea/sonoff-mqtt) - MicroPython scripts to control Sonoff/ESP8266 using MQTT.
* [micropython-sonoff-switch](https://github.com/kfricke/micropython-sonoff-switch) - Implements a MQTT controllable switch for the iTead Sonoff Switch using MicroPython.
* [micropython-thingspeak-mqtt-esp8266](https://github.com/miketeachman/micropython-thingspeak-mqtt-esp8266) - Publish and Subscribe to Thingspeak using MQTT with MicroPython running on ESP8266/ESP32 platforms.
* [uMQTT](https://github.com/andrewmk/uMQTT) - MQTT publish for MicroPython on the WiPy board.

#### NTP

* [esp8266_ntp_webserver](https://github.com/Roterfux/esp8266_ntp_webserver) - MicroPython + esp8266 + ntp + webserver.
* [micropython-ntpd](https://github.com/dave2/micropython-ntpd) - An implementation of an ntpd in MicroPython.
* [micropython_ntpserver](https://github.com/GrantGMiller/micropython_ntpserver) - An NTP server written for MicroPython.
* [micropython-ntpclient](https://github.com/wieck/micropython-ntpclient) - NTP client for MicroPython using uasyncio.

#### OneWire

* [Official OneWire](https://github.com/micropython/micropython/tree/master/drivers/onewire) - For devices using the OneWire bus, eg Dallas ds18x20.

#### Radio

* [micropython-radio](https://github.com/peterhinch/micropython-radio) - Protocols for nRF24L01 2.4Ghz radio modules.
* [micropython-rfsocket](https://github.com/wuub/micropython-rfsocket) - MicroPython implementation of popular 433MHzn based RFSockets.
* [Official nRF24L01](https://github.com/micropython/micropython/tree/master/drivers/nrf24l01) - Official driver for nRF24L01 2.4Ghz radio modules.
* [micropython_remote](https://github.com/peterhinch/micropython_remote) - Capture and replay 433MHz remote control codes. Control remote switched power adaptors.
* [micropython-ys-rf34t](https://github.com/mcauser/micropython-ys-rf34t) - MicroPython examples using YS-RF34T 433MHz ASK/OOK UART transceivers.

#### REPL

* [webrepl](https://micropython.org/webrepl) - MicroPython WebREPL.
* [zepl](https://gitlab.com/zepl1/zepl) - MicroPython WebREPL Console Application using ZeroMQ.
* [jupyter_micropython_remote](https://gitlab.com/alelec/jupyter_micropython_remote) - Jupyter kernel to directly execute code on a MicroPython board over the serial/web REPL.

#### RFID

* [micropython-mfrc522](https://github.com/wendlers/micropython-mfrc522) - Driver for NXP MFRC522 RFID reader/writer.
* [micropython-wiegand](https://github.com/pjz/micropython-wiegand) - Wiegand protocol reader.

#### RTC

* [micropython-tinyrtc-i2c](https://github.com/mcauser/micropython-tinyrtc-i2c) - Driver for DS1307 RTC and AT24C32N EEPROM.
* [Micropython_TinyRTC](https://github.com/AnthonyKNorman/Micropython_TinyRTC) - Driver for DS1307 RTC.

#### Serial

* [mpy-miniterm](https://github.com/jeffmakes/mpy-miniterm) - Tool for seamless serial debug and file synchronisation with MicroPython devices via the serial REPL.
* [micropython-modbus](https://gitlab.com/extel-open-source/micropython-modbus) - MicroPython port of modbus-tk.

#### SMTP

* [uMail](https://github.com/shawwwn/uMail) - A lightweight, scalable SMTP client for sending email in MicroPython.

#### Telnet

* [MicroTelnetServer](https://github.com/cpopp/MicroTelnetServer) - Simple telnet server for MicroPython and the ESP8266 allowing telnet clients access to the REPL.

#### WiFi

* [HueBridge](https://github.com/FRC4564/HueBridge) - Philips Hue Bridge.
* [micropython-wifimanager](https://github.com/mitchins/micropython-wifimanager) - A simple network configuration utility for MicroPython on the ESP8266 board.

#### Web

* [MicroWebSrv](https://github.com/jczic/MicroWebSrv) - A micro HTTP Web server that supports WebSockets, html/python language templating and routing handlers, for MicroPython (used on Pycom modules & ESP32).
* [MicroWebSrv2](https://github.com/jczic/MicroWebSrv2) - The last Micro Web Server for IoTs (MicroPython) or large servers (CPython), that supports WebSockets, routes, template engine and with really optimized architecture (mem allocations, async I/Os).
* [tinyweb](https://github.com/belyalov/tinyweb) - Simple and lightweight HTTP async server for MicroPython.
* [upy-websocket-server](https://github.com/BetaRavener/upy-websocket-server) - MicroPython (ESP8266) websocket server implementation.
* [micropython-captive-portal](https://github.com/amora-labs/micropython-captive-portal) - A captive portal demo for MicroPython.
* [uPyPortal](https://github.com/lemariva/uPyPortal) - A captive portal for MicroPython using ESP32 (WeMos).
* [ESP8266WebServer](https://github.com/codemee/ESP8266WebServer) - ESP8266 web server for MicroPython.
* [microCoAPy](https://github.com/insighio/microCoAPy) - A mini client/server implementation of CoAP (Constrained Application Protocol) into MicroPython.
* [micropyserver](https://github.com/troublegum/micropyserver) - MicroPyServer is a simple HTTP server for MicroPython projects.
* [MicroRESTCli](https://github.com/jczic/MicroRESTCli) - A micro JSON REST Web client based on MicroWebCli for MicroPython (used on Pycom modules & ESP32).
* [micropython-noggin](https://github.com/larsks/micropython-noggin) - A very simple web server for MicroPython.
* [uwebsockets](https://github.com/danni/uwebsockets) - MicroPython websockets implementation for ESP8266.
* [microdot](https://github.com/miguelgrinberg/microdot) - The impossibly small web framework for MicroPython.

#### Zigbee

* [ZbPy](https://github.com/osresearch/ZbPy) - MicroPython IEEE802.15.4 / Zigbee parser.

### Display

#### E-Paper

* [micropython-epaper](https://github.com/peterhinch/micropython-epaper) - Pyboard driver for Embedded Artists 2.7 inch e-paper display.
* [micropython-ili9341](https://bitbucket.org/thesheep/micropython-ili9341) - SSD1606 active matrix epaper display 128x180.
* [micropython-waveshare-epaper](https://github.com/mcauser/micropython-waveshare-epaper) - Drivers for various Waveshare e-paper modules.
* [micropython-waveshare-epd](https://github.com/ayoy/micropython-waveshare-epd) - Waveshare E-Paper Display driver for devices running Pycom-flavored MicroPython.

#### LCD Character

* [Grove_RGB_LCD](https://github.com/dda/MicroPython/blob/master/Grove_RGB_LCD.py) - Driver for SeeedStudio's Grove RGB LCD.
* [lcdi2c](https://github.com/slothyrulez/lcdi2c) - Driver for HD44780 compatible dot matrix LCDs.
* [micropython-charlcd](https://github.com/rdagger/micropython-charlcd) - Driver for HD44780 compatible LCDs.
* [micropython-i2c-lcd](https://github.com/Bucknalla/micropython-i2c-lcd) - Driver for I2C 2x16 LCD Screens.
* [micropython_grove_rgb_lcd_driver](https://github.com/KidVizious/micropython_grove_rgb_lcd_driver) - Driver for SeeedStudio's Grove RGB LCD.
* [pyboard-LCD-character-display](https://github.com/scitoast/pyboard-LCD-character-display) - PyBoard driver for HDD44780 compatible 1602 LCDs.
* [python_lcd](https://github.com/dhylands/python_lcd) - Driver for HD44780 compatible dot matrix LCDs.
* [micropython-lcd](https://github.com/wjdp/micropython-lcd) - Class for controlling the HD44780 from a MicroPython pyboard.

#### LCD Graphic

* [micropython-lcd-AQM1248A](https://github.com/forester3/micropython-lcd-AQM1248A) - ESP8266 driver for AQM1248A graphic LCD.
* [micropython-lcd160cr-gui](https://github.com/peterhinch/micropython-lcd160cr-gui) - Simple touch driven event based GUI for the Pyboard and LCD160CR colour display.
* [micropython-pcd8544](https://github.com/mcauser/micropython-pcd8544) - Driver for Nokia 5110 PCD8544 84x48 LCD modules.
* [micropython-st7565](https://github.com/nquest/micropython-st7565) - Driver for ST7565 128x64 LCDs.
* [micropython-st7920](https://github.com/ShrimpingIt/micropython-st7920) - Library for simple graphic primitives on ST7920 128x64 monochrome LCD panel using ESP8266 and SPI.
* [MicroPython_PCD8544](https://github.com/AnthonyKNorman/MicroPython_PCD8544) - ESP8266 driver for Nokia 5110 PCD8544.
* [Official LCD160CR](https://github.com/micropython/micropython/tree/master/drivers/display) - Driver for official MicroPython LCD160CR display with resistive touch sensor.
* [micropython-hx1230](https://github.com/mcauser/micropython-hx1230) - MicroPython library for HX1230 96x68 LCD modules.

#### LCD TFT

* [micropython-ili9341](https://bitbucket.org/thesheep/micropython-ili9341) - Collection of drivers for TFT displays, ILI9341, SH1106, SSD1606, ST7735.
* [micropython-ili934x](https://github.com/tuupola/micropython-ili934x) - SPI driver for ILI934X series based TFT / LCD displays.
* [MicroPython-ST7735](https://github.com/boochow/MicroPython-ST7735) - ESP32 version of GuyCarvers's ST7735 TFT LCD driver.
* [micropython-st7735](https://github.com/hosaka/micropython-st7735) - Driver for ST7735 TFT LCDs.
* [MicroPython_ST7735](https://github.com/AnthonyKNorman/MicroPython_ST7735) - Driver for ST7735 128x128 TFT.
* [SSD1963-TFT-Library-for-PyBoard](https://github.com/robert-hh/SSD1963-TFT-Library-for-PyBoard) - Driver for SSD1963 864x480 TFT LCDs.
* [ST7735](https://github.com/GuyCarver/MicroPython/blob/master/lib/ST7735.py) - Driver for ST7735 TFT LCDs.
* [micropython_ra8875](https://github.com/peterhinch/micropython_ra8875) - MicroPython device driver and nano-GUI for RA8875 based displays.
* [micropython-ili9341](https://github.com/rdagger/micropython-ili9341) - MicroPython ILI9341 display & XPT2046 touch screen driver.
* [st7789_mpy](https://github.com/devbis/st7789_mpy) - Fast pure-C driver for MicroPython that can handle display modules on ST7789 chip.
* [st7789py_mpy](https://github.com/devbis/st7789py_mpy) - Slow MicroPython driver for 240x240 ST7789 display without CS pin from Ali Express, written in MicroPython.
* [micropython-ili9341](https://github.com/jeffmer/micropython-ili9341) - MicroPython Driver for ILI9341 display.
* [micropython-ili9341](https://github.com/tkurbad/micropython-ili9341) - ILI9341 TFT driver for MicroPython on ESP32.

#### LED Matrix

* [micropython-ht1632c](https://github.com/vrialland/micropython-ht1632c) - Driver for HT1632C 32x16 bicolor led matrix.
* [micropython-matrix8x8](https://github.com/JanBednarik/micropython-matrix8x8) - Driver for AdaFruit 8x8 LED Matrix display with HT16K33 backpack.
* [micropython-max7219](https://github.com/mcauser/micropython-max7219) - Driver for MAX7219 8x8 LED matrix modules.
* [micropython-wemos-led-matrix-shield](https://github.com/mactijn/micropython-wemos-led-matrix-shield) - Driver for Wemos D1 Mini Matrix LED shield, using TM1640 chip.
* [micropython-wemos-led-matrix](https://github.com/mattytrentini/micropython-wemos-led-matrix) - Driver for Wemos D1 Mini Matrix LED shield, using TM1640 chip.
* [micropython-max7219](https://github.com/vrialland/micropython-max7219) - MicroPython driver for MAX7219 8x8 LED matrix.

#### LED Segment

* [LKM1638](https://github.com/arikb/LKM1638) - Driver for JY-LKM1638 displays based on TM1638 controller.
* [max7219_8digit](https://github.com/pdwerryhouse/max7219_8digit) - Driver for MAX7219 8-digit 7-segment LED modules.
* [micropython-max7219](https://github.com/JulienBacquart/micropython-max7219) - Driver for MAX7219 8-digit 7-segment LED modules.
* [micropython-my9221](https://github.com/mcauser/micropython-my9221) - Driver for MY9221 10-segment LED bar graph modules.
* [micropython-tm1637](https://github.com/mcauser/micropython-tm1637) - Driver for TM1637 quad 7-segment LED modules.
* [micropython-tm1638](https://github.com/mcauser/micropython-tm1638) - Driver for TM1638 dual quad 7-segment LED modules with switches.
* [micropython-tm1640](https://github.com/mcauser/micropython-tm1640) - Driver for TM1740 8x8 LED matrix modules.
* [micropython-tm1640](https://gitlab.com/robhamerling/micropython-tm1640) - MicroPython Library for 16 digits 7-segment displays controlled by a TM1640.

#### LEDs

* [micropython-morsecode](https://github.com/mampersat/micropython-morsecode) - Blink an LED with morse coded message.
* [micropython-p9813](https://github.com/mcauser/micropython-p9813) - Driver for P9813 RGB LED used in SeeedStudio's Grove Chainable RGB LED.
* [micropython-ws2812-7seg](https://github.com/HubertD/micropython-ws2812-7seg) - 7-segment display using WS2812 RGB LEDs.
* [micropython-ws2812](https://github.com/JanBednarik/micropython-ws2812) - Driver for WS2812 RGB LEDs.
* [Official APA102](http://docs.micropython.org/en/latest/esp8266/quickref.html#apa102-driver) - ESP8266 APA102/DotStar RGB LED driver.
* [Official WS2811](http://docs.micropython.org/en/latest/esp8266/quickref.html#neopixel-driver) - ESP8266 WS2811/NeoPixel RGB LED driver.
* [tlc5940-micropython](https://github.com/oysols/tlc5940-micropython) - Driver for TLC5940 16 channel LED driver.
* [ws2812-SPI](https://github.com/nickovs/ws2812-SPI) - An efficient micropython WS2812 (NeoPixel) driver.
* [micropython-ws2801](https://github.com/HeMan/micropython-ws2801) - A MicroPython library to interface with strands of WS2801 RGB LEDs.
* [tlc5947-rgb-micropython](https://gitlab.com/peterzuger/tlc5947-rgb-micropython) - Driver for the TLC5947 24 channel 12-bit PWM LED driver.

#### OLED

* [Grove_OLED](https://github.com/dda/MicroPython/blob/master/Grove_OLED.py) - Driver for SSD1327 used by SeeedStudio's Grove OLED Display 1.12" v1.0.
* [micropython-nano-gui](https://github.com/peterhinch/micropython-nano-gui) - Nano-Gui provides a limited set of GUI objects (widgets) for displays whose display driver is subclassed from the framebuf class. Which includes LCD and OLED displays.
* [micropython-oled](https://bitbucket.org/thesheep/micropython-oled) - Collection of drivers for monochrome OLED displays, PCD8544, SH1106, SSD1306, UC1701X.
* [micropython-ssd1327](https://github.com/mcauser/micropython-ssd1327) - Driver for SSD1327 128x128 4-bit greyscale OLED displays.
* [micropython-ssd1351](https://github.com/rdagger/micropython-ssd1351) - Driver for SSD1351 OLED displays.
* [MicroPython_SSD1306](https://github.com/AnthonyKNorman/MicroPython_SSD1306) - ESP8266 driver for SSD1306 OLED 128x64 displays.
* [Official SSD1306](https://github.com/micropython/micropython/tree/master/drivers/display) - Driver for SSD1306 128x64 OLED displays.
* [SH1106](https://github.com/robert-hh/SH1106) - Driver for the SH1106 OLED display.

### IO

#### ADC

* [ads1x15](https://github.com/robert-hh/ads1x15) - Driver for the ADS1015/ADS1115 ADC, I2C interface.
* [micropython-ads1015](https://bitbucket.org/thesheep/micropython-ads1015) - ADS1015 12-Bit and ADS1115 16-bit ADC, 4 channels with programmable gain, I2C interface.
* [Micropython_ADS1115](https://github.com/AnthonyKNorman/Micropython_ADS1115) - ADS1115 16-bit ADC, 4 channels with programmable gain, I2C interface.
* [ADS7818](https://github.com/robert-hh/ADS7818) - Python class interfacing the ADS7818 AD-converter.
* [micropython-ads1219](https://github.com/miketeachman/micropython-ads1219) - MicroPython module for the Texas Instruments ADS1219 ADC.
* [micropython-hx711](https://github.com/SergeyPiskunov/micropython-hx711) - MicroPython driver for HX711 24-Bit Analog-to-Digital Converter.

#### DAC

* [micropython-mcp4725](https://github.com/wayoda/micropython-mcp4725) - Driver for the MCP4725 I2C DAC.

#### GPIO

* [micropython-inputs](https://github.com/alanmitchell/micropython-inputs) - Classes to count pulses, debounce digital inputs, and calculate moving averages of analog inputs for a MicroPython board.
* [ubutton](https://gitlab.com/WiLED-Project/ubutton) - A MicroPython library for controlling reading and debouncing pushbutton inputs, including "short" and "long" press callbacks.
* [micropython-debounce-switch](https://github.com/selfhostedhome/micropython-debounce-switch) - MicroPython Class for Debouncing Switches.

#### IO-Expander

* [MCP23017-ESP8266-Miniature-Driver](https://github.com/forkachild/MCP23017-ESP8266-Miniature-Driver) - Driver for MCP23017 16-bit I/O Expander.
* [micropython-mcp230xx](https://github.com/ShrimpingIt/micropython-mcp230xx) - Driver for MCP23017 and MCP23008 GPIO expanders.
* [micropython-mcp23017](https://github.com/mcauser/micropython-mcp23017) - MicroPython driver for MCP23017 16-bit I/O Expander.
* [micropython-pcf8574](https://github.com/mcauser/micropython-pcf8574) - MicroPython driver for PCF8574 8-Bit I2C I/O Expander with Interrupt.
* [micropython-pcf8575](https://github.com/mcauser/micropython-pcf8575) - MicroPython driver for PCF8575 16-Bit I2C I/O Expander with Interrupt.
* [micropython-pcf8591](https://gitlab.com/cediddi/micropython-pcf8591) - MicroPython driver for PCF8591 8-Bit I2C I/O Expander.

#### Joystick

* [micropython-nunchuck](https://github.com/kfricke/micropython-nunchuck) - Driver for Nunchuk game controller, I2C interface.

#### PWM

* [upwmcontroller](https://gitlab.com/WiLED-Project/upwmcontroller) - A MicroPython library for controlling PWM outputs in an asyncio loop, with features including fading and blinking.

#### Rotary Encoder

* [micropython-rotary](https://github.com/miketeachman/micropython-rotary) - MicroPython module to read a rotary encoder.
* [uencoder](https://gitlab.com/WiLED-Project/uencoder) - A MicroPython library for reading from a rotary encoder.

#### Waveform Generator

* [Micropython-AD9833](https://github.com/KipCrossing/Micropython-AD9833) - Pyboard driver for AD9833, spi interface.

### Motion

#### DC Motor

* [L298N](https://github.com/GuyCarver/MicroPython/blob/master/lib/L298N.py) - Driver for the L298N dual h-bridge motor controller.

#### Servo

* [micropython-pca9685](https://bitbucket.org/thesheep/micropython-pca9685) - 16-channel 12-bit PWM/servo driver.

#### Stepper

* [micropython-upybbot](https://github.com/jeffmer/micropython-upybbot) - A4988 driver for bipolar stepper motors.
* [uln2003](https://github.com/IDWizard/uln2003) - Driver for 5V 28BYJ-48 stepper motors.
* [micropython-multiaxis](https://gitlab.com/olivier.len02/micropython-multiaxis) - Multiaxis with MicroPython ESP32 and DRV8825.

### Sensors

#### Accelerometer Digital

* [ADXL345-with-Pyboard](https://github.com/AbhinayBandaru/ADXL345-with-Pyboard) - Driver for ADXL345 16g 3-axis accelerometer.
* [adxl345_micropython](https://github.com/fanday/adxl345_micropython) - Driver for ADXL345 16g 3-axis accelerometer.
* [micropython-lis2hh12](https://github.com/tuupola/micropython-lis2hh12) - I2C driver for LIS2HH12 3-axis accelerometer.
* [MMA7660](https://github.com/Bucknalla/MicroPython-3-Axis-Accelerometer/blob/master/MMA7660.py) - Driver for MMA7660 1.5g 3-axis accelerometer.

#### Air Quality

* [CCS811](https://github.com/Ledbelly2142/CCS811) - CCS811 Air Quality Sensor.
* [upython-aq-monitor](https://github.com/ayoy/upython-aq-monitor) - Air Quality monitor using PMS5003 sensor and WiPy.
* [micropython-pms7003](https://github.com/pkucmus/micropython-pms7003) - MicroPython driver for the PMS7003 Air Quality Sensor.
* [pms5003_micropython](https://github.com/kevinkk525/pms5003_micropython) - Driver for pms5003 air quality sensor for MicroPython.
* [micropython-pms5003-minimal](https://github.com/miketeachman/micropython-pms5003-minimal) - Driver for pms5003 air quality sensor for MicroPython.
* [polly](https://github.com/g-sam/polly) - SDS011 pollution sensor + Wemos D1 mini pro + MicroPython.

#### Barometer

* [micropython-bme280](https://github.com/kevbu/micropython-bme280) - Driver for the Bosch BME280 temperature/pressure/humidity sensor.
* [micropython-bmp180](https://github.com/micropython-IMU/micropython-bmp180) - Driver for Bosch BMP180 temperature, pressure and altitude sensor.
* [mpy_bme280_esp8266](https://github.com/catdog2/mpy_bme280_esp8266) - Bosch BME280 temperature/pressure/humidity sensor.
* [wipy_bme280](https://bitbucket.org/oscarBravo/wipy_bme280) - Driver for the Bosch BME280 temperature/pressure/humidity sensor.
* [BME280](https://github.com/robert-hh/BME280) - MicroPython driver for the BME280 sensor, target platform Pycom devices.
* [micropython-bmp280](https://github.com/dafvid/micropython-bmp280) - Module for the BMP280 sensor.

#### Camera

* [micropython-ov2640](https://github.com/namato/micropython-ov2640) - MicroPython class for OV2640 camera.
* [Nikon-Trigger-for-MicroPython](https://github.com/Thekegman/Nikon-Trigger-for-MicroPython) - Remote trigger for a Nikon camera using an IR LED. For PyBoard v1.1.

#### Compass

* [micropython-esp8266-hmc5883l](https://github.com/gvalkov/micropython-esp8266-hmc5883l) - 3-axis digital compass on the ESP8266.
* [QMC5883](https://github.com/robert-hh/QMC5883) - Python class for the QMC5883 Three-Axis Digital Compass IC.

#### Current

* [micropythonINA219](https://github.com/kabel42/micropythonINA219) - Driver for INA219 current sensor.
* [pyb_ina219](https://github.com/chrisb2/pyb_ina219) - Driver for INA219 current sensor.

#### Distance IR

* [micropython-gp2y0e03](https://bitbucket.org/thesheep/micropython-gp2y0e03) - IR-LED distance measuring sensor using Sharp GP2Y0E03.
* [micropython-vl6180](https://bitbucket.org/thesheep/micropython-vl6180) - Time-of-Flight sensor, ambient light sensor & IR emitter.

#### Distance Laser

* [micropython-vl53l0x](https://bitbucket.org/thesheep/micropython-vl53l0x) - Time-of-Flight laser-ranging sensor.
* [Qwiic_TOF_Module_RFD77402](https://github.com/ZIOCC/Qwiic_TOF_Module_RFD77402) - Qwiic TOF Module (RFD77402) time-of-flight rangefinding module.

#### Distance Ultrasonic

* [micropython-hcsr04](https://github.com/rsc1975/micropython-hcsr04) - Driver for HC-SR04 ultrasonic distance sensors.

#### Energy

* [ATM90E26_Micropython](https://github.com/whatnick/ATM90E26_Micropython) - Driver for ATM90E26 energy metering device.
* [MCP39F521](https://github.com/warpme/MCP39F521) - ESP8266 scripts for reading MCP39F521 power monitors.

#### Gaseous

* [micropython-MQ](https://github.com/kartun83/micropython-MQ) - Drivers for MQ series gas sensors.
* [MQ135](https://github.com/rubfi/MQ135) - Driver for MQ135 gas sensor.
* [CCS811](https://github.com/Notthemarsian/CCS811) - Basic MicroPython driver for CCS811 on ESP8266 boards.
* [micropython-scd30](https://github.com/agners/micropython-scd30) - MicroPython I2C driver for Sensirion SCD30 CO2 sensor module.

#### Light

* [MicroPython-SI1145](https://github.com/neliogodoi/MicroPython-SI1145) - SI1145 UV index, IR, visible light and proximity sensor.
* [micropython-tsl2561](https://github.com/kfricke/micropython-tsl2561) - Driver for the TSL2561 illumination sensor from TAOS / ams.
* [mpy_bh1750fvi_esp8266](https://github.com/catdog2/mpy_bh1750fvi_esp8266) - ESP8266 driver for BH1750FVI sensor.
* [bh1750](https://github.com/PinkInk/upylib/tree/master/bh1750) - BH1750 i2c digital light sensor driver.

#### Motion Inertial

* [micropython-bmx055](https://github.com/micropython-IMU/micropython-bmx055) - Driver for Bosch BMX055 IMU sensor.
* [micropython-bno055](https://github.com/deshipu/micropython-bno055) - Bosch Sensortec BNO055 9DOF IMU sensor, I2C interface.
* [micropython-lsm9ds0](https://github.com/micropython-IMU/micropython-lsm9ds0) - LSM9DS0 g-force linear acceleration, gauss magnetic and dps angular rate sensors.
* [micropython-mpu9250](https://github.com/tuupola/micropython-mpu9250) - I2C driver for MPU9250 9-axis motion tracking device.
* [micropython-mpu9x50](https://github.com/micropython-IMU/micropython-mpu9x50) - Driver for the InvenSense MPU9250 inertial measurement unit.
* [MPU6050-ESP8266-MicroPython](https://github.com/adamjezek98/MPU6050-ESP8266-MicroPython) - ESP8266 driver for MPU6050 accelerometer/gyroscope.
* [py-mpu6050](https://github.com/larsks/py-mpu6050) - ESP8266 driver for MPU6050 accelerometer/gyroscope.
* [micropython-mpu6886](https://github.com/tuupola/micropython-mpu6886) - MicroPython I2C driver for MPU6886 6-axis motion tracking device.
* [micropython-fusion](https://gitlab.com/nnayo/micropython-fusion) - Sensor fusion calculates heading, pitch and roll from the outputs of motion tracking devices.

#### Soil Moisture

* [micropython-chirp](https://github.com/robberwick/micropython-chirp) - Driver for the Chirp Soil Moisture Sensor.

#### Temperature Analog

* [micropython-max31855](https://bitbucket.org/thesheep/micropython-max31855) - Thermocouple amplifier, SPI interface.
* [max31856](https://github.com/alinbaltaru/max31856) - Precision thermocouple to digital converter with linearization, SPI interface.

#### Temperature Digital

* [bme680-mqtt-micropython](https://github.com/robmarkcole/bme680-mqtt-micropython) - Driver for BME680 gas, pressure, temperature and humidity sensor.
* [LM75-MicroPython](https://github.com/OldhamMade/LM75-MicroPython) - Driver for LM75 digital temperature sensor, I2C interface.
* [micropython-am2320](https://github.com/mcauser/micropython-am2320) - Aosong AM2320 temperature and humidity sensor, I2C interface.
* [micropython-dht12](https://github.com/mcauser/micropython-dht12) - Aosong DHT12 temperature and humidity sensor, I2C interface.
* [micropython-hdc1008](https://github.com/kfricke/micropython-hdc1008) - Driver for the Texas Instruments HDC1008 humidity and temperature sensor.
* [micropython-mcp9808](https://github.com/kfricke/micropython-mcp9808) - Driver for the Microchip MCP9808 temperature sensor.
* [micropython-mpl115a2](https://github.com/khoulihan/micropython-mpl115a2) - Pyboard driver for the MPL115A2 barometric pressure sensor.
* [micropython-sht30](https://github.com/rsc1975/micropython-sht30) - Driver for SHT30 temperature and humidity sensor.
* [micropython-sht31](https://github.com/kfricke/micropython-sht31) - Driver for the SHT31 temperature and humidity sensor.
* [micropython-Si7005](https://github.com/Smrtokvitek/micropython-Si7005) - Driver for Si7005 relative humidity and temperature sensor.
* [micropython-si7021](https://bitbucket.org/thesheep/micropython-si7021) - SI7021 Temperature and humidity sensor, I2C interface.
* [micropython-si7021](https://github.com/chrisbalmer/micropython-si7021) - SI7021 Temperature and humidity sensor, I2C interface.
* [micropython-Si705x](https://github.com/billyrayvalentine/micropython-Si705x) - Silicon Labs Si705x series of temperature sensors, I2C interface.
* [micropython-Si70xx](https://github.com/billyrayvalentine/micropython-Si70xx) - Silicon Labs Si70xx series of relative humidity and temperature sensors, I2C interface.
* [micropython-tmp102](https://github.com/khoulihan/micropython-tmp102) - Driver for TMP102 digital temperature sensor.
* [Official DHT11+DHT12](https://github.com/micropython/micropython/blob/master/drivers/dht/dht.py) - ESP8266 driver for DHT11 and DHT12 temperature and humidity sensor.
* [SHT10_uPython](https://github.com/Omgitskillah/SHT10_uPython) - Driver for SHT10 temperature and humidity sensor.
* [sht25-micropython](https://github.com/Miceuz/sht25-micropython) - Driver for SHT25 temperature and humidity sensor.

#### Temperature IR

* [micropython-mlx90614](https://github.com/mcauser/micropython-mlx90614) - Driver for Melexis MLX90614 IR temperature sensor.

#### Touch Capacitive

* [micropython-mpr121](https://github.com/mcauser/micropython-mpr121) - Driver for MPR121 capacitive touch keypads and breakout boards.
* [micropython-ttp223](https://github.com/mcauser/micropython-ttp223) - Examples using TTP223 capacitive touch module.

#### Touch Resistive

* [XPT2046-touch-pad-driver](https://github.com/robert-hh/XPT2046-touch-pad-driver) - Driver for XPT2046 touch pad controller used in many TFT modules.

### Scheduling

* [micropython-mcron](https://github.com/fizista/micropython-mcron) - MicroCRON is a time-based task scheduling program for MicroPython.

### Storage

#### Database

* [uPyMySQL](https://github.com/dvrhax/uPyMySQL) - Pure uPython MySQL Client.
* [micropython-redis](https://github.com/dwighthubbard/micropython-redis) - A redis client implementation designed for use with MicroPython.
* [picoredis](https://github.com/SpotlightKid/picoredis) - A very minimal Redis client (not only) for MicroPython.
* [micropg](https://github.com/nakagami/micropg) - PostgreSQL database driver for MicroPython.
* [nmongo](https://github.com/nakagami/nmongo) - MongoDB client for CPython and MicroPython, with mongo shell like APIs.

#### EEPROM

* [micropython_eeprom](https://github.com/peterhinch/micropython_eeprom) - MicroPython device drivers for nonvolatile memory chips (EEPROM, FRAM, Flash).

#### FRAM

* [micropython-fram](https://github.com/rolandvs/micropython-fram) - Pyboard driver for Ferroelectric RAM module.

## Community

* [MicroPython Forum](https://forum.micropython.org/) - Online community of over 6400 users discussing all things related to MicroPython.
* [MicroPython on Twitter](https://twitter.com/micropython?lang=en) - Follow MicroPython on Twitter for latest news and updates.
* [MicroPython on Facebook](https://www.facebook.com/micropython) - Like MicroPython on Facebook for competitions, news and updates.
* [Melbourne MicroPython Meetup](https://www.meetup.com/en-AU/MicroPython-Meetup) - Regular meetup at CCHS in Melbourne, Australia.
* [Slack](https://slack-micropython.herokuapp.com/) - Get an automated invite to the micropython.slack.com workspace.
* [Discord](https://discord.gg/5GjKNE) - Get an invite to the MicroPython Discord server.

## Books

* [Programming with MicroPython: Embedded Programming with Microcontrollers and Python](http://shop.oreilly.com/product/0636920056515.do) - By Nicholas H. Tollervey. ISBN 9781491972731.
* [MicroPython for the Internet of Things: A Beginner's Guide to Programming with Python on Microcontrollers](https://www.apress.com/gp/book/9781484231227) - By Charles Bell. ISBN 9781484231227.
* [MicroPython Cookbook](https://www.packtpub.com/au/application-development/micropython-cookbook) - By Marwan Alsabbagh. ISBN 9781838649951.
* [Python for Microcontrollers: Getting Started with MicroPython](https://www.amazon.com.au/Python-Microcontrollers-Getting-Started-MicroPython/dp/1259644537) - By Donald Norris. ISBN 9781259644535.
* [Advanced Programming in MicroPython By Example](https://www.amazon.com/Advanced-Programming-MicroPython-Example-Magda/dp/1090900937) - By Yury Magda. ISBN 9781090900937.
* [MicroPython Projects](https://www.packtpub.com/au/iot-hardware/micropython-projects) - By Jacob Beningo. ISBN 9781789958034.

## Resources

* [MicroPython](http://micropython.org) - Project website. Test drive the pyboard. Try MicroPython online with unicorn.
* [MicroPython on GitHub](https://github.com/micropython/micropython) - Submit bug reports, follow and join in development on GitHub.
* [MicroPython Official Documentation](http://docs.micropython.org/) - For various ports, including quick reference, general information, examples and tutorials.
* [MicroPython Wiki](http://wiki.micropython.org/Home) - Community generated documentation and examples of the features of MicroPython and the pyboard.
* [MicroPython Newsletter](http://micropython.org/newsletter) - Subscribe to the MicroPython newsletter for news and announcements including new features and new products.
* [MicroPython Store](https://store.micropython.org/) - Where you can buy the pyboard, housings, skins, books, connectors and peripherals.
* [MicroPython on Wikipedia](https://en.wikipedia.org/wiki/MicroPython) - MicroPython on Wikipedia.

## Development

### Code Generation

* [micropy-cli](https://github.com/BradenM/micropy-cli) - Micropy Cli is a project management/generation tool for writing MicroPython code in modern IDEs such as VSCode.
* [micropython-stubber](https://github.com/Josverl/micropython-stubber) - Generate and use stubs for different MicroPython firmwares to use with vscode and/or pylint.

### Debugging

* [esp32-backtrace](https://github.com/tve/esp32-backtrace) - ESP32 Exception Stack Backtrace Analyzer.

### IDEs

* [JetBrains MicroPython Plugin](https://plugins.jetbrains.com/plugin/9777-micropython) - Support for MicroPython devices in IntelliJ IDEA and PyCharm.
* [Micropython IDE for VSCode](https://marketplace.visualstudio.com/items?itemName=dphans.micropython-ide-vscode) - MicroPython IDE for VSCode README.
* [Micropython-REPLink for VSCode](https://marketplace.visualstudio.com/items?itemName=SWC-Fablab.micropython-replink) - Handy shortcuts for interacting with a MicroPython REPL terminal.
* [Mu Editor](https://codewith.mu/) -  Code with Mu: a simple Python\MicroPythonb\CircuitPython editor for beginner programmers.
* [Thonny IDE](https://thonny.org/) - Thonny: Python IDE for beginners.
* [intellij-micropython](https://github.com/vlasovskikh/intellij-micropython) - Plugin for MicroPython devices in IntelliJ and PyCharm.

### Shells

#### On Device

* [upy-shell](https://github.com/dhylands/upy-shell) - A simple command line based shell for MicroPython.
* [Micropython-Editor](https://github.com/robert-hh/Micropython-Editor) - Small on-board editor for PyBoard, WiPy, ESP8266, ESP32, PyCom and Adafruit devices written in Python.

#### On Host

* [rshell](https://github.com/dhylands/rshell) - Copy or Sync files to Boards, enter REPL from your terminal.
* [ampy](https://github.com/scientifichackers/ampy) - Adafruit MicroPython Tool - Utility to interact with a MicroPython board over a serial connection.
* [mpfshell](https://github.com/wendlers/mpfshell) - A simple shell based file explorer for ESP8266 and WiPy.

## Miscellaneous

* [MicroPython Kickstarter](https://www.kickstarter.com/projects/214379695/micro-python-python-for-microcontrollers) - 1,931 backers pledged £97,803 to help bring this project to life.
* [MicroPython on the ESP8266 Kickstarter](https://www.kickstarter.com/projects/214379695/micropython-on-the-esp8266-beautifully-easy-iot) - 1,399 backers pledged £28,534 to help bring this project to life.

## Contributing

Contributions and suggestions are always welcome! Please take a look at the [contribution guidelines](https://github.com/mcauser/awesome-micropython/blob/master/contributing.md) first.

I will keep some pull requests open if I'm not sure whether those libraries are awesome, you could vote for them by adding 👍 to them.
