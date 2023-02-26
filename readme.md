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

[MicroPython](https://micropython.org/) is a lean and efficient implementation of the Python 3 programming language that includes a small subset of the Python standard library and is optimised to run on microcontrollers and in constrained environments.

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
  * [Threading](#threading)
  * [User Interface](#user-interface)
* [Community](#community)
* [Tutorials](#tutorials)
* [Books](#books)
* [Frameworks](#frameworks)
* [Resources](#resources)
* [Development](#development)
  * [Code Generation](#code-generation)
  * [Debugging](#debugging)
  * [IDEs](#ides)
  * [Logging](#logging)
  * [Shells](#shells)
* [Miscellaneous](#miscellaneous)

## Libraries

Other places you can look for MicroPython Libraries:

* [PyPi](https://pypi.org/search/?c=Programming+Language+%3A%3A+Python+%3A%3A+Implementation+%3A%3A+MicroPython) - This filter shows just the MicroPython libraries on PyPi. Note: You cannot `pip install` MicroPython libraries. See the [MicroPython docs](https://docs.micropython.org/en/latest/reference/packages.html) for more information on managing packages with MicroPython.
* [GitHub Search](https://github.com/search?q=micropython) - Search GitHub for repositories containing MicroPython.
* [GitHub Topic - MicroPython](https://github.com/topics/micropython) - Browse GitHub Topics for projects tagged with MicroPython.
* [Libraries.io](https://libraries.io/search?q=micropython) - Libraries.io query for MicroPython.
* [GitLab Explore](https://gitlab.com/explore?sort=latest_activity_desc&utf8=%E2%9C%93&name=micropython&sort=latest_activity_desc) - Explore repositories on GitLab.
* [Codeberg Explore](https://codeberg.org/explore/repos?tab=&sort=recentupdate&q=micropython) - Explore repositories on Codeberg.

### AI

* [MicroMLP](https://github.com/jczic/MicroMLP) - A micro neural network multilayer perceptron for MicroPython (used on ESP32 and Pycom modules).
* [MicroPython-NeuralNetwork](https://gitlab.com/olivierlenoir/MicroPython-NeuralNetwork) - Neural Network for MicroPython.

### Analytics

* [uMath](https://github.com/albaEDA/uMath) - Computer Algebra for microcontrollers.
* [micropython-ulab](https://github.com/v923z/micropython-ulab) - A NumPy-like fast vector module for MicroPython.
* [micropython-fourier](https://github.com/peterhinch/micropython-fourier) - Fast Fourier transform in MicroPython's inline ARM assembler.
* [Filters](https://github.com/peterhinch/micropython-filters) - FIR filters using ARM Thumb assembler. Using an online utility you can go from a graph
of required frequency response to a filter implementation.
* [ulinalg](https://github.com/jalawson/ulinalg) - Small size matrix handling module with a few linear algebra operations specifically for MicroPython (Python 3).
* [micropython-mtx](https://gitlab.com/nickoala/micropython-mtx) - Fast Matrix Multiplication and Linear Solver on MicroPython.
* [micropython-vec](https://gitlab.com/nickoala/micropython-vec) - Vector Operations on MicroPython.
* [MicroPython_Statistics](https://github.com/rcolistete/MicroPython_Statistics) - Statistics module for MicroPython.
* [MicroPython-Matrix](https://gitlab.com/olivierlenoir/MicroPython-Matrix) - MicroPython basic matrix operations.

### Audio

* [micropython-jq6500](https://github.com/rdagger/micropython-jq6500) - Driver for JQ6500 UART MP3 modules.
* [KT403A-MP3](https://github.com/jczic/KT403A-MP3) - Driver for KT403A, used by DFPlayer Mini and Grove MP3 v2.0.
* [micropython-buzzer](https://github.com/fruch/micropython-buzzer) - Play Nokia compose and mid files on buzzers.
* [micropython-dfplayer](https://github.com/ShrimpingIt/micropython-dfplayer) - Driver for DFPlayer Mini using UART.
* [micropython-longwave](https://github.com/MattMatic/micropython-longwave) - WAV player for MicroPython board.
* [micropython-vs1053](https://github.com/peterhinch/micropython-vs1053) - Asynchronous driver for VS1053b MP3 player.
* [micropython-midi](https://github.com/cjbarnes18/micropython-midi) - A MIDI implementation example for MicroPython.
* [upy-rtttl](https://github.com/dhylands/upy-rtttl) - Python Parser for Ring Tone Text Transfer Language (RTTTL).
* [micropython-i2s-examples](https://github.com/miketeachman/micropython-i2s-examples) - Examples for I2S support on microcontrollers that run MicroPython.
* [micropython-osc](https://github.com/SpotlightKid/micropython-osc) - A minimal OSC client and server library for MicroPython.

### Communications

#### APIs

* [micropython-utelegram](https://github.com/jordiprats/micropython-utelegram) - Telegram API wrapper for MicroPython.
* [uEagle](https://github.com/jcalbert/uEagle) - MicroPython Rainforest EAGLE client.
* [micropython-youtube-api](https://github.com/UnexpectedMaker/micropython-youtube-api) - YouTube API in MicroPython.
* [micropython_esp8266_tweetbot](https://github.com/ayoko/micropython_esp8266_tweetbot) - Tweet bot for MicroPython v1.8.4 (ESP8266).
* [telegram-upy](https://github.com/gabrielebarola/telegram-upy) - Telegram API wrapper for MicroPython.
* [micropython-thingspeak](https://github.com/radeklat/micropython-thingspeak) - Library for sending data to thingspeak.com from IoT devices running MicroPython (such as ESP8266).
* [micropython_pushbullet](https://github.com/gsampallo/micropython_pushbullet) - Simple example of how to use PushBullet with MicroPython on ESP8266.
* [esp32-youtube-display](https://github.com/alvarowolfx/esp32-youtube-display) - Display YouTube metrics using Google API and MicroPython.
* [micropython-spotify-web-api](https://github.com/tltx/micropython-spotify-web-api) - A library for using Spotify's web API from a IoT device with MicroPython.

#### Authentication

* [micropython-firebase-auth](https://github.com/WoolDoughnut310/micropython-firebase-auth) - Firebase Auth implementation for MicroPython.

#### Bluetooth

* [PyBoard-HC05-Android](https://github.com/KipCrossing/PyBoard-HC05-Android) - Pyboard HC05 Bluetooth adapter example application.
* [uble](https://github.com/dmazzella/uble) - Lightweight Bluetooth Low Energy driver written in pure Python for MicroPython.
* [MicroPythonBLEHID](https://github.com/Heerkog/MicroPythonBLEHID) - Human Interface Device (HID) over Bluetooth Low Energy (BLE) GATT library for MicroPython.
* [upyble](https://github.com/Carglglz/upyble) - Command line tool for Bluetooth Low Energy MicroPython devices.
* [micropython-xiaomi-ble-adv-parse](https://codeberg.org/scy/micropython-xiaomi-ble-adv-parse) - Passively retrieve sensor data from some Xiaomi Bluetooth Low Energy (BLE) sensors.
* [mijia-temphum-upy](https://codeberg.org/scy/mijia-temphum-upy) - MicroPython library to read certain Xiaomi Mijia BLE temperature & humidity sensors.

#### CAN

* [micropython-spacecan](https://gitlab.com/alphaaomega/micropython-spacecan) - Spacecan is a MicroPython implementation of the SpaceCAN protocol for embedded systems.
* [Robomaster-Micropython](https://github.com/JohnieBraaf/Robomaster-Micropython) - Robomaster S1 - MicroPython CAN BUS controller.
* [micropython-mcp2515](https://github.com/jxltom/micropython-mcp2515) - MicroPython MCP2515 driver, porting from Arduino MCP2515 CAN interface library.

#### Compression

* [ufastlz](https://github.com/dmazzella/ufastlz) - MicroPython wrapper for FastLZ, a lightning-fast lossless compression library.

#### Cryptography

* [mpyaes](https://github.com/iyassou/mpyaes) - MicroPython module for AES encryption.
* [micropython-aes](https://github.com/piaca/micropython-aes) - AES algorithm with pure python implementation.
* [ucrypto](https://github.com/dmazzella/ucrypto) - MicroPython package for doing fast RSA and elliptic curve cryptography, specifically digital signatures. ECDSA API design inspired from fastecdsa and implementation based on tomsfastmath.
* [ucryptoauthlib](https://github.com/dmazzella/ucryptoauthlib) - Lightweight driver for Microchip Crypto Authentication secure elements written in pure Python for MicroPython.
* [embit](https://github.com/diybitcoinhardware/embit) - A minimal Bitcoin library for MicroPython and Python 3 with a focus on embedded systems.
* [microotp](https://github.com/gdassori/microotp) - An ESP8266 MicroPython OTP Generator.
* [micropython-rsa-signing](https://github.com/artem-smotrakov/micropython-rsa-signing) - RSA signing on MicroPython.
* [micropython-cryptomsg](https://github.com/jacklinquan/micropython-cryptomsg) - A MicroPython module to encrypt and decrypt messages with AES CBC mode.

#### DNS

* [ICantBelieveItsNotDNS](https://github.com/yschaeff/ICantBelieveItsNotDNS) - "I Can't Believe It's Not DNS!" (ICBIND) is an authoritative DNS server for the ESP8266 written in MicroPython.
* [MicroDNSSrv](https://github.com/jczic/MicroDNSSrv) - A micro DNS server for MicroPython to simply respond to A queries on multi-domains with or without wildcards (used on Pycom modules & ESP32).
* [tinydns](https://github.com/belyalov/tinydns) - Very simple DNS async server for MicroPython.
* [micropython-captiveportal](https://github.com/metachris/micropython-captiveportal) -  Minimal async captive portal for MicroPython (compatible with uasyncio v3/MicroPython 1.13+ as well as earlier versions).
* [Micropython-DNSServer-Captive-Portal](https://github.com/p-doyle/Micropython-DNSServer-Captive-Portal) - MicroPython WiFi AP Captive Portal with DNS and Web Server.

#### Ethernet

* [Official WIZnet5k](https://github.com/andrewleech/wiznet_ioLibrary_Driver) - Driver for the WIZnet5x00 series of Ethernet controllers.
* [micropy-ENC28J60](https://github.com/przemobe/micropy-ENC28J60) - ENC28J60 Ethernet chip driver for MicroPython (RP2).
* [RP2040 Ethernet example](https://github.com/SteveSEK/Raspberry-Pi-Pico-MicroPython-Ethernet) - Ethernet driver, example Python code and YouTube.

#### FTP

* [micropython-ftplib](https://github.com/SpotlightKid/micropython-ftplib) - An FTP client library for MicroPython.
* [FTP-Server-for-ESP8266-ESP32-and-PYBD](https://github.com/robert-hh/FTP-Server-for-ESP8266-ESP32-and-PYBD) - Small FTP server for ESP8266/ESP32/Pyboard on the MicroPython platform.
* [MicroFTPServer](https://github.com/cpopp/MicroFTPServer) - Minimal FTP Server that can run on an ESP8266 with MicroPython.
* [micropython-uaioftp](https://github.com/cwyark/micropython-uaioftp) - Lightweight FTP library for MicroPython.

#### GPS

* [micropyGPS](https://github.com/inmcm/micropyGPS) - Full featured GPS NMEA sentence parser.
* [micropython-gnssl76l](https://github.com/tuupola/micropython-gnssl76l) - MicroPython I2C driver for Quectel GNSS L76-L (GPS).
* [mpy-agps](https://github.com/pulkin/mpy-agps) - MicroPython implementation of assisted location services (AGPS).
* [Asynchronous GPS driver](https://github.com/peterhinch/micropython-async/blob/master/v3/docs/GPS.md) - Receive and parse GPS data as a uasyncio task.

#### GSM

* [micropython-upyphone](https://github.com/jeffmer/micropython-upyphone) - A GSM phone using Pyboard and SIM800l.
* [micropython-sim800](https://github.com/olablt/micropython-sim800) - MicroPython driver for SIM800.
* [sim800](https://github.com/basanovase/sim800) - Library for interfacing with SIM800 module in MicroPython.
* [MicroPython-AM7020](https://github.com/JiekangHuang/MicroPython-AM7020) - MicroPython driver for AM7020 Narrowband Internet of Things (NBIoT) module.

#### IoT

* [microhomie](https://github.com/microhomie/microhomie) - MicroPython implementation of the Homie MQTT convention for IoT.
* [uPyEcho](https://github.com/lemariva/uPyEcho) - Emulated Belkin WeMo device that works with Amazon Echo (Alexa) using MicroPython on an ESP32.
* [SonosRemote](https://github.com/foosel/SonosRemote) - A remote for Sonos installations running on an ESP8266 and using Sonos HTTP API.
* [micropython-home-assistant](https://gitlab.com/aapjeisbaas/micropython-home-assistant) - MicroPython-based scripts to extend your Home Assistant-driven home automation projects.
* [micropython-iot](https://github.com/peterhinch/micropython-iot) - An approach to designing IoT applications using ESP8266, ESP32 or Pyboard D endpoints.
* [iot-core-micropython](https://github.com/GoogleCloudPlatform/iot-core-micropython) - Use MicroPython to connect to Google Cloud IoT Core.
* [SmartUPy](https://github.com/lemariva/SmartUPy) - Controlling "Tuya-type" smart power outlets using MicroPython.
* [aws-iot-GET-POST-loop](https://github.com/manningt/aws-iot-GET-POST-loop) - MicroPython code which uses the AWS IoT REST API to GET/POST device state info.
* [sensor-mqtt-homeassistant](https://github.com/DougWilkinson/sensor-mqtt-homeassistant) - An ESP8266/ESP32 MicroPython-based sensor platform for GPIO, DHT, analog, LED and more. Includes remote updates for .py code from web server and MQTT/Home Assistant integration.

#### IR

* [micropython-necir](https://github.com/MattMatic/micropython-necir) - NEC infrared capture for TL1838 IR receiver LEDs.
* [Micropython-IR](https://github.com/designerPing/Micropython-IR) - Pyboard infrared remote sniff and replay.
* [micropython_ir](https://github.com/peterhinch/micropython_ir) - Nonblocking device drivers to receive from IR remotes and for IR "blaster" apps.
* [micropython-amg88xx](https://github.com/peterhinch/micropython-amg88xx) - Driver for Grid-EYE thermal infrared array sensor (Adafruit 3538).
* [micropython-ys-irtm](https://github.com/mcauser/micropython-ys-irtm) - MicroPython examples for YS-IRTM 5V NEC Infrared UART transceivers.
* [esp8266_ir](https://github.com/ruoyu0088/esp8266_ir) - Control IR signal by WebSocket.
* [micropython_espX_IR_Transceiver](https://github.com/gamefunc/micropython_espX_IR_Transceiver) - MicroPython ESP32 IR Transceiver.
* [pico-ir](https://github.com/bartoszadamczyk/pico-ir) - IR library for Raspberry Pi Pico.

#### LoRaWAN

* [uPyLoRaWAN](https://github.com/lemariva/uPyLoRaWAN) - ESP32 using MicroPython meets LoRa and LoRaWAN.
* [SX127x_driver_for_MicroPython_on_ESP8266](https://github.com/Wei1234c/SX127x_driver_for_MicroPython_on_ESP8266) - SX127x (LoRa transceiver) driver for (Micro)Python on ESP8266/ESP32/Raspberry Pi.
* [LightLora_MicroPython](https://github.com/MZachmann/LightLora_MicroPython) - Lightweight Interrupt-driven Semtech SX127x Library for MicroPython.
* [u-lora](https://github.com/martynwheeler/u-lora) - Raspi-lora for MicroPython.
* [sx127x_esp](https://github.com/azorg/sx127x_esp) - Connect Ra-01 module base on LoRaTM sx127x chip to ESP8266/ESP32 under MicroPython.
* [nanoserver](https://github.com/gradoj/nanoserver) - MicroPython embedded LoRaWAN server.
* [micropySX126X](https://github.com/ehong-tl/micropySX126X) - Semtech SX126X LoRa driver for MicroPython and CircuitPython.

#### MDNS

* [micropython-mdns](https://github.com/cbrand/micropython-mdns) - A pure Python implementation of MDNS with support for Service Discovery.

#### Modbus

* [micropython-modbus](https://gitlab.com/extel-open-source/micropython-modbus) - MicroPython port of modbus-tk.
* [micropython-modbus](https://github.com/techbase123/micropython-modbus) - Modbus Master library for MicroPython ESP32 devices. Based on pycom-modbus from Pycom.
* [mp_modbus](https://github.com/eydam-prototyping/mp_modbus) - Modbus library for MicroPython.
* [micropython-modbus](https://github.com/brainelectronics/micropython-modbus) - ModBus TCP and RTU library supporting client and host mode. Based on pycom-modbus from Pycom.

#### MQTT

* [micropython-mqtt](https://github.com/peterhinch/micropython-mqtt) - A 'resilient' asynchronous MQTT driver. Plus a means of using an ESP8266 to bring MQTT to non-networked targets.
* [MQBoard](https://github.com/tve/mqboard) - A micro-framework for using MQTT with asyncio on MicroPython boards, primarily on the ESP32.
* [pysmartnode](https://github.com/kevinkk525/pysmartnode) -  MicroPython Smart Home framework.
* [umqtt_aws_iot](https://github.com/juwul/umqtt_aws_iot) - Publish UMQTT messages with MicroPython to AWS IoT.
* [sonoff-mqtt by davea](https://github.com/davea/sonoff-mqtt) - MicroPython scripts to control Sonoff/ESP8266 using MQTT.
* [micropython-sonoff-switch](https://github.com/kfricke/micropython-sonoff-switch) - Implements an MQTT-controllable switch for the iTead Sonoff Switch using MicroPython.
* [micropython-thingspeak-mqtt-esp8266](https://github.com/miketeachman/micropython-thingspeak-mqtt-esp8266) - Publish and Subscribe to ThingSpeak using MQTT with MicroPython running on ESP8266/ESP32 platforms.
* [uMQTT](https://github.com/andrewmk/uMQTT) - MQTT publish for MicroPython on the WiPy board.
* [micropython-mqtt](https://github.com/chrismoorhouse/micropython-mqtt) - Async MQTT library with auto reconnect for MicroPython devices such as the ESP32 or Pycom devices.
* [micropython-adafruit-mqtt-esp8266](https://github.com/miketeachman/micropython-adafruit-mqtt-esp8266) - Using MQTT to Publish/Subscribe to Adafruit IO. MicroPython/CircuitPython implementation on ESP8266/ESP32.
* [MicropythonCayenneMQTTClient](https://github.com/uraich/MicropythonCayenneMQTTClient) - A port of the Python Cayenne MQTT Client to MicroPython.
* [mqtt_upython](https://github.com/matbgn/mqtt_upython) - MQTT Client using MicroPython on ESP8266.

#### NFC

* [micropython-nfc](https://github.com/rolandvs/micropython-nfc) - Using NFC with MicroPython.
* [micropython_pn532](https://github.com/luiz-brandao/micropython_pn532) - Driver for PN532 NFC/RFID breakout boards based on Adafruit CircuitPython (UART).
* [NFC_PN532_SPI](https://github.com/Carglglz/NFC_PN532_SPI) - Partial port of Adafruit CircuitPython to MicroPython of PN532 NFC/RFID control library (SPI).

#### NTP

* [esp8266_ntp_webserver](https://github.com/Roterfux/esp8266_ntp_webserver) - MicroPython + ESP8266 + NTP + web server.
* [micropython-ntpd](https://github.com/dave2/micropython-ntpd) - An implementation of an NTP daemon in MicroPython.
* [micropython_ntpserver](https://github.com/GrantGMiller/micropython_ntpserver) - An NTP server written for MicroPython.
* [micropython-ntpclient](https://github.com/wieck/micropython-ntpclient) - NTP client for MicroPython using uasyncio.

#### OneWire

* [Official OneWire](https://github.com/micropython/micropython-lib/tree/master/micropython/drivers/bus/onewire) - For devices using the OneWire bus, eg Dallas DS18x20.
* [Onewire_DS18X20](https://github.com/robert-hh/Onewire_DS18X20) - Classes for driving the DS18x20 sensor with the OneWire protocol for Pycom MicroPython.

#### Onkyo EISCP

* [eiscp-micropython](https://github.com/cbrand/eiscp-micropython) - MicroPython port for the Onkyo-EISCP protocol used, among others, by Pioneer.

#### OTA

* [micropython-ota-updater](https://github.com/rdehuyss/micropython-ota-updater) - OTA Updater for MicroPython.
* [Micropython-ESP32-OTA](https://github.com/AkhileshThorat/Micropython-ESP32-OTA) - MicroPython updater based on rdehuyss/micropython-ota-updater.
* [senko](https://github.com/RangerDigital/senko) - Simplest OTA update solution for your MicroPython projects.

#### Radio

* [micropython-radio](https://github.com/peterhinch/micropython-radio) - Protocols for nRF24L01 2.4GHz radio modules.
* [micropython-rfsocket](https://github.com/wuub/micropython-rfsocket) - MicroPython implementation of popular 433MHz-based RFSockets.
* [Official nRF24L01](https://github.com/micropython/micropython-lib/tree/master/micropython/drivers/radio/nrf24l01) - Official driver for nRF24L01 2.4GHz radio modules.
* [micropython_remote](https://github.com/peterhinch/micropython_remote) - Capture and replay 433MHz remote control codes. Control remote switched power adaptors.
* [micropython-ys-rf34t](https://github.com/mcauser/micropython-ys-rf34t) - MicroPython examples using YS-RF34T 433MHz ASK/OOK UART transceivers.
* [FM_Talkie](https://github.com/Wei1234c/FM_Talkie) - FM Walkie Talkie using RDA5820N.
* [micropython-TEA5767](https://github.com/alankrantas/micropython-TEA5767) - MicroPython ESP8266/ESP32 driver for TEA5767 FM radio module.
* [micropython-ppm-decoder](https://github.com/dastultz/micropython-ppm-decoder) - Utility for decoding an R/C receiver PPM frame signal.
* [ESP32-433Mhz-Receiver-and-Tools](https://github.com/Aschhoff/ESP32-433Mhz-Receiver-and-Tools) - ESP32 433MHz receiver written in MicroPython and tools for Windows.

#### REPL

* [webrepl](https://micropython.org/webrepl) - MicroPython WebREPL.
* [zepl](https://gitlab.com/zepl1/zepl) - MicroPython WebREPL Console Application using ZeroMQ.
* [jupyter_micropython_remote](https://gitlab.com/alelec/jupyter_micropython_remote) - Jupyter kernel to directly execute code on a MicroPython board over the serial/web REPL.
* [FBConsole](https://github.com/boochow/FBConsole) - Framebuffer console class for MicroPython.

#### RFID

* [micropython-mfrc522](https://github.com/wendlers/micropython-mfrc522) - Driver for NXP MFRC522 RFID reader/writer.
* [micropython-wiegand](https://github.com/pjz/micropython-wiegand) - Wiegand protocol reader.
* [urdm6300](https://github.com/membermatters/urdm6300) - A MicroPython driver for the popular RDM6300 RFID card reader.

#### RTC

* [micropython-tinyrtc-i2c](https://github.com/mcauser/micropython-tinyrtc-i2c) - Driver for DS1307 RTC and AT24C32N EEPROM.
* [Micropython_TinyRTC](https://github.com/AnthonyKNorman/Micropython_TinyRTC) - Driver for DS1307 RTC.
* [micropython-mcp7940](https://github.com/mattytrentini/micropython-mcp7940) - Driver for the Microchip MCP7940 RTC.
* [micropython-ds1302-rtc](https://github.com/omarbenhamid/micropython-ds1302-rtc) - DS1302 RTC Clock driver for MicroPython.
* [DS3231micro](https://github.com/notUnique/DS3231micro) - MicroPython library for DS3231.

#### Serial

* [mpy-miniterm](https://github.com/jeffmakes/mpy-miniterm) - Tool for seamless serial debug and file synchronisation with MicroPython devices via the serial REPL.
* [MicroPython-MorseCode](https://gitlab.com/olivierlenoir/MicroPython-MorseCode) - International Morse Code using a microcontroller with MicroPython.
* [I2C Slave](https://github.com/peterhinch/micropython-async/blob/master/v3/docs/I2C.md) - Uses the Pyboard's I2C slave mode to implement a full duplex asynchronous link. Principal use case is for ESP8266 which has only one UART.

#### Serialization

* [micropython-msgpack](https://github.com/peterhinch/micropython-msgpack) - MessagePack serialisation library optimised for MicroPython.
* [micropython-uprotobuf](https://github.com/jazzycamel/micropython-uprotobuf) - A lightweight implementation of Google's Protocol Buffers (protobuf) for MicroPython.
* [minipb](https://github.com/dogtopus/minipb) - Mini Protobuf {de}serializer in pure Python.

#### SOCKS

* [micropython-socks](https://github.com/kost/micropython-socks) - MicroPython library implementing SOCKS server.

#### SMTP

* [uMail](https://github.com/shawwwn/uMail) - A lightweight, scalable SMTP client for sending email in MicroPython.

#### TCP

* [us2n](https://github.com/tiagocoutinho/us2n) - MicroPython bridge between UART and TCP for the ESP32.

#### Telnet

* [MicroTelnetServer](https://github.com/cpopp/MicroTelnetServer) - Simple telnet server for MicroPython and the ESP8266 allowing telnet clients access to the REPL.

#### VoIP

* [uPyVoip](https://github.com/RetepRelleum/uPyVoip) - VoIP for MicroPython ESP32 with Interactive Voice Response.

#### WiFi

* [HueBridge](https://github.com/FRC4564/HueBridge) - Philips Hue Bridge.
* [micropython-wifimanager](https://github.com/mitchins/micropython-wifimanager) - A simple network configuration utility for MicroPython on the ESP8266 board.
* [WiFiManager](https://github.com/tayfunulu/WiFiManager) - WiFi manager for ESP8266 - ESP12 - ESP32 - MicroPython.
* [Micropython-ESP-WiFi-Manager](https://github.com/brainelectronics/Micropython-ESP-WiFi-Manager) - WiFi Manager to configure and connect to networks.

#### Web

* [MicroWebSrv](https://github.com/jczic/MicroWebSrv) - A micro HTTP web server that supports WebSockets, HTML/Python language templating and routing handlers, for MicroPython (used on Pycom modules & ESP32).
* [MicroWebSrv2](https://github.com/jczic/MicroWebSrv2) - The last micro web server for IoTs (MicroPython) or large servers (CPython), that supports WebSocket, routes, template engine and with really optimized architecture (mem allocations, async I/Os).
* [tinyweb](https://github.com/belyalov/tinyweb) - Simple and lightweight HTTP async server for MicroPython.
* [upy-websocket-server](https://github.com/BetaRavener/upy-websocket-server) - MicroPython (ESP8266) WebSocket server implementation.
* [micropython-captive-portal](https://github.com/amora-labs/micropython-captive-portal) - A captive portal demo for MicroPython.
* [uPyPortal](https://github.com/lemariva/uPyPortal) - A captive portal for MicroPython using ESP32 (Wemos).
* [ESP8266WebServer](https://github.com/codemee/ESP8266WebServer) - ESP8266 web server for MicroPython.
* [microCoAPy](https://github.com/insighio/microCoAPy) - A mini client/server implementation of CoAP (Constrained Application Protocol) into MicroPython.
* [micropyserver](https://github.com/troublegum/micropyserver) - MicroPyServer is a simple HTTP server for MicroPython projects.
* [MicroRESTCli](https://github.com/jczic/MicroRESTCli) - A micro JSON REST web client based on MicroWebCli for MicroPython (used on Pycom modules & ESP32).
* [micropython-noggin](https://github.com/larsks/micropython-noggin) - A very simple web server for MicroPython.
* [uwebsockets](https://github.com/danni/uwebsockets) - MicroPython WebSocket implementation for ESP8266.
* [microdot](https://github.com/miguelgrinberg/microdot) - The impossibly small web framework for MicroPython.
* [micropython-nanoweb](https://github.com/hugokernel/micropython-nanoweb) - Full async MicroPython web server with small memory footprint.
* [MicroWebCli](https://github.com/jczic/MicroWebCli) - A micro HTTP web client for MicroPython (used on Pycom modules & ESP32).
* [micropython-configserver](https://github.com/carstenblt/micropython-configserver) - Captive portal for MicroPython including a dumb DNS server and a web server to configure WiFi networks.
* [micropython-aioweb](https://github.com/wybiral/micropython-aioweb) - A minimalist asyncio web framework for MicroPython.
* [thimble](https://github.com/DavesCodeMusings/thimble) - A tiny web framework for MicroPython.

#### Zigbee

* [ZbPy](https://github.com/osresearch/ZbPy) - MicroPython IEEE802.15.4 / Zigbee parser.

### Display

#### E-Paper

* [micropython-ili9341](https://github.com/mcauser/deshipu-micropython-ili9341) - SSD1606 active matrix ePaper display 128x180.
* [micropython-waveshare-epaper](https://github.com/mcauser/micropython-waveshare-epaper) - Drivers for various Waveshare ePaper modules.
* [micropython-waveshare-epd](https://github.com/ayoy/micropython-waveshare-epd) - Waveshare ePaper Display driver for devices running Pycom-flavored MicroPython.
* [ssd1675a](https://github.com/mattytrentini/ssd1675a) - Driver for SSD1675-based e-paper displays.
* [Inkplate-micropython](https://github.com/SolderedElectronics/Inkplate-micropython) - MicroPython driver for Inkplate boards.
* [micropython-inkplate6](https://github.com/tve/micropython-inkplate6) - MicroPython driver for the Inkplate 6.
* [eInk-micropython](https://github.com/dhallgb/eInk-micropython) - eInk library for Waveshare 4.3inch device on MicroPython.
* [eink](https://github.com/chevdor/eink) - An eInk, ePaper display driver for MicroPython and ESP32.
* [micropython_DEPG0213BN](https://github.com/Inqbus/micropython_DEPG0213BN) - Pure MicroPython driver for the DEPG0213BN eInk display found on the TTGO T5 V2.3 ESP32 boards.

#### Fonts

* [micropython-font-to-py](https://github.com/peterhinch/micropython-font-to-py) - A Python 3 utility to convert fonts to Python source capable of being frozen as bytecode.
* [writer](https://github.com/peterhinch/micropython-font-to-py/blob/master/writer/WRITER.md) - A simple way to render above Python fonts to displays whose driver is subclassed from `framebuf`.
* [ssd1306big](https://github.com/nickpmulder/ssd1306big) - A font for MicroPython on 128x64 pixel SSD1306 OLED display.

#### Graphics

* [micropython-stage](https://github.com/python-ugame/micropython-stage) - A MicroPython port of the Stage game library.
* [micropython-png](https://github.com/Ratfink/micropython-png) - Derivative of PyPNG for use with MicroPython.
* [mpy-img-decoder](https://github.com/remixer-dec/mpy-img-decoder) - PNG and JPEG decoder / parser / renderer in pure MicroPython.
* [micropython-oled-progressbars](https://github.com/follower46/micropython-oled-progressbars) - A collection of progress bars for use with ESP8266 and ESP32 on OLED displays.
* [microplot](https://github.com/romilly/microplot) - Simple MicroPython plotting package.

#### GUI

* [lvgl](https://github.com/lvgl/lv_binding_micropython) - An object-oriented, component-based high-level GUI library with MicroPython binding.
* [micropython-lcd160cr-gui](https://github.com/peterhinch/micropython-lcd160cr-gui) - Simple touch-driven event based GUI for the Pyboard and LCD160CR colour display.
* [micropython_ra8875](https://github.com/peterhinch/micropython_ra8875) - MicroPython device driver and nano-GUI for RA8875 based displays.
* [micropython-nano-gui](https://github.com/peterhinch/micropython-nano-gui) - A tiny display-only GUI with a limited set of GUI objects (widgets) for displays whose display driver is subclassed from the `framebuf` class. With drivers for TFT, ePaper and OLED displays.
* [micro-gui](https://github.com/peterhinch/micropython-micro-gui) - Derived from nano-gui and supporting the same displays and hosts, this provides for user
input via push buttons or a navigation joystick and an optional rotary encoder.
* [TFT-GUI](https://github.com/peterhinch/micropython-tft-gui) - A fast touch GUI for large displays based on SSD1963 controller with XPT2046 touch controller.
* [micropython-nextion](https://github.com/brainelectronics/micropython-nextion) - Control Nextion displays using MicroPython.

#### LCD Character

* [Grove_RGB_LCD](https://github.com/dda/MicroPython/blob/master/Grove_RGB_LCD.py) - Driver for SeeedStudio's Grove RGB LCD.
* [lcdi2c](https://github.com/slothyrulez/lcdi2c) - Driver for HD44780-compatible dot matrix LCDs.
* [micropython-charlcd](https://github.com/rdagger/micropython-charlcd) - Driver for HD44780-compatible LCDs.
* [micropython-i2c-lcd](https://github.com/Bucknalla/micropython-i2c-lcd) - Driver for I2C 2x16 LCD Screens.
* [pyboard-LCD-character-display](https://github.com/scitoast/pyboard-LCD-character-display) - Pyboar driver for HDD44780-compatible 1602 LCDs.
* [python_lcd](https://github.com/dhylands/python_lcd) - Driver for HD44780-compatible dot matrix LCDs.
* [micropython-lcd](https://github.com/wjdp/micropython-lcd) - Class for controlling the HD44780 from a MicroPython Pyboard.
* [HD44780-lcd-upy](https://gitlab.com/rafalosa/HD44780-lcd-upy) - MicroPython module for controlling a generic HD44780 LCD.
* [LCM1602-14_LCD_Library](https://github.com/Bhavithiran97/LCM1602-14_LCD_Library) - driver for AIP31068L [3.3 V I2C and SPI 1602 Serial Character LCDs](https://www.cytron.io/p-3v3-i2c-and-spi-1602-serial-character-lcd).

#### LCD Graphic

* [micropython-lcd-AQM1248A](https://github.com/forester3/micropython-lcd-AQM1248A) - ESP8266 driver for AQM1248A graphic LCD.
* [micropython-pcd8544](https://github.com/mcauser/micropython-pcd8544) - Driver for Nokia 5110 PCD8544 84x48 LCD modules.
* [micropython-st7565](https://github.com/nquest/micropython-st7565) - Driver for ST7565 128x64 LCDs.
* [micropython-st7920](https://github.com/ShrimpingIt/micropython-st7920) - Library for simple graphic primitives on ST7920 128x64 monochrome LCD panel using ESP8266 and SPI.
* [MicroPython_PCD8544](https://github.com/AnthonyKNorman/MicroPython_PCD8544) - ESP8266 driver for Nokia 5110 PCD8544.
* [Official LCD160CR](https://github.com/micropython/micropython-lib/tree/master/micropython/drivers/display/lcd160cr) - Driver for official MicroPython LCD160CR display with resistive touch sensor.
* [micropython-hx1230](https://github.com/mcauser/micropython-hx1230) - MicroPython library for HX1230 96x68 LCD modules.
* [micropython-SHARP_Memory_Display](https://github.com/pramasoul/micropython-SHARP_Memory_Display) - MicroPython driver for SHARP memory display.

#### LCD TFT

* [micropython-ili9341](https://github.com/mcauser/deshipu-micropython-ili9341) - Collection of drivers for TFT displays, ILI9341, SH1106, SSD1606, ST7735.
* [micropython-ili934x](https://github.com/tuupola/micropython-ili934x) - SPI driver for ILI934X series based TFT / LCD displays.
* [MicroPython-ST7735](https://github.com/boochow/MicroPython-ST7735) - ESP32 version of GuyCarvers's ST7735 TFT LCD driver.
* [micropython-st7735](https://github.com/hosaka/micropython-st7735) - Driver for ST7735 TFT LCDs.
* [MicroPython_ST7735](https://github.com/AnthonyKNorman/MicroPython_ST7735) - Driver for ST7735 128x128 TFT.
* [SSD1963-TFT-Library-for-PyBoard-and-RP2040](https://github.com/robert-hh/SSD1963-TFT-Library-for-PyBoard-and-RP2040) - SSD1963 TFT Library for Pyboard and Raspberry Pi Pico.
* [ST7735](https://github.com/GuyCarver/MicroPython/blob/master/lib/ST7735.py) - Driver for ST7735 TFT LCDs.
* [micropython-ili9341](https://github.com/rdagger/micropython-ili9341) - MicroPython ILI9341 display & XPT2046 touch screen driver.
* [st7789_mpy](https://github.com/devbis/st7789_mpy) - Fast pure-C driver for MicroPython that can handle display modules on ST7789 chip.
* [st7789py_mpy](https://github.com/devbis/st7789py_mpy) - Slow MicroPython driver for 240x240 ST7789 display without CS pin from AliExpress, written in MicroPython.
* [micropython-ili9341](https://github.com/jeffmer/micropython-ili9341) - MicroPython Driver for ILI9341 display.
* [micropython-ili9341](https://github.com/tkurbad/micropython-ili9341) - ILI9341 TFT driver for MicroPython on ESP32.
* [st7789_mpy](https://github.com/russhughes/st7789_mpy) - Fast MicroPython driver for ST7789 display module written in C.
* [st7789py_mpy](https://github.com/russhughes/st7789py_mpy) - Driver for 320x240, 240x240 and 135x240 ST7789 displays written in MicroPython.
* [ili9342c_mpy](https://github.com/russhughes/ili9342c_mpy) - ILI9342C Fast 'C' Driver for MicroPython (M5Stack Core).
* [gc9a01py](https://github.com/russhughes/gc9a01py) - GC9A01 Display driver in MicroPython.
* [gc9a01_mpy](https://github.com/russhughes/gc9a01_mpy) - Fast MicroPython driver for GC9A01 display modules written in C.
* [st7735-esp8266-micropython](https://github.com/cheungbx/st7735-esp8266-micropython) - An ESP8266 MicroPython library for ST7735 160x80, 128x128, 128x160 TFT LCD displays.
* [TTGO-ST7789-MicroPython](https://github.com/schumixmd/TTGO-ST7789-MicroPython) - MicroPython ST7789 display driver for TTGO T-Display ESP32 CP2104 WiFi Bluetooth Module 1.14 Inch LCD.
* [st7735_micropython](https://github.com/cheungbx/st7735_micropython) - ST7735 MicroPython drivers for 80x160, 128x128, 128x160 for ESP8266.
* [ili934x-micropython](https://gitlab.com/mhepp63/ili934x-micropython) - Library for using ILI9341 display drivers with MicroPython.
* [micropython-st7735-esp8266](https://gitlab.com/mo_krauti/micropython-st7735-esp8266) - MicroPython driver for ST7735 TFT displays on the ESP8266.

#### LED Matrix

* [micropython-ht1632c](https://github.com/vrialland/micropython-ht1632c) - Driver for HT1632C 32x16 bicolor LED matrix.
* [micropython-matrix8x8](https://github.com/JanBednarik/micropython-matrix8x8) - Driver for Adafruit 8x8 LED Matrix display with HT16K33 backpack.
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
* [TM74HC595](https://github.com/Sakartu/TM74HC595) - Driver for shift register-controlled 5 pin display modules.

#### LEDs

* [micropython-morsecode](https://github.com/mampersat/micropython-morsecode) - Blink an LED with Morse Coded message.
* [micropython-p9813](https://github.com/mcauser/micropython-p9813) - Driver for P9813 RGB LED used in SeeedStudio's Grove chainable RGB LED.
* [micropython-ws2812-7seg](https://github.com/HubertD/micropython-ws2812-7seg) - 7-segment display using WS2812 RGB LEDs.
* [micropython-ws2812](https://github.com/JanBednarik/micropython-ws2812) - Driver for WS2812 RGB LEDs.
* [Official APA102](https://docs.micropython.org/en/latest/esp8266/quickref.html#apa102-driver) - ESP8266 APA102/DotStar RGB LED driver.
* [Official WS2811](https://docs.micropython.org/en/latest/esp8266/quickref.html#neopixel-driver) - ESP8266 WS2811/NeoPixel RGB LED driver.
* [tlc5940-micropython](https://github.com/oysols/tlc5940-micropython) - Driver for TLC5940 16 channel LED driver.
* [ws2812-SPI](https://github.com/nickovs/ws2812-SPI) - An efficient MicroPython WS2812 (NeoPixel) driver.
* [micropython-ws2801](https://github.com/HeMan/micropython-ws2801) - A MicroPython library to interface with strands of WS2801 RGB LEDs.
* [tlc5947-rgb-micropython](https://gitlab.com/peterzuger/tlc5947-rgb-micropython) - Driver for the TLC5947 24 channel 12-bit PWM LED driver.
* [Hybotics_Micropython_HT16K33](https://github.com/hybotics/micropython-ht16k33) - MicroPython driver for the HT16K33, a LED matrix, 7-Segment Numeric, and 14-Segment Alphanumeric display driver IC.
* [micropython-rgbled](https://github.com/Warringer/micropython-rgbled) - This wrapper module aims to reduce the work needed to work with NeoPixel (WS2812) and DotStar (APA102) RGB LED strips and matrixes.
* [micropython_fastled](https://github.com/kdschlosser/micropython_fastled) - Port of FastLED to MicroPython.
* [micropython_quickled](https://github.com/thebaron88/micropython_quickled) - MicroPython module which allows Python to pump data into the WS2811 LEDs at full speed.
* [micropython-rgb-led-driver](https://gitlab.com/Athanaze/micropython-rgb-led-driver) - Tiny driver to control an RGB LED with PWM.
* [micropython-dotstar](https://github.com/mattytrentini/micropython-dotstar) - A MicroPython port of the Adafruit CircuitPython APA102/DotStar library.

#### OLED

* [Grove_OLED](https://github.com/dda/MicroPython/blob/master/Grove_OLED.py) - Driver for SSD1327 used by SeeedStudio's Grove OLED Display 1.12" v1.0.
* [micropython-oled](https://github.com/mcauser/deshipu-micropython-oled) - Collection of drivers for monochrome OLED displays, PCD8544, SH1106, SSD1306, UC1701X.
* [micropython-ssd1327](https://github.com/mcauser/micropython-ssd1327) - Driver for SSD1327 128x128 4-bit greyscale OLED displays.
* [micropython-ssd1351](https://github.com/rdagger/micropython-ssd1351) - Driver for SSD1351 OLED displays.
* [MicroPython_SSD1306](https://github.com/AnthonyKNorman/MicroPython_SSD1306) - ESP8266 driver for SSD1306 OLED 128x64 displays.
* [Official SSD1306](https://github.com/micropython/micropython-lib/tree/master/micropython/drivers/display/ssd1306) - Driver for SSD1306 128x64 OLED displays.
* [SH1106](https://github.com/robert-hh/SH1106) - Driver for the SH1106 OLED display.
* [micropython-ssd1309](https://github.com/rdagger/micropython-ssd1309) - MicroPython SSD1309 Monochrome OLED Display Driver.
* [sh1107-micropython](https://github.com/nemart69/sh1107-micropython) - MicroPython driver for SH1107-based OLED display (64x128).

#### Printer

* [micropython-thermal-printer](https://github.com/ayoy/micropython-thermal-printer) - The MicroPython port of Python Thermal Printer by Adafruit.

### IO

#### ADC

* [ads1x15](https://github.com/robert-hh/ads1x15) - Driver for the ADS1015/ADS1115 ADC, I2C interface.
* [micropython-ads1015](https://github.com/mcauser/deshipu-micropython-ads1015) - ADS1015 12-Bit and ADS1115 16-bit ADC, 4 channels with programmable gain, I2C interface.
* [Micropython_ADS1115](https://github.com/AnthonyKNorman/Micropython_ADS1115) - ADS1115 16-bit ADC, 4 channels with programmable gain, I2C interface.
* [ADS7818](https://github.com/robert-hh/ADS7818) - Python class interfacing the ADS7818 AD-converter.
* [micropython-ads1219](https://github.com/miketeachman/micropython-ads1219) - MicroPython module for the Texas Instruments ADS1219 ADC.
* [micropython-hx711](https://github.com/SergeyPiskunov/micropython-hx711) - MicroPython driver for HX711 24-Bit Analog-to-Digital Converter.
* [MicroPython-ADC_Cal](https://github.com/matthias-bs/MicroPython-ADC_Cal) - ESP32 ADC driver using reference voltage calibration value from efuse.
* [micropython-pcf8591](https://gitlab.com/cediddi/micropython-pcf8591) - MicroPython driver for PCF8591 ADC/DAC, I2C interface.

#### DAC

* [micropython-mcp4725](https://github.com/wayoda/micropython-mcp4725) - Driver for the MCP4725 I2C DAC.
* [mcp4728](https://github.com/openfablab/mcp4728) - Helper library for the Microchip MCP4728 I2C 12-bit Quad DAC.

#### GPIO

* [micropython-inputs](https://github.com/alanmitchell/micropython-inputs) - Classes to count pulses, debounce digital inputs, and calculate moving averages of analog inputs for a MicroPython board.
* [ubutton](https://gitlab.com/WiLED-Project/ubutton) - A MicroPython library for controlling reading and debouncing pushbutton inputs, including "short" and "long" press callbacks.
* [micropython-debounce-switch](https://github.com/selfhostedhome/micropython-debounce-switch) - MicroPython Class for Debouncing Switches.

#### IO-Expander

* [micropython-mcp230xx](https://github.com/ShrimpingIt/micropython-mcp230xx) - Driver for MCP23017 and MCP23008 GPIO expanders.
* [micropython-mcp23017](https://github.com/mcauser/micropython-mcp23017) - MicroPython driver for MCP23017 16-bit I/O Expander.
* [micropython-pcf8574](https://github.com/mcauser/micropython-pcf8574) - MicroPython driver for PCF8574 8-Bit I2C I/O Expander with Interrupt.
* [micropython-pcf8575](https://github.com/mcauser/micropython-pcf8575) - MicroPython driver for PCF8575 16-Bit I2C I/O Expander with Interrupt.

#### Joystick

* [micropython-nunchuck](https://github.com/kfricke/micropython-nunchuck) - Driver for Nunchuk game controller, I2C interface.

#### Keyboard

* [micropython-keyboard](https://github.com/mcameron/micropython-keyboard) - 47 key keyboard running on a MicroPython Pyboard.
* [pico-rgbkeypad](https://github.com/martinohanlon/pico-rgbkeypad) - A Python class for controlling the Pimoroni RGB Keypad for Raspberry Pi Pico.
* [micropython-aiobutton](https://github.com/jacklinquan/micropython-aiobutton) - A MicroPython module for asyncio button.

#### Potentiometers

* [micropython-ad840x](https://github.com/dsiggi/micropython-ad840x) - MicroPython SPI-based manipulation of the AD series digital potentiometers AD8400, AD8402 and AD8403.
* [mcp4131](https://github.com/scruss/mcp4131) - MicroPython module to control MicroChip's MCP4131 SPI digital potentiometer.

#### Power Management

* [AXP202_PythonLibrary](https://github.com/lewisxhe/AXP202_PythonLibrary) - MicroPython AXP202 Library.
* [micropython_hourly_sleeper_library](https://github.com/costastf/micropython_hourly_sleeper_library) - A MicroPython library that enables an ESP8266 to sleep for hourly increments for a setup amount of hours.

#### PWM

* [upwmcontroller](https://gitlab.com/WiLED-Project/upwmcontroller) - A MicroPython library for controlling PWM outputs in an asyncio loop, with features including fading and blinking.

#### Rotary Encoder

* [micropython-rotary](https://github.com/miketeachman/micropython-rotary) - MicroPython module to read a rotary encoder.
* [uencoder](https://gitlab.com/WiLED-Project/uencoder) - A MicroPython library for reading from a rotary encoder.
* [encodermenu](https://github.com/sgall17a/encodermenu) - Simple GUI menu for MicroPython using a rotary encoder and basic display.
* [encoderLib](https://github.com/BramRausch/encoderLib) - MicroPython library to handle a rotary encoder.
* [rotary-encoder](https://github.com/gurgleapps/rotary-encoder) - MicroPython code to drive a KY-040 rotary encoder.
* [micropython-encoder-knob](https://github.com/infinite-tree/micropython-encoder-knob) - A very simple lightweight encoder knob library with button support.
* [encoders](https://github.com/peterhinch/micropython-samples/blob/master/encoders/ENCODERS.md) - Short document explaining issues around encoder technology.
* [asynchronous encoder driver](https://github.com/peterhinch/micropython-async/blob/master/v3/primitives/encoder.py) - Interface an encoder to uasyncio code.

#### Shift Registers

* [micropython-74hc595](https://github.com/mcauser/micropython-74hc595) - MicroPython driver for 74HC595 8-bit shift registers.
* [MicroPython-SN74HCS264](https://gitlab.com/olivierlenoir/MicroPython-SN74HCS264) - MicroPython Driver for SN74HCS264 8-Bit Parallel-Out Serial Shift Registers With Schmitt-Trigger Inputs and Inverted Outputs.

#### Waveform Generator

* [Micropython-AD9833](https://github.com/KipCrossing/Micropython-AD9833) - Pyboard driver for AD9833, SPI interface.
* [Clock_Generators](https://github.com/Wei1234c/Clock_Generators) - Clock generators (Si5351 for now) toolbox.
* [Signal_Generators](https://github.com/Wei1234c/Signal_Generators) - Signal generators (AD9833, AD9834, AD9850, ADF4351) toolbox.
* [ad9850_signalgen](https://github.com/brenn/ad9850_signalgen) - MicroPython library for AD9850 synthesizer.

### Motion

#### DC Motor

* [L298N](https://github.com/GuyCarver/MicroPython/blob/master/lib/L298N.py) - Driver for the L298N dual H-bridge motor controller.
* [MicroPython-L298](https://gitlab.com/olivierlenoir/MicroPython-L298) - Drive L298 dual H-bridge with MicroPython.

#### Servo

* [micropython-pca9685](https://github.com/mcauser/deshipu-micropython-pca9685) - 16-channel 12-bit PWM/servo driver.

#### Stepper

* [micropython-upybbot](https://github.com/jeffmer/micropython-upybbot) - A4988 driver for bipolar stepper motors.
* [uln2003](https://github.com/IDWizard/uln2003) - Driver for 5V 28BYJ-48 stepper motors.
* [micropython-multiaxis](https://gitlab.com/olivierlenoir/micropython-multiaxis) - Multiaxis with MicroPython ESP32 and DRV8825.
* [ticlib](https://github.com/jphalip/ticlib) - Driver for Pololu Tic stepper motor controllers.
* [AccelStepper-MicroPython](https://github.com/pedromneto97/AccelStepper-MicroPython) - AccelStepper Library for MicroPython - ESP32.
* [pystepper](https://github.com/marcio-pessoa/pystepper) - MicroPython Stepper Motor Sequence Control.
* [uPySteppers](https://github.com/lemariva/uPySteppers) - DIY rotating platform using an ESP32 connected to WiFi.
* [microPython_AMIS-30543](https://github.com/capella-ben/microPython_AMIS-30543) - MicroPython library for Stepper Driver control using AMIS-30543 driver.

### Sensors

#### Accelerometer Digital

* [ADXL345-with-Pyboard](https://github.com/AbhinayBandaru/ADXL345-with-Pyboard) - Driver for ADXL345 16g 3-axis accelerometer.
* [adxl345_micropython](https://github.com/fanday/adxl345_micropython) - Driver for ADXL345 16g 3-axis accelerometer.
* [MicroPython-LIS3DH](https://github.com/tinypico/tinypico-micropython/tree/master/lis3dh%20library) - I2C driver for LIS3DH 3-axis accelerometer.
* [micropython-lis2hh12](https://github.com/tuupola/micropython-lis2hh12) - I2C driver for LIS2HH12 3-axis accelerometer.
* [MMA7660](https://github.com/Bucknalla/MicroPython-3-Axis-Accelerometer/blob/master/MMA7660.py) - Driver for MMA7660 1.5g 3-axis accelerometer.
* [ADXL345_spi_micropython](https://github.com/AlekseyFedorovich/ADXL345_spi_micropython) - Library for interacting through the SPI protocol with an 'Analog Devices ADXL345' accelerometer from an MCU flashed with MicroPython.

#### Air Quality

* [CCS811](https://github.com/Ledbelly2142/CCS811) - CCS811 Air Quality Sensor.
* [upython-aq-monitor](https://github.com/ayoy/upython-aq-monitor) - Air Quality monitor using PMS5003 sensor and WiPy.
* [micropython-pms7003](https://github.com/pkucmus/micropython-pms7003) - MicroPython driver for the PMS7003 Air Quality Sensor.
* [pms5003_micropython](https://github.com/kevinkk525/pms5003_micropython) - Driver for PMS5003 air quality sensor for MicroPython.
* [micropython-pms5003-minimal](https://github.com/miketeachman/micropython-pms5003-minimal) - Driver for P air quality sensor for MicroPython.
* [polly](https://github.com/g-sam/polly) - SDS011 pollution sensor + Wemos D1 mini pro + MicroPython.

#### Barometer

* [micropython-bme280](https://github.com/kevbu/micropython-bme280) - Driver for the Bosch BME280 temperature/pressure/humidity sensor.
* [micropython-bmp180](https://github.com/micropython-IMU/micropython-bmp180) - Driver for Bosch BMP180 temperature, pressure and altitude sensor.
* [mpy_bme280_esp8266](https://github.com/catdog2/mpy_bme280_esp8266) - Bosch BME280 temperature/pressure/humidity sensor.
* [BME280](https://github.com/robert-hh/BME280) - MicroPython driver for the BME280 sensor, target platform Pycom devices.
* [micropython-bmp280](https://github.com/dafvid/micropython-bmp280) - Module for the BMP280 sensor.
* [micropython_bme280_i2c](https://github.com/triplepoint/micropython_bme280_i2c) - A MicroPython module for communicating with the Bosch BME280 temperature, humidity, and pressure sensor.
* [MicroPython-BME280](https://github.com/neliogodoi/MicroPython-BME280) - Driver to digital sensor of Temperature, Pressure and Humidity.
* [micropython-bmp180](https://gitlab.com/flowolf/micropython-bmp180) - A module for MicroPython which provides a class for the BMP180 pressure sensor.
* [BMP390](https://github.com/octaprog7/BMP390) - MicroPython module for BMP390 pressure & temperature sensor.
* [BMP180](https://github.com/octaprog7/BMP180) - MicroPython module for BMP180 pressure & temperature sensor.

#### Battery

* [Micropython-LC709203F](https://github.com/scopelemanuele/Micropython-LC709203F) - A simple MicroPython library for LC709293F Fuel Gauge.

#### Biometric

* [micropython-fingerprint](https://github.com/chrisb2/micropython-fingerprint) - MicroPython library for reading Grow and ZhianTec fingerprint sensors.
* [MAX30102-MicroPython-driver](https://github.com/n-elia/MAX30102-MicroPython-driver) - A MAX30102 driver ported to MicroPython. It should also work for MAX30105.

#### Camera

* [micropython-ov2640](https://github.com/namato/micropython-ov2640) - MicroPython class for OV2640 camera.
* [Nikon-Trigger-for-MicroPython](https://github.com/Thekegman/Nikon-Trigger-for-MicroPython) - Remote trigger for a Nikon camera using an IR LED. For Pyboard v1.1.
* [micropython-camera-driver](https://github.com/lemariva/micropython-camera-driver) - OV2640 camera driver for MicroPython on ESP32.
* [esp32-cam-micropython](https://github.com/shariltumin/esp32-cam-micropython) - MicroPython ESP32-CAM.
* [uPyCam](https://github.com/lemariva/uPyCam) - Take a photo with an ESP32-CAM running MicroPython.
* [OV2640_uPy](https://github.com/FunPythonEC/OV2640_uPy) - OV2640 camera library for MicroPython.
* [MQTT-Cam](https://github.com/jono-allen/MQTT-Cam) - ESP32-CAM MicroPython MQTT AWS S3 Uploader.

#### Colour

* [micropython-tcs34725](https://gitlab.com/robhamerling/micropython-tcs34725) - Driver class for TCS34725 and TCS34727 color sensors.

#### Compass

* [micropython-esp8266-hmc5883l](https://github.com/gvalkov/micropython-esp8266-hmc5883l) - 3-axis digital compass on the ESP8266.
* [QMC5883](https://github.com/robert-hh/QMC5883) - Python class for the QMC5883 Three-Axis Digital Compass IC.
* [microPython_AS5600L](https://github.com/capella-ben/microPython_AS5600L) - MicroPython driver for AS5600L magnet rotary position sensor.

#### Current

* [micropythonINA219](https://github.com/kabel42/micropythonINA219) - Driver for INA219 current sensor.
* [pyb_ina219](https://github.com/chrisb2/pyb_ina219) - Driver for INA219 current sensor.
* [INA219](https://github.com/robert-hh/INA219) - INA219 MicroPython driver.
* [TI_INA226_micropython](https://github.com/elschopi/TI_INA226_micropython) - MicroPython driver for Texas Instruments INA226 power measuring IC.
* [micropython-current-monitor](https://gitlab.com/n.rj.powers/micropython-current-monitor) - Current monitor using the INA219 and an SSD1306 OLED.

#### Distance IR

* [micropython-gp2y0e03](https://github.com/mcauser/deshipu-micropython-gp2y0e03) - IR-LED distance measuring sensor using Sharp GP2Y0E03.
* [micropython-vl6180](https://github.com/mcauser/deshipu-micropython-vl6180) - Time-of-Flight sensor, ambient light sensor & IR emitter.

#### Distance Laser

* [micropython-vl53l0x](https://github.com/mcauser/deshipu-micropython-vl53l0x) - Time-of-Flight laser-ranging sensor.
* [Qwiic_TOF_Module_RFD77402](https://github.com/ZIOCC/Qwiic_TOF_Module_RFD77402) - Qwiic TOF Module (RFD77402) time-of-flight rangefinding module.
* [VL53L0X](https://github.com/uceeatz/VL53L0X) - MicroPython Library for LiDAR Sensor VL53L0X.
* [vl53l1x_pico](https://github.com/drakxtwo/vl53l1x_pico) - MicroPython driver for the VL53L1X ToF sensor.
* [tf-luna-micropython](https://github.com/davmoz/tf-luna-micropython) - A simple MicroPython I2C library for TF-Luna LiDAR Module.
* [vl53l5cx](https://github.com/mp-extras/vl53l5cx) - MicroPython and CircuitPython Package for the [VL53L5CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html) (4x4/8x8 ToF sensor array).

#### Distance Ultrasonic

* [micropython-hcsr04](https://github.com/rsc1975/micropython-hcsr04) - Driver for HC-SR04 ultrasonic distance sensors.
* [micropython-us100](https://github.com/kfricke/micropython-us100) - MicroPython driver for the US-100 sonar distance sensor.

#### Dust

* [pyGP2Y](https://github.com/amigcamel/pyGP2Y) - MicroPython library for the Sharp GP2Y1014AU0F Dust Sensor.

#### Energy

* [ATM90E26_Micropython](https://github.com/whatnick/ATM90E26_Micropython) - Driver for ATM90E26 energy metering device.
* [MCP39F521](https://github.com/warpme/MCP39F521) - ESP8266 scripts for reading MCP39F521 power monitors.
* [micropython-p1meter](https://github.com/Josverl/micropython-p1meter) - A ESP32 sensor to read a p1 electricity meter and publish this to MQTT and Home Assistant, written in MicroPython.
* [esp32-solar2](https://github.com/octopusengine/esp32-solar2) - Simple solar regulator - MicroPython project.

#### Gaseous

* [micropython-MQ](https://github.com/kartun83/micropython-MQ) - Drivers for MQ series gas sensors.
* [MQ135](https://github.com/rubfi/MQ135) - Driver for MQ135 gas sensor.
* [CCS811](https://github.com/Notthemarsian/CCS811) - Basic MicroPython driver for CCS811 on ESP8266 boards.
* [micropython-scd30](https://github.com/agners/micropython-scd30) - MicroPython I2C driver for Sensirion SCD30 CO2 sensor module.
* [micropython-sgp40](https://github.com/agners/micropython-sgp40) - MicroPython I2C driver for SGP40 VOC sensor module.
* [MICS6814-Micropython-driver](https://gitlab.com/DanNduati/MICS6814-Micropython-driver) - ESP32 MicroPython driver for the Pimoroni MICS6814 breakout board.

#### Light

* [MicroPython-SI1145](https://github.com/neliogodoi/MicroPython-SI1145) - SI1145 UV index, IR, visible light and proximity sensor.
* [micropython-tsl2561](https://github.com/kfricke/micropython-tsl2561) - Driver for the TSL2561 illumination sensor from TAOS / ams.
* [mpy_bh1750fvi_esp8266](https://github.com/catdog2/mpy_bh1750fvi_esp8266) - ESP8266 driver for BH1750FVI sensor.
* [bh1750](https://github.com/PinkInk/upylib/tree/master/bh1750) - BH1750 I2C digital light sensor driver.
* [micropython-max44009](https://github.com/mcauser/micropython-max44009) - MicroPython driver for the MAX44009 ambient light sensor.
* [veml7700](https://github.com/palouf34/veml7700) - Library for MicroPython for VEML7700 light sensor.
* [MicroPython_MAX44009_driver](https://github.com/rcolistete/MicroPython_MAX44009_driver) - MicroPython driver for MAX44009 light sensor.

#### Motion Inertial

* [micropython-bmx055](https://github.com/micropython-IMU/micropython-bmx055) - Driver for Bosch BMX055 IMU sensor.
* [micropython-bno055](https://github.com/deshipu/micropython-bno055) - Bosch Sensortec BNO055 9DOF IMU sensor, I2C interface.
* [micropython-lsm9ds0](https://github.com/micropython-IMU/micropython-lsm9ds0) - LSM9DS0 g-force linear acceleration, Gauss magnetic and DPS angular rate sensors.
* [micropython-mpu9250](https://github.com/tuupola/micropython-mpu9250) - I2C driver for MPU9250 9-axis motion tracking device.
* [micropython-mpu9x50](https://github.com/micropython-IMU/micropython-mpu9x50) - Driver for the InvenSense MPU9250 inertial measurement unit.
* [MPU6050-ESP8266-MicroPython](https://github.com/adamjezek98/MPU6050-ESP8266-MicroPython) - ESP8266 driver for MPU6050 accelerometer/gyroscope.
* [py-mpu6050](https://github.com/larsks/py-mpu6050) - ESP8266 driver for MPU6050 accelerometer/gyroscope.
* [micropython-mpu6886](https://github.com/tuupola/micropython-mpu6886) - MicroPython I2C driver for MPU6886 6-axis motion tracking device.
* [micropython-fusion](https://github.com/micropython-IMU/micropython-fusion) - Sensor fusion calculates heading, pitch and roll from the outputs of motion tracking devices.
* [flight_controller](https://github.com/wagnerc4/flight_controller) - MicroPython flight controller.
* [micropython-bno055](https://github.com/micropython-IMU/micropython-bno055) - Bosch BNO055 driver for MicroPython. IMU with hardware sensor fusion.
* [micropython-mpu6050-mqtt-streamer](https://github.com/mozanunal/micropython-mpu6050-mqtt-streamer) - Stream data from MPU6050 to MQTT server using MicroPython on ESP8266.
* [upy-motion](https://github.com/OneMadGypsy/upy-motion) - A simple MPU6050 driver written in MicroPython.
* [micropython-bno08x-rvc](https://github.com/rdagger/micropython-bno08x-rvc) - MicroPython library for BNO08x.
* [micropython-mpu9250](https://gitlab.com/nnayo/micropython-mpu9250) - MicroPython MPU-9250 (MPU-6500 + AK8963) I2C driver.

#### Pressure

* [ms5803-micropython](https://github.com/minyiky/ms5803-micropython) - A MicroPython implementation of the driver for an MS5803 pressure & temperature sensor.
* [MPL3115A2_MicroPython](https://github.com/PinsonJonas/MPL3115A2_MicroPython) - MicroPython library for the MPL3115A2 altimeter.

#### Proximity

* [uPy_APDS9960](https://github.com/rlangoy/uPy_APDS9960) - MicroPython proximity library for ESP8266 using APDS9960.

#### Radiation

* [micropython-geiger](https://github.com/Josep/micropython-geiger) - Geiger counter with MicroPython card.
* [ESPGeiger](https://github.com/biemster/ESPGeiger) - MicroPython library for the ESP8266 Geiger counter.

#### Soil Moisture

* [micropython-chirp](https://github.com/robberwick/micropython-chirp) - Driver for the Chirp Soil Moisture Sensor.
* [MicroPython-MiFlora](https://github.com/matthias-bs/MicroPython-MiFlora) - Xiaomi Mi Flora (aka flower care) BLE plant sensors (soil moisture/conductivity/light intensity/temperature).

#### Spectral

* [AS726X_LoPy](https://github.com/jajberni/AS726X_LoPy) - MicroPython driver for the AS726X spectral sensor.

#### Temperature Analog

* [micropython-max31855](https://github.com/mcauser/deshipu-micropython-max31855) - Thermocouple amplifier, SPI interface.
* [max31856](https://github.com/alinbaltaru/max31856) - Precision thermocouple to digital converter with linearization, SPI interface.
* [mcp9700](https://gitlab.com/CrispyCrafter/mcp9700) - Generic MicroPython driver for MCP9700.

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
* [micropython-si7021](https://github.com/mcauser/deshipu-micropython-si7021) - SI7021 Temperature and humidity sensor, I2C interface.
* [micropython-si7021](https://github.com/chrisbalmer/micropython-si7021) - SI7021 Temperature and humidity sensor, I2C interface.
* [micropython-Si705x](https://github.com/billyrayvalentine/micropython-Si705x) - Silicon Labs Si705x series of temperature sensors, I2C interface.
* [micropython-Si70xx](https://github.com/billyrayvalentine/micropython-Si70xx) - Silicon Labs Si70xx series of relative humidity and temperature sensors, I2C interface.
* [micropython-tmp102](https://github.com/khoulihan/micropython-tmp102) - Driver for TMP102 digital temperature sensor.
* [Official DHT11+DHT12](https://github.com/micropython/micropython-lib/tree/master/micropython/drivers/sensor/dht) - ESP8266 driver for DHT11 and DHT12 temperature and humidity sensor.
* [sht25-micropython](https://github.com/Miceuz/sht25-micropython) - Driver for SHT25 temperature and humidity sensor.
* [micropython-tmp1075](https://github.com/mattytrentini/micropython-tmp1075) - Driver for the TI TMP1075 temperature sensor.
* [micropython-sht11](https://github.com/2black0/micropython-sht11) - Driver for Sensirion SHT11 temperature and humidity sensor.
* [micropython-lm75a](https://github.com/mcauser/micropython-lm75a) - Driver for the NXP LM75A digital temperature sensor.
* [BME680-Micropython](https://github.com/robert-hh/BME680-Micropython) - MicroPython driver for the BME680 sensor.
* [htu21d-esp8266](https://github.com/julianhille/htu21d-esp8266) - This is a MicroPython module / class to measure data from the HTU21D.
* [HTU21D](https://github.com/peterhinch/micropython-async/blob/master/v3/docs/HTU21D.md) - Asynchronous driver for HTU21D temperature and humidity sensor.
* [esp-sht3x-micropython](https://github.com/HAIZAKURA/esp-sht3x-micropython) - A SHT3x (SHT30/31/35) library for ESP8266/ESP32 with MicroPython.
* [sht25-micropython](https://gitlab.com/miceuz/sht25-micropython) - MicroPython implementation of API of SHT25 humidity and temperature sensor.
* [micropython-sht30](https://github.com/schinckel/micropython-sht30) - SHT30 sensor driver in pure Python based on I2C bus.
* [micropython_ahtx0](https://github.com/targetblank/micropython_ahtx0) - MicroPython driver for the AHT10 and AHT20 temperature and humidity sensors.
* [sht85](https://github.com/octaprog7/sht85) - MicroPython driver for the [Sensiron SHT85](https://sensirion.com/products/catalog/SHT85/) humidity and temperature sensor.
* [micropython-zacwire]([https://github.com/targetblank/micropython_ahtx0](https://github.com/mdaeron/micropython-zacwire)) - MicroPython driver for the ZACwire protocol used in TSic temperature sensors (506F).

#### Temperature IR

* [micropython-mlx90614](https://github.com/mcauser/micropython-mlx90614) - Driver for Melexis MLX90614 IR temperature sensor.

#### Touch Capacitive

* [micropython-mpr121](https://github.com/mcauser/micropython-mpr121) - Driver for MPR121 capacitive touch keypads and breakout boards.
* [micropython-ttp223](https://github.com/mcauser/micropython-ttp223) - Examples using TTP223 capacitive touch module.
* [micropython-TTP229-BSF](https://github.com/alankrantas/micropython-TTP229-BSF) - MicroPython ESP8266/ESP32 driver for TTP229-BSF 16-key capacitive keypad in serial interface mode.
* [uFT6336U](https://github.com/fantasticdonkey/uFT6336U) - MicroPython I2C driver for the Focus LCDs FT6336U capacitive touch panel controller IC.

#### Touch Resistive

* [XPT2046-touch-pad-driver](https://github.com/robert-hh/XPT2046-touch-pad-driver) - Driver for XPT2046 touch pad controller used in many TFT modules.

### Scheduling

* [micropython-mcron](https://github.com/fizista/micropython-mcron) - MicroCRON is a time-based task scheduling program for MicroPython.
* [micropython-scron](https://github.com/fizista/micropython-scron) - SimpleCRON is a time-based task scheduling program inspired by the well-known cron program for Unix systems.
* [Schedule](https://github.com/peterhinch/micropython-async/blob/master/v3/docs/SCHEDULE.md) - A scheduler for uasyncio based applications. Schedule events at specified times and dates.
* [micropython-aioschedule](https://github.com/ThinkTransit/micropython-aioschedule) - A persistent uasyncio scheduler that supports deepsleep between task runs.

### Storage

#### Database

* [uPyMySQL](https://github.com/dvrhax/uPyMySQL) - Pure MicroPython MySQL Client.
* [micropython-redis](https://github.com/dwighthubbard/micropython-redis) - A Redis client implementation designed for use with MicroPython.
* [picoredis](https://github.com/SpotlightKid/picoredis) - A very minimal Redis client (not only) for MicroPython.
* [micropg](https://github.com/nakagami/micropg) - PostgreSQL database driver for MicroPython.
* [nmongo](https://github.com/nakagami/nmongo) - MongoDB client for CPython and MicroPython, with MongoDB shell-like APIs.
* [MicroPyDatabase](https://github.com/sungkhum/MicroPyDatabase) - A low-memory JSON-based database for MicroPython.
* [micropython-firebase-realtime-database](https://github.com/ckoever/micropython-firebase-realtime-database) - Firebase implementation for MicroPython optimized for ESP32.
* [micropython-firebase-firestore](https://github.com/WoolDoughnut310/micropython-firebase-firestore) - Firebase Firestore implementation for MicroPython.
* [uSQLite](https://github.com/spatialdude/usqlite) - SQLite library module for MicroPython.

#### EEPROM

* [micropython_eeprom](https://github.com/peterhinch/micropython_eeprom) - Cross-platform MicroPython device drivers for memory chips (EEPROM, FRAM, Flash, PSRAM).
* [mb_24x256_512](https://github.com/MarksBench/mb_24x256_512) - Very simple MicroPython module/driver for Microchip 24x256 and 24x512 I2C EEPROM devices.

#### Flash

* [micropython_data_to_py](https://github.com/peterhinch/micropython_data_to_py) - A Python 3 utility to convert an arbitrary binary file to Python source for freezing as bytecode in Flash.
* [micropython-winbond](https://github.com/brainelectronics/micropython-winbond) - Interact with Winbond W25Q Flash chips via SPI.

#### FRAM

* [micropython-fram](https://github.com/rolandvs/micropython-fram) - Pyboard driver for Ferroelectric RAM module.

#### PSRAM

* [mb_PSRAM_64Mb_SPI](https://github.com/MarksBench/mb_PSRAM_64Mb_SPI) - Very simple MicroPython module to use a generic 64Mbit PSRAM (ie Adafruit 4677) with a Raspberry Pi Pico (RP2040).

#### SRAM

* [mb_23LC1024](https://github.com/MarksBench/mb_23LC1024) - Very simple MicroPython module to use a Microchip 23LC1024 SPI SRAM with a Raspberry Pi Pico (RP2040).
* [mb_47x16](https://github.com/MarksBench/mb_47x16) - Very simple MicroPython module/driver for Microchip 47x16 EERAM devices (47L/47C).

### Threading

* [MicroWorkers](https://github.com/jczic/MicroWorkers) - A micro workers class that easily manages a pool of threads to optimise simultaneous jobs and jobs endings, for MicroPython (used on Pycom modules & ESP32).

### User Interface

* [upymenu](https://github.com/jplattel/upymenu) - MicroPython Menu for LCD Displays.

## Community

* [MicroPython Discussions on GitHub](https://github.com/orgs/micropython/discussions) - GitHub discussions for all things related to MicroPython.
* [MicroPython Forum (archive)](https://forum.micropython.org/) - Archived community conversations on all things related to MicroPython.
* [Discord](https://discord.com/invite/qw7d8bv) - Get an invite to the MicroPython Discord server.
* [MicroPython on Mastodon / Fediverse](https://fosstodon.org/@MicroPython) - Follow MicroPython in the Fediverse.
* [MicroPython on Twitter](https://twitter.com/micropython) - Follow MicroPython on Twitter for latest news and updates.
* [MicroPython on Facebook](https://www.facebook.com/micropython) - Like MicroPython on Facebook for competitions, news and updates.
* [Melbourne MicroPython Meetup](https://www.meetup.com/en-AU/MicroPython-Meetup) - Regular meetup at CCHS in Melbourne, Australia.

## Tutorials

* [uasyncio](https://github.com/peterhinch/micropython-async/blob/master/v3/docs/TUTORIAL.md) - Write asynchronous code which interfaces to hardware devices.
* [Asynchronous drivers](https://github.com/peterhinch/micropython-async/blob/master/v3/docs/DRIVERS.md) - Tutorial and code for asynchronous interfaces to switches, pushbuttons, encoders and ADCs.
* [Pyboard micropower](https://github.com/peterhinch/micropython-micropower) - Tutorial and code for low power applications on Pyboard 1.x and Pyboard D.
* [3D rotation with quaternions](https://github.com/peterhinch/micropython-samples/blob/master/QUATERNIONS.md) - Tutorial and code for the easy way to do 3D rotation.
* [Miguel Grinberg](https://blog.miguelgrinberg.com/category/MicroPython) - MicroPython and the Internet of Things.
* [Bhavesh Kakwani](https://bhave.sh/) - MicroPython videos + written tutorials.
* [CoderDojo Twin Cities MicroPython](https://www.coderdojotc.org/micropython/) - Full coding curriculum for teaching MicroPython to children.
* [MicroPython Tutorials for ESP32 boards](https://www.upesy.com/blogs/tutorials/tutorials-for-esp32-with-micropython-code) - Tutorials with code examples to learn the basic of MicroPython with ESP32 boards.
* [Learn MicroPython with a Pi Pico board](https://www.upesy.com/blogs/tutorials/tutorials-for-rpi-pi-pico-with-micropython-code) - Tutorials on MicroPython with the Raspberry Pi Pico / RP240 boards.

## Books

* [Programming with MicroPython: Embedded Programming with Microcontrollers and Python](https://www.oreilly.com/library/view/programming-with-micropython/9781491972724/) - By Nicholas H. Tollervey. ISBN 9781491972731.
* [MicroPython for the Internet of Things: A Beginner's Guide to Programming with Python on Microcontrollers](https://link.springer.com/book/10.1007/978-1-4842-3123-4) - By Charles Bell. ISBN 9781484231227.
* [Beginning MicroPython with the Raspberry Pi Pico: Build Electronics and IoT Projects](https://link.springer.com/book/10.1007/978-1-4842-8135-2) - By Charles Bell. ISBN 9781484281345.
* [MicroPython Cookbook](https://www.packtpub.com/au/application-development/micropython-cookbook) - By Marwan Alsabbagh. ISBN 9781838649951.
* [Python for Microcontrollers: Getting Started with MicroPython](https://www.mheducation.com.au/python-for-microcontrollers-getting-started-with-micropython-9781259644535-aus) - By Donald Norris. ISBN 9781259644535.
* [Advanced Programming in MicroPython By Example](https://www.amazon.com/Advanced-Programming-MicroPython-Example-Magda/dp/1090900937) - By Yury Magda. ISBN 9781090900937.
* [MicroPython Projects](https://www.packtpub.com/au/iot-hardware/micropython-projects) - By Jacob Beningo. ISBN 9781789958034.
* [Get Started with MicroPython on Raspberry Pi Pico](https://store.rpipress.cc/products/get-started-with-micropython-on-raspberry-pi-pico) - By Gareth Halfacree and Ben Everard. ISBN 9781912047864.
* [MicroPython for Microcontrollers](https://www.elektor.com/micropython-for-microcontrollers-e-book) - By Günter Spanner. ISBN 9783895764370.
* [MicroPython for the Raspberry Pi Pico W: A gentle introduction to programming digital circuits with Python](https://www.amazon.com/MicroPython-Raspberry-Pico-introduction-programming/dp/B0BKSCV18D) - By Miguel Grinberg. ISBN 9798361302710.

## Frameworks

* [micrOS](https://github.com/BxNxM/micrOS) - MicroPython-based IoT Framework.
* [terkin-datalogger](https://github.com/hiveeyes/terkin-datalogger) - Flexible data logger application for MicroPython and CPython.
* [perthensis](https://codeberg.org/scy/perthensis) - Perthensis: an asynchronous framework for MicroPython.
* [meerkat](https://github.com/crdietrich/meerkat) - I2C Data Acquisition for MicroPython and Raspberry Pi.

## Resources

* [MicroPython](https://micropython.org) - Project website. Test drive the Pyboard. Try MicroPython online with Unicorn.
* [MicroPython on GitHub](https://github.com/micropython/micropython) - Submit bug reports, follow and join in development on GitHub.
* [MicroPython Official Documentation](https://docs.micropython.org/) - For various ports, including quick reference, general information, examples and tutorials.
* [MicroPython Wiki](https://github.com/micropython/micropython/wiki) - Community generated documentation and examples of the features of MicroPython and the Pyboard.
* [MicroPython Newsletter](https://micropython.org/newsletter) - Subscribe to the MicroPython newsletter for news and announcements including new features and new products.
* [MicroPython Store](https://store.micropython.org/) - Where you can buy the Pyboard, housings, skins, books, connectors and peripherals.
* [MicroPython on Wikipedia](https://en.wikipedia.org/wiki/MicroPython) - MicroPython on Wikipedia.
* [awesome-micropythons](https://github.com/adafruit/awesome-micropythons) - The many forks & ports of MicroPython.

## Development

### Code Generation

* [micropy-cli](https://github.com/BradenM/micropy-cli) - Micropy CLI is a project management/generation tool for writing MicroPython code in modern IDEs such as Visual Studio Code.
* [micropython-stubber](https://github.com/Josverl/micropython-stubber) - Generate and use stubs for different MicroPython firmwares to use with Visual Studio Code and/or Pylint.
* [micropy-stubs](https://github.com/BradenM/micropy-stubs) - Automatically Generated Stub Packages for Micropy-Cli and whomever else.
* [micropython-extmod-generator](https://github.com/prusnak/micropython-extmod-generator) - Generator for MicroPython external modules written in C.
* [micropython-package-template](https://github.com/brainelectronics/micropython-package-template) - GitHub workflow supported MicroPython package template with deploys to the [Python Package Index](https://pypi.org/) on a push to the main branch and test deploys to the [Test Python Package Index](https://test.pypi.org/) on PRs.

### Debugging

* [esp32-backtrace](https://github.com/tve/esp32-backtrace) - ESP32 Exception Stack Backtrace Analyzer.
* [micropython-aiosentry](https://github.com/devbis/micropython-aiosentry) - Asynchronous Sentry.io micro client for MicroPython.
* [micropython-usyslog](https://github.com/kfricke/micropython-usyslog) - Simple remote syslog client for MicroPython.
* [Asynchronous monitor](https://github.com/peterhinch/micropython-monitor) - Use a Raspberry Pico and a logic analyser or scope to monitor asynchronous code.

### IDEs

* [BIPES](https://bipes.net.br/ide/) - Web-based IDE for MicroPython with file manager, editor, code generation from blocks, IoT dashboard and Serial/USB/Bluetooth/WebREPL console on the web browser. Source: [https://github.com/BIPES](https://github.com/BIPES).
* [ESP32-MPY-Jama](https://github.com/jczic/ESP32-MPY-Jama) - Tool for managing Espressif ESP32 microcontrollers with MicroPython.
* [JetBrains IntelliJ/PyCharm MicroPython Plugin](https://plugins.jetbrains.com/plugin/9777-micropython) - Plugin for MicroPython devices in IntelliJ and PyCharm.
* [MicroPython IDE for VSCode](https://marketplace.visualstudio.com/items?itemName=dphans.micropython-ide-vscode) - MicroPython IDE for Visual Studio Code.
* [MicroPython-REPLink for VSCode](https://marketplace.visualstudio.com/items?itemName=SWC-Fablab.micropython-replink) - Handy shortcuts for interacting with a MicroPython REPL terminal.
* [Mu Editor](https://codewith.mu/) -  Code with Mu: a simple Python/MicroPython/CircuitPython editor for beginner programmers.
* [Thonny IDE](https://thonny.org/) - Thonny: Python IDE for beginners.
* [Pyboard File Manager](https://github.com/joewez/PyboardFileManager) - Pyboard File Manager: Windows GUI for Pyboard.py compatible devices.

### Logging

* [micropython-ulogger](https://github.com/whales-chen/micropython-ulogger) - Lightweight log module customized for MicroPython.

### Shells

#### On Device

* [upy-shell](https://github.com/dhylands/upy-shell) - A simple command line-based shell for MicroPython.
* [Micropython-Editor](https://github.com/robert-hh/Micropython-Editor) - Small on-board editor for Pyboard, WiPy, ESP8266, ESP32, PyCom and Adafruit devices written in Python.

#### On Host

* [rshell](https://github.com/dhylands/rshell) - Copy or sync files to boards, enter REPL from your terminal.
* [ampy](https://github.com/scientifichackers/ampy) - Utility to interact with a MicroPython board over a serial connection.
* [mpbridge](https://github.com/AmirHmZz/mpbridge) - A file system bridge to synchronize and manage files on a device running MicroPython.
* [mpfshell](https://github.com/wendlers/mpfshell) - A simple shell-based file explorer for ESP8266 and WiPy.
* [mpsync](https://github.com/thilomichael/mpsync) - A tool that automatically synchronizes code to a MicroPython board.
* [mpremote](https://github.com/micropython/micropython/blob/master/tools/mpremote/README.md) - Powerful official shell that supports mounting the host's current directory on the target. Run code without changing the target's filesystem.

## Miscellaneous

* [MicroPython Kickstarter](https://www.kickstarter.com/projects/214379695/micro-python-python-for-microcontrollers) - 1,931 backers pledged £97,803 to help bring this project to life.
* [MicroPython on the ESP8266 Kickstarter](https://www.kickstarter.com/projects/214379695/micropython-on-the-esp8266-beautifully-easy-iot) - 1,399 backers pledged £28,534 to help bring this project to life.

## Contributing

Contributions and suggestions are always welcome! Please take a look at the [contribution guidelines](https://github.com/mcauser/awesome-micropython/blob/master/contributing.md) first.

I will keep some pull requests open if I'm not sure whether those libraries are awesome, you could vote for them by adding 👍 to them.
