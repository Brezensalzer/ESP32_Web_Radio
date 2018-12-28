This code is based on the great project from Nick Koumaris here: https://educ8s.tv/esp32-internet-radio/ .
There is also an instructable page: https://www.instructables.com/id/Internet-Radio-Using-an-ESP32/

Code changes:
* the station list is fetched on startup from a remote web server
* multitasking and buffering - the mp3 player code is running in a separate task (freeRTOS), the mp3 chunks are buffered in a queue
* Nextion HMI adapted to my 2.8" version of the display. Text is displayed dynamically with an art deco font. Buttons and a slider are used as controls (vs. physical buttons in the original project)
* Nextion display is connected to "Serial2" (ESP32 pins 16 RXD & 17 TXD) - "Serial" is connected to the USB converter and gives trouble when uploading a sketch.
* Station and volume control with rotary encoders

Used libraries:
* https://github.com/baldram/ESP_VS1053_Library
* https://github.com/itead/ITEADLIB_Arduino_Nextion
* Rotary encoder code from https://github.com/Edzelf/ESP32-Radio

Hardware:
* ESP32 board (Node32s)
* Nextion Display NX3224T028_011
* Level shifter 5V - 3.3V, 2 channels: Nextion runs on 5V, ESP32 (RX pin!) on 3.3V 
* MP3 module "LC Technology VS1003/1053 MP3 CODEC"
* Adafruit Stereo 2.1W Class D Audio Amplifier - TPA2012 (no noise!)
* Adafruit Speaker - 3" Diameter - 4 Ohm 3 Watt, 2x
* KY-040 Rotary Encoder, 2x

Plan:
* building a wooden artdeco enclosure: http://www.vintageradio.nl/radio%27s/rubis_type_72_engels.htm

3D Drawings:
* FreeCAD, step and stl files are now available.
* My first version will be 3D printed

<p align="center">
  <img src="./Rubis/Rubis_printed.png" width="800"/>
</p>
