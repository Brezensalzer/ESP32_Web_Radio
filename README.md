This code is based on the great project from Nick Koumaris here: https://educ8s.tv/esp32-internet-radio/ .
There is also an instructable page: https://www.instructables.com/id/Internet-Radio-Using-an-ESP32/

Code changes:
* the station list is fetched on startup from a remote web server
* multitasking and buffering - the mp3 player code is running in a separate task (freeRTOS), the mp3 chunks are buffered in a queue
* Nextion HMI adapted to my 2.8" version of the display. Text is displayed dynamically with an art deco font. Buttons and a slider are used as controls (vs. physical buttons in the original project)
* Nextion display is connected to "Serial2" (ESP32 pins 16 RXD & 17 TXD) - "Serial" is connected to the USB converter and gives trouble when uploading a sketch.

Used libraries:
* https://github.com/baldram/ESP_VS1053_Library
* https://github.com/itead/ITEADLIB_Arduino_Nextion
* https://github.com/igorantolic/ai-esp32-rotary-encoder

Hardware:
* ESP32 board (Node32s)
* Nextion Display NX3224T028_011
* Level shifter 5V - 3.3V, 2 channels: Nextion runs on 5V, ESP32 (RX pin!) on 3.3V 
* MP3 module "LC Technology VS1003/1053 MP3 CODEC"
* Adafruit Stereo 2.1W Class D Audio Amplifier - TPA2012 (no noise!)
* Adafruit Speaker - 3" Diameter - 4 Ohm 3 Watt, 2x

Problems:
* I'd like to use rotary encoders for station and volume control. While the example sketch from the library just works on a "blank" ESP32, it does not work in my more complex code. I have no idea why.

Plan:
* building a wooden artdeco enclosure. Search for "Rubis 70 Belgium 1933 artdeco".
