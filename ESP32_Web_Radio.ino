// ESP32_Web_Radio.ino
//
// Project based on
// ESP32 Internet Radio Project     v1.00 
// http://educ8s.tv/esp32-internet-radio

#include <VS1053.h>  //https://github.com/baldram/ESP_VS1053_Library
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <Nextion.h>
#include <Arduino.h>
#include "RadioStation.h"
#include "AiEsp32RotaryEncoder.h"
#include "config.h"

//------------------------------------------------------------------------------
// mp3 module
//------------------------------------------------------------------------------
  #define VS1053_CS    32 
  #define VS1053_DCS   33  
  #define VS1053_DREQ  35 
  uint8_t volume = 90; // volume level 0-100
  uint8_t mp3buff[32];   // vs1053 likes 32 bytes at a time
  
  VS1053 player(VS1053_CS, VS1053_DCS, VS1053_DREQ);
  
  //------------------------------------------------------------------------------
  void initMP3Decoder()
  //------------------------------------------------------------------------------
    {
      player.begin();
      player.switchToMp3Mode(); // optional, some boards require this
      player.setVolume(volume);
    }

//------------------------------------------------------------------------------
// freertos multitasking and queueing
//------------------------------------------------------------------------------
  #define QUEUELENGTH 256
  QueueHandle_t xQueueMp3;
  
  // Dimensions the buffer that the task being created will use as its stack.
  // NOTE:  This is the number of bytes the stack will hold, not the number of
  // words as found in vanilla FreeRTOS.
  #define STACK_SIZE 32768
  TaskHandle_t xHandle = NULL;
  
  //------------------------------------------------------------------------------
  void task_mp3play(void * pvParameters)
  //------------------------------------------------------------------------------
  {
    byte        mp3chunk[32];
    uint16_t    queue_depth;

    // loop forever
    while(true)
    {
      // check queue depth. Audio buffer has to be at least 25% full
      queue_depth = uxQueueMessagesWaiting(xQueueMp3);
      if(queue_depth > (QUEUELENGTH / 4))
      {
        // Receive a message on the created queue.  Block for 10 ticks if a
        // message is not immediately available.
        if( xQueueReceive( xQueueMp3, &( mp3chunk ), ( TickType_t ) 10 ) )
        {
             // we now can consume mp3chunk
             player.playChunk(mp3chunk, 32);
        }
      }
    }
  }
  
  //------------------------------------------------------------------------------
  void mp3Flush()   // clear queue from previous station
  //------------------------------------------------------------------------------
  {
      char rc;
      while(rc != pdPASS)
          rc = xQueueReset(xQueueMp3);
  }

//------------------------------------------------------------------------------
// radio stations
//------------------------------------------------------------------------------
  StationList stations;

//------------------------------------------------------------------------------
// Nextion
//------------------------------------------------------------------------------
  // don't use UART0, conflict with sketch upload
  // use UART2 on Pins 16 RXD & 17 TXD
  // has to be changed in NexConfig.h (!)
  // #define nexSerial Serial2
  
  NexText t0 = NexText(0, 2, "t0");
  NexButton b0 = NexButton(0, 3, "b0"); // next station
  NexButton b1 = NexButton(0, 4, "b1"); // previous station
  NexSlider h0 = NexSlider(0, 5, "h0"); // loudness
  
  // Register objects to the touch event list.
  NexTouch *nex_listen_list[] = 
  {
      &b0, &b1, &h0,
      NULL
  };

  //------------------------------------------------------------------------------
  void b0PopCallback(void *ptr) // next station
  //------------------------------------------------------------------------------
  {
    if(stations.radioStation < (stations.numStations - 1))
      stations.radioStation++;
    else
      stations.radioStation = 0;
    t0.setText(stations.station[stations.radioStation].label);      
  }
  
  //------------------------------------------------------------------------------
  void b1PopCallback(void *ptr) // prev station
  //------------------------------------------------------------------------------
  {
    if(stations.radioStation > 0)
      stations.radioStation--;
    else
      stations.radioStation = stations.numStations - 1;
    t0.setText(stations.station[stations.radioStation].label);      
  }
  
  //------------------------------------------------------------------------------
  void h0PopCallback(void *ptr)
  //------------------------------------------------------------------------------
  {
    uint32_t number = 0;
    
    h0.getValue(&number);
    volume = (int) number;
    player.setVolume(volume);
  }

//------------------------------------------------------------------------------
// Rotary Encoders
// not working yet, I have no idea why. Interrupts not working?
//------------------------------------------------------------------------------
  #define ROTARY_ENCODER1_CLK_PIN 21
  #define ROTARY_ENCODER1_DT_PIN 22
  AiEsp32RotaryEncoder encoderStation = AiEsp32RotaryEncoder(ROTARY_ENCODER1_CLK_PIN, ROTARY_ENCODER1_DT_PIN, -1, -1);
/*  
  #define ROTARY_ENCODER2_CLK_PIN 16
  #define ROTARY_ENCODER2_DT_PIN 4
  AiEsp32RotaryEncoder encoderVolume = AiEsp32RotaryEncoder(ROTARY_ENCODER2_CLK_PIN, ROTARY_ENCODER2_DT_PIN, -1, -1);
*/  
  //------------------------------------------------------------------------------
  void encoderStation_loop() 
  //------------------------------------------------------------------------------
  {
    //lets see if anything changed
    int16_t encoderDelta = encoderStation.encoderChanged();
    
    //optionally we can ignore whenever there is no change
    if (encoderDelta == 0) 
    {
      return;
    }
    else
    {
      stations.radioStation = encoderStation.readEncoder();
      t0.setText(stations.station[stations.radioStation].label);
    }         
  }
/*  
  //------------------------------------------------------------------------------
  void encoderVolume_loop() 
  //------------------------------------------------------------------------------
  {
    //lets see if anything changed
    int16_t encoderDelta = encoderVolume.encoderChanged();
    
    //optionally we can ignore whenever there is no change
    if (encoderDelta == 0) return;

    volume = encoderVolume.readEncoder();
  }
*/
//------------------------------------------------------------------------------
// Network
//------------------------------------------------------------------------------
  WiFiClient  client; 

  //------------------------------------------------------------------------------
  void connectToWIFI()
  //------------------------------------------------------------------------------
  {
    WiFi.begin(ssid, pass);
      while (WiFi.status() != WL_CONNECTED) 
      {
        delay(500);
      }
  }

  //------------------------------------------------------------------------------
  void station_connect (int station_no ) 
  //------------------------------------------------------------------------------
  {
    if (client.connect(stations.station[station_no].host, stations.station[station_no].port ))
    {       
      client.print(String("GET ") + stations.station[station_no].path + " HTTP/1.1\r\n" +
                 "Host: " + stations.station[station_no].host + "\r\n" + 
                 "Connection: close\r\n\r\n");   
      t0.setText(stations.station[station_no].label);      
    }
  }

  //------------------------------------------------------------------------------
  void mp3stream()
  //------------------------------------------------------------------------------
  {
    uint16_t    queue_depth, queue_avail;
    uint16_t    num_chunks;
    
    // how many chunks are in the tcp buffer?
    num_chunks = client.available() / 32;

    // avoid queue overflow
    queue_depth = uxQueueMessagesWaiting(xQueueMp3);
    queue_avail = QUEUELENGTH - queue_depth;
    if (queue_avail < num_chunks)
    {
        num_chunks = queue_avail;
    }
    
    // fill queue with mp3 chunks
    while (num_chunks > 0)
    {
        uint8_t bytesread = client.read(mp3buff, 32);
        // Send a chunk.  Wait for 10 ticks for space to become
        // available if necessary.
        if( xQueueSend( xQueueMp3, ( void * ) &mp3buff, ( TickType_t ) 10 ) != pdPASS )
        { } // Failed to post the message, even after 10 ticks.
        num_chunks--;
    }
  }

//------------------------------------------------------------------------------
// setup
//------------------------------------------------------------------------------
void setup () 
{
    SPI.begin();
    
    encoderStation.begin();
    encoderStation.setup([]{encoderStation.readEncoder_ISR();});
    encoderStation.setBoundaries(0, stations.numStations-1, true);
    encoderStation.reset(0);
    encoderStation.enable();
/*    
    encoderVolume.begin();
    encoderVolume.setup([]{encoderVolume.readEncoder_ISR();});
    encoderVolume.setBoundaries(75,100,false);
    encoderVolume.reset(90);
    encoderVolume.enable(); */

    /* Set the baudrate which is for debug and communicate with Nextion screen. */
    nexInit();

    /* Register the pop event callback function of the current button component. */
    b0.attachPop(b0PopCallback, &b0);
    b1.attachPop(b1PopCallback, &b1);
    h0.attachPop(h0PopCallback, &h0);
    
    // connect to access point
    t0.setText("Connecting to WiFi...");      
    connectToWIFI();

    // import station list
    t0.setText("Reading station list...");      
    HTTPClient  http;
    http.begin(url_stationlist);
    int httpCode = http.GET();
    if(httpCode == HTTP_CODE_OK)
    {
      String payload = http.getString();
      stations.parseStations(payload);
    } 

    // initialize the mp3 board
    t0.setText("Initialize MP3 board..."); 
    initMP3Decoder();

    // Create a queue capable of containing 32 messages a 32 bytes values.
    t0.setText("Creating queue...");      
    xQueueMp3 = xQueueCreate( QUEUELENGTH, // The number of items the queue can hold.
                              32 );        // The size of each item in the queue

    // Create the mp3 player task
    t0.setText("Creating mp3 player task..."); 
    xTaskCreate(
                  task_mp3play,   // Function that implements the task.
                  "mp3player",    // Text name for the task.
                  STACK_SIZE,     // Stack size in bytes, not words.
                  NULL,           // Parameter passed into the task.
                  1,              // Priority at which the task is created.
                  &xHandle);      // Task handle to keep track of created task
}

//------------------------------------------------------------------------------
// loop
//------------------------------------------------------------------------------
void loop() 
{
    encoderStation_loop();
    //encoderVolume_loop();

    // switch radio station
    if(stations.radioStation!=stations.previousRadioStation)
    {
        vTaskSuspend( xHandle );      // suspend mp3 player task
        mp3Flush();                   // clear mp3 queue from previous station
        station_connect(stations.radioStation);
        stations.previousRadioStation = stations.radioStation;
        vTaskResume( xHandle ); 
    }

    // read tcp stream and fill queue  
    mp3stream();

   /*
    * When a pop or push event occured every time,
    * the corresponding component[right page id and component id] in touch event list will be asked.
    */
    nexLoop(nex_listen_list);   
}    
