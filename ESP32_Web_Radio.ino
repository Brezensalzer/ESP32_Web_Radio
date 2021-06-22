// ESP32_Web_Radio.ino
//
// Project based on
// ESP32 Internet Radio Project     v1.00 
// http://educ8s.tv/esp32-internet-radio

#include <VS1053.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <Nextion.h>
#include <Arduino.h>
#include "RadioStation.h"
#include "config.h"

#define DEBUG false
#define LOGSERVER      "192.168.1.4"
#define UDP_LOGGER
#include <SimpleUDPLogger.h>

//------------------------------------------------------------------------------
// mp3 module
//------------------------------------------------------------------------------
  #define VS1053_CS    32 
  #define VS1053_DCS   33  
  #define VS1053_DREQ  35 
  #define MIN_VOLUME  75
  #define MAX_VOLUME  100
  uint8_t volume = 82; // volume level 0-100
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
  #define QUEUELENGTH 1024
  QueueHandle_t xQueueMp3;
  
  // Dimensions the buffer that the task being created will use as its stack.
  // NOTE:  This is the number of bytes the stack will hold, not the number of
  // words as found in vanilla FreeRTOS.
  #define STACK_SIZE 65536
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
        // Receive a message on the created queue.  Block for 10 ticks if a
        // message is not immediately available.
        if( xQueueReceive( xQueueMp3, &( mp3chunk ), ( TickType_t ) 10 ) )
        {
             // we now can consume mp3chunk
             player.playChunk(mp3chunk, 32);
             yield();
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

//------------------------------------------------------------------------------
// Rotary Encoders
// copied from https://github.com/Edzelf/ESP32-Radio
//------------------------------------------------------------------------------
  #define sv DRAM_ATTR static volatile    // alias

  //-----------------------------------------------------------------------------
  // radio station
  //-----------------------------------------------------------------------------
  #define ENC_STATION_CLK_PIN 21
  #define ENC_STATION_DT_PIN 22
  sv int16_t    rotationcount_station = 0;
  int16_t       oldcount_station = 0;

  //-----------------------------------------------------------------------------
  static void IRAM_ATTR isr_enc_station()
  //-----------------------------------------------------------------------------
  {
    sv uint32_t     old_state = 0x0001 ;                          // Previous state
    sv int16_t      locrotcount = 0 ;                             // Local rotation count
    uint8_t         act_state = 0 ;                               // The current state of the 2 PINs
    uint8_t         inx ;                                         // Index in enc_state
    sv const int8_t enc_states [] =                               // Table must be in DRAM (iram safe)
    { 0,                    // 00 -> 00
      -1,                   // 00 -> 01                           // dt goes HIGH
      1,                    // 00 -> 10
      0,                    // 00 -> 11
      1,                    // 01 -> 00                           // dt goes LOW
      0,                    // 01 -> 01
      0,                    // 01 -> 10
      -1,                   // 01 -> 11                           // clk goes HIGH
      -1,                   // 10 -> 00                           // clk goes LOW
      0,                    // 10 -> 01
      0,                    // 10 -> 10
      1,                    // 10 -> 11                           // dt goes HIGH
      0,                    // 11 -> 00
      1,                    // 11 -> 01                           // clk goes LOW
      -1,                   // 11 -> 10                           // dt goes HIGH
      0                     // 11 -> 11
    } ;
    // Read current state of CLK, DT pin. Result is a 2 bit binary number: 00, 01, 10 or 11.
    act_state = ( digitalRead ( ENC_STATION_CLK_PIN ) << 1 ) +
                  digitalRead ( ENC_STATION_DT_PIN ) ;
    inx = ( old_state << 2 ) + act_state ;                        // Form index in enc_states
    locrotcount += enc_states[inx] ;                              // Get delta: 0, +1 or -1
    if ( locrotcount == 4 )
    {
      rotationcount_station++ ;                                   // Divide by 4
      locrotcount = 0 ;
    }
    else if ( locrotcount == -4 )
    {
      rotationcount_station-- ;                                   // Divide by 4
      locrotcount = 0 ;
    }
    old_state = act_state ;                                       // Remember current status
  }

  //-----------------------------------------------------------------------------
  // volume
  //-----------------------------------------------------------------------------
  #define ENC_VOLUME_CLK_PIN 25
  #define ENC_VOLUME_DT_PIN 26
  sv int16_t    rotationcount_volume = volume;
  int16_t       oldcount_volume = 0;

  //-----------------------------------------------------------------------------
  static void IRAM_ATTR isr_enc_volume()
  //-----------------------------------------------------------------------------
  {
    sv uint32_t     old_state = 0x0001 ;                          // Previous state
    sv int16_t      locrotcount = 0 ;                             // Local rotation count
    uint8_t         act_state = 0 ;                               // The current state of the 2 PINs
    uint8_t         inx ;                                         // Index in enc_state
    sv const int8_t enc_states [] =  { 0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0 } ;
    // Read current state of CLK, DT pin. Result is a 2 bit binary number: 00, 01, 10 or 11.
    act_state = ( digitalRead ( ENC_VOLUME_CLK_PIN ) << 1 ) +
                  digitalRead ( ENC_VOLUME_DT_PIN ) ;
    inx = ( old_state << 2 ) + act_state ;                        // Form index in enc_states
    locrotcount += enc_states[inx] ;                              // Get delta: 0, +1 or -1
    if ( locrotcount == 4 )
    {
      rotationcount_volume++ ;                                    // Divide by 4
      locrotcount = 0 ;
    }
    else if ( locrotcount == -4 )
    {
      rotationcount_volume-- ;                                    // Divide by 4
      locrotcount = 0 ;
    }
    old_state = act_state ;                                       // Remember current status
  }

  //------------------------------------------------------------------------------
  void encoderStation_loop() 
  //------------------------------------------------------------------------------
  {
    //lets see if anything changed
    if ( rotationcount_station != oldcount_station )
    {
      if ( rotationcount_station > stations.numStations )
      {
        rotationcount_station = 0;
      }
      if ( rotationcount_station < 0 )
      {
        rotationcount_station = stations.numStations;
      }
      oldcount_station = rotationcount_station;
      stations.radioStation = rotationcount_station;  
    }
  }
  
  //------------------------------------------------------------------------------
  void encoderVolume_loop() 
  //------------------------------------------------------------------------------
  {
    //lets see if anything changed
    if ( rotationcount_volume != oldcount_volume )
    {
      if ( rotationcount_volume > MAX_VOLUME )
      {
        rotationcount_volume = MAX_VOLUME;
      }
      if ( rotationcount_volume < MIN_VOLUME )
      {
        rotationcount_volume = MIN_VOLUME;
      }
      oldcount_volume = rotationcount_volume;
      volume = (int) rotationcount_volume;
      player.setVolume(volume); 
      UDP_LOG_INFO("set volume to %lu", volume);
    }
  }

//------------------------------------------------------------------------------
// Network
//------------------------------------------------------------------------------
  HTTPClient http;
  WiFiClient * client; 

  //------------------------------------------------------------------------------
  void connectToWIFI()
  //------------------------------------------------------------------------------
  {
    if(DEBUG) {Serial.print("Connecting to Wifi");}
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) 
    {
      if(DEBUG) {Serial.print(".");}
      delay(500);
    }
    if(DEBUG) {Serial.println("\nWifi connected");}  
  }

  //------------------------------------------------------------------------------
  void station_connect (int station_no ) 
  //------------------------------------------------------------------------------
  {
    if (http.connected())
    {
      http.end(); // stop old connection
    }
    boolean isConnected = false;

    char rc[4];
    int httpCode;  
    char url[100];
    const char* headerNames[] = { "Location" };
    String Location;
    int pos;
    
    if(DEBUG) { Serial.println(stations.station[station_no].label); }
    if(DEBUG) { Serial.println(stations.station[station_no].url); }
    UDP_LOG_INFO(stations.station[station_no].label);      
    UDP_LOG_INFO(stations.station[station_no].url);

    t0.setText(stations.station[station_no].label);                 
    
    //-------------------------------
    //--- check for http redirect ---
    //-------------------------------
    http.begin(stations.station[station_no].url);
    http.collectHeaders(headerNames, sizeof(headerNames)/sizeof(headerNames[0]));
    httpCode = http.GET();    

    //------- follow redirect -----------
    if (httpCode == 302)
    {
      if(DEBUG) { Serial.println("following redirect"); }
      UDP_LOG_INFO("following redirect");
      Location = http.header("Location");
      pos = Location.indexOf('?');
      if (pos == -1)
      {
        Location.toCharArray(url, Location.length()+1);
      }
      else
      {
        Location.toCharArray(url, pos+1);
      }
      if(DEBUG) { Serial.println(url); }
      UDP_LOG_INFO(url);

      http.end();
      http.begin(url);
      httpCode = http.GET();
    }
    //-------------------------------

    if (httpCode == HTTP_CODE_OK)
    {
      client = http.getStreamPtr();
      isConnected = true;
    }
    else
    {
      if(DEBUG) { Serial.println("connection to station failed"); }
      UDP_LOG_INFO("connection to station failed");
      t0.setText("Station Error");            
    }
  }

  //------------------------------------------------------------------------------
  void mp3stream()
  //------------------------------------------------------------------------------
  {
    uint16_t    queue_depth, queue_avail;
    uint16_t    num_chunks;
    
    // how many chunks are in the tcp buffer?
    num_chunks = client->available() / 32;

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
        uint8_t bytesread = client->read(mp3buff, 32);
        // Send a chunk.  Wait for 10 ticks for space to become
        // available if necessary.
        if( xQueueSend( xQueueMp3, ( void * ) &mp3buff, ( TickType_t ) 10 ) != pdPASS )
        { // Failed to post the message, even after 10 ticks.
          t0.setText("Error writing to queue!");
          UDP_LOG_ERROR("Error writing to queue!"); 
          if(DEBUG) { Serial.println("Error writing to queue!"); }
        } 
        num_chunks--;
    }
  }

//------------------------------------------------------------------------------
// setup
//------------------------------------------------------------------------------
void setup () 
{
    if(DEBUG) { Serial.begin(115200); }
    SPI.begin();
    
    /* Set the baudrate which is for DEBUG and communicate with Nextion screen. */
    nexInit();

    // connect to access point
    t0.setText("Connecting to WiFi...");      
    connectToWIFI();

    // setup rsyslog
    UDP_LOG_BEGIN(LOGSERVER, LOG_MODE_DEBUG);
    UDP_LOG_INFO("Starting Rubis Internet Radio");

    // import station list
    if(DEBUG) { Serial.println("Reading station list..."); }
    UDP_LOG_INFO("Reading station list...");
    t0.setText("Reading station list...");      
    HTTPClient  http;
    http.begin(url_stationlist);
    int httpCode = http.GET();
    if(httpCode == HTTP_CODE_OK)
    {
      String payload = http.getString();
      stations.parseStations(payload);
    } 
    http.end();
    
    // initialize the mp3 board
    if(DEBUG) { Serial.println("Initializing MP3 board..."); }
    UDP_LOG_INFO("Initializing MP3 board...");
    t0.setText("Initializing MP3 board..."); 
    initMP3Decoder();
    if (!player.isChipConnected())
    { 
      if(DEBUG) { Serial.println("MP3 board failed!"); }
      UDP_LOG_ERROR("MP3 board failed!");
      t0.setText("MP3 board failed!");
      while(true)
        delay(1000);
    }
    
    // initialize rotary encoder
    pinMode(ENC_STATION_CLK_PIN, INPUT_PULLUP);
    pinMode(ENC_STATION_DT_PIN, INPUT_PULLUP);
    attachInterrupt ( digitalPinToInterrupt(ENC_STATION_CLK_PIN), isr_enc_station,   CHANGE ) ;
    attachInterrupt ( digitalPinToInterrupt(ENC_STATION_DT_PIN),  isr_enc_station,   CHANGE ) ;   
    pinMode(ENC_VOLUME_CLK_PIN, INPUT_PULLUP);
    pinMode(ENC_VOLUME_DT_PIN, INPUT_PULLUP);
    attachInterrupt ( digitalPinToInterrupt(ENC_VOLUME_CLK_PIN), isr_enc_volume,   CHANGE ) ;
    attachInterrupt ( digitalPinToInterrupt(ENC_VOLUME_DT_PIN),  isr_enc_volume,   CHANGE ) ;   
    
    // Create a queue capable of containing 32 messages a 32 bytes values.
    t0.setText("Creating queue...");      
    xQueueMp3 = xQueueCreate( QUEUELENGTH, // The number of items the queue can hold.
                              32 );        // The size of each item in the queue

    // Create the mp3 player task
    if(DEBUG) { Serial.println("Creating mp3 player task..."); }
    UDP_LOG_INFO("Creating mp3 player task...");
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
    encoderVolume_loop();

    // switch radio station
    if(stations.radioStation!=stations.previousRadioStation)
    {
        vTaskSuspend( xHandle );      // suspend mp3 player task
        mp3Flush();                   // clear mp3 queue from previous station
        station_connect(stations.radioStation);
        stations.previousRadioStation = stations.radioStation;
        mp3stream();  // prefill Queue
        vTaskResume( xHandle ); 
    }

    // read tcp stream and fill queue  
    mp3stream();

    // house keeping
    yield();
}
