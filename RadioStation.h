/*
 * RadioStation.h
 * class for webradio
 */

#include <Arduino.h>

#ifndef _RadioStation
#define _RadioStation

class RadioStation
{
  public:
    char    label[25];
    char    host[50];
    char    path[50];
    int port;
    // default constructor
    RadioStation() : label(), host(), path(), port(0)
    {    }
};

#define MAXSTATIONS 10

class StationList
{
  public:
    RadioStation  station[MAXSTATIONS];   // declare an object of class 
    uint16_t numStations = 0;                  // number of available stations
    uint16_t radioStation = 0;                 // index of choosen station 
    uint16_t previousRadioStation = -1;
    //default constructor
    StationList()  {}
    void parseStations(String lines);
};

#endif
