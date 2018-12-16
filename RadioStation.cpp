/*
 * RadioStation.cpp
 */

#include "RadioStation.h"

//------------------------------------------------------------------------------
// parse station list from downloaded file
//------------------------------------------------------------------------------
void StationList::parseStations(String lines)      
{
  int newlineAt, semicolonAt;
  String line;
  String label, host, path;

  while (lines.indexOf("\n") != -1)
  {
    // find first newline
    newlineAt = lines.indexOf("\n");
    // grab line
    line = lines.substring(0, newlineAt);
    // shorten string
    lines.remove(0, newlineAt+1);
    while (line.indexOf(";") != -1)
    {
      // find first semicolon
      semicolonAt = line.indexOf(";");
      // grab field
      label = line.substring(0, semicolonAt);
      label.toCharArray(station[numStations].label, label.length()+1);
      // shorten string
      line.remove(0, semicolonAt+1);
    
      // repeat for each field
      semicolonAt = line.indexOf(";");
      host  = line.substring(0, semicolonAt);
      host.toCharArray(station[numStations].host, host.length()+1);
      line.remove(0, semicolonAt+1);
    
      semicolonAt = line.indexOf(";");
      path  = line.substring(0, semicolonAt);
      path.toCharArray(station[numStations].path, path.length()+1);
      line.remove(0, semicolonAt+1);
    
      // the rest
      station[numStations].port  = line.toInt();        
      break;
    }
    numStations++;
    if (numStations > MAXSTATIONS)
      { break; }
  }
}
