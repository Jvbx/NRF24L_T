

#include "gnss_utils.h"


 char   nmea_line[255]={0}; 
 uint8_t nmea_pos = 0;
 uint8_t msg_end = 0;
 uint8_t msg_start = 0;
  uint8_t msg_wait_crc = 2;
 uint8_t msg_ready = 0;
 
 
 
uint8_t nmea_getmessage(uint8_t* data)
{
 uint8_t rbuf = *data;
 
 
  if (rbuf == '$')
   
   { 
    uint8_t tmp_pos = 0;
    while (tmp_pos<255) 
     {
     nmea_line[tmp_pos] = 0; 
     tmp_pos++;
     }
    nmea_pos = 0;
    nmea_line[nmea_pos] = rbuf;     
    msg_start = 1; 
    msg_end = 0; 
    msg_ready = 0;
    msg_wait_crc = 2;
    nmea_pos++;
    return 0;
   } 
   
  if ((rbuf=='*')&(nmea_pos<255))
   {
    nmea_line[nmea_pos] = rbuf;
    msg_end = 1;
    nmea_pos++;
    return 0;
   }
   
 if ((!msg_ready)&(nmea_pos<255))
  {
   nmea_line[nmea_pos] = rbuf; 
   if ((nmea_line[nmea_pos-1] == '*')|(nmea_line[nmea_pos-2] == '*')) 
   {
    msg_wait_crc--;
    if ((msg_wait_crc) == 0) 
     {
      msg_ready = 1;
      msg_start = 0;
      msg_end = 0;
      
      
      return 1;
      
     }
   
   } 
   nmea_pos++;   
   
  }
   return 0;
}

