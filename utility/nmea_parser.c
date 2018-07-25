#include <stdlib.h>     /* atoi */
#include <stdint.h>
#include <string.h>
#include "nmea_parser.h"
char SLongitudePtr[12]=""; 
char Time[12]=""; //время
char Status[2]=""; //валидность
char SLatitude[16]="";  //Латитуда
char NS[3]="";                          // 
char SLongitude[12]="";         //Лонгитуда 
char EW[3]="";                          // 
char CourseTrue[10]="";                 // курс
char Data[12]="";                               //Дата
char SatCount[4]="";                    //используемых спутников
char AltitudaMSL[12]="";            //высота
char ViewSat[4];   //
char COG[8]="";                 //      
char COGstat[4]="";             //
char Speed[8]="";                       //скорость
char SpeedAlt[8]="";    //
char UNUSED[32]="";                     //мусорка, тут все данные, которые не нужны
char Knot[8]="";
char *const RMC[]={Time,Status,SLatitude,NS,SLongitude,EW,UNUSED,CourseTrue,Data,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
char *const GGA[]={UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,SatCount,UNUSED,AltitudaMSL,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
char *const GSV[]={UNUSED,UNUSED,ViewSat,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
char *const VTG[]={COG,COGstat, UNUSED,UNUSED,Knot,UNUSED,Speed,UNUSED,UNUSED,UNUSED};
unsigned char GLONAS_COUNT=0;
unsigned char GPS_COUNT=0;
volatile char DataDone=0;
unsigned char DataValid=0;
uint8_t i = 0; 
static uint8_t accum[255];






void NMEA_Parser(uint8_t data)
{
static unsigned char ByteCount=0xff;
static unsigned int MsgType;                                    
static char *MsgTxt=(char*)&MsgType;     
static unsigned char ComaPoint=0xff;
static unsigned char CharPoint=0;

accum[i] = data;
i++;
if (i>=255) 
{i = 0;}
 //return;
 
 
if(data=='$'){ByteCount=0;ComaPoint=0xff;MsgTxt=(char*)&MsgType; DataDone=0;return;} //ждем начала стрки
if(ByteCount==0xff) return;                                                                     //
ByteCount++;
if(ByteCount<=1)        return;                                                         //
if(ByteCount<6&&ByteCount>1)            //берем 4 символа заголовка
        {
        *MsgTxt=data;   //и делаем из него число                                        
        MsgTxt++;
        return;
        }  
//
//SLongitudePtr=RMC[2];
memcpy(SLongitudePtr,RMC[2],12);
switch(MsgType)
        {
        case    0x434D5250:                             //GPRMC         
        case    0x434D524E:                             //GNRMC 
                if(data==',') {ComaPoint++;     CharPoint=0;RMC[ComaPoint][0]=0;return;}
                if(data=='*') {MsgType=0;DataDone=1;return;}
                RMC[ComaPoint][CharPoint++]=data;
                RMC[ComaPoint][CharPoint]=0;
                return;
        case    0x41474750:                             //PGGA
        case    0x4147474e:                             //NGGA
                if(data==',')  {ComaPoint++;    CharPoint=0;GGA[ComaPoint][0]=0;return;}
                if(data=='*') {MsgType=0;DataDone=1;return;}
                GGA[ComaPoint][CharPoint++]=data;
                GGA[ComaPoint][CharPoint]=0;
                return;
        case    0x47545650:             //PVTG
                if(data==',')  {ComaPoint++;    CharPoint=0;VTG[ComaPoint][0]=0;return;}
                if(data=='*') {return;}
                VTG[ComaPoint][CharPoint++]=data;
                VTG[ComaPoint][CharPoint]=0;
                return;
        case    0x4754564e:             //NVTG
                if(data==',')  {ComaPoint++;    CharPoint=0;VTG[ComaPoint][0]=0;return;}
                if(data=='*') {return;}
                VTG[ComaPoint][CharPoint++]=data;
                VTG[ComaPoint][CharPoint]=0;
                return;
        case    0x56534750:             //PGSV          
                if(data==',')  {ComaPoint++;    CharPoint=0;GSV[ComaPoint][0]=0;return;}
                if(data=='*')  {GPS_COUNT=(char)atoi(ViewSat);MsgType=0;DataDone=1;return;}
                GSV[ComaPoint][CharPoint++]=data;
                GSV[ComaPoint][CharPoint]=0;
                return;
        case    0x5653474c:             //LGSV          
                if(data==',')  {ComaPoint++;    CharPoint=0;GSV[ComaPoint][0]=0;return;}
                if(data=='*') {GLONAS_COUNT=(char)atoi(ViewSat);MsgType=0;DataDone=1;return;}
                GSV[ComaPoint][CharPoint++]=data;
                GSV[ComaPoint][CharPoint]=0;
                return;
        default:        ByteCount=0xff;break;
        }       
ByteCount=0xff;
}

