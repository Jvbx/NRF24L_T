#ifndef __NMEA_H__
#define __NMEA_H__

#ifdef __cplusplus
 extern "C" {
#endif

extern char Time[12]; //время
extern char Status[2]; //валидность
extern char SLatitude[16];  //Латитуда
extern char NS[3];                          // 
extern char SLongitude[12];         //Лонгитуда 
extern char EW[3];                          // 
extern char CourseTrue[10];                 // курс
extern char Data[12];                               //Дата
extern char SatCount[4];                    //используемых спутников
extern char AltitudaMSL[12];            //высота
extern char ViewSat[4];   //
extern char COG[8];                 //      
extern char COGstat[4];             //
extern char Speed[8];                       //скорость
extern char SpeedAlt[8];    //
extern char Knot[8];
extern unsigned char GLONAS_COUNT;
extern unsigned char GPS_COUNT;
extern volatile char DataDone;
extern unsigned char DataValid;
	 
	 
void NMEA_Parser(unsigned char data);
	 
	 
	 
#ifdef __cplusplus
}
#endif	 
#endif /* __NMEA_H__ */

