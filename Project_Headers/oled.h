/*
 * oled.h
 *
 *  Created on: Mar 9, 2016
 *      Author: zhongwei
 */

#ifndef OLED_H_
#define OLED_H_

#include "includes.h"


//	SIU.PCR[72].R = 0x0200;//OLED     E8 
//	SIU.PCR[74].R = 0x0200;//OLED     E10 
//	SIU.PCR[75].R = 0x0200;//OLED     E11 
//	SIU.PCR[42].R = 0x0200;//OLED     C10 
//	SIU.PCR[17].R = 0x0200;//OLED     B1 

#define OLED_CS_PIN   72	///< 片选 (Chip Select) //72
#define OLED_DC_PIN   74	///< Data/Command Control //74
#define OLED_RST_PIN  75	///< Reset //75
#define OLED_SDA_PIN  42	///< SPI Data MISO //42
#define OLED_SCL_PIN  17	///< SPI Clock //17

#define OLED_CS   SIU.GPDO[OLED_CS_PIN].R ///< 片选 (Chip Select)
#define OLED_DC   SIU.GPDO[OLED_DC_PIN].R ///< Data/Command Control
#define OLED_RST  SIU.GPDO[OLED_RST_PIN].R ///< Reset
#define OLED_SDA  SIU.GPDO[OLED_SDA_PIN].R ///< SPI Data MISO
#define OLED_SCL  SIU.GPDO[OLED_SCL_PIN].R ///< SPI Clock

#define OLED_CLK 50000 ///< 80MHz 在此表示为80,000
#define OLED_COL 128 ///< 屏幕宽度
#define OLED_ROW 64 ///< 屏幕高度
#define OLED_PAGE 8 ///< 屏幕页数

#define OLED_CLEAR OLED_Fill(0x00)

void OLED_Init(void);
void OLED_Cmd(BYTE cmd);
void OLED_Page(BYTE gray);
void OLED_Fill(BYTE gray);
void OLED_SetPointer(BYTE col, BYTE page);
void OLED_Delay(WORD ms);
void OLED_display();
void OLED_Num(int num);
void OLED_Float(float num_f);
void OLED_Char(unsigned char c);
void OLED_Str(char * s);
void OLED_Test(void);


#endif /* OLED_H_ */
