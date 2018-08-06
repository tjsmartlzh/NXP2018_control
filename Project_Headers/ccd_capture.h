/*
 * ccd_capture.h
 *
 *  Created on: Feb 1, 2018
 *      Author: SanqingQu
 */

#ifndef CCD_CAPTURE_H_
#define CCD_CAPTURE_H_

#include"MPC5604B.h"
#include"uart.h"
#include"pit.h"
#include"delay.h"
#include"gpio.h"

#define CCDR_SI  SIU.GPDO[33].R   //定义右线性传感器的端口 SI  C1
#define CCDR_CLK SIU.GPDO[36].R   //定义右线性传感器的端口 CLK C4
#define CCDR_LED SIU.GPDO[39].R   //用于调试 定义检测到EDGE则右转向灯亮  C7
void ccd_init();

void ccd_capture();

void ccd_threshold_detect();
void ccd_max_min_send(int pic[]);
void ccd_pix60pix100_send(int pic[]);

uint8_t ccd_edge_detect(uint8_t sta_pix,uint8_t end_pix,int threshold,int pic[]);

void ccd_img_send();

#endif /* CCD_CAPTURE_H_ */
