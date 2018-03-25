/*
 * initModeAndClock.c
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */

#include "MPC5604B.h"
void initModesAndClock(void)
{
	
	//模式配置: 使能DRUN, RUN0, SAFE, RESET模式
	  ME.MER.R = 0x0000001D;   

	//初始化锁相环，
	//外部晶振为8MHz，设置PLL0为64MHz  // 0x05400100: 0000 0101 0100 0000 0000 0001 0000 0000
	//设置IDF=2,ODF=4,NDIV=64;
	//锁相环输出时钟phi=(clkin*NDIV)/(IDF*ODF)=(8*64)/(2*4)=64MHz
	  CGM.FMPLL_CR.R = 0x05400100;

	//RUN0配置: 主电压调节器打开，Data Flash处于正常模式。Code Flash处于正常模式。
	//使能锁相环，锁相环输出时钟作为系统时钟。
	  ME.RUN[0].R   = 0x001F0074;           
	  
	//外设运行配置0: 外设在所有模式下都工作
	  ME.RUNPC[0].R = 0x000000FE;           
	 
	  // SIUL: 选择 ME.RUNPC[0] 的配置  
	  ME.PCTL[68].R = 0x00;                 

	// 设置进入RUN0模式
	  ME.MCTL.R = 0x40005AF0;               //写入模式和密钥
	  ME.MCTL.R = 0x4000A50F;               //写入模式和反密钥

	//等待模式转换完成
	  while(ME.GS.B.S_MTRANS) {};           
	//验证进入了RUN0模式
	  while(ME.GS.B.S_CURRENTMODE != 4) {}  
	
}
