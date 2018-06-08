/*
 * uart.h
 *
 *  Created on: Mar 25, 2018
 *      Author: dell
 */

#ifndef UART_H_
#define UART_H_

/*
 * Entscheiden Sie PCR[18] als TX und PCR[19] als RX.
 * */



#include"MPC5604B.h"
//extern uint8_t datain[3];
typedef struct
{
   float k1;
   float k2;
   float b1;
   float b2;
}pram_t;

#define LEFT 1
#define RIGHT 2
#define FORWARD 3
#define BEHIND 4

void LINFlex_TX(unsigned char data);
void LINFlex_3TX(unsigned char data);

void BlueTx(char *send);
void Blue3Tx(char *send);

void LINFlex_RX(void);
void LINFlex_3RX(void);

void LINFlex_RX_Interrupt(void);

void initLINFlex_0_UART(uint8_t pri);
void initLINFlex_3_UART(uint8_t pri);


void f2s(float f, char* str);//��������f�����������������λС����ת��Ϊ�ַ���.

void UARTitosTX (int n);//������nת��Ϊ�ַ������ô��ڷ���
char* Int_to_char(int n);

#endif /* UART_H_ */
