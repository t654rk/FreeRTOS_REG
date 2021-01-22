#ifndef	_USART_H
#define	_USART_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define APBCLK   48000000UL
#define BAUDRATE 9600UL

/*
typedef struct USART_prop       // property - свойство, реквизит )))
{
  uint8_t usart_buf[64];
  uint8_t usart_cnt;
} USART_buff;
#define usart_buf_len 64


#define BUF_SIZE 64
#define BUF_MASK (BUF_SIZE-1)
typedef struct usart_prop       //кольцевой (циклический) буфер
{
  uint8_t cycleBuf[BUF_SIZE];
  uint8_t idxIN;      //"указатель" хвоста буфера 
  uint8_t idxOUT;      //"указатель" головы буфера
} volatile usart_ring;


void RING_Clear(usart_ring volatile * buf);
uint8_t RING_GetCount(usart_ring volatile * buf);
void RING_Put(volatile uint8_t value, usart_ring volatile * buf);
uint8_t RING_Get(usart_ring volatile * buf);
void RING_Read(usart_ring volatile * buf, char *data, uint8_t len);
void RING_Write(char *data, usart_ring volatile * buf, uint8_t len);


void WIN_RxCallBack(void);
void GSM_RxCallBack(void);
*/
#endif