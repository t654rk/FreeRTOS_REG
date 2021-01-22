#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f051x8.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "timers.h"  
#include "cmsis_os.h"   // osDelayUntil
/*
typedef struct Calculation_Integr     {
  uint8_t Simpl_Number;
  uint32_t Integr_Sin_A;                   // Переменные суммы по 24 точкам синуса
  uint32_t Integr_Sin_B;
  uint32_t Integr_Sin_C;
  uint32_t Integr_Sin_D;
} Calc_Integr; 
*/  
/************************************************** Defines *******************************************************************/
#define APBCLK   48000000UL
#define BAUDRATE 9600UL

TaskHandle_t xDMA1_Channel1_IRQ_Handle;

xQueueHandle Signal1PeriodData;         // Очередь для передачи из прерывания TIM15 значения периода сигнала

xQueueHandle USARTPeriodPrint;          // Очередь печати значения периода сигнала
xQueueHandle USARTRMSPrint;             // Очередь печати значения RMS сигнала
xQueueHandle FrequencyERRORPrint;       // Очередь печати сообщения об ошибке, когда TIM14 обновляется быстрей, чем считает vTaskTIM15_IRQ

xQueueHandle Integr_Simpl_Queue;        // Очередь сумм квадратов для рассчета квадратных корней



xSemaphoreHandle xBinarySemaphoreTIM15;         // Семафор псевдообработчика прерывания счета периода таймером TIM15
xSemaphoreHandle xBinarySemaphoreDMA1_Channel1; // Семафор псевдообработчика прерывания окончания передачи DMA1_Channel1 значений массива значений АЦП

SemaphoreHandle_t xCountingSemaphoreADCSpeedNormal;
// Счетный семафор прерывания TIM14. Нужен для контроля успеваем ли измерить, передать и посчитать значения симпла
// до запуска начала новых измерений

/******************************************** Use tasks FreeRTOS **************************************************************/


//void MCO_out (void);
void generator(uint32_t frequency);
void LedsInit (void);
void TIM1_Init(void);
void ADC_Init(void);
void TIM14_Init(uint16_t psc, uint16_t arr);
void TIM15_Init(void);
uint16_t sqrt_GLS (uint32_t x);
void USART1_Init (void);
void USART1_Error (void);
void USART1_Line_Rdy (void);
void USART1_Send (char chr);
void USART1_Send_String (char* str);

void vTaskLed1 (void *argument);
void vTaskLed2 (void *argument);
void vTaskSignal1PeriodMeasuring (void *argument);
void vTaskUSART1Print (void *argument);

void vTaskTIM15_IRQ (void *argument);
void vTaskDMA1_Channel1_IRQ (void *argument);
void vTaskPeriod_End (void *argument);




#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */