#include "stm32f051x8.h"
#include "UART.h"
#include <string.h>


//______________________________________________________________________________
void USART1_Init (void){
	
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;              // GPIO port A clock enable
        
        GPIOB->OTYPER &= ~GPIO_OTYPER_OT_6;             // Output push-pull (reset state)
        GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR6;             // No pull-up, pull-down
        GPIOB->MODER |= GPIO_MODER_MODER6_1;            // Alternate function mode
        GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1;     // 40 MHz High speed
        
        GPIOB->MODER |= GPIO_MODER_MODER7_1;           // Alternate function mode
        GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1;    // 40 MHz High speed

// Это если подключить на PA9 & PA10
//        GPIOA->AFR[1]   |= 0x1 << 4;                    // USART1_TX
//        GPIOA->AFR[1]   |= 0x1 << 8;                    // USART1_RX
//        GPIOA->AFR[1]   |= 0x110;                       // И даже вот так! )))
        
// А если подключить на PB6 & PB7, то можно ни фига не писать, потому что там это AF0, а там и так нули
        GPIOB->AFR[0]   &= ~(0xF << 24);                    // USART1_TX
        GPIOB->AFR[0]   &= ~(0xF << 28);                    // USART1_RX
        
//        RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;        // USART1 reset Непонятно как он сбрасывается.
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

// В stm32f0 Есть забавное прерывание по приему символа с определенным кодом
        USART1->CR2 |=   0x24<<USART_CR2_ADD_Pos; //Activate when receive '$'
//        USART1->CR2 |=   0x12<<USART_CR2_ADD_Pos; //Activate when receive '\r''
//        USART1->CR2 |=   0x15<<USART_CR2_ADD_Pos; //Activate when receive '\n'
        
        
        RCC->CFGR3     &= ~RCC_CFGR3_USART1SW;          // USART1SW[1:0] bits Почистить оба бита
        RCC->CFGR3     |=  RCC_CFGR3_USART1SW_SYSCLK;   //System clock (SYSCLK) selected as USART1 clock
//        RCC->CFGR3     |=  RCC_CFGR3_USART1SW_PCLK;     // PCLK clock used as USART1 clock source
	
	USART1->CR1 &= ~USART_CR1_M;                    // Данные - 8 бит
        USART1->CR2 &= ~USART_CR2_STOP;                 // 1 стоп-бит
        USART1->BRR =(APBCLK+BAUDRATE/2)/BAUDRATE;      // Скорость usart
//        USART1->BRR = 0x1388;                            // Скорость usart 9600
	
	USART1->CR1 |= USART_CR1_TE;                    // USART1 ON, TX ON, RX ON
	USART1->CR1 |= USART_CR1_RE;
	USART1->CR1 |= USART_CR1_UE;
        // If the USART is disabled (UE=0) during an auto baud rate operation, the BRR value may be corrupted.
        
//        while(!(USART1->ISR & USART_ISR_TC)); // polling idle frame Transmission
//        USART1->ICR |= USART_ICR_TCCF; // clear TC flag
        
        // Все прерывания перенесены в Main
        
//      USART1->CR1 |= USART_CR1_CMIE; // Enable CM interrupt прерывание Character match (совпадение символов)
	
//	USART1->CR1 |= USART_CR1_RXNEIE;                // RXNE Int ON	
//      USART1->CR1 |= USART_CR1_TXEIE;                 // TXNE Int ON	
//	NVIC_EnableIRQ(USART1_IRQn);
	
}
//______________________________________________________________________________
void USART1_Error (void){
}
//______________________________________________________________________________
void USART1_Line_Rdy (void){
}
//______________________________________________________________________________
void USART1_Send (char chr){
	
	while (!(USART1->ISR & USART_ISR_TC));
	USART1->TDR = chr;
	
}
//______________________________________________________________________________
void USART1_Send_String (char* str){
	
	uint8_t i = 0;
	
	while(str[i])
	USART1_Send (str[i++]);
	
}
/*
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern __IO uint8_t USART1_RX_Flag;
extern __IO uint8_t USART2_RX_Flag;

__IO uint8_t win_in_usart_symbol;
__IO uint8_t gsm_in_usart_symbol;

__IO usart_ring usart_ring_win;
__IO usart_ring usart_ring_gsm;



//------------------------------------------------------------------------------
void RING_Clear(usart_ring __IO * buf)
{
    buf->idxIN = 0;
    buf->idxOUT = 0;
}
//------------------------------------------------------------------------------
uint8_t RING_GetCount(usart_ring __IO * buf)
{
    return (buf->idxIN - buf->idxOUT)&BUF_MASK;
}
//------------------------------------------------------------------------------
void RING_Put(__IO uint8_t value, usart_ring __IO * buf)
{
    buf->cycleBuf[buf->idxIN++] = value;
    buf->idxIN &= BUF_MASK;
}
//------------------------------------------------------------------------------
uint8_t RING_Get(usart_ring __IO * buf)
{
    uint8_t value = buf->cycleBuf[buf->idxOUT++];
    buf->idxOUT &= BUF_MASK;
    return value;
}
//------------------------------------------------------------------------------
void RING_Read(usart_ring __IO * buf, char *data, uint8_t len)
{
    for(uint8_t i = 0; i < len; i++)
    {
        data[i] = buf->cycleBuf[buf->idxOUT++];
        buf->idxOUT &= BUF_MASK;
    }
}
//------------------------------------------------------------------------------
void RING_Write(char *data, usart_ring __IO * buf, uint8_t len)
{
    for(uint8_t i = 0; i < len; i++)
    {
        buf->cycleBuf[buf->idxIN++] = data[i];
        buf->idxOUT &= BUF_MASK;
    }
}
//------------------------------------------------------------------------------
void WIN_RxCallBack(void)
{
//  RING_Put(win_in_usart_symbol, &usart_ring_win);
  usart_ring_win.cycleBuf[usart_ring_win.idxIN++] = win_in_usart_symbol;
  usart_ring_win.idxIN &= BUF_MASK;
  if(win_in_usart_symbol == '\r')
	{
//                RING_Put('\n', &usart_ring_win);
                usart_ring_win.cycleBuf[usart_ring_win.idxIN++] = '\n';
                usart_ring_win.idxIN &= BUF_MASK;
                USART1_RX_Flag = 1;
	}
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&win_in_usart_symbol, 1);  
}

//------------------------------------------------------------------------------
void GSM_RxCallBack(void)
{
//  RING_Put(gsm_in_usart_symbol, &usart_ring_gsm);
  usart_ring_gsm.cycleBuf[usart_ring_gsm.idxIN++] = gsm_in_usart_symbol;
  usart_ring_gsm.idxIN &= BUF_MASK;
  if(gsm_in_usart_symbol == '\n')
	{
                USART2_RX_Flag = 1;
	}
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&gsm_in_usart_symbol, 1);  
}
*/
/*
//////////////// проверка и установка скорости 19200, нужна один раз /////////////////
void chek_speed(void)
{
  for(uint8_t i = 0; i < 7; i++)
  {
	  uint32_t sp = 0;

	  if(i == 0) sp = 2400;
	  else if(i == 1) sp = 4800;
	  else if(i == 2) sp = 9600;
	  else if(i == 3) sp = 19200;
	  else if(i == 4) sp = 38400;
	  else if(i == 5) sp = 57600;
	  else if(i == 6) sp = 115200;

	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = sp;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
		  Error_Handler();
	  }

	  char str[16] = {0,};
	  HAL_UART_Transmit(&huart2, (uint8_t*)"AT\r\n", strlen("AT\r\n"), 1000);
	  HAL_Delay(300);

	  if(gsm_available()) //если модуль что-то прислал
	  {
		  uint16_t i = 0;

		  while(gsm_available())
		  {
			  str[i++] = gsm_read();
			  if(i > 15) break;
			  HAL_Delay(1);
		  }

		  if(strstr(str, "OK") != NULL)
		  {
			  char buf[64] = {0,};
			  snprintf(buf, 64, "Uart modem was %lu, switched to 57600\n", huart1.Init.BaudRate);
			  HAL_UART_Transmit(&huart1, (uint8_t*)win10.usart_buf, strlen(win10.usart_bufz), 100);
			  HAL_UART_Transmit(&huart2, (uint8_t*)"AT+IPR=57600\r\n", strlen("AT+IPR=57600\r\n"), 1000);
			  HAL_Delay(250);
			  MX_USART2_UART_Init();
			  break;
		  }
	  }
  }
}*/