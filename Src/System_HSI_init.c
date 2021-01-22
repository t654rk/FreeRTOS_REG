#include "stm32f051x8.h"

// ��! � ���� ������ �������� � ������ startup_stm32f0xx.s, �� STM-�� ��� ����� ���������
// � ����� �������� �� ������� HSI... � �� �� ����� � RCC_CFGR_PLLMUL12 ���������� ����� ������ ��������
void SystemInit (void)
    	{   
        
    	//Turn ON HSI
        RCC->CR |= RCC_CR_HSION;
  
        //Wait until it's stable 
        while (!(RCC->CR & RCC_CR_HSIRDY));
    	
    	FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
        
    	RCC->CFGR  |= RCC_CFGR_PLLSRC_HSI_DIV2;
    	RCC->CFGR  |= RCC_CFGR_PLLMUL12; 
    	
    	RCC->CR |= RCC_CR_PLLON; 
    	while(!(RCC->CR & RCC_CR_PLLRDY));
    	
    	//Set PLL as SYSCLK
    	RCC->CFGR |= RCC_CFGR_SW_PLL | RCC_CFGR_SWS_PLL;
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {};
    	
    	}