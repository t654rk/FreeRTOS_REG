#include "stm32f051x8.h"

//______________________________________________________________________________
void MCO_out (void){
	
	RCC->AHBENR     |= RCC_AHBENR_GPIOAEN;                     // enable clock for port A
	
        GPIOA->MODER    |= GPIO_MODER_MODER8_1;         // Альтернативная функция
        GPIOA->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR8;      // gpio speed 50 MHz
	GPIOA->AFR[1]   |= 0x0 << 0;   // Запись дебильная, но правильная так как там и так ноль после reset
        // Например
        // GPIOA->AFR[1]   |= 0x1 << 4;        // USART1_TX
        // GPIOA->AFR[1]   |= 0x1 << 8;        // USART1_RX
        //AFRLy selection:
        //0000: AF0       1000: AF8
        //0001: AF1       1001: AF9
        //0010: AF2       1010: AF10
        //0011: AF3       1011: AF11
        //0100: AF4       1100: AF12
        //0101: AF5       1101: AF13
        //0110: AF6       1110: AF14
        //0111: AF7       1111: AF15
        // Смотрим тавлицу в заголовке слева-направо номера альтернативных функций (Alternate functions pg. 44 Datasheet)
        // По умолчанию (после сброса) все ножки портов GPIOx настроены на альтернативную функцию AF0.
        // Вот этот номер: "Адин, два, тры... сэмь!" в шестнадцатеричном виде и пишем
        // в ->AFR[0], если лапа от 0 до 7 и в ->AFR[1] если лапа от 8 до 15
        // со смещением 4*N (N начинается с 0, а не с 1)чтобы метко попасть
        // в поля регистра GPIO_AFRL & GPIO_AFRH ->AFR[0] & AFR[1] соответственно.
        // Эти регистры 32-битные, и все их биты поделены на четверки бит, каждая четверка соответствует отдельной ножке порта GPIOx
        // http://microsin.net/programming/arm/stm32f407-gpio-pins-alternate-function.html

        //        RCC->CFGR |= RCC_CFGR_MCO_SYSCLK;
//        RCC->CFGR |= RCC_CFGR_MCO_HSE;          // Посмотрел осциллографом, все корректно
//        RCC->CFGR |= RCC_CFGR_MCO_HSI;          // Посмотрел осциллографом, все корректно
        RCC->CFGR |= RCC_CFGR_MCO_PLL;          // PllCLK/2
}