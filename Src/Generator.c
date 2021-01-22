#include "stm32f051x8.h"
#include "Sin_DAC_3V.h"    // static volatile uint16_t Sin_DAC [24] = {};

#define APBCLK   48000000UL

// DACoutput = VDDA x DOR / 4096

void DAC_Init(void);
void DMA1_Init(void);
void TIM6_init(void);

//______________________________________________________________________________
static uint16_t TIM6_prescalerRegister;
static uint16_t TIM6_autoReloadRegister;
//______________________________________________________________________________
void generator(uint32_t frequency) {
  
      if (frequency <= 50)
      {
	TIM6_prescalerRegister = 60 - 1;
	TIM6_autoReloadRegister = APBCLK / 60 / frequency / 24;       // 24 - колличество точек синуса
      }
      if ((frequency > 50) && (frequency <= 100))
      {
	TIM6_prescalerRegister = 2 - 1;
	TIM6_autoReloadRegister = APBCLK / 2 / frequency / 24;       // 24 - колличество точек синуса
      }
      if (frequency > 100)
      {
	TIM6_prescalerRegister = 0;
	TIM6_autoReloadRegister = APBCLK / frequency / 24;       // 24 - колличество точек синуса
      }

    DAC_Init();
    DMA1_Init();
    TIM6_init();
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
//______________________________________________________________________________
void DAC_Init(void)
{
  GPIOA->MODER |= GPIO_MODER_MODER4;       // 11: Analog mode Чтобы избежать паразитного потребления
  
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  
  DAC->CR |= DAC_CR_EN1;
  
  DAC->CR &= ~DAC_CR_BOFF1;
//  DAC->CR |= DAC_CR_BOFF1;
  // Когда буфер отключен, выходное сопротивление канала может достигать 15 кОм.
  // Для получения точности преобразования 1%, сопротивление нагрузки (между DAC_OUT и VSS)
  // должно быть не меньше 1.5 МОм. При включённом буфере требования к нагрузке куда более мягкие,
  // сопротивление нагрузки должно быть от 5 кОм или выше, а ёмкость - не более 50 пФ.
  // С другой стороны, при отключённом буфере на выходе преобразователя можно получить напряжение
  // из полного диапазона 0..VREF+. Буфер же обеспечивает работу только с сигналами в диапазоне 0.2 В..VDDA-0.2 В.
  
  DAC->CR |= DAC_CR_TEN1;       // DAC channel1 Trigger enable
  //  DAC->CR |= DAC_CR_TSEL1;      // Вывод данных по програмному событию DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;    // программное события запуска
  DAC->CR &= ~DAC_CR_TSEL1;     // Timer6 TRGO event Используется, только если бит TEN1 = 1 (триггер ЦАП 1 включен).
  DAC->CR |= DAC_CR_DMAEN1;     // DAC channel1 DMA enable
  DAC->CR |= DAC_CR_DMAUDRIE1;  // DAC channel1 DMA Underrun Interrupt enable Разрешение прерывания из-за недогрузки DMA

 }

//______________________________________________________________________________
void DMA1_Init(void)
{
RCC->AHBENR |= RCC_AHBENR_DMA1EN;       // Включили тактирование DMA1

        DMA1_Channel3->CCR |= DMA_CCR_MINC;     // Memory increment mode                   Инкрементный режим адреса памяти
        DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0; // будем передавать 16 бит данных
        DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0;
        DMA1_Channel3->CCR |= DMA_CCR_CIRC;    // цыклическая передача  Configure increment, size (32-bits), interrupts, transfer from memory to peripheral and circular mode
        DMA1_Channel3->CCR |= DMA_CCR_DIR;     // чтение с памяти

        DMA1_Channel3->CNDTR = 24;                      // Счетчик массива Configure the number of DMA tranfer to be performs on channel 3
        DMA1_Channel3->CPAR = (uint32_t) &DAC->DHR12R1; // Configure the peripheral data register address
        DMA1_Channel3->CMAR = (uint32_t) Sin_DAC;       // Configure the memory address

        DMA1_Channel3->CCR |= DMA_CCR_EN;      // вкл. ПДП

}

//______________________________________________________________________________
void TIM6_init(void)
{
        RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;             // тактирование таймера

        TIM6->PSC = TIM6_prescalerRegister;             // предделитель
        TIM6->ARR = TIM6_autoReloadRegister;            // переполнение две секунды 
        TIM6->CR1 |= TIM_CR1_ARPE;     //Включен режим предварительной записи регистра автоперезагрузки

//        TIM6->DIER |= TIM_DIER_UIE;             // прерывание по переполнению все-таки наверно не нужно
        TIM6->DIER |= TIM_DIER_UDE;             // Update DMA request enable Здесь событие, а не прерывание.
        //  Поэтому обработчик не нужен. Само прерывание тоже не включаем
                
        TIM6->CR2 |= TIM_CR2_MMS_1;     // Событие обновления используется как выход триггера
        TIM6->CR1 |= TIM_CR1_CEN;               // запуск счета                 
}

//______________________________________________________________________________
void DAC_ConversionSpeedError (void)
{
    DAC->CR &= ~DAC_CR_EN1;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    TIM6->CR1 &= ~TIM_CR1_CEN;
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    GPIOC->BSRR |= GPIO_BSRR_BS_8;
}
