#include "stm32f10x.h"
#include "GPIOReg.h"


#define TIMEMAX (86400)


struct Clock {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
};
void ClockConvertTime(struct Clock* clock, uint32_t time) {
	clock->seconds = time % 60;
	clock->minutes = (time / 60) % 60;
	clock->hours = (time / 3600) % 60;
}


enum {
	ClockModeClock = 0,
	ClockModeSetClock,
	ClockModeSetAlarm
} ClockMode;


const uint32_t time_changes[7] = { 0, 36000, 3600, 600, 60, 10, 1 };


uint8_t active = 0;
uint32_t clock_time;
struct Clock clock;
uint32_t alarm_time;
struct Clock alarm;
uint8_t buzzer;


void SystemCoreClockConfigure(void) {
  RCC->CR |= RCC_CR_HSION;      						                // Enable HSI
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                   // Wait for HSI Ready

  RCC->CFGR = RCC_CFGR_SW_HSI;                              // HSI is system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);   // Wait for HSI used as system clock
	
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                          // HCLK = SYSCLK
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;                         // APB1 = HCLK/4
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                         // APB2 = HCLK

  RCC->CR &= ~RCC_CR_PLLON;                                 // Disable PLL

  //  PLL configuration:  = HSE * 2 (24 MHz)
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
  RCC->CFGR |=  (RCC_CFGR_PLLSRC_HSE| RCC_CFGR_PLLMULL2);

  RCC->CR |= RCC_CR_PLLON;                                  // Enable PLL
  while((RCC->CR & RCC_CR_PLLRDY) == 0);									  // Wait till PLL is ready

  RCC->CFGR &= ~RCC_CFGR_SW;                                // Select PLL as system clock source
  RCC->CFGR |=  RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);   // Wait till PLL is system clock src
}

void TIM3_IRQHandler(void) {
	TIM3->SR &= ~TIM_SR_UIF;
	
	GPIOA->ODR ^= 0x1 << 5;
	if (ClockMode == ClockModeClock) {
		clock_time = (clock_time + 1) % TIMEMAX;
		ClockConvertTime(&clock, clock_time);
		if (clock_time == alarm_time) { buzzer = 1; }
	}
}
void TIM3_Init(void) {
	RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN; 				 						// TIM 3 clock enable
	
	TIM3->PSC = 24000 - 1;																		// dt = 1s
	TIM3->ARR = 1000 - 1;
	
	TIM3->DIER |= TIM_DIER_UIE;										 						// Enable TIM3 interrupt
	NVIC_EnableIRQ(TIM3_IRQn);														 		// Enable TIM3 interrupt	in interrupt controller
	TIM3->CR1 |= TIM_CR1_CEN;														 			// TIM3 counter enable
}

void GPIO_Init (void) {
  RCC->APB2ENR |= (1UL << 2);																// Enable GPIOA clock
	// PA5
  GPIOA->CRL &= ~(GPIO_MODE_CONF << GPIO_PORT_5);
  GPIOA->CRL |= (GPIO_MODE_OUTPUT_10MHz | GPIO_CONF_OUTPUTG_PUSHPULL) << GPIO_PORT_5;
	// PA6
  GPIOA->CRL &= ~(GPIO_MODE_CONF << GPIO_PORT_6);
  GPIOA->CRL |= (GPIO_MODE_OUTPUT_10MHz | GPIO_CONF_OUTPUTG_PUSHPULL) << GPIO_PORT_6;
}

void EXTI0_IRQHandler(void) {      
	EXTI->PR |= EXTI_PR_PR0;
	
	ClockMode = (ClockMode + 1) % 3;
	active = 0;
}
void EXTI1_IRQHandler(void) {      
	EXTI->PR |= EXTI_PR_PR1;
	
	if (ClockMode != ClockModeClock) {
		active = (active + 1) % 7;
		if (active == 0) { ClockMode = ClockModeClock; }
	}
	buzzer = 0;
}
void EXTI2_IRQHandler(void) {      
	EXTI->PR |= EXTI_PR_PR2;
	
	if (ClockMode == ClockModeSetClock) {
		clock_time = (clock_time + time_changes[active]) % TIMEMAX;
		ClockConvertTime(&clock, clock_time);
	}
	else if (ClockMode == ClockModeSetAlarm) {
		alarm_time = (alarm_time + time_changes[active]) % TIMEMAX;
		ClockConvertTime(&alarm, alarm_time);
	}
}
void EXTI3_IRQHandler(void) {      
	EXTI->PR |= EXTI_PR_PR3;
	
	if (ClockMode == ClockModeSetClock) {
		clock_time = (clock_time - time_changes[active]) % TIMEMAX;
		ClockConvertTime(&clock, clock_time);
	}
	else if (ClockMode == ClockModeSetAlarm) {
		alarm_time = (alarm_time - time_changes[active]) % TIMEMAX;
		ClockConvertTime(&alarm, alarm_time);
	}
}
void Buttons_Init(void) {
	RCC->APB2ENR |= (1UL << 3);																// Enable GPIOB clock
	
	// PB0 (function)
  GPIOB->CRL &= ~(GPIO_MODE_CONF << GPIO_PORT_0);
  GPIOB->CRL |= (GPIO_MODE_INPUT | GPIO_CONF_INPUT_PULL) << GPIO_PORT_0;
	GPIOB->BSRR |= GPIO_BSRR_BR0;
	AFIO->EXTICR [0] |= AFIO_EXTICR1_EXTI0_PB;
	EXTI->RTSR |= EXTI_RTSR_TR0;
	EXTI->IMR |= EXTI_IMR_MR0;
	NVIC_EnableIRQ(EXTI0_IRQn);
	// PB1 (accept)
  GPIOB->CRL &= ~(GPIO_MODE_CONF << GPIO_PORT_1);
  GPIOB->CRL |= (GPIO_MODE_INPUT | GPIO_CONF_INPUT_PULL) << GPIO_PORT_1;
	GPIOB->BSRR |= GPIO_BSRR_BR1;
	AFIO->EXTICR [0] |= AFIO_EXTICR1_EXTI1_PB;
	EXTI->RTSR |= EXTI_RTSR_TR1;
	EXTI->IMR |= EXTI_IMR_MR1;
	NVIC_EnableIRQ(EXTI1_IRQn);
	// PB2 (add)
  GPIOB->CRL &= ~(GPIO_MODE_CONF << GPIO_PORT_2);
  GPIOB->CRL |= (GPIO_MODE_INPUT | GPIO_CONF_INPUT_PULL) << GPIO_PORT_2;
	GPIOB->BSRR |= GPIO_BSRR_BR2;
	AFIO->EXTICR [0] |= AFIO_EXTICR1_EXTI2_PB;
	EXTI->RTSR |= EXTI_RTSR_TR2;
	EXTI->IMR |= EXTI_IMR_MR2;
	NVIC_EnableIRQ(EXTI2_IRQn);
	// PB3 (sub)
  GPIOB->CRL &= ~(GPIO_MODE_CONF << GPIO_PORT_3);
  GPIOB->CRL |= (GPIO_MODE_INPUT | GPIO_CONF_INPUT_PULL) << GPIO_PORT_3;
	GPIOB->BSRR |= GPIO_BSRR_BR3;
	AFIO->EXTICR [0] |= AFIO_EXTICR1_EXTI3_PB;
	EXTI->RTSR |= EXTI_RTSR_TR3;
	EXTI->IMR |= EXTI_IMR_MR3;
	NVIC_EnableIRQ(EXTI3_IRQn);
}

// radix = 0x0A
// radix = 0x10
int main (void) {
	SystemCoreClockConfigure();
	SystemCoreClockUpdate();
	
	GPIO_Init();
	Buttons_Init();
	
	TIM3_Init();
	
	while (1) { GPIOA->ODR ^= (buzzer & 0x1) << 6;}
	
	return 0;
}

