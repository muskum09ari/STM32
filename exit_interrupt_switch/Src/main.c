// TOGGLE LED WITH interrupt OF SWITCH 2

#include <stm32f4xx.h>  // CMSIS header for STM32F4xx
#include <stdint.h>

int main(void)
{
	RCC->AHB1ENR |= (1<<1); // GPIOb Enable, Bit 2 of AHB1ENR port b

	// Configure PB13 as output (LED2)
	GPIOB->MODER &=~(1<<27); //  led 2
	GPIOB->MODER |= (1<<26); //  led 2

	//switch-2
	GPIOB->MODER &=~(3<<6); //switch 2

	RCC->APB2ENR |= (1<<14); // syscfg en pg 248

	SYSCFG->EXTICR[0] &=~(1<<15); //pg 291
	SYSCFG->EXTICR[0] &=~(1<<14);
	SYSCFG->EXTICR[0] &=~(1<<13);
	SYSCFG->EXTICR[0] |=(1<<12); // Set to GPIOB for EXTI3

	// Configure EXTI for Switch2 (PB4)
	EXTI->RTSR |=(1<<3); // // Rising edge trigger PAGE 385
	EXTI->IMR |=(1<<3); //  Enable interrupt mask for EXTI3 PG 384

	NVIC_EnableIRQ(EXTI3_IRQn);	// EXTI interrupt enable


	while(1);
}

// Interrupt Handler for EXTI3
void EXTI3_IRQHandler()
{
	if (EXTI->PR & (1 << 3))
	{
		GPIOB->ODR ^= (1 << 13);
		for( unsigned int i=0; i<10000; i++);
		EXTI->PR |= (1 << 3);
	}
}


