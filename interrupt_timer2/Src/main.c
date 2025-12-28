

#include <stdint.h>


uint32_t volatile *NVICIRQ_En = (uint32_t *)0xE000E100;

uint32_t *RCC_AHB1ENR = (uint32_t*)0x40023830; // RCC AHB1 Clock Enable Register
uint32_t *RCC_APB1ENR = (uint32_t*)0x40023840; // RCC APB1 Clock Enable Register
uint32_t *GPIOB_MODER = (uint32_t*)0x40020400; // GPIOB Mode Register (Switches and LEDs)
uint32_t *GPIOB_ODR = (uint32_t*)0x40020414;   // GPIOB Output Data Register (Control LEDs)
uint32_t *TIM2_PSC = (uint32_t*)0x40000028;    // Timer2 Prescaler Register offset is 28 and timer address memory map.
uint32_t *TIM2_ARR = (uint32_t*)0x4000002C;    // Timer2 Auto-Reload Register PAGE 642
uint32_t *TIM2_CR1 = (uint32_t*)0x40000000;    // Timer2 Control Register 1
uint32_t *TIM2_SR = (uint32_t*)0x40000010;     // Timer2 Status Register
uint32_t *TIM2_DIER = (uint32_t*)0x4000000C;   // Timer2 DMA/Interrupt Enable Register

void TIM2_IRQHandler(void);

int main(void)
{
	// Enable GPIOB clock for button input
	    *RCC_AHB1ENR |= (1<<1);

	    // Enable Timer 2 clock on APB1
	    	*RCC_APB1ENR |= (1<<0);

	    // Configure PB13, PB14, PB15 as Outputs for LEDs
	        *GPIOB_MODER |= (1 << 26);   // Set PB13 as output
	        *GPIOB_MODER &= ~(1 << 27);  // Clear bit 27


	 // Configure Timer 2 with a 1 ms tick using 16 MHz clock
	        *TIM2_PSC = 16000-1;  // Prescaler
	        *TIM2_ARR = 1000;     // Auto-reload for 1 second



	              // Enable Timer 2 Update Interrupt
	              *TIM2_DIER |= (1 << 0);

	              *NVICIRQ_En |= (1 << 28); // Enable TIM2 IRQ in NVIC (IRQ28) page 373

	              // Start Timer 2
	              *TIM2_CR1 |= (1 << 0);

	    	while(1);
}

// Timer 2 Interrupt Service Routine (ISR)
void TIM2_IRQHandler(void)

{ *TIM2_SR &=~(1<<0);//clear UIF
*GPIOB_ODR ^= (1<<13); //toggle led
}



