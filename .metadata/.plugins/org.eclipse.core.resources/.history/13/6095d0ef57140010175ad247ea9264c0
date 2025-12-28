/*Design a system where a button triggers ADC conversion via EXTI interrupt.
 * The ADC value should determine the PWM duty cycle. Display the duty cycle and ADC value on LCD.
 * Show the corresponding brightness change in LED using PWM.
 * Further, once the PWM is updated, trigger a USART message saying "PWM updated" on LCD.  */
// usart doesnt show pwm update bcz no third line


#include <stdint.h>
#include<stdio.h>
#include <STM32f405xx.h>
#include <lcd.h>

void usart3_init();
uint8_t usart3_read();
void ADC_init();
uint16_t ADC_Read();
void PWM_init();
void EXTI9_5_IRQHandler();
void update_PWM(uint16_t);
void usart3_write_string(const char* str);

volatile uint8_t update_flag = 0;


uint16_t result_ch11;
char buf[16];
float Vol_ch11;
uint8_t dutyCycle;

void usart3_init()
{
	RCC->APB1ENR |=(1<<18);// ENABLE USART CLOCK
	RCC->AHB1ENR |=(1<<2);// EBABLE GPIO C CLOCK ENABLE BCZ pc10(Tx) and pc11(Rx)
	GPIOC->MODER |=(1<<23); // alternate function so 10 Transmission
	GPIOC->MODER &=~(1<<22);
	GPIOC->MODER |=(1<<21);//Reciver for pc 11
	GPIOC->MODER &=~(1<<20);

	//alternate function PC10 AND PC11 refer pg 272 usart on af7 so pg 286 so af7 0111.
	//pc11
		GPIOC->AFR[1] &=~(1<<15);
		GPIOC->AFR[1] |=(1<<14);
		GPIOC->AFR[1] |=(1<<13);
		GPIOC->AFR[1] |=(1<<12);

		//pc10
	GPIOC->AFR[1] &=~(1<<11);
	GPIOC->AFR[1] |=(1<<10);
	GPIOC->AFR[1] |=(1<<9);
	GPIOC->AFR[1] |=(1<<8);

	USART3->BRR=0x0683;
	// enable transmitter enable at bit 3 and receiver enable bit 2
	USART3->CR1 |=((1<<3)|(1<<2));// PG 1012 USART MODE ENABLE

	USART3->CR2=0; //sets all CR2 to 0 means stop bits,clock disabled
	USART3->CR3=0; // all things disabled works like uart
	USART3->CR1 |=(1<<13); // enable usart
}
void usart3_write(uint8_t val)
{
	while (!(USART3->SR & (1 << 7)));  // Wait until TXE is set
	USART3->DR = val;
}

// Definition
void usart3_write_string(const char* str) {
    while (*str) {
        usart3_write(*str);
        str++;
    }
}

void ADC_init(){

	// Enable ADC1 clock - Page 187
		RCC->APB2ENR |= (1<<8);

	//Turning MODER for ADC for PC1
	GPIOC->MODER |= (1<<2);
	GPIOC->MODER |=(1<<3);

	// Ensure ADC is disabled before configuration
		ADC1->CR2 &= ~(1<<0);

		// Configure the sequence register for channel 11 - Page 423
		ADC1->SQR3 = (11 << 0);

		// Set sequence length to 1 (1 conversion) - Page 422
		ADC1->SQR1 = 0;

		// Enable scan mode -
		ADC1->CR1 |= (1<<8);

		// Enable ADC -
		ADC1->CR2 |= (1<<0);

}

uint16_t ADC_Read()
{
	// Start conversion
	ADC1->CR2 |= (1<<30);

	// Wait for conversion to complete using bit 1
	while(!(ADC1->SR & (1<<1)));

	// Read ADC result for channel 11 -
	result_ch11 = ADC1->DR;
	return result_ch11;
}

void PWM_init()
{
	// Enable GPIOC Clock for LED on PC6 (TIM3 CH1)
	RCC->AHB1ENR |= (1<<2);

	GPIOC->MODER |= (1<<13);
	GPIOC->MODER &= ~(1<<12); // Alternate Function Mode for PC6

		// Set AF2 (TIM3_CH1) for PC6
		GPIOC->AFR[0] |= (2<<24);

	// Enable TIM3 Clock
	RCC->APB1ENR |= (1<<1);

	// Configure TIM3 for PWM Mode
	TIM3->PSC = 84 - 1; // Prescaler to get 1 MHz
	TIM3->ARR = 1000 - 1; // Period for 1 kHz PWM

	TIM3->CCMR1 &= ~(1 << 4); // (6,5,4)=110 PWM MODE 1 pg 636 oc1m
	TIM3->CCMR1 |= (1 << 5);
	TIM3->CCMR1 |= (1 << 6);

	// low for 0 to 6 / alternate function pC6 bcz af1 0010 (27,26,25,24) PG 286
			GPIOC->AFR[0] &= ~(1 << 24);
			GPIOC->AFR[0] &= ~(1 << 26);
			GPIOC->AFR[0] &= ~(1 << 27);
			GPIOC->AFR[0] |= (1 << 25);

	TIM3->CCER |= (1<<0); // Enable Channel 1
	TIM3->CCR1 = 0; // Initial duty cycle 0%

	// Enable Timer
	TIM3->CR1 |= (1<<0);
}



void update_PWM(uint16_t adc_value)
{
	dutyCycle = (adc_value * 100) / 4095; // Scale 0-4095 to 0-100%
	TIM3->CCR1 = (dutyCycle * TIM3->ARR) / 100; // Set PWM duty cycle
}
// EXTI init for PB7 button
void EXTI7_init() {
	//Clock enable for pb7
		RCC->AHB1ENR |= (1<<1);


		//configure for switch PB7
		GPIOB->MODER &=~(3<<6);

		RCC->APB2ENR |= (1<<14); // Enable SYSCFG for EXTI configuration

		//SYSCGG EXTIRCR PG 291 BCZ SWITCH 1 IS PB 7 SO BIT IS PB0001 SO register 2
			SYSCFG->EXTICR[1]|= (1<<12);
			SYSCFG->EXTICR[1]&= ~(1<<13);
			SYSCFG->EXTICR[1]&=~(1<<14);
			SYSCFG->EXTICR[1]&=	~(1<<15);

			// Enable rising edge trigger for Switch1,   Interrupt Mask Register.

					//When a bit in the IMR is set to 1, the corresponding external interrupt is unmasked(enabled)
					// and can generate an interrupt request (IRQ) to the NVIC (Nested Vector Interrupt Controller)

				EXTI->RTSR |= (1<<7);//Enables interrupt for Switch 1 (PB7)
				EXTI->IMR |= (1<<7);//risng edge

				NVIC_EnableIRQ(EXTI9_5_IRQn); // enable
}

//ISR when button is pressed
void EXTI9_5_IRQHandler()
{
	if (EXTI->PR & (1 << 7))
	{
		EXTI->PR |= (1 << 7); // Clear pending bit

		result_ch11 = ADC_Read();     // Read ADC
		update_PWM(result_ch11);      // Update PWM based on ADC
		update_flag = 1;              // Set flag to update display & USART in main loop
	}
}


int main(void){
			lcd_gpio_init();
				lcd_init();
				ADC_init();
				PWM_init();
				usart3_init();
				EXTI7_init();
				while (1)
				{
					if (update_flag)
					{
						update_flag = 0; // Clear flag

						// Update LCD
						lcd(0x80, 0);
						lcd_string("                ");  // Clear 16 chars
						lcd(0x80, 0);
						lcd_string("ADC: ");
						single_print(result_ch11);

						Vol_ch11 = (float)(result_ch11 * 3.3) / 4095;
						sprintf(buf, "V: %.2fV", Vol_ch11);
						lcd_string(buf);

						lcd(0xC0, 0);
						lcd_string("D:");
						sprintf(buf, "%d%%", dutyCycle);
						lcd_string(buf);
						lcd_string(" PWM upd");

						// USART message
						usart3_write_string("PWM updated\r\n");
					}
				}


}
