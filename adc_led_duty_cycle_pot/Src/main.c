
/*Develop a logic to read the values of pot voltage using ADC and reflect the changes
  of ADC values to the pwm duty cycle in proportion, demonstrate it on LED,
 and display the ADC value and duty cycle on LCD.*/

#include <stdint.h>
#include<stdio.h>
#include <STM32f405xx.h>
#include <lcd.h>

uint16_t result_ch11;
char buf[16];
float Vol_ch11;
uint8_t dutyCycle;

void ADC_init()
{
	// Configure PC1 as analog mode (Channel 11) - Page 281
	GPIOC->MODER |= (1<<2) | (1<<3);

	// Enable ADC1 clock - Page 187
	RCC->APB2ENR |= (1<<8);

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
	TIM3->CCMR1 |= (6<<4); // PWM Mode 1 on Channel 1
	TIM3->CCER |= (1<<0); // Enable Channel 1
	TIM3->CCR1 = 0; // Initial duty cycle 0%

	// Enable Timer
	TIM3->CR1 |= (1<<0);
}

void ADC_Read()
{
	// Start conversion
	ADC1->CR2 |= (1<<30);

	// Wait for conversion to complete using bit 1
	while(!(ADC1->SR & (1<<1)));

	// Read ADC result for channel 11 -
	result_ch11 = ADC1->DR;
}

void update_PWM(uint16_t adc_value)
{
	dutyCycle = (adc_value * 100) / 4095; // Scale 0-4095 to 0-100%
	TIM3->CCR1 = (dutyCycle * TIM3->ARR) / 100; // Set PWM duty cycle
}

int main(void)
{
	lcd_gpio_init();
	lcd_init();
	ADC_init();
	PWM_init();

	while(1)
	{
		ADC_Read();

		// Update PWM based on ADC Channel 11 (PC1)
		update_PWM(result_ch11);

		// Display for channel 11 (PC1)
		lcd(0x80,0);
		lcd_string("Ch11 ADC: ");
		single_print(result_ch11);
		Vol_ch11 = (float)(result_ch11 * 3.3) / 4095;
		sprintf(buf, "Vol: %.3fV", Vol_ch11);
		lcd_string(buf);

		// Display Duty Cycle
		lcd(0xC0,0);
		lcd_string("Duty Cycle: ");
		sprintf(buf, "%d%%", dutyCycle);
		lcd_string(buf);
	}
}
