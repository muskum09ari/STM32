

#include <stdint.h>
#include<stdio.h>
#include <STM32f405xx.h>
#include <lcd.h>

// adc port 2 pc2 channel 12
char buf[16];
float Vol;
uint16_t result;

int ADC_conversion()
{
	ADC1->CR2 |=(1<<30); // start conversion
	// Wait for conversion to complete
		while(!(ADC1->SR & (1<<1))); //bit 1 is 1 when conversion is complete
	// Read ADC result -
		result = ADC1->DR;
		return result;
}

int main(void)
{
	lcd_gpio_init();
    lcd_init();

	// PG 281 AS PC2 FOR VARIABLE REGISTER AS analog so 11 so moder 1 as pc2
	GPIOC->MODER |= (1<<4);
	GPIOC->MODER |=(1<<5);

	//SET Enable for adc clock pg 187
	RCC->APB2ENR |=(1<<8);

	// Ensure ADC is disabled before configuration
	ADC1->CR2 &= ~(1<<0);

	//  PAGE 422 SQR1 REGULAR CHANNEL SEQUANCE LENGTH FOR channel1 SO BIT has to be 11
		ADC1->SQR1 &=~(1<<20);
		ADC1->SQR1 &=~(1<<21);
		ADC1->SQR1 &=~(1<<22);
		ADC1->SQR1 &=~(1<<23);

	// Configure ADC to use channel 12 (PC2) SO SQ2 5 to 9 bit as 12- Page 423
	// MAKE SEQUENCE REGISTER BIT 0 TO 4 AS B bcz we are using channel 11 SO SQ1 0 TO 4  PG 423
			    ADC1->SQR3 &=~(1<<9);
				ADC1->SQR3 |=(1<<8);
				ADC1->SQR3 |=(1<<7);
				ADC1->SQR3 &=~(1<<6);
				ADC1->SQR3 &=~(1<<5);

				ADC1->CR1 = 0xC; // for 12th channel


	// Enable Scan Mode -
	ADC1->CR1 |=(1<<8);



	// Enable ADC -
	ADC1->CR2 |=(1<<0);

	while(1){

		result = ADC_conversion();

		lcd(0x80,0);
		lcd_string("ADC Value2 :");

		single_print(result);

		Vol = (float)(result*3.3) /4095; // Adjust for 3.3V reference
		sprintf(buf,"Vol:%.3fV",Vol);
		lcd(0xC0,0);
		lcd_string(buf);
	}
}
