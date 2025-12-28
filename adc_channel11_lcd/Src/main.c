

#include <stdint.h>
#include<stdio.h>
#include <STM32f405xx.h>
#include <lcd.h>

// adc port 2 pc1 channel 11 user namunal
char buf[16];
float Vol;
uint16_t result;

int ADC_conversion()
{
	ADC1->CR2 |=(1<<30); // start conversion
	// Wait for conversion to complete
		while(!(ADC1->SR & (1<<1))); //bit 2 is 1 so shift 1 to set check
	// Read ADC result - Page 409
		result = ADC1->DR;
		return result;

}

int main(void)
{
	lcd_gpio_init();
    lcd_init();

	// PG 281 AS PC1 FOR VARIABEL REGISTER AS analog so 11 so moder 1 as pc1

	GPIOC->MODER |= (1<<2);
	GPIOC->MODER |=(1<<3);

	//SET ENable for adc clock pg 187
	RCC->APB2ENR |=(1<<8);

	// Ensure ADC is disabled before configuration
	ADC1->CR2 &= ~(1<<0);

	// MAKE SEQUENCE REGISTER SQ1 BIT 0 TO 4 AS B bcz we are using channel 11 SO SQ1 0 TO 4  PG 423
	    ADC1->SQR3 &=~(1<<4);
		ADC1->SQR3 |=(1<<3);
		ADC1->SQR3 &=~(1<<2);
		ADC1->SQR3 |=(1<<1);
		ADC1->SQR3 |=(1<<0);

	// PAGE 422 SQR1 REGULAR CHANNEL SEQUANCE LENGTH FOR channel1 SO BIT has to be 11
	ADC1->SQR1 &=~(1<<20);
	ADC1->SQR1 &=~(1<<21);
	ADC1->SQR1 &=~(1<<22);
	ADC1->SQR1 &=~(1<<23);



	ADC1->CR1 |=(1<<8);// SCAN MODE ON
	// Set bits 0 to 4 as 11 (binary representation)
		//ADC1->CR1 |= (0xB);

	    ADC1->CR1 &=~(1<<4);
		ADC1->CR1 |=(1<<3);
		ADC1->CR1 &=~(1<<2);
		ADC1->CR1 |=(1<<1);
		ADC1->CR1 |=(1<<0);

	ADC1->CR2 |=(1<<0);// set bit 0 as 1 enable adc

	while(1){

		result = ADC_conversion();

	       lcd(0x80,0);
			lcd_string("CH11");

			single_print(result);

			Vol = (float)(result*3) /4095;
			sprintf(buf,"Vol:%.3f",Vol);
			lcd(0xC0,0);
			lcd_string(buf);
	}
}
