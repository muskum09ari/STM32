//ADC channel 11 port 2 and 12 port1 & 2 together in one program
#include <stdint.h>
#include <stm32f405xx.h>
#include "lcd.h"
#include <stdio.h>

void lcd_gpio_init(void);
void lcd_init(void);
void lcd(uint8_t val,uint8_t cmd);
void lcd_string(char* str);
void single_print(uint32_t val);
void debounce(void);
void counter_init(void);

void adc1_init(void);
void adc2_init(void);
void adc1_conversion(void);
void adc2_conversion(void);

int var1;
float Vol1;
char buf1[30];

int var2;
float Vol2;
char buf2[30];
int main(void)
{
    /* Loop forever */
	lcd_gpio_init();
	lcd_init();
	adc1_init();
	adc2_init();
	while(1)
	{
		adc1_conversion();
		adc2_conversion();
		lcd(0x80,0);
		lcd_string("ADCp1:");//port 2 as channel 11
		single_print(var1);
		Vol1 = (float)(var1*3) /4095;
		sprintf(buf1,"Vol:%.3f",Vol1);
		lcd_string(buf1);
		lcd(0xC0,0);
		lcd_string("ADCp2:"); // port 1 as channel 12
		single_print(var2);
		Vol2 = (float)(var2*3) /4095;
		sprintf(buf2,"Vol:%.3f",Vol2);
		lcd_string(buf2);
	}

}
void adc1_init()
{
	RCC->AHB1ENR|=(1<<2);  //Enable clock for port c
	GPIOC->MODER|=(1<<2);  //enable gpioc moder for pc-1
	GPIOC->MODER|=(1<<3);
	RCC->APB2ENR|=(1<<8); //enable clock for adc-1
	ADC1->CR2=0; //make cr2 of adc-1 as 0
	ADC1->SQR1&=~(1<<20);
	ADC1->SQR1&=~(1<<21);
	ADC1->SQR1&=~(1<<22);
	ADC1->SQR1&=~(1<<23);

	ADC1->SQR3|=(1<<0);    //make the bits 4,3,2,1,0 as B(hex) since channel-no. 11 is used with binary value 01011
	ADC1->SQR3|=(1<<1);
	ADC1->SQR3&=~(1<<2);
	ADC1->SQR3|=(1<<3);
	ADC1->SQR3&=~(1<<4);

	ADC1->CR1|=(1<<8);

	ADC1->CR1|=(1<<0);    //make the bits 4,3,2,1,0 as B(hex) since channel-no. 11 is used with binary value 01011
	ADC1->CR1|=(1<<1);
	ADC1->CR1&=~(1<<2);
	ADC1->CR1|=(1<<3);
	ADC1->CR1&=~(1<<4);

	ADC1->CR2|=(1<<0);
}
void adc1_conversion()
{
	ADC1->CR2|=(1<<30);
	while(!(ADC1->SR&=~(1<<1)));
	var1=ADC1->DR;
}
void adc2_init()
{
	RCC->AHB1ENR|=(1<<2);  //Enable clock for port c
	GPIOC->MODER|=(1<<4);  //enable gpioc moder for pc-2
	GPIOC->MODER|=(1<<5);
	RCC->APB2ENR|=(1<<9); //enable clock for adc-2
	ADC2->CR2=0; //make cr2 of adc-2 as 0
	ADC2->SQR1&=~(1<<20);
	ADC2->SQR1&=~(1<<21);
	ADC2->SQR1&=~(1<<22);
	ADC2->SQR1&=~(1<<23);

	ADC2->SQR3&=~(1<<0);    //make the bits 4,3,2,1,0 as B(hex) since channel-no. 12 is used with binary value 01100
	ADC2->SQR3&=~(1<<1);
	ADC2->SQR3|=(1<<2);
	ADC2->SQR3|=(1<<3);
	ADC2->SQR3&=~(1<<4);

	ADC2->CR1|=(1<<8);    //scan mode enable

	ADC2->CR1&=~(1<<0);    //make the bits 4,3,2,1,0 as B(hex) since channel-no. 12 is used with binary value 01100
	ADC2->CR1&=~(1<<1);
	ADC2->CR1|=(1<<2);
	ADC2->CR1|=(1<<3);
	ADC2->CR1&=~(1<<4);

	ADC2->CR2|=(1<<0);
}
void adc2_conversion()
{
    ADC2->CR2|=(1<<30);
	while(!(ADC2->SR&(1<<1)));
	var2=ADC2->DR;
}
