#include <STM32F405xx.h>
#include "lcd.h"
#include <stdio.h>

void ADC_Config(void) {
    RCC->APB2ENR |= (1 << 8); // Enable ADC1 clock

    ADC->CCR |= (1 << 23);  // Enable temperature sensor
    ADC1->SQR3 = 16;        // ADC1, Channel 16 (Temp Sensor)
    ADC1->SMPR1 |= (3 << 18); // Sampling time 56 cycles

    ADC1->CR2 |= (1 << 0);  // Enable ADC
}

uint16_t ADC_Read(void) {
    ADC1->CR2 |= (1 << 30); // Start ADC conversion
    while (!(ADC1->SR & (1 << 1))); // Wait for conversion
    return ADC1->DR; // Return ADC value
}

float GetTemperature(void) {
    uint16_t adc_value = ADC_Read();
    float V_sense = (adc_value * 3.3) / 4095; // Convert ADC value to voltage
    float temperature = ((V_sense - 0.76) / 0.0025) + 25; // Convert to °C
    return temperature;
}

int main(void) {
    lcd_gpio_init();
    lcd_init();
    ADC_Config();

    while (1) {
        float temperature = GetTemperature();

        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "Temp: %.1fC", temperature);

        lcd(0x01, 0); // Clear LCD
        lcd_string(temp_str);

        for (volatile int i = 0; i < 1000000; i++); // Simple delay
    }
}
/* prth jha
#include <stdint.h>
#include <stm32f405xx.h>

#include "lcd.h"
#include <stdio.h>

void adc_init(void);
void adc_conversion(void);
int var;
char buf[30];
float Vol;
float temperature;

int main(void)
{

	lcd_gpio_init();
	lcd_init();
	adc_init();
	while(1)
	{
		adc_conversion();
		lcd(0x80,0);
	    lcd_string("ADC Value:");
	    single_print(var);
		Vol = (float)(var*3.3) /4095;
		temperature=(float)((Vol - 0.76) / 0.0025) + 25; //convert to celsius
	    sprintf(buf,"Temperature:%.3f",temperature);
	    lcd(0xC0,0);
	    lcd_string(buf);
	}

}

void adc_init()
{
	    //RCC->AHB1ENR|=(1<<2);  //Enable clock for port c
		GPIOC->MODER|=(1<<2);  //enable gpioc moder for pc-2 analog mode for adc
		GPIOC->MODER|=(1<<3);
		RCC->APB2ENR|=(1<<8); //enable clock for adc-1
		ADC1->CR2=0; //make cr2 of adc-2 as 0
		ADC1->SQR1&=~(1<<20);
		ADC1->SQR1&=~(1<<21);
		ADC1->SQR1&=~(1<<22);
		ADC1->SQR1&=~(1<<23);

		ADC1->SQR3&=~(1<<0);    //make the bits 4,3,2,1,0 as 00001 since channel-no. 16 is used with binary value 10000
		ADC1->SQR3&=~(1<<1);
		ADC1->SQR3&=~(1<<2);
		ADC1->SQR3&=~(1<<3);
		ADC1->SQR3|=(1<<4);

		//ADC1->SQR3=16;

		ADC1->CR1|=(1<<8);    //scan mode enable

		ADC1->CR1&=~(1<<0);    //make the bits 4,3,2,1,0 as 00001 since channel-no. 16 is used with binary value 10000
		ADC1->CR1&=~(1<<1);
		ADC1->CR1&=~(1<<2);
		ADC1->CR1&=~(1<<3);
		ADC1->CR1|=(1<<4);

		ADC1->SMPR1|=(1<<18);
		ADC1->SMPR1|=(1<<19);
		ADC1->SMPR1&=~(1<<20);

		ADC1->CR2|=(1<<0);
		ADC->CCR|=(1<<23);
}
void adc_conversion()
{
	    ADC1->CR2|=(1<<30);
		while(!(ADC1->SR&(1<<1)));
		var=ADC1->DR;
}
*/
