#include <stdint.h>
#include <stm32f405xx.h>

//adc led Develop a Fuel gauge by using 4 LEDS and a potentiometer(pot).
//The LEDs should show the value in a bar type logic, where with 25% of value, only one LED should glow(|   ),
//with 50%, the next(physically closest) LED should glow(||  ), with 75%, the next(physically closest) LED should glow(|||  )
//and with 100% value, all LEDS should glow(||||). The pot should be connected to an ADC input channel.
//Adjusting the potentiometer will indicate the level of fuel in tanks. Process ADC input within 0-100 in integers

void adc_init(void);
void adc_conversion(void);
uint32_t val;
int main(void)
{
    /* Loop forever */
	RCC->AHB1ENR|=(1<<2);   //enable rcc clock for port-c
	RCC->AHB1ENR|=(1<<1);   //enable rcc clock for port-b
	GPIOC->MODER|=(1<<12);  //enable gpioc moder led-1 pc-6
	GPIOC->MODER&=~(1<<13);
	GPIOB->MODER|=(1<<26);  //enable gpiob moder led-2 pb-13
	GPIOB->MODER&=~(1<<27);
	GPIOB->MODER|=(1<<28);  //enable gpiob moder led-3 pb-14
	GPIOB->MODER&=~(1<<29);
	GPIOB->MODER|=(1<<30);  //enable gpiob moder led-4 pb-15
	GPIOB->MODER&=~(1<<31);
	GPIOC->ODR|=(1<<6);
	GPIOB->ODR|=(1<<13);
	GPIOB->ODR|=(1<<14);
	GPIOB->ODR|=(1<<15);
	adc_init();
	while(1)
	{
		adc_conversion();
		if(val==1023)
		{
			GPIOC->ODR&=~(1<<6);
			GPIOB->ODR|=(1<<13);
			GPIOB->ODR|=(1<<14);
			GPIOB->ODR|=(1<<15);
		}
		else if(val==2047)
		{
			GPIOB->ODR&=~(1<<13);
			GPIOC->ODR|=(1<<6);
			GPIOB->ODR|=(1<<14);
			GPIOB->ODR|=(1<<15);
		}
		else if(val==3071)
		{
			GPIOB->ODR&=~(1<<14);
			GPIOB->ODR|=(1<<13);
			GPIOC->ODR|=(1<<6);
			GPIOB->ODR|=(1<<15);
		}
		else if(val==4095)
		{
			GPIOC->ODR&=~(1<<6);
			GPIOB->ODR&=~(1<<13);
			GPIOB->ODR&=~(1<<14);
			GPIOB->ODR&=~(1<<15);
		}
	}

}
void adc_init()
{
	RCC->AHB1ENR|=(1<<2);
	RCC->APB2ENR|=(1<<8);
	GPIOC->MODER|=(1<<2);
	GPIOC->MODER|=(1<<3);
	ADC1->CR2=0;
	ADC1->SQR1&=~(1<<20);
	ADC1->SQR1&=~(1<<21);
	ADC1->SQR1&=~(1<<22);
	ADC1->SQR1&=~(1<<23);

	ADC1->SQR3=11;
	ADC1->CR1|=(1<<8); //scan mode enable
	ADC1->CR1=11;
	ADC1->CR2|=(1<<0);
}

void adc_conversion()
{
	ADC1->CR2|=(1<<30);
	while(!(ADC1->SR&(1<<1)));
	val=ADC1->DR;
}
