#include <stdint.h>
#include <stm32f405xx.h>
#include "lcd.h"
// pin 51 and 52 usart
uint8_t read(void);
void write(uint8_t);
void usart_init(void);
void switch_press(void);

uint8_t res=0;
uint32_t current=0;
uint32_t last=1;
uint32_t cnt=0;

int main(void)
{
    /* Loop forever */
	RCC->AHB1ENR|=(1<<1); //enable rcc for switch
	GPIOB->MODER&=~(3<<14);
	lcd_gpio_init();
	lcd_init();
	usart_init();

	while(1)
		{
			current=(GPIOB->IDR&(1<<7));
			if(current!=last)
			{
				for (volatile int i = 0; i < 100000; i++);  // simple debounce
				current=(GPIOB->IDR&(1<<7));
				if(current==0)
				{
					cnt++;
				}
				last=current;
			}
			if(cnt>0)
			{
				write('A'+cnt-1);
		        for(unsigned int i=0;i<1000;i++);
		        res=read();
		        char buffer1[2]={res,'\0'};
		        lcd(0x80,0);
		        lcd_string(buffer1);
			}
		}



}

void usart_init()
{
	RCC->AHB1ENR|=(1<<2);  //clock enable for port-c
	RCC->APB1ENR|=(1<<18); //enable clock for usart3
	GPIOC->MODER&=~(1<<20); //enable gpio for usart3 pc-10 and pc-11 alternate mode
	GPIOC->MODER|=(1<<21);
	GPIOC->MODER&=~(1<<22);
	GPIOC->MODER|=(1<<23);

	GPIOC->AFR[1]|=(1<<8); //enable afrh af-7 for pc-10
	GPIOC->AFR[1]|=(1<<9);
	GPIOC->AFR[1]|=(1<<10);
	GPIOC->AFR[1]&=~(1<<11);

	GPIOC->AFR[1]|=(1<<12); //enable afrh af-7 for pc-11
	GPIOC->AFR[1]|=(1<<13);
	GPIOC->AFR[1]|=(1<<14);
	GPIOC->AFR[1]&=~(1<<15);

	USART3->BRR=0X0683;
	USART3->CR1|=(1<<2);  //enable reciever enable
	USART3->CR1|=(1<<3);  //enable transmitter enable
	USART3->CR2=0;
	USART3->CR3=0;
	USART3->CR1|=(1<<13); //enable usart
}

void write(uint8_t val)
{
	while(!(USART3->SR&=~(1<<7)));
	USART3->DR=val;
}

uint8_t read()
{
	while(!(USART3->SR&=~(1<<5)));
	return USART3->DR;
}
