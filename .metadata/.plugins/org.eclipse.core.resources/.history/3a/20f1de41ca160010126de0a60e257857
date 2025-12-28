#include <stdint.h>         //uart string code
#include <stm32f405xx.h>
#include <stdio.h>
#include "lcd.h"
#include <string.h>

// string use 51 and 52 wire for usart connection

void usart3_init(void);
uint8_t usart_read(void);
void usart_write(uint8_t);
//char string[30]="Hello";
uint8_t res=0;
int main(void)
{
    /* Loop forever */
	    lcd_gpio_init(); // Enable LCD GPIO
	    lcd_init();      // Initialize LCD
	    usart3_init();   // Initialize USART3

	    char *data= "Muskan kumari ";
	    for (int i = 0; data[i] != '\0'; i++) {
	    	usart_write(data[i]); // Send string
	    	for(int i=0; i<100; i++){};
	    	res = usart_read();
	    		//lcd(0x80,0);
	    	char buffer[2]={res,'\0'};
	    	lcd_string(buffer);
	    }

	//for(;;);
}
void usart3_init()
{
	RCC->AHB1ENR|=(1<<2);  //clock enable for gpioc
	RCC->APB1ENR|=(1<<18);  //enable clock for usart3
	GPIOC->MODER&=~(1<<20);  //enable gpio moder for pc-10 alternate function mode
	GPIOC->MODER|=(1<<21);
	GPIOC->MODER&=~(1<<22);  //enable gpio moder for pc-11 alternate function mode
	GPIOC->MODER|=(1<<23);
	GPIOC->AFR[1]|=(1<<8);  //enable gpio afr values for pc-10
	GPIOC->AFR[1]|=(1<<9);
	GPIOC->AFR[1]|=(1<<10);
	GPIOC->AFR[1]&=~(1<<11);
	GPIOC->AFR[1]|=(1<<12);  //enable gpio afr values for pc-11
	GPIOC->AFR[1]|=(1<<13);
	GPIOC->AFR[1]|=(1<<14);
	GPIOC->AFR[1]&=~(1<<15);
	USART3->BRR=0x0683;
	USART3->CR1|=(1<<2);
	USART3->CR1|=(1<<3);
	USART3->CR2=0;
	USART3->CR3=0;
	USART3->CR1|=(1<<13);
}

void usart_write(uint8_t val)
{
	while(!(USART3->SR&=~(1<<7)));
	USART3->DR=val;
}

uint8_t usart_read()
{
	while(!(USART3->SR&=~(1<<5)));
	return USART3->DR;
}
