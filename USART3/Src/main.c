#include <stdint.h>
#include <stm32f405xx.h>
// connect 51 with 52 wire
void usart3_init(void);
void usart3_write(uint8_t);
uint8_t usart3_read(void);

uint8_t result=0;

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
	while (!(USART3->SR & (1 << 7)));//pg 1008
	USART3->DR= val;
}

uint8_t usart3_read()
{
	while(!(USART3->SR & (1<<5)));// Read data register not empty
	return USART3->DR;

}

int main(void)
{
	usart3_init();

	    while (1)
	    {
	        usart3_write('H'); // Send 'H'
	        for (int i = 0; i < 100; i++); // Small delay
	        result = usart3_read();
	    }
}
