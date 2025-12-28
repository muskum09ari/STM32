#include <stdint.h>
#include <stm32f405xx.h>
#include "lcd.h"

uint8_t message[] = "HELLO DMA";
uint8_t res = 0;

void usart3_init(void);
void dma1_stream3_init(uint8_t *data, uint32_t len);
void delay(void);

int main(void)
{
    lcd_gpio_init();
    lcd_init();

    usart3_init();
    dma1_stream3_init(message, sizeof(message) - 1);

    lcd(0x80, 0);
    lcd_string("USART3 DMA TX");

    delay();  // wait kr, jab tak transmit ho jaye

    lcd(0xC0, 0);
    lcd_string("Rcd: ");

    //  RX se padh aur print
    for (int i = 0; i < 9; i++)
    {
        while (!(USART3->SR & (1 << 5)));  // RXNE
        res = USART3->DR;
        char buffer[2] = {res, '\0'};
        lcd_string(buffer);
        delay();
    }


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

    USART3->BRR = 0x0683;
    USART3->CR1 |= (1 << 3) | (1 << 2);  // TE + RE
    USART3->CR1 |= (1 << 13);           // UE
    USART3->CR3 |= (1 << 7);            // Enable DMA for TX
}

void dma1_stream3_init(uint8_t *data, uint32_t len)
{
    RCC->AHB1ENR |= (1 << 21);  // DMA1 clock enable

    DMA1_Stream3->CR &= ~1;
    while (DMA1_Stream3->CR & 1);

    DMA1_Stream3->PAR = (uint32_t)&USART3->DR;
    DMA1_Stream3->M0AR = (uint32_t)data;
    DMA1_Stream3->NDTR = 9;
    DMA1_Stream3->CR |= (1 << 4); // Transfer complete interrupt enable
    DMA1_Stream3->CR |= (1 << 2); // Transfer error interrupt enable

    DMA1_Stream3->CR &= ~(7 << 25); // clear channel
    DMA1_Stream3->CR |= (4 << 25);  // Channel 4

    DMA1_Stream3->CR |= (1 << 6);   // DIR: mem to periph
    DMA1_Stream3->CR |= (1 << 10);  // MINC
    DMA1_Stream3->CR &= ~((3 << 11) | (3 << 13)); // 8-bit

    DMA1_Stream3->FCR = 0;          // Direct mode
    DMA1_Stream3->CR |= 1;          // Enable DMA stream
}

void delay(void)
{
    for (volatile int i = 0; i < 100000; i++);
}
