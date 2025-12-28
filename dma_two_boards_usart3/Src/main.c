//usart two boards A is DMA board to usart peripheral usart 3 se tx aur usart 3 is received

#include <stm32f4xx.h>
#include "lcd.h"

char data[] = "DMA USART3 Send\r\n";

void dma1_stream3_init(void)
{
    RCC->AHB1ENR |= (1 << 21);

    DMA1_Stream3->CR &= ~1;
    while (DMA1_Stream3->CR & DMA_SxCR_EN);

    DMA1_Stream3->PAR = (uint32_t)&USART3->DR;
    DMA1_Stream3->M0AR = (uint32_t)data;
    DMA1_Stream3->NDTR = sizeof(data) - 1;

    // Channel 4 so 100 bit 27 to 25 usart tx
       DMA1_Stream3->CR &= ~(1 << 25);
       DMA1_Stream3->CR &= ~(1 << 26);
       DMA1_Stream3->CR &= (1 << 27);

    DMA1_Stream3->CR |=  (1 << 6); // Memory to peripheral 01
    DMA1_Stream3->CR &=  ~(1 << 7); // Memory to peripheral
    DMA1_Stream3->CR |= (1 << 10);  // Memory increment

    //byte(8bit) so 11-14= 0
        DMA1_Stream3->CR &= ~((1 << 11) |(1 << 12)| (1 << 13)|(1 << 14)); // Byte size for mem/periph

}

void usart3_init(void)
{
	RCC->APB1ENR |= (1 << 18);  // Enable USART3 clock
    RCC->AHB1ENR |= (1 << 2);   // Enable GPIOC clock

    GPIOC->MODER&=~(1<<20); //PC10 so 10*2=20
     GPIOC->MODER|=(1<<21);

     //Alternate mode 10 AFR7 0111 for pc10
         	GPIOC->AFR[1]|=(1<<8);
         	GPIOC->AFR[1]|=(1<<9);
         	GPIOC->AFR[1]|=(1<<10);
         	GPIOC->AFR[1]&=~(1<<11);

    USART3->BRR = 0x0683; // 9600 baud @ 16 MHz
    USART3->CR3 |= (1<<7); // Enable DMA for TX
    USART3->CR1 |= ((1<<3)| (1<<13)); //ue and te bit
}

int main(void)
{
    usart3_init();
    dma1_stream3_init();

    DMA1_Stream3->CR |= (1<<0);  // Start DMA

    while (1);
}

/*
#include <stm32f4xx.h>
#include "lcd.h"

void usart3_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    GPIOC->MODER |= (2 << (10 * 2)) | (2 << (11 * 2));
    GPIOC->AFR[1] |= (7 << ((10 - 8) * 4)) | (7 << ((11 - 8) * 4));

    USART3->BRR = 0x0683; // 9600 baud
    USART3->CR1 |= USART_CR1_RE | USART_CR1_UE;
}

uint8_t usart3_read(void)
{
    while (!(USART3->SR & USART_SR_RXNE));
    return USART3->DR;
}

int main(void)
{
    lcd_gpio_init();
    lcd_init();
    usart3_init();

    lcd(0x80, 0);
    lcd_string("USART3 RX:");

    while (1)
    {
        uint8_t ch = usart3_read();
        lcd(0xC0, 0);
        single_print(ch);
    }
}
 */

