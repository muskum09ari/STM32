#include <stdint.h>
#include <stm32f405xx.h>
#include <stdio.h>
#include "lcd.h"

// adc_toggle led

void adc_init(void);
void led_init(void);
volatile int var;
float Vol;
char buf[30];

int main(void)
{
    lcd_gpio_init();
    lcd_init();
    adc_init();
    led_init();  // Initialize PC6 as output

    NVIC_EnableIRQ(ADC_IRQn);

    while(1)
    {
        ADC1->CR2 |= (1 << 30);  // Start ADC Conversion
        lcd(0x80,0);
        lcd_string("ADC Value:");
        single_print(var);

        Vol = (float)(var * 3) / 4095;
        sprintf(buf, "Vol:%.3f", Vol);
        lcd(0xC0,0);
        lcd_string(buf);


    }
}

void adc_init()
{
    RCC->AHB1ENR |= (1 << 2);   // Enable clock for port C
    GPIOC->MODER |= (3 << 2);   // Set PC1 as Analog Mode



    RCC->APB2ENR |= (1 << 8);   // Enable ADC1 Clock

    ADC1->CR2 = 0;              // Disable ADC before configuring
    ADC1->SQR3 &= ~(0x1F << 0); // Clear previous selection
    ADC1->SQR3 |= (11 << 0);    // Set Channel 11 (PC1)

    ADC1->CR1 |= (1 << 5);      // Enable End of Conversion Interrupt
    ADC1->CR2 |= (1 << 0);      // Enable ADC1
}

void led_init()
{
    RCC->AHB1ENR |= (1 << 2);   // Enable clock for GPIOC
    GPIOC->MODER &= ~(3 << 12); // Clear MODER6 (PC6)
    GPIOC->MODER |= (1 << 12);  // Set PC6 as Output Mode
}

void ADC_IRQHandler(void)
{
    if (ADC1->SR & (1 << 1))  // Check EOC (End of Conversion) flag
    {
        var = ADC1->DR;       // Read ADC result
        ADC1->SR &= ~(1 << 1);  // Clear EOC flag

        GPIOC->ODR ^= (1 << 6);  // Toggle LED on PC6
        lcd(0x80, 0);
        lcd_string("ISR Triggered!");  // Debug message

        ADC1->CR2 |= (1 << 30);  // Start next ADC conversion
    }
}

