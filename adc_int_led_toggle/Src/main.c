#include <stdint.h>
#include <stdio.h>
#include <STM32F4xx.h>
#include "lcd.h"
//PC6 LED will toggle ON/OFF with each ADC conversion

//LCD will continuously display ADC voltage

char buf[16];
float Vol;
volatile uint16_t result; // Must be volatile since it changes in ISR

void ADC_init() {
    // Enable ADC1 clock (RCC->APB2ENR)
    RCC->APB2ENR |= (1 << 8);

    // Enable GPIOC clock for ADC (PC1) and LED (PC6)
    RCC->AHB1ENR |= (1 << 2);  // Enable clock for GPIOC

    // Configure PC1 as Analog mode (ADC Input)
    GPIOC->MODER |= (3 << 2);  // Set MODER1 to 11 (Analog mode)

    // Configure PC6 as Output for LED
    GPIOC->MODER |= (1 << 12);  // Set PC6 as output (bit 12 = 1, bit 13 = 0)
    GPIOC->MODER &= ~(1 << 13);

    // Ensure ADC is disabled before configuration
    ADC1->CR2 &= ~(1 << 0);

    // Select channel 11 (PC1) as first conversion in regular sequence
    ADC1->SQR3 |= (11 << 0);

    // Set ADC sampling time for channel 11 (SMPR1 register)
    ADC1->SMPR1 |= (3 << 3); // Sampling time selection for channel 11

    // Enable scan mode (CR1, bit 8)
    ADC1->CR1 |= (1 << 8);

    // Enable End of Conversion Interrupt (EOCIE)
    ADC1->CR1 |= (1 << 5);

    // Enable ADC1
    ADC1->CR2 |= (1 << 0);

    // Enable ADC interrupt in NVIC (IRQ18 for ADC1)
    NVIC_EnableIRQ(ADC_IRQn);
}

// Interrupt Service Routine (ISR) for ADC1
void ADC_IRQHandler(void) {
    if (ADC1->SR & (1 << 1)) { // Check if End Of Conversion (EOC) flag is set
        result = ADC1->DR; // Read ADC result (also clears EOC flag)

        // Toggle PC6 LED
        GPIOC->ODR ^= (1 << 6); // Flip LED state

        // Display ADC Value on LCD
        Vol = (float)(result * 3.0) / 4095;
        sprintf(buf, "Vol: %.3f", Vol);

        lcd(0x80, 0);
        lcd_string("CH11");
        single_print(result);

        lcd(0xC0, 0);
        lcd_string(buf);

        // Start next conversion
        ADC1->CR2 |= (1 << 30);
    }
}

int main(void) {
    lcd_gpio_init();
    lcd_init();
    ADC_init();

    // Start ADC Conversion
    ADC1->CR2 |= (1 << 30); // Start first conversion

    while (1) {
        // Main loop does nothing, ADC result is handled in interrupt
    }
}
