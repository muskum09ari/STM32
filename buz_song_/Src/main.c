
#include <stdint.h>
#include <STM32F405xx.h>

void delay(uint32_t time) {
    for (uint32_t i = 0; i < time * 4000; i++);
}

void configureGPIO() {
    // Enable GPIOC
    RCC->AHB1ENR |= (1 << 2);

    // Configure PC9 as output for buzzer
    GPIOC->MODER &= ~(3 << (9 * 2));
    GPIOC->MODER |= (1 << (9 * 2));
}

void buzzerOn() {
    GPIOC->ODR |= (1 << 9);
}

void buzzerOff() {
    GPIOC->ODR &= ~(1 << 9);
}

void playNote(uint32_t frequency, uint32_t duration) {
    uint32_t period = 1000000 / frequency;
    uint32_t half_period = period / 2;
    uint32_t cycles = (frequency * duration) / 1000;

    for (uint32_t i = 0; i < cycles; i++) {
        buzzerOn();
        delay(half_period);
        buzzerOff();
        delay(half_period);
    }
}

int main(void) {
    configureGPIO();

    // Play Mario tune
    while (1) {
        playNote(659, 150); // E5
        delay(150);
        playNote(659, 150); // E5
        delay(300);
        playNote(659, 150); // E5
        delay(300);
        playNote(523, 150); // C5
        delay(150);
        playNote(659, 150); // E5
        delay(300);
        playNote(784, 150); // G5
        delay(600);
        playNote(392, 150); // G4
        delay(600);

        // Pause before repeating
        delay(2000);
    }
}


