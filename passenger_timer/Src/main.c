#include <stdint.h>
#include <STM32f405xx.h>

void delay_ms(uint32_t ms);
void init_GPIO(void);
void init_EXTI(void);

uint8_t sw1_state = 0;
uint8_t sw2_state = 0;
uint8_t buzzer_on = 0;
uint32_t elapsed_time = 0;
uint32_t off_time = 1000;

int main(void)
{
    init_GPIO();
    init_EXTI();

    while (1)
    {
        if (sw1_state && !sw2_state)
        {
            buzzer_on = 1;
        }
        else
        {
            buzzer_on = 0;
            off_time = 1000;
            elapsed_time = 0;
            GPIOC->ODR &= ~(1 << 9); // Buzzer Off
            GPIOC->ODR &= ~(1 << 6); // LED1 Off
            GPIOB->ODR &= ~(1 << 13); // LED3 Off
        }

        if (buzzer_on)
        {
            if (elapsed_time >= 10000)
            {
                GPIOC->ODR ^= (1 << 9); // Toggle Buzzer
                GPIOC->ODR ^= (1 << 6); // Toggle LED1
                GPIOB->ODR ^= (1 << 13); // Toggle LED3

                if (!(GPIOC->ODR & (1 << 9))) // If buzzer is off, decrease off time
                {
                    if (off_time > 0)
                    {
                        off_time -= 100;
                    }
                }

                delay_ms(1000); // On Time: 1 Second
                delay_ms(off_time); // Off Time
            }
        }
    }
}

void init_GPIO(void)
{
    RCC->AHB1ENR |= (1 << 2) | (1 << 1); // Enable clock for GPIOC and GPIOB

    // Configure PC9 as output for buzzer
    GPIOC->MODER |= (1 << 18);
    GPIOC->MODER &= ~(1 << 19);

    // Configure PC6 as output for LED1
    GPIOC->MODER |= (1 << 12);
    GPIOC->MODER &= ~(1 << 13);

    // Configure PB13 as output for LED3
    GPIOB->MODER |= (1 << 26);
    GPIOB->MODER &= ~(1 << 27);

    // Configure PB7 and PB3 as input for switches
    GPIOB->MODER &= ~(3 << 14); // PB7
    GPIOB->MODER &= ~(3 << 6);  // PB3
}

void init_EXTI(void)
{
    RCC->APB2ENR |= (1 << 14); // Enable SYSCFG clock

    // Configure EXTI for PB7 (Switch1) and PB3 (Switch2)
    SYSCFG->EXTICR[1] |= (1 << 12); // PB7 for Switch1
    SYSCFG->EXTICR[0] |= (1 << 12); // PB3 for Switch2

    EXTI->FTSR |= (1 << 7) | (1 << 3);
    EXTI->IMR |= (1 << 7) | (1 << 3);

    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI3_IRQHandler(void)
{
    if (EXTI->PR & (1 << 3))
    {
        sw2_state ^= 1; // Toggle Seatbelt Sensor State
        EXTI->PR |= (1 << 3);
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & (1 << 7))
    {
        sw1_state ^= 1; // Toggle Seat Occupancy State
        EXTI->PR |= (1 << 7);
    }
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 4000; i++);
}
