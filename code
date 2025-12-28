# STM32



#include "stm32f405xx.h"
#include "lcd.h"
#include <string.h>


// === Integration Plan for Ultrasonic in Traffic System ===
// Includes CASE 1 and CASE 2 handling for Lane A and Lane B

uint32_t SystemCoreClock = 16000000;  // or whatever your real clock is


// Lane A Ultrasonic Pins
#define TRIG1_PIN   3   // PC3
#define ECHO1_PIN   6   // PC6
// Lane B Ultrasonic Pins
#define TRIG2_PIN   8   // PC8
#define ECHO2_PIN   12  // PC12

// Detection threshold and wait counters
#define DETECTION_THRESHOLD 1749
#define DETECTION_TIME_THRESHOLD 7  // seconds

uint8_t laneA_detect_counter = 0;
uint8_t laneB_detect_counter = 0;
uint8_t stateA, timerA;
uint8_t stateB, timerB;

// Security (Keypad & LCD) configuration
char passkey[5] = "2580";
char entered[5] = "";
char keypad_map[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

// TM1637 configuration
#define TM1637_A_CLK_PIN  10  // PB10
#define TM1637_A_DIO_PIN  11  // PB11
#define TM1637_B_CLK_PIN  11  // PC11
#define TM1637_B_DIO_PIN  10  // PC10

// LED configuration - First traffic light (PC0, PC1, PC2)
#define LEDA_RED    0   // PC0
#define LEDA_YELLOW 1   // PC1
#define LEDA_GREEN  2   // PC2

// LED configuration - Second traffic light (PA2, PA3, PA4)
#define LEDB_RED    2   // PA2
#define LEDB_YELLOW 3   // PA3
#define LEDB_GREEN  4   // PA4

// Keypad pins (Rows: PB2-PB5, Cols: PB6-PB9)
#define KEYPAD_ROW_START 2
#define KEYPAD_COL_START 6

const uint8_t digit_to_segment[] = {
    0x3F, 0x06, 0x5B, 0x4F,
    0x66, 0x6D, 0x7D, 0x07,
    0x7F, 0x6F
};
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t cycles = (SystemCoreClock / 1000000L) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

uint8_t Object_Detection(GPIO_TypeDef *trig_port, uint8_t trig_pin, GPIO_TypeDef *echo_port, uint8_t echo_pin) {
    uint32_t start = 0, end = 0, pulse;

    Trigger_Pulse(trig_port, trig_pin);

    // Wait for ECHO to go HIGH
    uint32_t timeout = DWT->CYCCNT;
    while (!(echo_port->IDR & (1 << echo_pin))) {
        if ((DWT->CYCCNT - timeout) > (SystemCoreClock / 1000 * 25)) return 0;
    }

    start = TIM2->CNT;

    // Wait for ECHO to go LOW
    timeout = DWT->CYCCNT;
    while ((echo_port->IDR & (1 << echo_pin))) {
        if ((DWT->CYCCNT - timeout) > (SystemCoreClock / 1000 * 25)) return 0;
    }

    end = TIM2->CNT;
    pulse = (end >= start) ? (end - start) : (0xFFFFFFFF - start + end);

    return (pulse < DETECTION_THRESHOLD);
}

void Trigger_Pulse(GPIO_TypeDef *GPIOx, uint8_t pin) {
    GPIOx->BSRR = (1 << pin);
    delay_us(10);
    GPIOx->BSRR = (1 << (pin + 16));
}

uint8_t Detect_Object(GPIO_TypeDef *trig_port, uint8_t trig_pin, GPIO_TypeDef *echo_port, uint8_t echo_pin) {
    uint32_t start = 0, end = 0, pulse;
    Trigger_Pulse(trig_port, trig_pin);
    uint32_t timeout = DWT->CYCCNT;
    while (!(echo_port->IDR & (1 << echo_pin))) {
        if ((DWT->CYCCNT - timeout) > (SystemCoreClock / 1000 * 25)) return 0;
    }
    start = TIM2->CNT;
    timeout = DWT->CYCCNT;
    while ((echo_port->IDR & (1 << echo_pin))) {
        if ((DWT->CYCCNT - timeout) > (SystemCoreClock / 1000 * 25)) return 0;
    }
    end = TIM2->CNT;
    pulse = (end >= start) ? (end - start) : (0xFFFFFFFF - start + end);
    return (pulse < DETECTION_THRESHOLD);
}


// Traffic system state
uint8_t traffic_system_active = 0;



void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

void delayms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 16000; i++);
}

void gpio_init(void) {
    // Enable clocks for all GPIO ports
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // === Ultrasonic Sensor Pins ===

    // TRIG1 (PC3), TRIG2 (PC8) as OUTPUT
    GPIOC->MODER &= ~((3 << (TRIG1_PIN * 2)) | (3 << (TRIG2_PIN * 2)));
    GPIOC->MODER |= ((1 << (TRIG1_PIN * 2)) | (1 << (TRIG2_PIN * 2)));

    // ECHO1 (PC6), ECHO2 (PC12) as INPUT with pull-down
    GPIOC->MODER &= ~((3 << (ECHO1_PIN * 2)) | (3 << (ECHO2_PIN * 2)));
    GPIOC->PUPDR &= ~((3 << (ECHO1_PIN * 2)) | (3 << (ECHO2_PIN * 2)));
    GPIOC->PUPDR |= ((2 << (ECHO1_PIN * 2)) | (2 << (ECHO2_PIN * 2)));


    // --- TM1637 A pins (PB10, PB11) ---
    int tm_pins_b[] = {10, 11};
    for (int i = 0; i < 2; i++) {
        int pin = tm_pins_b[i];
        GPIOB->MODER &= ~(3 << (pin * 2));
        GPIOB->MODER |= (1 << (pin * 2));
        GPIOB->OTYPER |= (1 << pin);
        GPIOB->ODR |= (1 << pin);
    }

    // --- TM1637 B pins (PC10, PC11) ---
    int tm_pins_c[] = {10, 11};
    for (int i = 0; i < 2; i++) {
        int pin = tm_pins_c[i];
        GPIOC->MODER &= ~(3 << (pin * 2));
        GPIOC->MODER |= (1 << (pin * 2));
        GPIOC->OTYPER |= (1 << pin);
        GPIOC->ODR |= (1 << pin);
    }

    // --- LED pins for Traffic Light A (PC0, PC1, PC2) ---
    for (int pin = 0; pin <= 2; pin++) {
        GPIOC->MODER &= ~(3 << (pin * 2));
        GPIOC->MODER |= (1 << (pin * 2));
        GPIOC->OTYPER &= ~(1 << pin);
    }

    // --- LED pins for Traffic Light B (PA2, PA3, PA4) ---
    for (int pin = 2; pin <= 4; pin++) {
        GPIOA->MODER &= ~(3 << (pin * 2));
        GPIOA->MODER |= (1 << (pin * 2));
        GPIOA->OTYPER &= ~(1 << pin);
    }

    // --- Keypad rows (PB2-PB5) ---
    for (int i = 2; i <= 5; i++) {
        GPIOB->MODER &= ~(3 << (i * 2));
        GPIOB->MODER |= (1 << (i * 2));
        GPIOB->ODR |= (1 << i);
    }

    // --- Keypad columns (PB6-PB9) ---
    for (int i = 6; i <= 9; i++) {
        GPIOB->MODER &= ~(3 << (i * 2));
        GPIOB->PUPDR &= ~(3 << (i * 2));
        GPIOB->PUPDR |= (1 << (i * 2));
    }
}

// Keypad Scanning
char scan_keypad(void) {
    for (int row = 0; row < 4; row++) {
        // Set all rows high
        GPIOB->ODR |= (0xF << 2);
        // Set current row low
        GPIOB->ODR &= ~(1 << (row + 2));
        delayms(5);

        for (int col = 0; col < 4; col++) {
            if ((GPIOB->IDR & (1 << (col + 6))) == 0) {
                while ((GPIOB->IDR & (1 << (col + 6))) == 0);
                delayms(50);
                return keypad_map[row][col];
            }
        }
    }
    return 0;
}

// TM1637 functions
void TM1637_start(GPIO_TypeDef* port, uint8_t clk, uint8_t dio) {
    port->ODR |= (1 << clk) | (1 << dio);
    delay_us(5);
    port->ODR &= ~(1 << dio);
    delay_us(5);
}

void TM1637_stop(GPIO_TypeDef* port, uint8_t clk, uint8_t dio) {
    port->ODR &= ~(1 << clk);
    delay_us(5);
    port->ODR &= ~(1 << dio);
    delay_us(5);
    port->ODR |= (1 << clk);
    delay_us(5);
    port->ODR |= (1 << dio);
    delay_us(5);
}

void TM1637_write_byte(GPIO_TypeDef* port, uint8_t clk, uint8_t dio, uint8_t b) {
    for (int i = 0; i < 8; i++) {
        port->ODR &= ~(1 << clk);
        delay_us(3);
        if (b & 0x01)
            port->ODR |= (1 << dio);
        else
            port->ODR &= ~(1 << dio);
        delay_us(3);
        port->ODR |= (1 << clk);
        delay_us(3);
        b >>= 1;
    }
    port->MODER &= ~(3 << (dio * 2));
    port->ODR &= ~(1 << clk);
    delay_us(5);
    port->ODR |= (1 << clk);
    delay_us(5);
    port->MODER |= (1 << (dio * 2));
}

void TM1637_display_digits(GPIO_TypeDef* port, uint8_t clk, uint8_t dio, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
    TM1637_start(port, clk, dio);
    TM1637_write_byte(port, clk, dio, 0x40);
    TM1637_stop(port, clk, dio);

    TM1637_start(port, clk, dio);
    TM1637_write_byte(port, clk, dio, 0xC0);
    TM1637_write_byte(port, clk, dio, digit_to_segment[d0]);
    TM1637_write_byte(port, clk, dio, digit_to_segment[d1]);
    TM1637_write_byte(port, clk, dio, digit_to_segment[d2]);
    TM1637_write_byte(port, clk, dio, digit_to_segment[d3]);
    TM1637_stop(port, clk, dio);

    TM1637_start(port, clk, dio);
    TM1637_write_byte(port, clk, dio, 0x88 | 0x07);
    TM1637_stop(port, clk, dio);
}

void display_seconds(GPIO_TypeDef* port, uint8_t clk, uint8_t dio, uint8_t seconds) {
    TM1637_display_digits(port, clk, dio, 0, 0, seconds / 10, seconds % 10);
}

// LED & Traffic Logic
void set_leds_A(uint8_t red, uint8_t yellow, uint8_t green) {
    if (red)
        GPIOC->ODR |= (1 << LEDA_RED);
    else
        GPIOC->ODR &= ~(1 << LEDA_RED);

    if (yellow)
        GPIOC->ODR |= (1 << LEDA_YELLOW);
    else
        GPIOC->ODR &= ~(1 << LEDA_YELLOW);

    if (green)
        GPIOC->ODR |= (1 << LEDA_GREEN);
    else
        GPIOC->ODR &= ~(1 << LEDA_GREEN);
}

void set_leds_B(uint8_t red, uint8_t yellow, uint8_t green) {
    if (red)
        GPIOA->ODR |= (1 << LEDB_RED);
    else
        GPIOA->ODR &= ~(1 << LEDB_RED);

    if (yellow)
        GPIOA->ODR |= (1 << LEDB_YELLOW);
    else
        GPIOA->ODR &= ~(1 << LEDB_YELLOW);

    if (green)
        GPIOA->ODR |= (1 << LEDB_GREEN);
    else
        GPIOA->ODR &= ~(1 << LEDB_GREEN);
}

uint8_t stateA = 0, timerA = 5;
uint8_t stateB = 2, timerB = 7;

void reset_traffic_states(void) {
    set_leds_A(1, 0, 0);
    set_leds_B(1, 0, 0);

    for (int i = 10; i > 0; i--) {
        display_seconds(GPIOB, TM1637_A_CLK_PIN, TM1637_A_DIO_PIN, i);
        display_seconds(GPIOC, TM1637_B_CLK_PIN, TM1637_B_DIO_PIN, i);
        delay_ms(150);
    }

    stateA = 0; timerA = 5;
    stateB = 2; timerB = 7;
}

// thoda theek hai
//
void integrate_ultrasonic_logic(void) {
    static uint8_t countA = 0, countB = 0;
    static uint8_t ultrasonic_granted_A = 0, ultrasonic_granted_B = 0;

    uint8_t us1 = Object_Detection(GPIOC, TRIG1_PIN, GPIOC, ECHO1_PIN);
    uint8_t us2 = Object_Detection(GPIOC, TRIG2_PIN, GPIOC, ECHO2_PIN);

    // Count if object detected continuously
    if (us1) countA++;
    else countA = 0;

    if (us2) countB++;
    else countB = 0;

    // LANE A - Give green only once when detected for threshold
    if (countA >= DETECTION_TIME_THRESHOLD && !ultrasonic_granted_A && stateA != 0) {
        // Ensure Lane B is not green now
        if (stateB != 0) {
            stateA = 0;
            timerA = 10;
            stateB = 2;
            timerB = 10;
            ultrasonic_granted_A = 1;
            ultrasonic_granted_B = 0;  // Reset B's trigger
        }
        countA = 0;
    }

    // LANE B - Give green only once when detected for threshold
    if (countB >= DETECTION_TIME_THRESHOLD && !ultrasonic_granted_B && stateB != 0) {
        if (stateA != 0) {
            stateB = 0;
            timerB = 10;
            stateA = 2;
            timerA = 10;
            ultrasonic_granted_B = 1;
            ultrasonic_granted_A = 0;  // Reset A's trigger
        }
        countB = 0;
    }

    // Reset flags when green phase finishes
    if (stateA != 0 && ultrasonic_granted_A) ultrasonic_granted_A = 0;
    if (stateB != 0 && ultrasonic_granted_B) ultrasonic_granted_B = 0;
}



void update_trafficA(void) {
    if (stateA == 0) {
        set_leds_A(0, 0, 1);
        display_seconds(GPIOB, TM1637_A_CLK_PIN, TM1637_A_DIO_PIN, timerA);
        if (--timerA == 0) { stateA = 1; timerA = 2; }
    } else if (stateA == 1) {
        set_leds_A(0, 1, 0);
        display_seconds(GPIOB, TM1637_A_CLK_PIN, TM1637_A_DIO_PIN, timerA);
        if (--timerA == 0) { stateA = 2; timerA = 7; }
    } else {
        set_leds_A(1, 0, 0);
        display_seconds(GPIOB, TM1637_A_CLK_PIN, TM1637_A_DIO_PIN, timerA);
        if (--timerA == 0) { stateA = 0; timerA = 5; }
    }
}

void update_trafficB(void) {
    if (stateB == 0) {
        set_leds_B(0, 0, 1);
        display_seconds(GPIOC, TM1637_B_CLK_PIN, TM1637_B_DIO_PIN, timerB);
        if (--timerB == 0) { stateB = 1; timerB = 2; }
    } else if (stateB == 1) {
        set_leds_B(0, 1, 0);
        display_seconds(GPIOC, TM1637_B_CLK_PIN, TM1637_B_DIO_PIN, timerB);
        if (--timerB == 0) { stateB = 2; timerB = 7; }
    } else {
        set_leds_B(1, 0, 0);
        display_seconds(GPIOC, TM1637_B_CLK_PIN, TM1637_B_DIO_PIN, timerB);
        if (--timerB == 0) { stateB = 0; timerB = 5; }
    }
}

// MAIN
int main(void) {
    gpio_init();
    LcdInit();

    lcd_print(0x80, "Welcome");
    DWT_Init();  // for delay_us()
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 16 - 1;  // 1 µs if HSI 16 MHz
    TIM2->ARR = 0xFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;



    while (1) {
        char key = scan_keypad();

        if (key == '*') {
            lcd_print(0x80, "Input Passkey:");

            int index = 0;

            while (1) {
                key = scan_keypad();

                if (key) {
                    if (key == '#') {
                        entered[index] = '\0';

                        if (strcmp(entered, passkey) == 0) {
                            lcd_print(0xC0, "Access Granted ");
                            delay_ms(1000);
                            traffic_system_active = 1;
                            break;
                        } else {
                            lcd_print(0xC0, "Wrong Passkey!");
                            delayms(1500);
                            lcd_print(0xC0, "Try Again!     ");
                            index = 0;
                        }
                    }
                    else if (index < 4 && key != '*' && key != '#') {
                        entered[index++] = key;
                        char buf[17] = "Entered: ";
                        for (int i = 0; i < index; i++) buf[9 + i] = '*';
                        buf[9 + index] = '\0';
                        lcd_print(0xC0, buf);
                    }
                }
            }
            break;
        }
    }

    if (traffic_system_active) {
        lcd_print(0x80, "Traffic System  ");
        lcd_print(0xC0, "Active          ");
        reset_traffic_states();

        while (1) {
            integrate_ultrasonic_logic();  // add before updates
            update_trafficA();
            update_trafficB();
            delay_ms(1000);
        }

    }

    return 0;
}
