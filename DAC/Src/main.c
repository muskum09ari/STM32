#include <stdint.h>
#include <STM32f405xx.h>

// for live expression use spiderman->then switch>live expression>resume>terminate

uint32_t analog;
int i;
const static int SineWave[]={2048,3071,3821,4095,3821,3071,2048,1024,274,0,274,1024};

void delayms(uint32_t delay)
{
	uint32_t i,j=0;
	for(i=0;i<delay;i++)
		for(j=0;j<16000;j++);
}

int main(void)
{
	RCC->AHB1ENR |=(1<<0);// enable gpioA
	GPIOA->MODER |=(1<<8);//pg 281 for settig analog mode
	GPIOA->MODER |=(1<<9); //MODER 4 ENABLE TO ANALOG MODE as pa4 dac
	RCC->APB1ENR |=(1<<29);//pg 183 RCC DAC 29th bit set for dac enable
	DAC->CR |=(1<<0);// enable dac channel
while (1)
    {
        for (i = 0; i < sizeof(SineWave) / sizeof(int); i++)
        {
            DAC->DHR12R1 = SineWave[i]; /* Write value of SineWave to DAC */
            analog = DAC->DHR12R1;      // Check variable in Debug mode
            delayms(20);
        }
    }
}


