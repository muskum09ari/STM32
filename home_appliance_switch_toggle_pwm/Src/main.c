//led-1 - fan
//led-2 - bulb
//led-3 - refrigerator
#include <stdint.h>
#include <stm32f405xx.h>

//Formulae used
/* Duty cycle=(ADC_Val/4096) *100
* F-sys=16MHz
* PSC=8
* F-timer=F-sys/PSC
* PWM-Freq=4Khz(by default)
* ARR=F-timer/PWM-Freq=500
* Duty cycle=(CCR1/ARR) *100
*/
uint32_t adc_read(void);
void adc_init(void);
void pwm_enable(uint32_t);
uint32_t var,val,duty_cycle;
int main(void)
{
    /* Loop forever */
	    //for fan(using adc and pwm)
	    RCC->AHB1ENR|=(1<<2); //enable clock for led-1(pc-6)
		GPIOC->MODER&=~(1<<12); //enable gpioc af-mode
		GPIOC->MODER|=(1<<13);
		GPIOC->AFR[0]&=~(1<<24);  //enable afrl for gpioc tim-3 af-2
		GPIOC->AFR[0]|=(1<<25);
		GPIOC->AFR[0]&=~(1<<26);
		GPIOC->AFR[0]&=~(1<<27);
		RCC->APB1ENR|=(1<<1); //enable clock for timer-3
		//initialize tim-3 psc,arr,cnt values
		TIM3->CNT=0;
		TIM3->PSC=8-1;
		TIM3->ARR=500-1;
		//set bits 4,5,6 in ccmr1 as 1,1,0
		TIM3->CCMR1|=(1<<4);
		TIM3->CCMR1|=(1<<5);
		TIM3->CCMR1&=~(1<<6);
		TIM3->CCMR1|=(1<<3);

		//enable ccer and cr1 bits
		TIM3->CCER|=(1<<0);
		TIM3->CR1|=(1<<0);
		adc_init();
		uint32_t current=0;
		uint32_t last=1;
		uint32_t count=0;
		while(1)
		{
			var=adc_read();
			duty_cycle=(int)((var*100)/4096); //calculating duty cycle with respect to adc-value which is read
			val=(duty_cycle*TIM3->ARR) / 100; //calculating ccr1 value
			current=(GPIOB->IDR>>7) & 1;
			if(current!=last)
			{
				current=(GPIOB->IDR>>7) & 1;
				if(current==0)
				{
					count++;
					if(count>5)
					{
						count=0;
					}
				}
				last=current;
			}
			if(count>0)
			{
				switch(count)
				{
				   case 1: pwm_enable(val);   //changing duty cycle and observe pwm
					       for(unsigned int i=0;i<5000;i++);
					       break;

				   case 2: pwm_enable(1.2*val);
					       for(unsigned int i=0;i<5000;i++);
					       break;

				   case 3: pwm_enable(0.6*val);
					       for(unsigned int i=0;i<5000;i++);
					       break;

				   case 4: pwm_enable(0.3*val);
				           for(unsigned int i=0;i<5000;i++);
				           break;

				   case 5: pwm_enable(0.1*val);
				           for(unsigned int i=0;i<5000;i++);
				           break;
				}
			}
			else
			{
				pwm_enable(val);
				for(unsigned int i=0;i<5000;i++);
			}
		    //for bulb(using interupt-external)-switch-2 led-2
		    RCC->AHB1ENR|=(1<<1);  //enable clock for gpiob enr
		    GPIOB->MODER|=(1<<26);  //make pb-13 as output gpiob moder
		    GPIOB->MODER&=~(1<<27);
		    GPIOB->MODER&=~(3<<6); //making pb-3 switch as input
		    RCC->APB2ENR|=(1<<14); //make bit 14 as 1
		    SYSCFG->EXTICR[0]|=(1<<12);
		    SYSCFG->EXTICR[0]&=~(1<<13);
		    SYSCFG->EXTICR[0]&=~(1<<14);
		    SYSCFG->EXTICR[0]&=~(1<<15);
		    EXTI->FTSR|=(1<<3);
		    EXTI->IMR|=(1<<3);

		    NVIC_EnableIRQ(EXTI3_IRQn);	// EXTI interrupt enable

		    //for refrigator(using external-interrupt)-switch-3 led-3
		    RCC->AHB1ENR|=(1<<1);   //enable clock for gpiob enr
		    GPIOB->MODER|=(1<<28);  //make pb-14 as output gpiob moder
		    GPIOB->MODER&=~(1<<29);
		    GPIOB->MODER&=~(3<<8);  //making pb-4 switch as input
		    RCC->APB2ENR|=(1<<14);  //make bit 14 as 1
		    SYSCFG->EXTICR[1]|=(1<<0);
		    SYSCFG->EXTICR[1]&=~(1<<1);
		    SYSCFG->EXTICR[1]&=~(1<<2);
		    SYSCFG->EXTICR[1]&=~(1<<3);
		    EXTI->FTSR|=(1<<4);
		    EXTI->IMR|=(1<<4);
		    NVIC_EnableIRQ(EXTI4_IRQn);	// EXTI interrupt enable*/
		}
	//for(;;);
}

void EXTI3_IRQHandler(void)  //switch-2 led-2 for bulb
{
	if(EXTI->PR&(1<<3))
	{
		GPIOB->ODR^=(1<<13);
		EXTI->PR|=(1<<3);
	}
}

void EXTI4_IRQHandler(void)  //switch-3 led-3 for refrigator
{
	if(EXTI->PR&(1<<4))
	{
		GPIOB->ODR^=(1<<14);
		EXTI->PR|=(1<<4);
	}
}

void adc_init()
{
	RCC->AHB1ENR|=(1<<2);  //Enable clock for port c
	GPIOC->MODER|=(1<<2);  //enable gpioc moder for pc-1
	GPIOC->MODER|=(1<<3);
	RCC->APB2ENR|=(1<<8); //enable clock for adc-1
	ADC1->CR2=0; //make cr2 of adc-1 as 0
	ADC1->SQR1&=~(1<<20);
	ADC1->SQR1&=~(1<<21);
	ADC1->SQR1&=~(1<<22);
	ADC1->SQR1&=~(1<<23);

	ADC1->SQR3|=(1<<0);    //make the bits 4,3,2,1,0 as B(hex) since channel-no. 11 is used with binary value 01011
	ADC1->SQR3|=(1<<1);
	ADC1->SQR3&=~(1<<2);
	ADC1->SQR3|=(1<<3);
	ADC1->SQR3&=~(1<<4);

	ADC1->CR1|=(1<<8);

	ADC1->CR1|=(1<<0);    //make the bits 4,3,2,1,0 as B(hex) since channel-no. 11 is used with binary value 01011
	ADC1->CR1|=(1<<1);
	ADC1->CR1&=~(1<<2);
	ADC1->CR1|=(1<<3);
	ADC1->CR1&=~(1<<4);

	ADC1->CR2|=(1<<0);
}
uint32_t adc_read()      //for controlling fan
{
	ADC1->CR2|=(1<<30);
	while(!(ADC1->SR&(1<<1)));
	return ADC1->DR;
}
void pwm_enable(uint32_t val)  //for controlling fan
{
	TIM3->CR1&=~(1<<0);
	TIM3->CCR1=val;
	TIM3->CR1|=(1<<0);
}
