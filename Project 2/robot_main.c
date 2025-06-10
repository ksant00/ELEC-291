#include "../Common/Include/stm32l051xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../Common/Include/serial.h"
#include "UART2.h"

#define F_CPU 32000000L
#define SYSCLK 32000000L
#define DEF_F 100000L // 10us tick
#define TICK_FREQ 1000L

volatile int PWM_Counter = 0;
unsigned int pwm_Lp = 0, pwm_Ln = 0, pwm_Rp = 0, pwm_Rn = 0;
unsigned int negative_input_1 = 0, negative_input_2=0, pwm_left,pwm_right;
volatile int sec_count = 0;
volatile int turn_count = 0;
volatile int turning = 0;
unsigned int distance = 4;
volatile int seconds = 0;

// LQFP32 pinout
//             ----------
//       VDD -|1       32|- VSS
//      PC14 -|2       31|- BOOT0
//      PC15 -|3       30|- PB7
//      NRST -|4       29|- PB6
//      VDDA -|5       28|- PB5
//       PA0 -|6       27|- PB4
//       PA1 -|7       26|- PB3
//       PA2 -|8       25|- PA15 (Used for RXD of UART2, connects to TXD of JDY40)
//       PA3 -|9       24|- PA14 (Used for TXD of UART2, connects to RXD of JDY40)
//       PA4 -|10      23|- PA13 (Used for SET of JDY40)
//       PA5 -|11      22|- PA12
//       PA6 -|12      21|- PA11
//       PA7 -|13      20|- PA10 (Reserved for RXD of UART1)
//       PB0 -|14      19|- PA9  (Reserved for TXD of UART1)
//       PB1 -|15      18|- PA8  (pushbutton)
//       VSS -|16      17|- VDD
//             ----------

// Uses SysTick to delay <us> micro-seconds. 
void Delay_us(unsigned char us)
{
	// For SysTick info check the STM32L0xxx Cortex-M0 programming manual page 85.
	SysTick->LOAD = (F_CPU/(1000000L/us)) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Delay_us(250);
}

void wait_1ms(void)
{
	// For SysTick info check the STM32l0xxx Cortex-M0 programming manual.
	SysTick->LOAD = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void delayms(int len)
{
	while(len--) wait_1ms();
}

void Hardware_Init(void)
{
	GPIOA->OSPEEDR=0xffffffff; // All pins of port A configured for very high speed! Page 201 of RM0451

	// Set up output pins
	RCC->IOPENR |= 0x00000001; // peripheral clock enable for port A
    GPIOA->MODER = (GPIOA->MODER & ~(BIT2|BIT3)) | BIT2; // Make pin PA1 output 
	GPIOA->OTYPER &= ~BIT1; // Push-pull
	
    GPIOA->MODER = (GPIOA->MODER & ~(BIT4|BIT5)) | BIT4; // Make pin PA2 output 
	GPIOA->OTYPER &= ~BIT2; // Push-pull
	
	GPIOA->MODER = (GPIOA->MODER & ~(BIT6|BIT7)) | BIT6; // Make pin PA3 output 
	GPIOA->OTYPER &= ~BIT3; // Push-pull
	
    GPIOA->MODER = (GPIOA->MODER & ~(BIT8|BIT9)) | BIT8; // Make pin PA4 output 
	GPIOA->OTYPER &= ~BIT4; // Push-pull

	/////////////////// radio
	GPIOA->MODER = (GPIOA->MODER & ~(BIT27|BIT26)) | BIT26; // Make pin PA13 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.

	GPIOA->MODER &= ~(BIT16 | BIT17); // Make pin PA8 input
	// Activate pull up for pin PA8:
	GPIOA->PUPDR |= BIT16; 
	GPIOA->PUPDR &= ~(BIT17);


	// Set up timer
	RCC->APB1ENR |= BIT0;  // turn on clock for timer2 (UM: page 177)
	TIM2->ARR = SYSCLK/TICK_FREQ;
	NVIC->ISER[0] |= BIT15; // enable timer 2 interrupts in the NVIC
	TIM2->CR1 |= BIT4;      // Downcounting    
	TIM2->CR1 |= BIT7;      // ARPE enable   
	TIM2->DIER |= BIT0;     // enable update event (reload event) interrupt 
	TIM2->CR1 |= BIT0;      // enable counting
	RCC->APB1ENR &= ~BIT0;  // turn on clock for timer2 (UM: page 177)    


	__enable_irq();
}

#define PIN_PERIOD (GPIOA->IDR&BIT8)

// GetPeriod() seems to work fine for frequencies between 300Hz and 600kHz.
// 'n' is used to measure the time of 'n' periods; this increases accuracy.
long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(SysTick->CTRL & BIT16) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(SysTick->CTRL & BIT16) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter
	
	SysTick->LOAD = 0xffffff;  // 24-bit counter reset
	SysTick->VAL = 0xffffff; // load the SysTick counter to initial value
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(SysTick->CTRL & BIT16) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(SysTick->CTRL & BIT16) return 0;
		}
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	return 0xffffff-SysTick->VAL;
}

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	GPIOA->ODR &= ~(BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2(s);
	egets2(buff, sizeof(buff)-1);
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.
	waitms(10);
	printf("Response: %s", buff);
}

void timer21_setup(void)
{

	RCC->APB2ENR |= BIT2;			// turn on clock for timer21
	TIM21->ARR = F_CPU / DEF_F - 1; // auto reload value
	NVIC->ISER[0] |= BIT20;			// enable timer 21

	TIM21->CR1 |= BIT4;	 // Downcounting
	TIM21->CR1 |= BIT0;	 // enable counting
	TIM21->CR1 |= BIT7;	 // ARPE enable
	TIM21->DIER |= BIT0; // enable update event (reload event) interrupt
	__enable_irq();
}

void TIM21_Handler(void)
{
	TIM21->SR &= ~BIT0; // clear update interrupt flag

	PWM_Counter++;

	if (negative_input_1 == 0)
	{
		if (pwm_left > PWM_Counter)
		{
			GPIOA->ODR |= BIT1;	 // PA1 as high
			GPIOA->ODR &= ~BIT2; // PA2 as low
		}
		else
		{
			GPIOA->ODR &= ~BIT1;
			GPIOA->ODR &= ~BIT2;
		}
	}
	else
	{
		if (pwm_left > PWM_Counter)
		{
			GPIOA->ODR |= BIT2;	 // PA2 as high
			GPIOA->ODR &= ~BIT1; // PA1 as low
		}
		else
		{
			GPIOA->ODR &= ~BIT1;
			GPIOA->ODR &= ~BIT2;
		}
	}
	if (PWM_Counter > 100) // THe period is 20ms
	{
		PWM_Counter = 0;
		GPIOA->ODR |= (BIT1 | BIT2);
	}
}

void timer22_setup(void)
{

	RCC->APB2ENR |= BIT5;			// turn on clock for timer22
	TIM22->ARR = F_CPU / DEF_F - 1; // auto reload value
	NVIC->ISER[0] |= BIT22;			// enable timer 22

	TIM22->CR1 |= BIT4;	 // Downcounting
	TIM22->CR1 |= BIT0;	 // enable counting
	TIM22->CR1 |= BIT7;	 // ARPE enable
	TIM22->DIER |= BIT0; // enable update event (reload event) interrupt
	__enable_irq();
}

void TIM22_Handler(void) 
{
	TIM22->SR &= ~BIT0; // clear update interrupt flag

	PWM_Counter++;

	if (negative_input_2 != 0)
	{
		if (pwm_right > PWM_Counter)
		{
			GPIOA->ODR |= BIT3;	 // PA1 as high
			GPIOA->ODR &= ~BIT4; // PA2 as low
		}
		else
		{
			GPIOA->ODR &= ~BIT3;
			GPIOA->ODR &= ~BIT4;
		}
	}
	else
	{
		if (pwm_right > PWM_Counter)
		{
			GPIOA->ODR |= BIT4;	 // PA2 as high
			GPIOA->ODR &= ~BIT3; // PA1 as low
		}
		else
		{
			GPIOA->ODR &= ~BIT3;
			GPIOA->ODR &= ~BIT4;
		}
	}
	if (PWM_Counter > 100) // THe period is 20ms
	{
		PWM_Counter = 0;
		GPIOA->ODR |= (BIT3 | BIT4);
	}
}

void TIM2_Handler(void)
{
	TIM2->SR &= ~BIT0; // clear update interrupt flag
	sec_count++;

	if (sec_count > 1000)
	{
		sec_count = 0;
		seconds++;
	}

	if ((seconds < distance) && (turning == 0))
	{
		pwm_left = 100;
		pwm_right = 100;
	}
	else if ((seconds >= distance) && (turning == 0))
	{
		turn_count++;
		turning = 1;
	}
	else;

	if ((turn_count == 1) && (turning == 1) && (seconds < 4 + distance))
	{
		pwm_left = 55;
		pwm_right = 0;
	}
	else if ((turn_count == 1) && (turning == 1) && (seconds >= 4 + distance))
	{
		turning = 0;
		seconds = 0;
	}
	else;

	if ((turn_count == 2) && (turning == 1) && (seconds < 4 + distance))
	{
		pwm_left = 0;
		pwm_right = 54;
	}
	else if ((turn_count == 2) && (turning == 1) && (seconds >= 4 + distance))
	{
		turning = 0;
		turn_count = 0;
		seconds = 0;
	}
	else;
}

int main(void)
{
	char robot[80];
	char remote[80];
	char c;
	unsigned int timeout;
	
	// Period VARs
	unsigned long int count;
	float T, f;

	Hardware_Init();
	initUART2(9600);

	timer21_setup();
	timer22_setup();
	
	waitms(1000); // Give putty some time to start.
	printf("\r\nJDY-40 test\r\n");

	
	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVID3F4D\r\n");  

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");
	
	printf("\r\nPress and hold a push-button attached to PA8 (pin 18) to transmit.\r\n");
	
	
	while (1)
	{
		
		if (ReceivedBytes2() > 0) // Something has arrived
		{

			egets2(remote, sizeof(remote) - 1);
			if (strlen(remote) == 11)
			{
				if (atoi(&remote[9]) == 0)
				{
					RCC->APB1ENR &= ~BIT0;  // turn off clock for timer2 
					if (atoi(remote) >= 0)
					{
						pwm_Lp = atoi(remote);
						pwm_Ln = 0;
					}
					else
					{
						pwm_Ln = atoi(&remote[1]);
						pwm_Lp = 0;
					}

					if (pwm_Lp == 0)
					{
						negative_input_1 = 1;
						pwm_left = pwm_Ln;
					}
					else
					{
						negative_input_1 = 0;
						pwm_left = pwm_Lp;
					}

					if (atoi(&remote[4]) >= 0)
					{
						pwm_Rp = atoi(&remote[4]);
						pwm_Rn = 0;
					}
					else
					{
						pwm_Rn = atoi(&remote[5]);
						pwm_Rp = 0;
					}

					if (pwm_Rp == 0)
					{
						negative_input_2 = 1;
						pwm_right = pwm_Rn;
					}
					else
					{
						negative_input_2 = 0;
						pwm_right = pwm_Rp;
					}
				}
				else
				{
					if (atoi(&remote[9]) == 1)
					negative_input_1 = 0;
					negative_input_2 = 0;
					distance = atoi(remote);
					RCC->APB1ENR |= BIT0;  // turn on clock for timer2
				}
			}

			__disable_irq();
			count = GetPeriod(150);
			__enable_irq();

			if (count > 0)
			{
				T = count / (4800000000.0); // Since we have the time of 100 periods, we need to divide by 100

				sprintf(robot, "%.2f\n", T*1000000000);
				eputs2(robot);
			}
		}
	}
}