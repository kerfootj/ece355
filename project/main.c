//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_spi.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescalers: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
#define myTIM3_PRESCALER ((unit16_t)0xFFFF)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

/* Prototypes */
void myGPIOA_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI_Init(void);
void writeToHC595(char data);
void writeToLCD(char data, int cmd_flag);
void writeStringToLCD(char* data);

/* Global Variables */
int debug = 0;
int global_frequency = 0;
int global_resistance = 0;

int main(int argc, char* argv[])
{
	trace_printf("This is the final project for ECE 355\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myTIM3_Init();		/* Initialize timer TIM3 */
	myEXTI_Init();		/* Initialize EXTI */

	while (1)
	{
		// Nothing is going on here...
	}

	return 0;

}


void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA0 as input */
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);

	/* Ensure no pull-up/pull-down for PA0 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);
}


void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	TIM2->DIER |= TIM_DIER_UIE;
}


// Refresh Display Timer
void myTIM3_Init()
{
	/* Enable clock for TIM3 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	/* Configure TIM3: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM3->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM3->PSC = myTIM2_PRESCALER;

	/* Update timer registers */
	TIM3->EGR = ((uint16_t)0x0001);

	/* Assign TIM interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM3_IRQn, 0);

	/* Enable TIM3 interrupts in NVIC */
	NVIC_EnableIRQ(TIM3_IRQn);

	/* Enable update interrupt generation */
	TIM3->DIER |= TIM_DIER_UIE;

	/* Start the timer*/
	TIM3->CR1 = TIM_CR1_CEN;
}


void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		TIM2->CR1 = TIM_CR1_CEN;
	}
}


void TIM3_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM3->SR & TIM_SR_UIF) != 0)
	{
		// Create resistance and frequency strings
		char freq[9];
		char rest[9];

		// Store char array in strings
		sprintf(freq, "F:%4dHz", global_frequency);
		sprintf(rest, "R:%4d%c", global_resistance, 0xF4);

		// Move Cursor to start of line for write
		writeToLCD(0x80, 1);

		// write frequency value

		// write resistance value

		/* Clear update interrupt flag */
		TIM3->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		TIM3->CR1 = TIM_CR1_CEN;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
//		if (first_edge) {
//			first_edge = 0;
		if (!(TIM2->CR1 & TIM_CR1_CEN)) {

			/* Reset current timer count */
			TIM2->CNT = (uint32_t)0x0;

			/* Start the timer */
			TIM2->CR1 = TIM_CR1_CEN;

		} else {

			/* Stop the timer */
			// TIM2->CR1 &= ~(TIM_CR1_CEN);
			TIM2->CR1 = (uint16_t)0x0;

			/* Read current timer value*/
			uint32_t count = TIM2->CNT;

			/* Calculate signal frequency and period */
			float freq = ((float)SystemCoreClock) / count;
			float period = 1000.0 / freq;

			if(debug)
				trace_printf("Count: %d\n", (int)count);

			/* Prints low due to integer truncation */
			trace_printf("Signal Frequency: %.0f Hz\n", freq);
			trace_printf("Signal Period: %.2f ms\n\n", period);

			//first_edge = 1;

		}

		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR |= EXTI_PR_PR1;

	}
}


void writeToHC595(char data)
{
	// Force LCK clock to zero
	GPIOB->BRR = GPIO_Pin_4;

	// Wait until SPI is ready
	while((SPI1->SR & SPI_SR_BSY) != 0);

	// Send 8 bits of data
	SPI_SendData8(SPI1, data);

	// Wait until SPI is not busy
	while((SPI1->SR & SPI_SR_BSY) != 0);

	// Set LCK clock to zero
	GPIOB->BSRR = GPIO_Pin_4;
}



void writeToLCD(char data, int cmdFlag)
{
	char RS = cmdFlag ? 0x00 : 0x40;
	char EN = 0x80;

	char upper = (data & 0xF0) >> 4;
	char lower = data & 0xF;

	writeToHC595(upper | RS);
	writeToHC595(upper | RS | EN);
	writeToHC595(upper | RS);
	writeToHC595(lower | RS);
	writeToHC595(lower | RS | EN);
	writeToHC595(lower | RS);
}


void writeStringToLCD(char* data)
{
	int n;
	// Write one char at a time
	for(n = 0; n < strlen(data); ++n)
	{
		writeToLCD(data[n], 0);
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
