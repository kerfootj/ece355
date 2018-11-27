// ----------------------------------------------------------------------------
// School: 	University of Victoria, Canada.
// Course: 	ECE 355 "Microprocessor-Based Systems"
//
// Author: 	Joel Kerfoot
// Date:	November 25, 2018
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations
// ----------------------------------------------------------------------------


/******************************************************************************
 * 									LIBRARIES								  *
 ******************************************************************************/

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_spi.h"


/******************************************************************************
 * 									PRAGMATICS								  *
 ******************************************************************************/

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/******************************************************************************
 * 									  DEFINES								  *
 ******************************************************************************/

/* Clock prescalers: TIM2, TIM3, TIM6 */
#define TIM2_PRESCALER ((uint16_t)0x0000)
#define TIM3_PRESCALER ((uint16_t)0xFFFF)
#define TIM6_PRESCALER ((uint16_t)4)

/* Maximum possible setting for overflow */
#define TIM2_PERIOD ((uint32_t)0xFFFFFFFF)

/* ADC and DAC*/
#define MAX_ADC_VALUE 4095 	// 12 bit ADC holds 0xFFF = 4095
#define MAX_DAC_VALUE 4095 	// 12 bit DAC holds 0xFFF = 4095
#define MAX_POT_VALUE 5000 	// 5k potentiometer
#define MAX_VOLTAGE (2.91) 	// Measured when DAC->DHR12R1 = DAC_MAX_VALUE
#define DIODE_DROP (1.0) 	// Voltage needed to overcome diode voltage


/******************************************************************************
 * 									 PROTOTYPES							  	  *
 ******************************************************************************/

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myTIM6_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void mySPI_Init(void);
void LCD_Init(void);
void LCD_4_Bit_mode(void);
void writeToHC595(char data);
void writeToLCD(char data, int cmd_flag);
void writeStringToLCD(char* data);
void wait(int ms);
float getPOTValue(void);
uint16_t offsetDAC(float value);


/******************************************************************************
 * 								   GLOBAL VARIABLES							  *
 ******************************************************************************/

int debug = 0;
int testFlag = 0;
int global_frequency = 0;
int global_resistance = 0;


/******************************************************************************
 * 								     IMPLEMENTATION						  	  *
 ******************************************************************************/

int main(int argc, char* argv[])
{
	trace_printf("This is the final project for ECE 355\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myTIM3_Init();		/* Initialize timer TIM3 */
	myTIM6_Init();		/* Initialize timer TIM6 */
	myEXTI_Init();		/* Initialize EXTI */
	myDAC_Init();		/* Initialize DAC */
	myADC_Init();		/* Initialize ADC */
	LCD_Init();			/* Initialize LCD */

	while (1)
	{
		float resistance_value = getPOTValue();
		global_resistance = (resistance_value/MAX_ADC_VALUE) * MAX_POT_VALUE;
		uint32_t control = offsetDAC(resistance_value);
		DAC->DHR12R1 = control;
	}
	return 0;
}

/*
 * Configures Port A to the following:
 * 		PA0: Input for ADC from POT
 * 		PA1: Detect edge transitions from NE555
 * 		PA4: Output DAC for timer control voltage
 */
void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA0 */
	GPIOA->MODER &= ~(GPIO_MODER_MODER0); // Input
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0); // Ensure no pull-up/pull-down

	/* Configure PA1 */
	GPIOA->MODER &= ~(GPIO_MODER_MODER1); // Input
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1); // Ensure no pull-up/pull-down

	/* Configure PA4 */
	GPIOA->MODER &= ~(GPIO_MODER_MODER4); // Output
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4); // Ensure no pull-up/pull-down
}


void myGPIOB_Init()
{
	/* Enable clock for GPIOB peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	GPIO_InitTypeDef GPIO_InitStruct;

	/* Configure MOSI and SCK pins PB3, PB5 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configure LCK pin PB4 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOB, &GPIO_InitStruct);
}


/* Frequency Timer */
void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = TIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = TIM2_PERIOD;

	/* Update timer registers */
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority 0 in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	TIM2->DIER |= TIM_DIER_UIE;
}


/* Refresh Display Timer */
void myTIM3_Init()
{
	/* Enable clock for TIM3 peripheral */
	RCC->APB1ENR |= RCC_APB1RSTR_TIM3RST;

	/* Configure TIM3: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on underflow only */
	TIM3->CR1 &= 0xFFFD;
	TIM3->CR1 |= 0x9D;

	/* Set clock prescaler value */
	TIM3->PSC = TIM3_PRESCALER;

	/* Set auto-reload delay*/
	TIM3->ARR = TIM3_PRESCALER;

	/* Set timer count*/
	TIM3->CNT = SystemCoreClock / TIM3_PRESCALER;

	/* Update timer registers */
	TIM3->EGR |= 0x1;

	/* Assign TIM interrupt priority 0 in NVIC */
	NVIC_SetPriority(TIM3_IRQn, 0);

	/* Enable TIM3 interrupts in NVIC */
	NVIC_EnableIRQ(TIM3_IRQn);

	/* Enable update interrupt generation */
	TIM3->DIER |= TIM_DIER_UIE;

	/* Start the timer*/
	TIM3->CR1 = TIM_CR1_CEN;
}


/* Wait timer */
void myTIM6_Init()
{
	/* Enable clock for TIM6 peripheral */
	RCC->APB1ENR |= RCC_APB1RSTR_TIM6RST;

	/* Configure TIM6: buffer auto-reload */
	TIM6->CR1 |= 0x84;

	/* Set clock prescaler value */
	TIM6->PSC = TIM6_PRESCALER;
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


void myDAC_Init()
{
	/* Enable DAC clock */
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	/* Enable DAC */
	DAC->CR |= DAC_CR_EN1;
}


void myADC_Init()
{
	/* Enable ADC clock */
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	/* Let the ADC self calibrate */
	ADC1->CR |= ADC_CR_ADCAL;

	/* Wait for calibration */
	while(ADC1->CR == ADC_CR_ADCAL);

	/*Configure ADC: continuous conversion, overrun mode */
	ADC1->CFGR1 |= (ADC_CFGR1_OVRMOD | ADC_CFGR1_CONT);

	/* Set ADC to channel 0 for PA0 */
	ADC1->CHSELR |= ADC_CHSELR_CHSEL0;

	/* Enable ADC */
	ADC1->CR |= ADC_CR_ADEN;

	/* Wait for stability */
	while(!(ADC1->ISR & ADC_ISR_ADRDY));
}


void LCD_Init ()
{
	/*Setup Port B and SPI*/
	myGPIOB_Init();
	mySPI_Init();

	/* 4 Bit Mode */
	LCD_4_Bit_mode();

	/* Display 2 Lines of 8 characters */
	writeToLCD(0x28, 1);
	wait(37);

	/* Disable Cursor */
	writeToLCD(0x0C, 1);
	wait(37);

	/* Auto-increment address after access */
	writeToLCD(0x06, 1);
	wait(37);

	/* Clear Display */
	writeToLCD(0x01, 1);
	wait(1550);
}


void mySPI_Init ()
{
	/* Enable SPI clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* Configure Pins */
	GPIO_PinAFConfig(GPIOB, 3, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOB, 5, GPIO_AF_0);

	/* Configure SPI1 */
	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);
}


void LCD_4_Bit_mode()
{
	/* LCD needs delay to display properly */
	writeToHC595(0x3);
	wait(1520);
	writeToHC595(0x3 | 0x80);
	wait(1520);
	writeToHC595(0x3);
	wait(1520);

	writeToHC595(0x3);
	wait(1520);
	writeToHC595(0x3 | 0x80);
	wait(1520);
	writeToHC595(0x3);
	wait(1520);

	writeToHC595(0x3);
	wait(1520);
	writeToHC595(0x3 | 0x80);
	wait(1520);
	writeToHC595(0x3);
	wait(1520);

	writeToHC595(0x2);
	wait(1520);
	writeToHC595(0x2 | 0x80);
	wait(1520);
	writeToHC595(0x2);
	wait(1520);


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
		char frequency[9];
		char resistance[9];

		// Store char array in strings
		sprintf(frequency, "F:%4dHz", global_frequency);
		sprintf(resistance, "R:%4d%c", global_resistance, 0xF4);

		trace_printf("%s\n%s", frequency, resistance);

		// Move cursor to start of line for write
		writeToLCD(0x80, 1);

		// Write frequency value
		writeStringToLCD(frequency);

		// Move cursor to second line
		writeToLCD(0xC0, 1);

		// Write resistance value
		writeStringToLCD(resistance);

		/* Clear update interrupt flag */
		TIM3->SR &= ~(TIM_SR_UIF);

		/* Set clock count*/
		TIM3->SR &= SystemCoreClock / TIM3_PRESCALER;

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
		/* Timer is Disabled */
		if (!(TIM2->CR1 & TIM_CR1_CEN)) {

			/* Reset current timer count */
			TIM2->CNT = (uint32_t)0x0;

			/* Start the timer */
			TIM2->CR1 = TIM_CR1_CEN;

		/* Timer is Enabled*/
		} else {

			/* Stop the timer */
			TIM2->CR1 &= ~(TIM_CR1_CEN);

			/* Read current timer value*/
			uint32_t count = TIM2->CNT;

			/* Calculate signal frequency and period */
			global_frequency = ((float)SystemCoreClock) / count;

			if(debug)
			{
				trace_printf("Signal Frequency: %.0f Hz\n", global_frequency);
				trace_printf("Signal Period: %.2f ms\n\n", global_resistance);
			}
		}

		/* Clear EXTI1 interrupt pending flag */
		EXTI->PR |= EXTI_PR_PR1;
	}
}


float getPOTValue()
{
	/* Start ADC conversion */
	ADC1->CR |= ADC_CR_ADSTART;

	/* Wait for conversion */
	while(!(ADC1->ISR & ADC_ISR_EOC));

	/* Reset conversion flag */
	ADC1->ISR &= ~(ADC_ISR_EOC);

	/* Data mask bit to obtain ADC data */
	uint32_t value = (ADC1->DR) & ADC_DR_DATA;

	return ((float)value);
}


uint16_t offsetDAC(float value)
{
	/* Map the DAC value to a voltage range */
	float normalizedDAC = value / MAX_DAC_VALUE;
	float outputVoltageRange = MAX_VOLTAGE - DIODE_DROP;
	float outputVoltage = (normalizedDAC * outputVoltageRange) + DIODE_DROP;

	/* Convert voltage back to a DAC level */
	float normOutputVoltage = outputVoltage / MAX_VOLTAGE;
	float outputDACValue = normOutputVoltage * MAX_DAC_VALUE;

	return ((uint16_t) outputDACValue);

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
	/* */
	char RS = cmdFlag ? 0x00 : 0x40;
	char EN = 0x80;

	/* Separate 8 bit data into 2 chunks */
	char high = (data & 0xF0) >> 4;
	char low = data & 0xF;

	writeToHC595(high | RS);
	writeToHC595(high | RS | EN);
	writeToHC595(high | RS);
	writeToHC595(low | RS);
	writeToHC595(low | RS | EN);
	writeToHC595(low | RS);
}


void writeStringToLCD(char* data)
{
	int n;
	// Write one char at a time
	for(n = 0; n < strlen(data); n++)
	{
		writeToLCD(data[n], 0);
	}
}


void wait(int ms)
{
	TIM6->ARR = 12 * ms;

	TIM6->CNT = 0;

	TIM6->CR1 |= 0x1;

	TIM6->SR = 0;

	while((TIM6->SR & 0x1) == 0);

	TIM6->CR1 &= 0xFFFE;
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
