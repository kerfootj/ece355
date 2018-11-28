// ----------------------------------------------------------------------------
// School: 	University of Victoria, Canada.
// Course: 	ECE 355 "Microprocessor-Based Systems"
//
// Version: 1.3.1
//
// Author: 	Joel Kerfoot
// Date:	November 25, 2018
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations
// ----------------------------------------------------------------------------


/******************************************************************************
 * 									REFERENCES								  *
 ******************************************************************************/

/* http://www.ece.uvic.ca/~daler/courses/ece355/interfacex.pdf */

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
#define myTIM2_PRESCALER ((uint16_t)0x0000)
#define myTIM3_PRESCALER ((uint16_t)0xFFFF)
#define myTIM6_PRESCALER ((uint16_t)4)

/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

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
void myADC_Init(void);
void myDAC_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myTIM6_Init(void);
void myEXTI_Init(void);
void mySPI_Init(void);
void myLCD_Init(void);

unsigned int convert_analog_to_digital(void);
void convert_digital_to_analog(unsigned int);
int calculate_resistance(unsigned int);

void write_to_LCD(char, int);
void write_string_to_LCD(char *);
void write_to_HC595(char);
void set_LCD_4_bit_mode();

void wait(int);


/******************************************************************************
 * 								   GLOBAL VARIABLES							  *
 ******************************************************************************/

int global_resistance = 0;
int global_frequency = 0;


/******************************************************************************
 * 								     IMPLEMENTATION						  	  *
 ******************************************************************************/

int main(int argc, char* argv[])
{

	/* Start initialization */
	myGPIOA_Init();
	myGPIOB_Init();
	myTIM2_Init();
	myTIM3_Init();
	myTIM6_Init();
	myEXTI_Init();
	myADC_Init();
	myDAC_Init();
	mySPI_Init();
	myLCD_Init();

	/* Pole ADC */
	while (1)
	{
		unsigned int value = convert_analog_to_digital();
		global_resistance = calculate_resistance(value);
		convert_digital_to_analog(value);
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

	/* Configure SCK and MOSI pins PB3, PB5 */
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

void myDAC_Init()
{
	/* Enable DAC clock */
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	/* Enable DAC */
	DAC->CR |= DAC_CR_EN1;
}


void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	RCC->APB1ENR |= RCC_APB1RSTR_TIM2RST;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority 0 in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	TIM2->DIER |= TIM_DIER_UIE;
}


void myTIM3_Init(void)
{
	/* Enable clock for TIM3 peripheral */
	RCC->APB1ENR |= RCC_APB1RSTR_TIM3RST;

	/* Configure TIM3: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on underflow only */
	TIM3->CR1 &= 0xFFFD;
	TIM3->CR1 |= 0x9D;

	/* Set clock prescaler value */
	TIM3->PSC = myTIM3_PRESCALER;

	/* Set auto-reloaded delay */
	TIM3->ARR = myTIM3_PRESCALER;

	/* Set timer count */
	TIM3->CNT = SystemCoreClock / myTIM3_PRESCALER;

	/* Update timer registers */
	TIM3->EGR |= 0x1;

	/* Assign TIM interrupt priority 0 in NVIC */
	NVIC_SetPriority(TIM3_IRQn, 0);

	/* Enable TIM3 interrupts in NVIC */
	NVIC_EnableIRQ(TIM3_IRQn);

	/* Enable update interrupt generation */
	TIM3->DIER |= TIM_DIER_UIE;

	/* Start the timer */
	TIM3->CR1 |= TIM_CR1_CEN;
}


void myTIM6_Init()
{
	/* Enable clock for TIM6 peripheral */
	RCC->APB1ENR |= RCC_APB1RSTR_TIM6RST;

	/* Configure TIM6: buffer auto-reload */
	TIM6->CR1 |= 0x84;

	/* Set clock prescaler value */
	TIM6->PSC = myTIM6_PRESCALER;
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


void mySPI_Init()
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


void myLCD_Init()
{

	set_LCD_4_bit_mode();

	/* Display 2 Lines of 8 characters */
	write_to_LCD(0x28, 1); //DL=0, N=1, F=0
	wait(50);

	/* Disable Cursor */
	write_to_LCD(0x0C, 1); //D=1, C=0, B=0
	wait(50);

	/* Auto-increment address after access */
	write_to_LCD(0x06, 1); //I/D=1, S=0
	wait(50);

	/* Clear Display */
	write_to_LCD(0x01, 1);
	wait(1600);
}

void set_LCD_4_bit_mode()
{
	/* Waits added to ensure that 4 bit mode is set correctly */

	write_to_HC595(0x3);
	wait(1520);
	write_to_HC595(0x3 | 0x80);
	wait(1520);
	write_to_HC595(0x3);

	wait(1520);
	write_to_HC595(0x3);
	wait(1520);
	write_to_HC595(0x3 | 0x80);
	wait(1520);
	write_to_HC595(0x3);
	wait(1520);

	write_to_HC595(0x3);
	wait(1520);
	write_to_HC595(0x3 | 0x80);
	wait(1520);
	write_to_HC595(0x3);
	wait(1520);

	write_to_HC595(0x2);
	wait(1520);
	write_to_HC595(0x2 | 0x80);
	wait(1520);
	write_to_HC595(0x2);
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
		// Relevant register: TIM2->SR
		TIM2->SR &= 0xFFFE;

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= 0x1;
	}
}


void TIM3_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM3->SR & TIM_SR_UIF) != 0)
	{

		/* Create frequency and resistance strings */
		char frequency[9];
		char resistance[9];
		sprintf(frequency, "F:%4dHz", global_frequency);
		sprintf(resistance, "R:%4d%c", global_resistance, 0xF4);

		/* Move cursor to start of first line and write frequency */
		write_to_LCD(0x80, 1);
		write_string_to_LCD(frequency);

		/* Move cursor to start of second line and write resistance */
		write_to_LCD(0xC0, 1);
		write_string_to_LCD(resistance);

		/* Clear update interrupt flag */
		TIM3->SR &= ~(TIM_SR_UIF);

		/* Set clock count */
		TIM3->CNT = SystemCoreClock / myTIM3_PRESCALER;

		/* Restart stopped timer */
		TIM3->CR1 |= 0x1;
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

			/* Calculate signal frequency */
			global_frequency = ((float)SystemCoreClock) / count;
		}

		/* Clear EXTI1 interrupt pending flag */
		EXTI->PR |= EXTI_PR_PR1;
	}
}


unsigned int convert_analog_to_digital()
{
    //Start the analog to digital conversion
	ADC1->CR |= ADC_CR_ADSTART;

	//Wait until conversion finishes
	while((ADC1->ISR & ADC_ISR_EOC) == 0);

	//Read the result of the conversion
	return ADC1->DR;
}


void convert_digital_to_analog(unsigned int value)
{
	//Clear the right 12 bits
	DAC->DHR12R1 &= ~DAC_DHR12R1_DACC1DHR;

	//Set the right 12 bits
	DAC->DHR12R1 |= value;
}

int calculate_resistance(unsigned int value)
{
	return (int)((5000.0 / 4095.0) * value);
}


void write_string_to_LCD(char * s)
{
    int i;
    for(i = 0; i < strlen(s); ++i)
    {
        write_to_LCD(s[i], 0);
    }
}


void write_to_LCD(char c, int cmdFlag)
{
	/* Apply offset to non commands */
    char RS = cmdFlag ? 0x00 : 0x40;
    char EN = 0x80;

    /* Split word into high and low chunks */
    char high = (c & 0xF0) >> 4;
    char low = c & 0xF;

    write_to_HC595(high | RS);
    write_to_HC595(high | RS| EN);
    write_to_HC595(high | RS);
    write_to_HC595(low | RS);
    write_to_HC595(low | RS | EN);
    write_to_HC595(low | RS);
}


void write_to_HC595(char data)
{
	/* Force LCK signal to 0 */
	GPIOB->BRR = GPIO_Pin_4;

	/* Wait until SPI1 is ready */
	while((SPI1->SR & SPI_SR_BSY) != 0);

	/* Send 8 bits of data */
	SPI_SendData8(SPI1, data);

	/* Wait until SPI1 is not busy */
	while((SPI1->SR & SPI_SR_BSY) != 0);

	/* Force LCK signal to 1 */
	GPIOB->BSRR = GPIO_Pin_4;

}


void wait(int ms)
{
    /* Set auto reload delay */
	TIM6->ARR = 12 * ms;

    /* Set count */
	TIM6->CNT = 0;

    /* Start timer */
	TIM6->CR1 |= 0x1;

    /* Clear status register */
	TIM6->SR = 0;

    /* Wait for timer overflow */
	while((TIM6->SR & 0x1) == 0);

    /* Stop timer */
	TIM6->CR1 &= ~(TIM_CR1_CEN);
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
