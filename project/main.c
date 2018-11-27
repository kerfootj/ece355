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

void wait(int);

unsigned int convertAnalogToDigital(void);
void convertDigitalToAnalog(unsigned int);
int calculateResistance(unsigned int);

void writeToLCD(char, int);
void writeStringToLCD(char *);
void sendCommandToLCD(char);
void HC595Write(char);
void set4BitLCDMode();


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
	myGPIOA_Init();
	myGPIOB_Init();
	myTIM2_Init(); //Initialize timer used for measuring frequency
	myEXTI_Init();
	myADC_Init();
	myDAC_Init();
	mySPI_Init();
	myTIM3_Init(); //Initialize timer used for measuring 1 second LCD print intervals
	myTIM6_Init(); //Initialize timer used for waiting for short periods
	myLCD_Init();


	while (1)
	{
		unsigned int digital = convertAnalogToDigital();
		global_resistance = calculateResistance(digital);
		convertDigitalToAnalog(digital);
	}

	return 0;
}


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
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1RSTR_TIM2RST;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 &= 0xFFED;
	TIM2->CR1 |= 0x8D;

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR |= 0x1;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
}

void myTIM3_Init(void)
{
	/* Enable clock for TIM3 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1RSTR_TIM3RST;

	/* Configure TIM3: buffer auto-reload, count down, stop on overflow,
	 * enable update events, interrupt on underflow only */
	// Relevant register: TIM3->CR1
	TIM3->CR1 &= 0xFFFD;
	TIM3->CR1 |= 0x9D;

	/* Set clock prescaler value */
	//Frequency is 732Hz
	TIM3->PSC = myTIM3_PRESCALER;

	/* Set auto-reloaded delay */
	TIM3->ARR = myTIM3_PRESCALER;

	//Should correspond to a value of 1 second
	TIM3->CNT = SystemCoreClock / myTIM3_PRESCALER;

	/* Update timer registers */
	// Relevant register: TIM3->EGR
	TIM3->EGR |= 0x1;

	/* Assign TIM3 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM3_IRQn, 0);

	/* Enable TIM3 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM3_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM3->DIER
	TIM3->DIER |= TIM_DIER_UIE;

	//Start timer
	TIM3->CR1 |= 0x1;
}

void myTIM6_Init()
{
	/* Enable clock for TIM6 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1RSTR_TIM6RST;

	/* Configure TIM6: buffer auto-reload,
	 * enable update events */
	TIM6->CR1 |= 0x84;


	/* Set clock prescaler value */
	//Frequency is 12MHz
	TIM6->PSC = myTIM6_PRESCALER;
}

void wait(int microseconds)
{
	//12 cycles with prescaler = 1 microsecond
	TIM6->ARR = 12 * microseconds;

	//Set count to 0
	TIM6->CNT = 0;

	//Start timer
	TIM6->CR1 |= 0x1;

	//Clear status register
	TIM6->SR = 0;

	//Wait until timer overflows
	while((TIM6->SR & 0x1) == 0);

	//Stop timer
	TIM6->CR1 &= 0xFFFE;
}

void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	//TODO: Use |= instead of =
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void mySPI_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_PinAFConfig(GPIOB, 3, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOB, 5, GPIO_AF_0);

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

	set4BitLCDMode();

	//2 lines of 8 characters are displayed
	sendCommandToLCD(0x28); //DL=0, N=1, F=0
	wait(37);

	//Curser not displayed and is not blinking
	sendCommandToLCD(0x0C); //D=1, C=0, B=0
	wait(37);

	//Auto increment DDRAM address after each access
	sendCommandToLCD(0x06); //I/D=1, S=0
	wait(37);

	//Clear display
	sendCommandToLCD(0x01);
	wait(1520);
}

void set4BitLCDMode()
{
	//We have lots of waits to make sure LCD is able to be set to 4 bit mode

	HC595Write(0x3);
	wait(1520);
	HC595Write(0x3 | 0x80);
	wait(1520);
	HC595Write(0x3);

	wait(1520);
	HC595Write(0x3);
	wait(1520);
	HC595Write(0x3 | 0x80);
	wait(1520);
	HC595Write(0x3);
	wait(1520);

	HC595Write(0x3);
	wait(1520);
	HC595Write(0x3 | 0x80);
	wait(1520);
	HC595Write(0x3);
	wait(1520);

	HC595Write(0x2);
	wait(1520);
	HC595Write(0x2 | 0x80);
	wait(1520);
	HC595Write(0x2);
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

		//Create strings holding resistance and frequency values
		char frequencyStr[9];
		char resistanceStr[9];
		sprintf(frequencyStr, "F:%4dHz", global_frequency);
		sprintf(resistanceStr, "R:%4d%c", global_resistance, 0xF4);

		//Move cursor to start of first line
		sendCommandToLCD(0x80);

		writeStringToLCD(frequencyStr);

		//Move cursor to start of second line
		sendCommandToLCD(0xC0);

		writeStringToLCD(resistanceStr);

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM3->SR &= 0xFFFE;

		//Should correspond to a value of 1 second
		TIM3->CNT = SystemCoreClock / myTIM3_PRESCALER;

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM3->CR1 |= 0x1;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	// Your local variables...
	static int edgeCounter = 0;

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//Even edge
		if(edgeCounter % 2 == 0) {

			/* Clear the count register */
			TIM2->CNT = 0;

			/* Start the timer */
			TIM2->CR1 |= 0x1;
		}
		else {
			//	- Calculate signal period and frequency.
			//	- Print calculated values to the console.
			//	  NOTE: Function trace_printf does not work
			//	  with floating-point numbers: you must use
			//	  "unsigned int" type to print your signal
			//	  period and frequency.

			/* Read out the count register */
			int count = TIM2->CNT;

			/* Stop the timer */
			TIM2->CR1 &= 0xFFFE;

			// Calculate the frequency
			double signalPeriod = count / (double) SystemCoreClock;
			global_frequency = (int) (1 / signalPeriod);

		}

		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR)
		EXTI->PR |= EXTI_PR_PR1;

		++edgeCounter;
	}
}

unsigned int convertAnalogToDigital()
{
    //Start the analog to digital conversion
	ADC1->CR |= ADC_CR_ADSTART;

	//Wait until conversion finishes
	while((ADC1->ISR & ADC_ISR_EOC) == 0);

	//Read the result of the conversion
	return ADC1->DR;
}

void convertDigitalToAnalog(unsigned int digital)
{
	//Clear the right 12 bits
	DAC->DHR12R1 &= ~DAC_DHR12R1_DACC1DHR;

	//Set the right 12 bits
	DAC->DHR12R1 |= digital;
}

int calculateResistance(unsigned int digital)
{
	return (int)((5000.0 / 4095.0) * digital);
}


void writeToLCD( char c, int isCommand)
{
    char RS = isCommand ? 0x00 : 0x40;
    char EN = 0x80;

    char highNibble = (c & 0xF0) >> 4;
    char lowNibble = c & 0xF;

    HC595Write(highNibble | RS);
    HC595Write(highNibble | RS| EN);
    HC595Write(highNibble | RS);
    HC595Write(lowNibble | RS);
    HC595Write(lowNibble | RS | EN);
    HC595Write(lowNibble | RS);
}

void writeStringToLCD(char * s)
{
    int i;
    for(i = 0; i < strlen(s); ++i)
    {
        writeToLCD(s[i], 0);
    }
}

void sendCommandToLCD(char c)
{
    writeToLCD(c, 1);
}

void HC595Write(char data)
{
    //Based off of example from http://www.ece.uvic.ca/~daler/courses/ceng355/interfacex.pdf

    /* Force your LCK signal to 0 */
	GPIOB->BRR = GPIO_Pin_4;

    /* Wait until SPI1 is ready (TXE = 1 or BSY = 0) */
	while((SPI1->SR & SPI_SR_BSY) != 0);

    /* Assumption: your data holds 8 bits to be sent */
    SPI_SendData8(SPI1, data);

    /* Wait until SPI1 is not busy (BSY = 0) */
	while((SPI1->SR & SPI_SR_BSY) != 0);

    /* Force your LCK signal to 1 */
	GPIOB->BSRR = GPIO_Pin_4;

}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
