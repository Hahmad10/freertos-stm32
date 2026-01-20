/* Traffic Light System
 * One-way traffic simulation with dynamic flow control
 * FreeRTOS on STM32F4 Discovery Board
 *
 * Architecture:
 * 4 Tasks - FlowAdj (reads pot), TrLight (starts timer chain),
 *             CarGen (spawns cars), Display (moves cars + updates LEDs)
 * 3 Queues - FlowRate, LightState (both length-1, overwrite/peek pattern),
 *             NewCar (length-10 FIFO buffer)
 * 3 Timers - Green, Yellow, Red (one-shot, chain into each other)
 *
 * All inter-task communication is through queues. No shared global data.
 */

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/* ---- Pin Definitions (all on GPIOC) ---- */
#define TLS_RED_PIN      GPIO_Pin_0 /* PC0: Red traffic light LED */
#define TLS_AMBER_PIN    GPIO_Pin_1 /* PC1: Yellow/Amber traffic light LED */
#define TLS_GREEN_PIN    GPIO_Pin_2 /* PC2: Green traffic light LED */
#define TLS_POT_PIN      GPIO_Pin_3 /* PC3: Potentiometer analog input (ADC) */
#define TLS_SR_DATA_PIN  GPIO_Pin_6 /* PC6: Shift register serial data in */
#define TLS_SR_CLOCK_PIN GPIO_Pin_7 /* PC7: Shift register clock (shared by all 3 SRs) */
#define TLS_SR_RESET_PIN GPIO_Pin_8 /* PC8: Shift register clear/reset (active low, shared) */

/* ---- Traffic Light States ---- */
#define LIGHT_GREEN  0
#define LIGHT_YELLOW 1
#define LIGHT_RED    2

/* ---- Timing Constants (milliseconds) ---- */
#define FLOW_READ_PERIOD_MS 100   /* How often the pot is read */
#define CAR_MOVE_PERIOD_MS  500   /* How often cars advance one LED (~2 LEDs/sec) */
#define YELLOW_DURATION_MS  2000  /* Yellow is always 2 seconds */
#define GREEN_MIN_MS        5000  /* Green at 0% flow = 5 seconds */
#define GREEN_MAX_MS        10000 /* Green at 100% flow = 10 seconds */
#define RED_MIN_MS          5000  /* Red at 100% flow = 5 seconds */
#define RED_MAX_MS          10000 /* Red at 0% flow = 10 seconds */

/* ---- Queue Lengths ---- */
#define FLOW_QUEUE_LENGTH  1  /* Length 1: only care about latest value */
#define LIGHT_QUEUE_LENGTH 1  /* Length 1: only care about latest state */
#define CAR_QUEUE_LENGTH   10 /* Length 10: buffer for car spawn events */

/* ---- Road Constants ---- */
#define ROAD_LENGTH   19 /* 19 green LEDs on the breadboard */
#define STOP_LINE_POS 7  /* Stop line at bit 7 (0-indexed from entry) */

/*-----------------------------------------------------------*/
/* Middleware: GPIO Initialization                      */
/* Configures all pins on GPIOC used by the traffic system. */
/*-----------------------------------------------------------*/

static void GPIO_Init_TLS(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Must enable the clock for GPIOC before configuring any pins on it */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Configure output pins:
     * PC0, PC1, PC2 = traffic light LEDs (directly driven by GPIO)
     * PC6, PC7, PC8 = shift register control lines (data, clock, reset)
     * Push-pull output: pin drives both high and low (vs open-drain) */
    GPIO_InitStruct.GPIO_Pin = TLS_RED_PIN | TLS_AMBER_PIN | TLS_GREEN_PIN |
                               TLS_SR_DATA_PIN | TLS_SR_CLOCK_PIN | TLS_SR_RESET_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;     /* Digital output */
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;    /* Push-pull (can source and sink current) */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; /* Switching speed */
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;  /* No internal pull-up or pull-down */
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Configure PC3 as analog input for potentiometer.
     * Analog mode is required for the ADC to read a voltage level. */
    GPIO_InitStruct.GPIO_Pin = TLS_POT_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; /* Analog mode for ADC */
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Shift register reset is active-low: HIGH = normal operation, LOW = clear all outputs.
     * Start with reset HIGH so the registers operate normally. */
    GPIO_SetBits(GPIOC, TLS_SR_RESET_PIN);
    /* Clock idles HIGH; data is latched on the falling edge */
    GPIO_SetBits(GPIOC, TLS_SR_CLOCK_PIN);
}

/*-----------------------------------------------------------*/
/* Middleware: ADC Initialization & Read                */
/* ADC1 Channel 13 reads the potentiometer voltage on PC3. */
/* 12-bit resolution: returns 0 (0V) to 4095 (3.3V). */
/*-----------------------------------------------------------*/

static void ADC_Init_TLS(void) {
    ADC_InitTypeDef ADC_InitStruct;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;

    /* Enable the clock for ADC1 (on the APB2 bus) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* ADC Common configuration (shared settings across ADC peripherals) */
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;                /* Only using ADC1, not dual/triple mode */
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;             /* ADC clock = APB2/4 */
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; /* Not using DMA */
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);

    /* ADC1 specific configuration */
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;                      /* 12-bit: 0 to 4095 */
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;                               /* Single channel, not scanning multiple */
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;                         /* Software-triggered, one conversion at a time */
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; /* No external trigger */
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;                      /* Result is right-aligned in the data register */
    ADC_InitStruct.ADC_NbrOfConversion = 1;                                  /* One channel per conversion */
    ADC_Init(ADC1, &ADC_InitStruct);

    /* Map ADC1 to Channel 13 (which is the fixed channel for PC3 on STM32F4).
     * 144-cycle sample time: longer = more accurate reading from the pot. */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_144Cycles);

    /* Turn on ADC1 */
    ADC_Cmd(ADC1, ENABLE);
}

/* Read a single ADC value. Triggers a conversion, waits for it, returns the result.
 * Blocking wait is fine here ADC conversions take microseconds. */
static uint16_t ADC_Read(void) {
    ADC_SoftwareStartConv(ADC1); /* Start a conversion */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
        ;                                /* Wait for End Of Conversion flag */
    return ADC_GetConversionValue(ADC1); /* Read and return the 12-bit result */
}

/*-----------------------------------------------------------*/
/* FreeRTOS Hook Functions                             */
/* These are required by FreeRTOS the project won't link */
/* without them. They handle error conditions.         */
/*-----------------------------------------------------------*/

/* Called if pvPortMalloc (FreeRTOS heap allocation) fails.
 * This means there isn't enough heap for a queue/task/timer.
 * Infinite loop = system halts so you can debug. */
void vApplicationMallocFailedHook(void) {
    for (;;)
        ;
}

/* Called if FreeRTOS detects a task has overflowed its stack.
 * Each task has a fixed stack size set in xTaskCreate. */
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
    (void)pcTaskName; /* Cast to void to suppress "unused parameter" warning */
    (void)pxTask;
    for (;;)
        ;
}

/* Called whenever the idle task runs (no other tasks are ready).
 * Here it just checks the remaining free heap as a sanity check. */
void vApplicationIdleHook(void) {
    volatile size_t xFreeStackSpace;
    xFreeStackSpace = xPortGetFreeHeapSize();
    if (xFreeStackSpace > 100) {
        /* Heap is healthy plenty of memory remaining */
    }
}

/* Basic hardware setup. NVIC_SetPriorityGrouping(0) configures the
 * ARM interrupt priority scheme for FreeRTOS compatibility. */
static void prvSetupHardware(void) {
    NVIC_SetPriorityGrouping(0);
}
