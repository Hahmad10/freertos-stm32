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
