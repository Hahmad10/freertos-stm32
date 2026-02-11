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
/* Simple PRNG (xorshift32)                            */
/* Lightweight random number generator for embedded use. */
/* XORs the state with bit-shifted versions of itself to */
/* produce pseudo-random numbers. Seeded from ADC noise. */
/*-----------------------------------------------------------*/
static uint32_t prng_state = 1;

static uint32_t prng_rand(void) {
    uint32_t x = prng_state;
    x ^= x << 13;   /* XOR with left-shifted self */
    x ^= x >> 17;   /* XOR with right-shifted self */
    x ^= x << 5;    /* XOR with left-shifted self again */
    prng_state = x; /* Save for next call */
    return x;       /* Returns a pseudo-random 32-bit number */
}

/*-----------------------------------------------------------*/
/* Queue Handles                                       */
/* These are the ONLY global state. All data flows through */
/* queues, not shared variables. Queues are thread-safe. */
/*-----------------------------------------------------------*/
static xQueueHandle xFlowRateQueue = NULL;   /* Carries flow rate (0-100) from FlowTask to others */
static xQueueHandle xLightStateQueue = NULL; /* Carries light state (0/1/2) from timer callbacks to DisplayTask */
static xQueueHandle xNewCarQueue = NULL;     /* Carries car spawn events from GeneratorTask to DisplayTask */

/* ---- Timer Handles ---- */
static TimerHandle_t xGreenTimer = NULL;  /* One-shot: fires when green phase ends */
static TimerHandle_t xYellowTimer = NULL; /* One-shot: fires when yellow phase ends */
static TimerHandle_t xRedTimer = NULL;    /* One-shot: fires when red phase ends */

/* ---- Task Forward Declarations ---- */
static void TrafficFlowTask(void *pvParameters);
static void TrafficLightTask(void *pvParameters);
static void TrafficGeneratorTask(void *pvParameters);
static void SystemDisplayTask(void *pvParameters);

/* ---- Timer Callback Declarations ---- */
static void vGreenTimerCallback(TimerHandle_t xTimer);
static void vYellowTimerCallback(TimerHandle_t xTimer);
static void vRedTimerCallback(TimerHandle_t xTimer);

/* ---- Hardware Setup ---- */
static void prvSetupHardware(void);

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
/* Middleware: Shift Register Driver (SN74HC164N)       */
/* 3 shift registers daisy-chained: 3 x 8 bits = 24 bits. */
/* We use 19 of those for the green car LEDs.          */
/* Data is clocked in LSB-first on the falling clock edge. */
/*-----------------------------------------------------------*/

/* Small busy-wait delay (~1 microsecond at 168MHz).
 * The shift register needs brief pauses between signal changes. */
static void SR_Delay(void) {
    volatile uint32_t i;
    for (i = 0; i < 50; i++)
        ;
}

/* Clear all shift register outputs to 0 by pulsing reset LOW then HIGH. */
static void ShiftRegister_Reset(void) {
    GPIO_ResetBits(GPIOC, TLS_SR_RESET_PIN); /* Pull reset LOW clears all outputs */
    SR_Delay();
    GPIO_SetBits(GPIOC, TLS_SR_RESET_PIN); /* Pull reset HIGH normal operation resumes */
    SR_Delay();
}

/* Write a 24-bit value to the shift register chain.
 * Each bit controls one LED. Bit 0 = first LED (traffic entry).
 *
 * How it works:
 * 1. Reset clears all outputs
 * 2. For each of 24 bits (LSB first):
 *    a. Set the data pin (PC6) to the bit value (high or low)
 *    b. Create a falling edge on the clock (PC7): HIGH LOW HIGH
 *    c. The falling edge shifts the data bit into the register
 * 3. After 24 clock pulses, all bits are in place across all 3 registers */
static void ShiftRegister_Write(uint32_t data) {
    int i;
    ShiftRegister_Reset(); /* Start fresh clear all outputs */

    /* Clock out 24 bits, LSB first (bit 0 goes in first, ends up at far end of chain) */
    for (i = 0; i <= 23; i++) {
        /* Set data pin to current bit value */
        if (data & (1 << i)) {
            GPIO_SetBits(GPIOC, TLS_SR_DATA_PIN); /* Bit is 1: data HIGH */
        } else {
            GPIO_ResetBits(GPIOC, TLS_SR_DATA_PIN); /* Bit is 0: data LOW */
        }
        SR_Delay();

        /* Create falling edge on clock to latch the data bit */
        GPIO_ResetBits(GPIOC, TLS_SR_CLOCK_PIN); /* Clock LOW (falling edge data latched) */
        SR_Delay();
        GPIO_SetBits(GPIOC, TLS_SR_CLOCK_PIN); /* Clock back to HIGH (idle state) */
        SR_Delay();
    }
}

/*-----------------------------------------------------------*/
/* Helper: Duration Calculations                       */
/* Green and red durations scale linearly with flow rate. */
/* At max flow: green=10s, red=5s (green is 2x red). */
/* At min flow: green=5s, red=10s (red is 2x green). */
/* Yellow is always constant at 2 seconds.             */
/*-----------------------------------------------------------*/

/* More traffic   longer green light to let more cars through.
 * flow=0: 5000 + 0 = 5s. flow=100: 5000 + 5000 = 10s. */
static uint32_t CalculateGreenDuration(uint16_t flow_rate) {
    return GREEN_MIN_MS + ((GREEN_MAX_MS - GREEN_MIN_MS) * flow_rate) / 100;
}

/* More traffic   shorter red light so cars wait less.
 * flow=0: 10000 - 0 = 10s. flow=100: 10000 - 5000 = 5s. */
static uint32_t CalculateRedDuration(uint16_t flow_rate) {
    return RED_MAX_MS - ((RED_MAX_MS - RED_MIN_MS) * flow_rate) / 100;
}

/*-----------------------------------------------------------*/
/* Helper: Build Shift Register Output                 */
/* Maps the 19-bit car_positions bitmask directly to the */
/* shift register output. Bits 0-18 = LEDs 0-18. No gap. */
/* Traffic lights are on GPIO, NOT in the shift register. */
/*-----------------------------------------------------------*/

static uint32_t BuildShiftRegisterOutput(uint32_t car_positions, uint8_t light_state) {
    return car_positions & 0x7FFFF; /* Mask to 19 bits (0x7FFFF = 0b1111111111111111111) */
}

/*-----------------------------------------------------------*/
/* Timer Callbacks Traffic Light State Machine            */
/*                                                     */
/* These form a chain: each callback changes the light state */
/* and starts the next timer. The cycle repeats forever: */
/* GREEN -> (timer fires) -> YELLOW -> (timer fires) -> RED */
/*     -> (timer fires) -> GREEN -> ...                        */
/*                                                     */
/* Callbacks run in the FreeRTOS timer service task context, */
/* not in any of our application tasks.                */
/*-----------------------------------------------------------*/

/* Called when the green phase expires. Switches to yellow and starts yellow timer. */
static void vGreenTimerCallback(TimerHandle_t xTimer) {
    uint8_t state = LIGHT_YELLOW;
    xQueueOverwrite(xLightStateQueue, &state); /* Tell all tasks: light is now YELLOW */
    xTimerStart(xYellowTimer, 0);              /* Start the 2-second yellow timer */
}

/* Called when yellow expires. Switches to red.
 * Reads the current flow rate to calculate how long red should last. */
static void vYellowTimerCallback(TimerHandle_t xTimer) {
    uint16_t flow_rate = 50; /* Default 50% if queue is empty (safety net) */
    uint8_t state = LIGHT_RED;
    xQueueOverwrite(xLightStateQueue, &state); /* Tell all tasks: light is now RED */

    /* Read current flow rate to decide red duration */
    xQueuePeek(xFlowRateQueue, &flow_rate, 0); /* Peek: read without removing from queue */
    /* xTimerChangePeriod both changes the period AND starts the timer */
    xTimerChangePeriod(xRedTimer, pdMS_TO_TICKS(CalculateRedDuration(flow_rate)), 0);
}

/* Called when red expires. Switches to green.
 * Reads the current flow rate to calculate how long green should last. */
static void vRedTimerCallback(TimerHandle_t xTimer) {
    uint16_t flow_rate = 50; /* Default 50% if queue is empty (safety net) */
    uint8_t state = LIGHT_GREEN;
    xQueueOverwrite(xLightStateQueue, &state); /* Tell all tasks: light is now GREEN */

    /* Read current flow rate to decide green duration */
    xQueuePeek(xFlowRateQueue, &flow_rate, 0);
    xTimerChangePeriod(xGreenTimer, pdMS_TO_TICKS(CalculateGreenDuration(flow_rate)), 0);
}

/*-----------------------------------------------------------*/
/* Task: Traffic Flow Adjustment (Priority 2)          */
/* Reads the potentiometer via ADC every 100ms and publishes */
/* the flow rate (0-100%) to the FlowRate queue.       */
/* Other tasks peek this queue to get the current flow rate. */
/*-----------------------------------------------------------*/

static void TrafficFlowTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount(); /* Record start time for periodic execution */
    uint16_t adc_value;
    uint16_t flow_rate;

    while (1) {
        /* Read potentiometer. Divide by 10 to scale raw ADC into usable range. */
        adc_value = ADC_Read() / 10;

        /* Convert ADC value to 0-100% flow rate.
         * Cast to uint32_t first to prevent overflow during multiplication:
         * e.g. 3800 * 100 = 380,000 which exceeds uint16_t max of 65,535. */
        flow_rate = (uint16_t)((uint32_t)adc_value * 100 / 3800);

        /* Debug output to SWV ITM console (visible in TrueSTUDIO) */
        printf("ADC: %u Flow: %u%%\n", (unsigned int)adc_value, (unsigned int)flow_rate);

        /* Publish flow rate to queue. xQueueOverwrite always succeeds:
         * it replaces whatever is in the length-1 queue with the new value.
         * This way, readers always get the LATEST flow rate, not a stale one. */
        xQueueOverwrite(xFlowRateQueue, &flow_rate);

        /* Sleep until exactly 100ms after last wake. vTaskDelayUntil compensates
         * for execution time so the period is consistent (vs vTaskDelay which
         * would drift by adding execution time on top of the delay). */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(FLOW_READ_PERIOD_MS));
    }
}

/*-----------------------------------------------------------*/
/* Task: Traffic Light State Machine (Priority 3 highest) */
/* Initializes the light to GREEN, starts the first timer, */
/* then sleeps forever. All subsequent state transitions are */
/* handled by the timer callbacks above.               */
/*-----------------------------------------------------------*/

static void TrafficLightTask(void *pvParameters) {
    uint16_t flow_rate = 50; /* Default if flow queue not populated yet */
    uint8_t initial_state = LIGHT_GREEN;

    /* Set initial light state to GREEN */
    xQueueOverwrite(xLightStateQueue, &initial_state);

    /* Wait up to 500ms for FlowTask to put a value in the flow queue.
     * This gives the flow task time to do its first ADC read. */
    xQueuePeek(xFlowRateQueue, &flow_rate, pdMS_TO_TICKS(500));

    /* Start the green timer. xTimerChangePeriod sets the duration based on
     * current flow rate AND starts the timer. Once this fires, the callback
     * chain (    greenyellowredgreen    ...) runs forever on its own. */
    xTimerChangePeriod(xGreenTimer, pdMS_TO_TICKS(CalculateGreenDuration(flow_rate)), 0);

    /* Nothing left to do. Sleep forever.
     * FreeRTOS tasks must never return they must loop forever or delete themselves. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*-----------------------------------------------------------*/
/* Task: Traffic Generator (Priority 2)                */
/* Every 500ms (same rate as car movement), decides whether */
/* to spawn a new car based on flow rate. Higher flow = */
/* higher probability of spawning.                     */
/*-----------------------------------------------------------*/

static void TrafficGeneratorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount(); /* For fixed-rate periodic execution */
    uint16_t flow_rate = 0;                         /* Local copy of flow rate, updated each tick */
    uint8_t new_car = 1;                            /* Value sent to queue just a flag meaning "car exists" */
    uint32_t spawn_chance;                          /* Probability out of 100 that a car spawns this tick */

    while (1) {
        /* Read the latest flow rate. Peek = read without removing.
         * Timeout 0 = don't block, just use last known value if queue is empty. */
        xQueuePeek(xFlowRateQueue, &flow_rate, 0);

        /* Calculate spawn probability based on flow rate.
         * flow=0      spawn_chance = 17 + 0 = 17% (sparse: ~1 car per 6 ticks, ~6 LED gap)
         * flow=50     spawn_chance = 17 + 16 = 33% (moderate traffic)
         * flow=100    spawn_chance = 17 + 33 = 50% (dense traffic)
         * 17 = base chance (always some traffic), 33 = range added by flow */
        spawn_chance = 17 + (83 * flow_rate) / 100;

        /* Roll a random number 0-99. If it's below spawn_chance, spawn a car.
         * Example: spawn_chance=17 numbers 0-16 trigger a spawn 17% chance.
         * prng_rand() returns a large random uint32_t, % 100 gives 0-99. */
        if ((prng_rand() % 100) < spawn_chance) {
            /* Send a car event to the NewCar queue. DisplayTask will receive it.
             * Timeout 0 = if queue is full (10 items), just drop it, don't block. */
            xQueueSend(xNewCarQueue, &new_car, 0);
        }

        /* Sleep until next 500ms tick. Same rate as DisplayTask so car spawning
         * and car movement stay synchronized. */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAR_MOVE_PERIOD_MS));
    }
}

/*-----------------------------------------------------------*/
/* Task: System Display (Priority 1 lowest)              */
/*                                                     */
/* Every 500ms:                                        */
/* 1. Reads the current traffic light state            */
/* 2. Moves all cars forward by one position (bitmask) */
/* 3. Adds a new car at the entry if one was spawned */
/* 4. Writes the car positions to the shift registers */
/* 5. Sets the traffic light GPIO pin                  */
/*                                                     */
/* Cars are stored as a uint32_t bitmask. Each bit = 1 LED: */
/* Bit 0 = entry (leftmost LED on board)               */
/* Bit 7 = stop line (where cars stop on red/yellow) */
/* Bit 18 = exit (rightmost LED, cars disappear here) */
/*-----------------------------------------------------------*/

static void SystemDisplayTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t car_positions = 0;        /* Bitmask: each 1-bit = a car at that LED position */
    uint8_t light_state = LIGHT_GREEN; /* Local copy of current light color */
    uint8_t new_car;                   /* Buffer for receiving from NewCar queue */
    uint32_t sr_output;                /* What gets sent to the shift registers */
    uint32_t new_positions;            /* Temporary bitmask built during car movement */
    int pos;                           /* Current position being processed */
    int next_pos;                      /* Position the car wants to move to */

    while (1) {
        /* --- STEP 1: Get current traffic light state --- */
        /* Peek (read without removing) so other tasks can also read it */
        xQueuePeek(xLightStateQueue, &light_state, 0);

        /* --- STEP 2: Move existing cars --- */
        /* Build a fresh bitmask (new_positions) by processing each car.
         * Iterate RIGHT TO LEFT (pos 18 down to 0) so that cars further
         * ahead are placed first, allowing us to check for collisions
         * when processing cars behind them. */
        new_positions = 0;
        for (pos = ROAD_LENGTH - 1; pos >= 0; pos--) {
            /* Skip this position if there's no car here */
            if (!(car_positions & (1 << pos)))
                continue;

            next_pos = pos + 1; /* Where this car wants to move */

            /* Car at the end of the road (bit 18)? It exits   just don't
             * add it to new_positions, so it disappears. */
            if (next_pos >= ROAD_LENGTH)
                continue;

            /* Car is at the STOP LINE and light is red or yellow?
             * Stay put   don't move past the stop line. */
            if (pos == STOP_LINE_POS && light_state != LIGHT_GREEN) {
                new_positions |= (1 << pos); /* Keep car at same position */
                continue;
            }

            /* Car is PAST the stop line (already in or through the intersection).
             * These cars keep moving regardless of light color, but must still
             * check for collision (next position occupied). */
            if (pos > STOP_LINE_POS) {
                if (new_positions & (1 << next_pos)) {
                    new_positions |= (1 << pos); /* Blocked by car ahead stay */
                } else {
                    new_positions |= (1 << next_pos); /* Free move forward */
                }
                continue;
            }

            /* Car is BEFORE the stop line. Check if next position is occupied
             * (causes pile-up / bumper-to-bumper behind a stopped car). */
            if (new_positions & (1 << next_pos)) {
                new_positions |= (1 << pos); /* Blocked stay (pile up) */
                continue;
            }

            /* Nothing blocking move forward one position */
            new_positions |= (1 << next_pos);
        }
        car_positions = new_positions; /* Replace old positions with new */

        /* --- STEP 3: Add new car at the entry (bit 0) --- */
        /* Drain the NewCar queue. xQueueReceive removes items (vs Peek).
         * Only actually place one car per tick, and only if bit 0 is empty. */
        while (xQueueReceive(xNewCarQueue, &new_car, 0) == pdTRUE) {
            if (!(car_positions & 0x01)) { /* Is position 0 (entry) empty? */
                car_positions |= 0x01;     /* Place a car at the entry */
                break;                     /* One car per tick max */
            }
        }

        /* Safety: mask to 19 bits to discard any bits beyond the road length */
        car_positions &= ((1 << ROAD_LENGTH) - 1);

        /* --- STEP 4: Write car positions to shift registers --- */
        printf("cars:%u light:%u\n", (unsigned int)car_positions, (unsigned int)light_state);
        sr_output = BuildShiftRegisterOutput(car_positions, light_state);
        ShiftRegister_Write(sr_output); /* Clocks 24 bits out to the 3 daisy-chained SRs */

        /* --- STEP 5: Set traffic light GPIO pins --- */
        /* Turn all three LEDs off first, then turn on the correct one.
         * Traffic lights are driven directly by GPIO, NOT through the shift register. */
        GPIO_ResetBits(GPIOC, TLS_RED_PIN | TLS_AMBER_PIN | TLS_GREEN_PIN);
        if (light_state == LIGHT_GREEN)
            GPIO_SetBits(GPIOC, TLS_GREEN_PIN);
        if (light_state == LIGHT_YELLOW)
            GPIO_SetBits(GPIOC, TLS_AMBER_PIN);
        if (light_state == LIGHT_RED)
            GPIO_SetBits(GPIOC, TLS_RED_PIN);

        /* Sleep until next 500ms tick */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAR_MOVE_PERIOD_MS));
    }
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
