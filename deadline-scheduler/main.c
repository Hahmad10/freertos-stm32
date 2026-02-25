// Deadline-Driven Scheduler (EDF) on FreeRTOS
//
//
// Overview:
// 1. Generator creates user tasks periodically and tells the DDS about them
// 2. DDS keeps a sorted list (earliest deadline first) and gives the
//     top task the highest priority so FreeRTOS runs it
// 3. When a task finishes, it tells the DDS, which moves it to the
//     completed list and promotes the next task
// 4. If a task misses its deadline, a timer fires and the DDS moves
//     it to the overdue list
// 5. Monitor periodically prints how many tasks are active/completed/overdue
//
// PRIORITY TRICK (how EDF works on a fixed-priority RTOS):
// DDS=3 (highest, always preempts everything for scheduling decisions)
// AUX=2 (generator + monitor, run when DDS is idle)
// RUN=1 (only ONE user task has this at a time -- the earliest deadline)
// PAUSE=0 (all other user tasks sit here, never get CPU)

#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"

#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

//------------------------------------------------------------------
// These need to be 1 for FreeRTOS features we use
//------------------------------------------------------------------
#define INCLUDE_vTaskSuspend             1
#define INCLUDE_vTaskDelete              1
#define configUSE_TIMERS                 1
#define configSUPPORT_DYNAMIC_ALLOCATION 1

//------------------------------------------------------------------
// Priority levels (higher number = runs first)
// DDS = 3 (always runs first when a message arrives)
// AUX = 2 (generator and monitor - run when no DDS work)
// RUN = 1 (the ONE user task that should be executing)
// PAUSE = 0 (all other user tasks wait here)
//------------------------------------------------------------------
#define PAUSE_PRIO 0
#define RUN_PRIO   1
#define AUX_PRIO   2
#define DDS_PRIO   3

#define QUEUE_LEN        10
#define MONITOR_INTERVAL 100 // monitor prints every 100ms


//==================================================================
// UTILITY FUNCTIONS
//==================================================================

// Returns current time in ms (read from the system clock timer)
uint32_t getCurrentTime(void) {
    return (uint32_t)pvTimerGetTimerID(systemClockTimer);
}

// Busy-wait for exactly 'ms' milliseconds.
// NOT the same as vTaskDelay - this keeps the CPU busy (simulates real work).
void delayMS(uint32_t ms) {
    uint32_t startTick;
    for (; ms > 0; ms--) {
        startTick = xTaskGetTickCount();
        while ((xTaskGetTickCount() - startTick) == 0)
            ;
    }
}

static void setupHardware(void) {
    NVIC_SetPriorityGrouping(0);
}

//==================================================================
// FREERTOS HOOKS - specific function names FreeRTOS calls automatically
// if enabled in FreeRTOSConfig.h. We implement them, FreeRTOS calls them.
//==================================================================

// Called when pvPortMalloc returns NULL (out of heap).
void vApplicationMallocFailedHook(void) {
    printf("SYSTEM OUT OF MEMORY");
    for (;;)
        ;
}

// Called when a task's stack usage exceeds its allocated stack.
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
    (void)pxTask;
    (void)pcTaskName;
    for (;;)
        ;
}

// Runs whenever NO task is ready (CPU is idle). Could put CPU in low-power sleep.
void vApplicationIdleHook(void) {}
