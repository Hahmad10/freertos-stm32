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

//------------------------------------------------------------------
// DATA STRUCTURES
//------------------------------------------------------------------

// Task can be periodic (repeating) or aperiodic (one-time)
typedef enum { PERIODIC,
               APERIODIC } taskType;

// All the info we track for each deadline-driven task
// DD-Task = user-defined F-Task + EDF scheduling information
typedef struct ddTask {
    TaskHandle_t taskHandle; // FreeRTOS handle (so we can change priority / delete it)
                             // NOTE: starts as NULL, filled in by insertTask() via xTaskCreate
    TaskFunction_t taskFunc; // pointer to the function (userTask1, userTask2, etc.)
    taskType type;
    char *taskID; // name like "T1", "T2", "T3"
    uint32_t taskNum;
    uint32_t releaseTime;    // when the task was released (ms)
    uint32_t absDeadline;    // when it MUST finish by (ms) -- this is what EDF sorts by
    uint32_t completionTime; // when it actually finished (filled in by removeTask)
} ddTask_t;

// Linked list node - each node holds one task + pointer to next
typedef struct ddTaskNode {
    ddTask_t task;
    struct ddTaskNode *next;
} ddTaskNode_t;

// Message types that get sent to the DDS through the queue
// NOTE: ordering matters! 0-2 are scheduling events (trigger pause+update),
//      3-6 are queries (just return data). Code uses "eventType >= GET_LISTS"
//      to distinguish them efficiently.
typedef enum {
    RELEASE_EVENT,  // 0 - "hey DDS, new task just arrived"
    COMPLETE_EVENT, // 1 - "hey DDS, the running task just finished"
    OVERDUE_EVENT,  // 2 - "hey DDS, deadline timer fired - task is late"
    GET_LISTS,      // 3 - "DDS, send me all 3 lists"
    GET_ACTIVE,     // 4 - "DDS, send me the active list"
    GET_COMPLETE,   // 5 - "DDS, send me the complete list"
    GET_OVERDUE     // 6 - "DDS, send me the overdue list"
} EventType;

// The actual message struct that goes through the queue
typedef struct {
    EventType reqType;
    ddTask_t task; // only meaningful for RELEASE_EVENT; zeroed ({0}) for all others
} DDSMessage_t;

xQueueHandle ddsEventQueue = 0;    // main queue: everyone -> DDS
xQueueHandle ddsResponseQueue = 0; // response queue: DDS -> monitor

SemaphoreHandle_t taskReleaseSemaphore = 0;   // timer gives this to wake generator
SemaphoreHandle_t monitorUpdateSemaphore = 0; // timer gives this to wake monitor

TimerHandle_t systemClockTimer = 0;    // ticks every 1ms, tracks time
TimerHandle_t deadlineCheckTimer = 0;  // one-shot, fires when deadline passes
TimerHandle_t taskGenerationTimer = 0; // periodic, triggers generator
TimerHandle_t monitorReportTimer = 0;  // periodic, triggers monitor

//------------------------------------------------------------------
// FUNCTION PROTOTYPES
//------------------------------------------------------------------
static void setupHardware(void);
void delayMS(uint32_t ms);
uint32_t getCurrentTime(void);

BaseType_t releaseTask(ddTask_t task);
BaseType_t completeTask(void);
void getTaskLists(ddTaskNode_t **active, ddTaskNode_t **overdue, ddTaskNode_t **complete);
void getActiveList(ddTaskNode_t **active);
void getCompleteList(ddTaskNode_t **complete);
void getOverdueList(ddTaskNode_t **overdue);

BaseType_t insertTask(ddTaskNode_t **activeList, ddTask_t newTask);
BaseType_t removeTask(ddTaskNode_t **activeList, ddTaskNode_t **retiredList);
BaseType_t pauseScheduler(ddTaskNode_t **activeList);
BaseType_t updateScheduler(ddTaskNode_t **activeList);

void deadlineSchedulerTask(void *pvParameters);
void taskGeneratorTask(void *pvParameters);
void monitorTask(void *pvParameters);
void userTask1(void *pvParameters);
void userTask2(void *pvParameters);
void userTask3(void *pvParameters);

void deadlineTimerCallback(TimerHandle_t xTimer);
void systemTimerCallback(TimerHandle_t xTimer);
void referenceTimerCallback(TimerHandle_t xTimer);


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
