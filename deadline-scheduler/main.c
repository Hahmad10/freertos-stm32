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
// TEST BENCH PARAMETERS
//
// Bench 1: Utilization = 95/500 + 150/500 + 250/750 = 0.82 (schedulable)
// Bench 2: U = 95/250 + 150/500 + 250/750 = 1.01 (overloaded)
// Bench 3: U = 100/500 + 200/500 + 200/500 = 1.00 (borderline)
//------------------------------------------------------------------

// ---- Test Bench 1 ----
//#define TASK_GEN_INTERVAL 250
//#define TASK1_EXEC       95
//#define TASK1_PERIOD     500
//#define TASK2_EXEC       150
//#define TASK2_PERIOD     500
//#define TASK3_EXEC       250
//#define TASK3_PERIOD     750

//// ---- Test Bench 2 ----
//#define TASK_GEN_INTERVAL 250
//#define TASK1_EXEC        95
//#define TASK1_PERIOD      250
//#define TASK2_EXEC        150
//#define TASK2_PERIOD      500
//#define TASK3_EXEC        250
//#define TASK3_PERIOD      750

//// ---- Test Bench 3 ----
#define TASK_GEN_INTERVAL 500
#define TASK1_EXEC        100
#define TASK1_PERIOD      500
#define TASK2_EXEC        200
#define TASK2_PERIOD      500
#define TASK3_EXEC        200
#define TASK3_PERIOD      500

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
// DDS TASK - the brain of the scheduler (HIGHEST PRIORITY = 3)
//
// WHY highest priority? Must preempt ANY user task instantly when an
// event arrives -- otherwise scheduling decisions would be delayed and
// tasks could miss deadlines while waiting for DDS to respond.
//
// Sits in a loop waiting for messages. When one arrives:
// RELEASE -> add new task to active list (sorted by deadline)
// COMPLETE -> move head task to completed list
// OVERDUE -> move head task to overdue list (deadline missed)
// GET_*     -> send requested list back through response queue
//
// Before any scheduling change: pause the running task (priority 0)
// After any scheduling change: promote new head (priority 1) + arm deadline timer
//==================================================================
void deadlineSchedulerTask(void *pvParameters) {
    ddTaskNode_t *activeList = NULL;   // tasks waiting to run (sorted by deadline)
    ddTaskNode_t *overdueList = NULL;  // tasks that missed their deadline
    ddTaskNode_t *completeList = NULL; // tasks that finished on time

    for (;;) {
        DDSMessage_t ddsMessage;

        // Block here until someone sends us a message
        if (!xQueueReceive(ddsEventQueue, &ddsMessage, portMAX_DELAY))
            continue;

        EventType eventType = ddsMessage.reqType;

        // enum values 0-2 = scheduling events, 3+ = queries.
        // Queries just return data; scheduling events need pause+update cycle.
        if (eventType >= GET_LISTS) {
            switch (eventType) {
                case GET_LISTS:
                    xQueueSend(ddsResponseQueue, &activeList, 0);
                    xQueueSend(ddsResponseQueue, &completeList, 0);
                    xQueueSend(ddsResponseQueue, &overdueList, 0);
                    break;
                case GET_ACTIVE:
                    xQueueSend(ddsResponseQueue, &activeList, 0);
                    break;
                case GET_COMPLETE:
                    xQueueSend(ddsResponseQueue, &completeList, 0);
                    break;
                case GET_OVERDUE:
                    xQueueSend(ddsResponseQueue, &overdueList, 0);
                    break;
                default:
                    break;
            }
        } else {
            // SCHEDULING EVENT: "freeze" the running task first (drop to priority 0)
            // so it can't interfere while we rearrange the active list.
            pauseScheduler(&activeList);

            switch (eventType) {
                case RELEASE_EVENT:
                    // New task arrived - insert it in the right spot (sorted by deadline)
                    insertTask(&activeList, ddsMessage.task);
                    break;

                case COMPLETE_EVENT:
                    // Running task finished - move from active to completed
                    removeTask(&activeList, &completeList);
                    break;

                case OVERDUE_EVENT:
                    // Deadline timer fired - if head task is actually past deadline, move to overdue
                    if (activeList && activeList->task.absDeadline < getCurrentTime())
                        removeTask(&activeList, &overdueList);
                    break;

                default:
                    break;
            }

            // Now promote the new head task and re-arm the deadline timer
            updateScheduler(&activeList);
        }

        // MEMORY MANAGEMENT: each ddTaskNode_t is ~52 bytes. Completed nodes stay
        // in completeList so the monitor can count them. We defer freeing until heap
        // pressure because the monitor needs the data. 5000 = enough buffer to still
        // allocate new tasks + stacks while cleanup runs.
        if (xPortGetFreeHeapSize() < 5000) {
            printf("Memory Full\n");
            ddTaskNode_t *freedNode;
            while (completeList != NULL) {
                freedNode = completeList;
                completeList = completeList->next;
                vPortFree(freedNode);
            }
        }
    }
}

//==================================================================
// INSERT TASK - add a new task to the active list in deadline order
//
// Walks the list to find where the new task belongs (earlier deadline = closer to head).
// Also creates the actual FreeRTOS task at priority 0 (paused).
// updateScheduler() will set the head to priority 1 after this returns.
//==================================================================
BaseType_t insertTask(ddTaskNode_t **activeList, ddTask_t newTask) {
    if (*activeList == NULL) {
        // List is empty - new task becomes the head
        *activeList = (ddTaskNode_t *)pvPortMalloc(sizeof(ddTaskNode_t));
        (*activeList)->task = newTask;
        (*activeList)->next = NULL;
        // Create FreeRTOS task at priority 0 -- intentionally paused.
        // We don't want it running yet; DDS must finish scheduling first.
        // updateScheduler() will promote the head to priority 1 after this returns.
        xTaskCreate(newTask.taskFunc, newTask.taskID, configMINIMAL_STACK_SIZE,
                    NULL, 0, &((*activeList)->task.taskHandle));
    } else {
        // Walk the list to find the right position (sorted by deadline, earliest first).
        // The >= means same-deadline tasks keep insertion order (FIFO tiebreaker).
        // This matters in TB3 where all deadlines are equal -- T1 runs first because
        // the generator releases it first.
        ddTaskNode_t *currentNode = *activeList;
        ddTaskNode_t *previousNode = NULL;
        while (currentNode != NULL && newTask.absDeadline >= currentNode->task.absDeadline) {
            previousNode = currentNode;
            currentNode = currentNode->next;
        }
        // pvPortMalloc = FreeRTOS's malloc (deterministic timing, managed heap)
        ddTaskNode_t *newNode = (ddTaskNode_t *)pvPortMalloc(sizeof(ddTaskNode_t));
        newNode->task = newTask;
        newNode->next = currentNode;
        xTaskCreate(newTask.taskFunc, newTask.taskID, configMINIMAL_STACK_SIZE,
                    NULL, 0, &newNode->task.taskHandle);
        // Link it in
        if (previousNode != NULL)
            previousNode->next = newNode; // insert in middle or end
        else
            *activeList = newNode; // new earliest deadline = new head
    }
    return pdTRUE;
}

//==================================================================
// REMOVE TASK - take the HEAD off the active list
//
// WHY always head? In EDF, the running task = earliest deadline = head
// of the sorted list. On completion or overdue, it's always the head.
//
// The removed node gets prepended (O(1)) to whichever "retired" list
// you pass in. Node is NOT freed here -- stays for monitor to count.
// Freed later by DDS when heap runs low.
//==================================================================
BaseType_t removeTask(ddTaskNode_t **activeList, ddTaskNode_t **retiredList) {
    if (*activeList == NULL)
        return pdFALSE;

    ddTaskNode_t *currentNode = *activeList;
    *activeList = (*activeList)->next; // advance the list

    currentNode->task.completionTime = getCurrentTime();

    // This printf gives the release/completion timeline used to check EDF ordering
    printf("Task ID: %s, Release: %d, Complete: %d\n",
           currentNode->task.taskID,
           currentNode->task.releaseTime,
           currentNode->task.completionTime);

    // Kill the FreeRTOS task (it's sitting in for(;;) after completeTask, waiting for this)
    if (currentNode->task.taskHandle != NULL)
        vTaskDelete(currentNode->task.taskHandle);

    // Prepend to retired list (O(1) -- order doesn't matter for record-keeping)
    currentNode->next = *retiredList;
    *retiredList = currentNode;
    return pdTRUE;
}

//==================================================================
// PAUSE SCHEDULER - set the running task's priority to 0
// Called before any scheduling change so the running task doesn't
// interfere while we rearrange the list.
//==================================================================
BaseType_t pauseScheduler(ddTaskNode_t **activeList) {
    if (*activeList != NULL) {
        vTaskPrioritySet((*activeList)->task.taskHandle, PAUSE_PRIO);
        return pdTRUE;
    }
    return pdFALSE;
}

//==================================================================
// UPDATE SCHEDULER - promote the head task and arm the deadline timer
//
// Sets head of active list to RUN_PRIO (1) so FreeRTOS runs it.
// Arms the deadline timer to fire when this task's deadline arrives.
// If the deadline already passed, fire immediately (period = 1 tick).
//==================================================================
BaseType_t updateScheduler(ddTaskNode_t **activeList) {
    if (*activeList != NULL) {
        // >>> THIS is the line that makes tasks actually RUN <<<
        // Setting priority to 1 lets FreeRTOS schedule it when DDS yields.
        vTaskPrioritySet((*activeList)->task.taskHandle, RUN_PRIO);

        // Re-arm deadline timer for the NEW head task's deadline.
        // xTimerChangePeriod sets the duration, xTimerReset starts it from zero.
        uint32_t now = getCurrentTime();
        if ((*activeList)->task.absDeadline <= now) {
            xTimerChangePeriod(deadlineCheckTimer, 1, 0); // already late, fire now -> OVERDUE
        } else {
            xTimerChangePeriod(deadlineCheckTimer,
                               pdMS_TO_TICKS((*activeList)->task.absDeadline - now), 0);
        }
        xTimerReset(deadlineCheckTimer, 0);
        return pdTRUE;
    } else {
        // No tasks left - stop the deadline timer
        xTimerStop(deadlineCheckTimer, 0);
        return pdFALSE;
    }
}

//==================================================================
// TASK GENERATOR - creates new tasks at the right times
//
// Wakes up every TASK_GEN_INTERVAL ms (via semaphore from timer).
// Checks each task: if enough time has passed (elapsed % period == 0),
// it's time to release a new instance of that task.
// Builds a ddTask_t struct and sends it to the DDS via releaseTask().
//==================================================================
void taskGeneratorTask(void *pvParameters) {
    // Elapsed counters track the IDEAL schedule (not real time).
    // Using modulo on clean counters avoids drift from real-time checks.
    uint32_t task1Elapsed = 0, task2Elapsed = 0, task3Elapsed = 0;
    uint32_t delta = TASK_GEN_INTERVAL;   // each invocation = one interval passed
    xSemaphoreGive(taskReleaseSemaphore); // self-trigger so tasks release at t=0

    for (;;) {
        if (xSemaphoreTake(taskReleaseSemaphore, portMAX_DELAY) == pdTRUE) {
            uint32_t now = getCurrentTime();

            // Check each task: is it time to release a new instance?
            // Deadline = elapsed + period (NOT now + period) -- aligns with theoretical schedule.
            if ((task1Elapsed % TASK1_PERIOD) == 0) {
                ddTask_t t = {NULL, userTask1, PERIODIC, "T1", 1, now, task1Elapsed + TASK1_PERIOD, 0};
                releaseTask(t);
            }
            if ((task2Elapsed % TASK2_PERIOD) == 0) {
                ddTask_t t = {NULL, userTask2, PERIODIC, "T2", 2, now, task2Elapsed + TASK2_PERIOD, 0};
                releaseTask(t);
            }
            if ((task3Elapsed % TASK3_PERIOD) == 0) {
                ddTask_t t = {NULL, userTask3, PERIODIC, "T3", 3, now, task3Elapsed + TASK3_PERIOD, 0};
                releaseTask(t);
            }

            task1Elapsed += delta;
            task2Elapsed += delta;
            task3Elapsed += delta;
        }
    }
}

//==================================================================
// MONITOR TASK - prints system status periodically
//
// Wakes up every MONITOR_INTERVAL ms (via semaphore from timer).
// Asks DDS for the 3 lists, counts how many tasks are in each,
// and prints the result.
//==================================================================
void monitorTask(void *pvParameters) {
    ddTaskNode_t *taskLists[3] = {NULL, NULL, NULL};
    int taskCounts[3] = {0, 0, 0};

    for (;;) {
        if (xSemaphoreTake(monitorUpdateSemaphore, portMAX_DELAY) == pdTRUE) {
            // Ask DDS for each list
            getActiveList(&taskLists[0]);
            getCompleteList(&taskLists[1]);
            getOverdueList(&taskLists[2]);

            // Count nodes in each list
            for (int i = 0; i < 3; i++) {
                taskCounts[i] = 0;
                ddTaskNode_t *node = taskLists[i];
                while (node != NULL) {
                    taskCounts[i]++;
                    node = node->next;
                }
            }

            printf("Time = %u | Active = %d | Complete = %d | Overdue = %d\n\n",
                   (unsigned int)getCurrentTime(), taskCounts[0], taskCounts[1], taskCounts[2]);
        }
    }
}

//==================================================================
// DDS INTERFACE FUNCTIONS
// These are called by other tasks to communicate with the DDS.
// They just package a message and send it through the queue.
//==================================================================

// Tell DDS a new task needs to be scheduled
BaseType_t releaseTask(ddTask_t task) {
    DDSMessage_t msg = {RELEASE_EVENT, task};
    return xQueueSend(ddsEventQueue, &msg, portMAX_DELAY);
}

// Tell DDS the running task just finished
BaseType_t completeTask(void) {
    DDSMessage_t msg = {COMPLETE_EVENT, {0}};
    return xQueueSend(ddsEventQueue, &msg, portMAX_DELAY);
}

// Request-response pattern: send GET_* to DDS, block until DDS responds with pointer.
// xQueueReset clears stale data from a previous interrupted request.
void getActiveList(ddTaskNode_t **active) {
    if (uxQueueSpacesAvailable(ddsResponseQueue) < QUEUE_LEN)
        xQueueReset(ddsResponseQueue);
    DDSMessage_t req = {GET_ACTIVE, {0}};
    xQueueSend(ddsEventQueue, &req, portMAX_DELAY);
    xQueueReceive(ddsResponseQueue, active, portMAX_DELAY);
}

void getCompleteList(ddTaskNode_t **complete) {
    if (uxQueueSpacesAvailable(ddsResponseQueue) < QUEUE_LEN)
        xQueueReset(ddsResponseQueue);
    DDSMessage_t req = {GET_COMPLETE, {0}};
    xQueueSend(ddsEventQueue, &req, portMAX_DELAY);
    xQueueReceive(ddsResponseQueue, complete, portMAX_DELAY);
}

void getOverdueList(ddTaskNode_t **overdue) {
    if (uxQueueSpacesAvailable(ddsResponseQueue) < QUEUE_LEN)
        xQueueReset(ddsResponseQueue);
    DDSMessage_t req = {GET_OVERDUE, {0}};
    xQueueSend(ddsEventQueue, &req, portMAX_DELAY);
    xQueueReceive(ddsResponseQueue, overdue, portMAX_DELAY);
}

void getTaskLists(ddTaskNode_t **active, ddTaskNode_t **overdue, ddTaskNode_t **complete) {
    if (uxQueueSpacesAvailable(ddsResponseQueue) < QUEUE_LEN)
        xQueueReset(ddsResponseQueue);
    DDSMessage_t req = {GET_LISTS, {0}};
    xQueueSend(ddsEventQueue, &req, portMAX_DELAY);
    xQueueReceive(ddsResponseQueue, active, portMAX_DELAY);
    xQueueReceive(ddsResponseQueue, overdue, portMAX_DELAY);
    xQueueReceive(ddsResponseQueue, complete, portMAX_DELAY);
}

//==================================================================
// USER TASKS - simulate real work by busy-waiting
//
// delayMS = busy-wait (keeps CPU occupied, simulates real computation).
// NOT vTaskDelay -- that would yield the CPU and not model real workload.
// completeTask = sends COMPLETE_EVENT to DDS.
// for(;;) = can't return from a FreeRTOS task (would crash). Spins at
// low priority until DDS processes the completion and calls vTaskDelete.
//==================================================================
void userTask1(void *pvParameters) {
    delayMS(TASK1_EXEC);
    completeTask();
    for (;;)
        ; // DDS will delete this task
}

void userTask2(void *pvParameters) {
    delayMS(TASK2_EXEC);
    completeTask();
    for (;;)
        ;
}

void userTask3(void *pvParameters) {
    delayMS(TASK3_EXEC);
    completeTask();
    for (;;)
        ;
}

//==================================================================
// TIMER CALLBACKS
//==================================================================

// Deadline timer fired = task may have missed its deadline.
// "ToFront" = overdue events jump ahead of pending releases (they're urgent).
// "FromISR" = timer callbacks run in ISR-like context; blocking calls are illegal.
void deadlineTimerCallback(TimerHandle_t xTimer) {
    DDSMessage_t msg = {OVERDUE_EVENT, {0}};
    BaseType_t woken = pdFALSE;
    xQueueSendToFrontFromISR(ddsEventQueue, &msg, &woken);
    // If DDS (higher priority) woke up, yield to it immediately
    if (woken)
        taskYIELD();
}

// System clock: time stored in the timer's ID field (no global variable needed).
// pvTimerGetTimerID reads it, vTimerSetTimerID writes it.
void systemTimerCallback(TimerHandle_t xTimer) {
    uint32_t t = (uint32_t)pvTimerGetTimerID(xTimer);
    vTimerSetTimerID(xTimer, (void *)(t + 1));
}

// Shared callback -- both timers just give a semaphore, no need for separate callbacks.
// TimerID 2 = generator timer, TimerID 3 = monitor timer.
// xSemaphoreGiveFromISR = ISR-safe (non-blocking). taskYIELD = if the woken task has
// higher priority than whatever was running, switch to it now (don't wait for next tick).
void referenceTimerCallback(TimerHandle_t xTimer) {
    BaseType_t woken = pdFALSE;
    uint32_t id = (uint32_t)pvTimerGetTimerID(xTimer);
    if (id == 2)
        xSemaphoreGiveFromISR(taskReleaseSemaphore, &woken);
    else
        xSemaphoreGiveFromISR(monitorUpdateSemaphore, &woken);
    if (woken)
        taskYIELD();
}

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
