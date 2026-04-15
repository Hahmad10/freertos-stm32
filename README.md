# FreeRTOS on STM32F4

Two real-time systems I built in C on FreeRTOS for an STM32F4 Discovery board in early 2026.

| Project | What it is |
|---|---|
| [Traffic Light System](traffic-light) | A one-lane road with a traffic light. Four tasks, three queues and a chain of one-shot timers drive 19 car LEDs through shift registers, and a potentiometer sets the traffic flow. |
| [Deadline-Driven Scheduler](deadline-scheduler) | An Earliest-Deadline-First scheduler built on top of FreeRTOS's fixed-priority scheduler by changing task priorities at run time. Tested at 82%, 100% and 101% CPU utilization. |

## Building

Each folder has a single `main.c`. It expects the usual STM32F4 Discovery FreeRTOS project layout:

```
FreeRTOS_Source/   FreeRTOS kernel and the ARM_CM4F port
Libraries/         CMSIS and the STM32F4xx Standard Peripheral Library
Utilities/         STM32F4-Discovery board support
src/main.c         <- this file
```

The kernel and the ST libraries aren't included, since they're third-party code. Drop `main.c` into an existing STM32F4 Discovery FreeRTOS project in TrueSTUDIO or STM32CubeIDE, then build and flash over ST-Link. Output goes to the SWV/ITM console.

## Note on the source

Both files compile cleanly against the STM32F4 and FreeRTOS headers.
