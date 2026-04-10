# Deadline-Driven Scheduler (EDF)

An Earliest-Deadline-First scheduler built on top of FreeRTOS, running on an STM32F4 Discovery board.

FreeRTOS only knows fixed priorities, so the scheduler runs as the highest-priority task. It keeps every released job in a list sorted by deadline, then gives the job with the earliest deadline the run priority and parks all the others at priority 0.

## How it works

**Priorities:**

| Level | Priority | Who |
|---|---|---|
| DDS | 3 | The scheduler task. It always preempts so it can react to events right away. |
| AUX | 2 | The task generator and the monitor |
| RUN | 1 | Exactly one user task: the one with the earliest deadline |
| PAUSE | 0 | Every other user task |

**Events:** everything talks to the scheduler through one event queue, with a response queue for list queries. The event types are:
- Release
- Complete
- Overdue
- Get-lists

**Insertion:** a new job is placed into the active list by deadline. The code handles four cases: empty list, new head, middle of the list and end of the list.

**Deadline timer:** a one-shot timer is re-armed for the head job's deadline. If it fires, that job is moved to the overdue list.

**Memory:** job nodes are allocated with `pvPortMalloc`. Completed nodes are freed in bulk when free heap drops below 5 KB, so the system can run indefinitely.

## Results

Measured on hardware over one hyperperiod:

| Test bench | Utilization | Released | Completed | Overdue | Miss rate |
|---|---|---|---|---|---|
| 1 | 82.3% | 8 | 8 | 0 | 0% |
| 2 | 101.3% | 11 | 10 | 1 | 9.1% |
| 3 | 100.0% | 3 | 2 | 1 | 33.3% |

- **Test bench 1:** completion times landed within 1 to 8 ms of the hand-computed EDF schedule.
- **Test bench 2:** the misses match the theory. The workload is over 100% by 20 ms per hyperperiod, so exactly one job misses.
- **Test bench 3:** this one is the interesting result. At exactly 100% utilization, EDF is schedulable on paper. In practice, scheduler overhead pushes the last task 1 ms past its deadline every period. The overhead comes from:
  - context switches
  - queue operations
  - the monitor task
  - `printf`

Switch between test benches by uncommenting the matching block of `#define`s near the top of `main.c`. Test bench 3 is the one currently enabled.
