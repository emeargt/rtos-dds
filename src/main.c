/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-----------------------------------------------------------*/
/* Defines */
#define TRUE 1
#define FALSE 0
#define BASIC_VIEW 1

#define mainQUEUE_LENGTH 100

#define TASK1_PERIOD 500 // in ms
#define TASK2_PERIOD 500 // in ms
#define TASK3_PERIOD 750 // in ms

#define TASK1_EXE_TIME 95 // in ms
#define TASK2_EXE_TIME 150 // in ms
#define TASK3_EXE_TIME 250 // in ms

#define SET_PRIORITY 1
#define RST_PRIORITY 0

typedef enum {PERIODIC, APERIODIC} task_type; // task_type may be PERIODIC or APERIODIC

typedef enum
{
	ACTIVE = 0,
	COMPLETE = 1,
	OVERDUE = 2
} list_type;

// struct for storing dd task parameters
struct dd_task {

	TaskHandle_t t_handle;
	task_type type;
	uint32_t task_id;
	uint32_t release_time; // release time in ticks
	uint32_t absolute_deadline; // absolute deadline in ticks
	uint32_t completion_time; // completion time in ticks

};

// struct for building linked dd task lists
struct dd_task_node {

	struct dd_task task;	// dd_task
	struct dd_task_node* next_task_p; // pointer to next task_node

};

struct task_info {

	uint32_t id;
	task_type type;
	uint32_t period;
	uint32_t execution_time;
};

/* Hardware setup functions */
static void prvSetupHardware(void);

/* F-Tasks */
// DDS
static void Deadline_Driven_Scheduler(void *pvParameters);

// Deadline Driven Task Generator
static void Task_Generator(void *pvParameters);

// Monitor task
static void Monitor_Task(void *pvParameters);

// User defined tasks
static void User_Task(void *pvParameters); // pass wait time with pvParameters

/* Deadline-driven functions
 * Interfaces to DDS
 */

void release_dd_task(TaskHandle_t t_handle, task_type type, uint32_t task_id, uint32_t absolute_deadline);
void complete_dd_task(uint32_t task_id);

// Get task lists
struct dd_task_node** get_active_dd_task_list(void);
struct dd_task_node** get_complete_dd_task_list(void);
struct dd_task_node** get_overdue_dd_task_list(void);

// sorting tasks
void swap(struct dd_task_node* a, struct dd_task_node* b);
void sort(struct dd_task_node** head);
void push(struct dd_task_node** head, struct dd_task* new_task_p);

void print_task_list(struct dd_task_node** head);

/* Queue handles */
xQueueHandle xQ_new_tasks = 0;
xQueueHandle xQ_release = 0;
xQueueHandle xQ_complete = 0;
xQueueHandle xQ_list_request = 0;
xQueueHandle xQ_list_mailbox = 0;

/* Software Timer handles */
TimerHandle_t task1_timer = 0;
TimerHandle_t task2_timer = 0;
TimerHandle_t task3_timer = 0;

// Timer callback
void vTimerCallback(TimerHandle_t xtimer);

/* F-Task handles */
TaskHandle_t h_dds = 0;
TaskHandle_t h_task_generator = 0;
TaskHandle_t h_monitor = 0;

int main(void)
{
	// Configure system
	prvSetupHardware();

	// Create queues
	xQ_new_tasks = xQueueCreate(mainQUEUE_LENGTH, sizeof(struct task_info)); // takes a number identifying which user task to generate
	xQ_release = xQueueCreate(mainQUEUE_LENGTH, sizeof(struct dd_task)); // holds new dd_task to release by dds
	xQ_complete = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t)); // holds task id
	xQ_list_request = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t)); // holds list_type
	xQ_list_mailbox = xQueueCreate(3, sizeof(struct dd_task_node**));

	// Add to the registry, for the benefit of kernel aware debugging
	vQueueAddToRegistry(xQ_new_tasks, "NewTasksQ");
	vQueueAddToRegistry(xQ_release, "ReleaseQ");
	vQueueAddToRegistry(xQ_complete, "CompleteQ");
	vQueueAddToRegistry(xQ_list_request, "ListRequestQ");
	vQueueAddToRegistry(xQ_list_mailbox, "ListMailboxQ");

	// Create timers used for task generation notification
	task1_timer = xTimerCreate("Task1", pdMS_TO_TICKS(TASK1_PERIOD), pdTRUE, (void*)0, vTimerCallback);
	task2_timer = xTimerCreate("Task2", pdMS_TO_TICKS(TASK2_PERIOD), pdTRUE, (void*)0, vTimerCallback);
	task3_timer = xTimerCreate("Task3", pdMS_TO_TICKS(TASK3_PERIOD), pdTRUE, (void*)0, vTimerCallback);

	xTaskCreate(Deadline_Driven_Scheduler, "DDS", configMINIMAL_STACK_SIZE, NULL, 5, &h_dds);
	xTaskCreate(Task_Generator, "TaskGenerator", configMINIMAL_STACK_SIZE, NULL, 3, &h_task_generator);
	//xTaskCreate(Monitor_Task, "Monitor", configMINIMAL_STACK_SIZE, NULL, 2, &h_monitor);

	// Start timers
	xTimerStart(task1_timer, 0);
	xTimerStart(task2_timer, 0);
	xTimerStart(task3_timer, 0);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

/*-----------------------------------------------------------*/

// function for pushing new dd_task_node to the top of the list
void push(struct dd_task_node** head, struct dd_task* new_task_p)
{
	struct dd_task_node* newNode = (struct dd_task_node*)pvPortMalloc(sizeof(struct dd_task_node));
	newNode->task = *new_task_p;
	newNode->next_task_p = *head;
	*head = newNode;
}

// function for swapping dd_task pointers between two dd_task_nodes. Called after comparison of dd_task priority
void swap(struct dd_task_node* a, struct dd_task_node* b)
{
	struct dd_task temp_task = a->task;
	a->task = b->task;
	b->task = temp_task;

}


void sort(struct dd_task_node** head)
{

	int swapped; // flag for indicating swap took place

	struct dd_task_node* ptr1; // temporary pointer to for list node
	struct dd_task_node* lptr = NULL;

	/* Checking for empty list */
	if (*head == NULL)
		return;

	do
	{
		swapped = FALSE;
		ptr1 = *head;

		while (ptr1->next_task_p != lptr)
		{
			if (ptr1->task.absolute_deadline > (ptr1->next_task_p)->task.absolute_deadline)
			{
				swap(ptr1, ptr1->next_task_p);
				swapped = TRUE;
			}
			ptr1 = ptr1->next_task_p;
		}
		lptr = ptr1;
	}
	while (swapped);
}

/*-----------------------------------------------------------*/

void release_dd_task(TaskHandle_t t_handle, task_type type, uint32_t task_id, uint32_t absolute_deadline)
{
	struct dd_task new_dd_task;
	new_dd_task.t_handle = t_handle;
	new_dd_task.type = type;
	new_dd_task.task_id = task_id;
	new_dd_task.absolute_deadline = absolute_deadline;

	if(xQueueSend(xQ_release, &new_dd_task, 0)) // send new_task to new task Queue
	{
		vTaskResume(h_dds);
	}
	else
	{
		printf("Failed to send Task%u to release queue.\n", (unsigned int)new_dd_task.task_id);
	}
}

void complete_dd_task(uint32_t task_id)
{
	if(xQueueSend(xQ_complete, &task_id, 0))
	{
		vTaskResume(h_dds);
	}
	else
	{
		printf("Failed to send Task%u to complete queue.\n", (unsigned int)task_id);
	}
}


struct dd_task_node** get_active_dd_task_list(void)
{
	struct dd_task_node** head = NULL;
	list_type list = ACTIVE;
	// request current active task list
	if(xQueueSend(xQ_list_request, &list, 0))
	{
		vTaskResume(h_dds);
	}
	if(!xQueueReceive(xQ_list_mailbox, &head, 1000))
	{
		printf("Failed to receive active task list.\n");
	}
	return head;
}


struct dd_task_node** get_complete_dd_task_list(void)
{
	struct dd_task_node** head = NULL;
	list_type list = COMPLETE;
	// request completed task list
	if(xQueueSend(xQ_list_request, &list, 0))
	{
		vTaskResume(h_dds);
	}
	if(!xQueueReceive(xQ_list_mailbox, &head, 1000))
	{
		printf("Failed to receive completed task list.\n");
	}
	return head;
}

struct dd_task_node** get_overdue_dd_task_list(void)
{
	struct dd_task_node** head = NULL;
	list_type list = OVERDUE;
	// request completed task list
	if(xQueueSend(xQ_list_request, &list, 0))
	{
		vTaskResume(h_dds);
	}
	if(!xQueueReceive(xQ_list_mailbox, &head, 1000))
	{
		printf("Failed to receive overdue task list.\n");
	}
	return head;
}

void print_task_list(struct dd_task_node** head)
{
	struct dd_task_node* temp_dd_task_node = *head;
	int i = 1;

	do
	{

	#ifdef BASIC_VIEW
		i++;
	#endif

	#ifdef ADVANCED_VIEW
		printf("#%-3d \t", i);
		printf("Task ID: %d \n", &(temp_dd_task_node->task_p->task_id));
		//keep listing task parameters as desired
		i++;
	#endif

	temp_dd_task_node = temp_dd_task_node->next_task_p;

	}
	while(temp_dd_task_node->next_task_p != NULL);

	printf("Number of tasks in list: %d \n", (i-1));

}

/*-----------------------------------------------------------*/

static void Deadline_Driven_Scheduler(void *pvParameters)
{
	struct dd_task_node *active_head = NULL;
	struct dd_task_node *complete_head = NULL;
	struct dd_task_node *overdue_head = NULL;
	while(1)
	{
		vTaskSuspend(h_dds);
		// if list requested
		list_type list_req;
		if(xQueueReceive(xQ_list_request, &list_req, 0))
		{
			switch(list_req)
			{
			case ACTIVE:
				xQueueSend(xQ_list_mailbox, &active_head, 0);
				break;
			case COMPLETE:
				xQueueSend(xQ_list_mailbox, &complete_head, 0);
				break;
			case OVERDUE:
				xQueueSend(xQ_list_mailbox, &overdue_head, 0);
				break;
			}
		}
		else
		{
			struct dd_task to_release;
			uint32_t complete_id;
			TickType_t cur_time = xTaskGetTickCount();
			if(xQueueReceive(xQ_release, &to_release, 0))
			{
				// get and assign release time
				to_release.release_time = cur_time; // set current tick count as release time
				// add new task to active task list
				push(&active_head, &to_release);
			}
			// if completed task
			if(xQueueReceive(xQ_complete, &complete_id, 0))
			{
				if(complete_id == active_head->task.task_id)
				{
					// pop completed task from active list

					// assign completion time to task

					// delete associated f-task to free up memory

					// check cur_time against deadline

				}
			}
			// sort active task list
			sort(&active_head);
			// reset task priorities
			struct dd_task_node *temp = active_head;
			while(temp != NULL)
			{
				vTaskPrioritySet(temp->task.t_handle, RST_PRIORITY);
				temp = temp->next_task_p;
			}
			// schedule earliest deadline task by setting priority
			// head of active task list
			vTaskPrioritySet(active_head->task.t_handle, SET_PRIORITY);
		}
	}
}

static void Task_Generator(void *pvParameters)
{
	struct task_info new_task;
	uint32_t id = 0;
	while(1)
	{
		vTaskSuspend(h_task_generator);
		while(uxQueueMessagesWaiting(xQ_new_tasks))
		{
			if(xQueueReceive(xQ_new_tasks, &new_task, 0))
			{
				char name[20];
				sprintf(name, "Task%u", (unsigned int)id);
				new_task.id = id;
				TaskHandle_t h_new = 0;
				uint32_t abs_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(new_task.period);
				xTaskCreate(User_Task, name, configMINIMAL_STACK_SIZE, (void*)new_task, 0, &h_new);
				release_dd_task(h_new, new_task.type, id, abs_deadline);
				++id;
			}
		}
	}
}

static void Monitor_Task(void *pvParameters)
{
	while(1)
	{
		vTaskSuspend(h_monitor);

		//struct dd_task_list** temp;

		printf("Active DD Task List Monitor Data: \n");
		print_task_list(get_active_dd_task_list());

		printf("Complete DD Task List Monitor Data: \n");
		print_task_list(get_complete_dd_task_list());

		printf("Overdue DD Task List Monitor Data: \n");
		print_task_list(get_overdue_dd_task_list());

	}
}

static void User_Task(void *pvParameters)
{
	while(1)
	{
		struct task_info* task = (struct task_info*)pvParameters;
		// delay for user task execution time
		TickType_t delay_ticks = xTaskGetTickCount() + pdMS_TO_TICKS(task->execution_time);
		while(xTaskGetTickCount() < delay_ticks);
		complete_dd_task(task->id);
	}
}

/*-----------------------------------------------------------*/

void vTimerCallback(TimerHandle_t xtimer)
{
	struct task_info new_task;
	if(xtimer == task1_timer)
	{
		new_task.type = PERIODIC;
		new_task.period = TASK1_PERIOD;
		new_task.execution_time = TASK1_EXE_TIME;
	}
	else if(xtimer == task2_timer)
	{
		new_task.type = PERIODIC;
		new_task.period = TASK2_PERIOD;
		new_task.execution_time = TASK2_EXE_TIME;
	}
	else if(xtimer == task3_timer)
	{
		new_task.type = PERIODIC;
		new_task.period = TASK3_PERIOD;
		new_task.execution_time = TASK3_EXE_TIME;
	}
	if(xQueueSend(xQ_new_tasks, &new_task, 0))
	{
		vTaskResume(h_task_generator);
	}
	else
	{
		printf("Timer Callback: Failed to add Task%u to queue.\n", (unsigned int)new_task.task_num);
	}
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq (&RCC_Clocks);
	(void) SysTick_Config (RCC_Clocks.HCLK_Frequency / 1000);

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}
