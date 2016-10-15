/*******************************************************************************
 *
 * @file Kernel_Internal.h
 *
 * @author Murat Cakmak
 *
 * @brief P-OS Kernel Internal Interface.
 *
 * @see https://github.com/P-LATFORM/P-OS/wiki
 *
 ******************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 P-OS
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *******************************************************************************/
#ifndef __KERNEL_INTERNAL_H
#define __KERNEL_INTERNAL_H

/********************************* INCLUDES ***********************************/
#include "Drv_Timer.h"
#include "Drv_CPUCore.h"

#include "UserStartupInfo.h"

#include "OSConfig.h"
#include "SysConfig.h"

#include "postypes.h"

/***************************** MACRO DEFINITIONS ******************************/

/*
 * Number of Kernel Tasks
 *
 *  - We just have an idle task.
 */
#define NUM_OF_KERNEL_TASKS				(1)

/*
 * Returns Numbers of User Tasks
 */
#define NUM_OF_USER_TASKS \
			(sizeof(startupApplications) / sizeof(void*))

/*
 * Number of all task including kernel and user tasks
 */
#define NUM_OF_ALL_TASKS \
			(NUM_OF_KERNEL_TASKS + NUM_OF_USER_TASKS)

/*
 * Size of idle stack.
 *
 *  128 byte should be enough
 *  TODO : Lower than 512 causes crash when MPU enabled. Find root cause and
			reduce to sutable size
 */
#define IDLE_TASK_STACK_SIZE		    (128)

/*
 * Idle Task Priority
 *  Lower Priority (0)
 */
#define IDLE_TASK_PRIORITY              (0)

/*
 * Kernel Task Creater Definition.
 * There is no difference between kernel and user task so use USER_TASK
 * definition for also kernel task
 */
#define KERNEL_TASK						OS_USER_TASK

/*
 * Start Point Definition (Function Prototype) for Kernel Tasks
 */
#define KERNEL_TASK_START_POINT			OS_USER_TASK_START_POINT

/*
 * Kernel Timer Priority
 *  Kernel Timer is a critical timer so priority selected as High.
 */
#define KERNEL_TIMER_PRIORITY           DRV_TIMER_PRI_HIGH

/*
 * Following defines are just wrapper definitions and covers Driver Layer APIs.
 *  A generic OS architecture should not dependent to external modules
 *  (like Driver or BSP). Therefore we uses wrapper functions to abstract
 *  external modules.
 */

/* Wrapper function definition to initialize CPU */
#define Kernel_InitializeCPU            Drv_CPUCore_Init

/* Wrapper function definition to start context switching */
#define Kernel_StartContextSwitching    Drv_CPUCore_CSStart

/* Wrapper function definition to initialize task stack */
#define Kernel_InitializeTask      		Drv_CPUCore_CSInitializeTask

/* Wrapper function definition to yield running task to */
#define Kernel_SwitchTo                 Drv_CPUCore_CSYieldTo

/* Wrapper function definition to create a Timer */
#define Kernel_CreatePreemptionTimer    Drv_Timer_Create

/* Wrapper function definition to start Timer */
#define Kernel_StartPreemptionTimer     Drv_Timer_Start

/* Wrapper function definitions to get time stamp */
#define Kernel_GetPreemptionTimeStamp   Drv_Timer_ReadElapsedTimeInUs

/********************************* VARIABLES *******************************/

/*
 * User Space Applications which are started at startup
 * (after system initialization)
 *
 * We use following user-defined container to access user applications
 */
extern void* startupApplications[];

/***************************** TYPE DEFINITIONS *******************************/
/*
 * Wrapper Timer Handle definition to abstract external definition in kernel.
 */
typedef TimerHandle KernelTimerHandle;

/*
 * Base type for User Task
 *
 *  Each static user task implements its type implicitly using OS_USER_TASK()
 *  macro and specifies task specific features. e.g stack size.
 *  We can handle task specific differencies using this base type.
 */
typedef struct
{
	/*
	 * Symbolic Symbol for Stack.
	 */
	//uint8_t stack[0];

	/*
	 * Stack size of User Task
	 */
	uint32_t stackSize;
	/*
	 * User Task start point.
	 *
	 * User task starts with that point (function).
	 * Program Counter (PC) is set to this value when task is started.
	 */
	OSUserTaskStartPoint taskStartPoint;
    /*
     * User Task Priority
     */
    uint32_t priority;

} UserAppBaseType;

/*
 * User Application
 */
typedef struct
{
	/* TCB of User Application */
	TCB tcb;
	/* User defined information of Application */
	UserAppBaseType* info;
} UserApp;
/*************************** FUNCTION DEFINITIONS *****************************/

#endif	/* __KERNEL_INTERNAL_H */
