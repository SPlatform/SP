/*******************************************************************************
 *
 * @file Drv_CPUCore.h
 *
 * @author Murat Cakmak
 *
 * @brief CPU Core Driver Interface
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
#ifndef __DRV_CPUCORE_H
#define __DRV_CPUCORE_H

/********************************* INCLUDES ***********************************/
#include "postypes.h"

/***************************** MACRO DEFINITIONS ******************************/

/***************************** TYPE DEFINITIONS *******************************/
typedef void(*Drv_CPUCore_TaskStartPoint)(void* arg);

/*
 * Task Control Block (TCB)
 *
 *  Includes all task specific information including required information for
 *	Context Switching.
 */
typedef struct TCB
{
	/*
	 * Actual top address of stack of task.
	 *
	 *  When a user task is started, task starts to use stack from end of
	 *  stack in descending order.
	 *  When a task preempted, we need to save actual position of stack to
	 *  backup next execution of task.
	 *
	 *	[IMP] This value must be first item in that data structure. When we pass
	 *	a TCB to context switcher mechanism, HW looks for first address.
	 */
	reg32_t* topOfStack;

	/* Task Specific MPU Info */
	struct
	{
		volatile uint32_t __internal;
		volatile uint32_t __internal2;
	} mpuInfo[4];
} TCB;
/*************************** FUNCTION DEFINITIONS *****************************/
/**
* Initializes actual CPU and its components/peripherals.
*
* @param none
* @return none
*/
void Drv_CPUCore_Init(void);

/*
 * Halts all system.
 *
 * @param none
 * @return none
 *
 */
void Drv_CPUCore_Halt(void);

/*
 * Starts Context Switching
 *  Configures HW for CS and starts first task
 *
 * @param initialTCB Initial (First) TCB (Task) for Context Switching
 *
 * @return none
 */
void Drv_CPUCore_CSStart(reg32_t* initialTCB);

/*
 * Switches running task to provided new TCB
 *
 * @param newTCB to be switched TCB
 *
 * @return none
 *
 */
void Drv_CPUCore_CSYieldTo(reg32_t* newTCB);

/*
 * Initializes task stack
 *
 * @param stack to be initialized task stack
 * @param stackSize Stack Size
 * @param taskStartPoint Start Point (Function) of Task
 *
 * @return top of stack after initialization. 
 *		   [IMP] Caller should keep top of stack address for new context switches.
 */
void Drv_CPUCore_CSInitializeTask(TCB* tcb, uint8_t* stack, uint32_t stackSize,
										   Drv_CPUCore_TaskStartPoint startPoint,
										   bool privileged);
/*
 * Initializes Memory Protection for whole system.
 *
 * @param none
 *
 * @return none
 */
void Drv_CPUCore_InitializeMPU(void);
#endif	/* __DRV_CPUCORE_H */
