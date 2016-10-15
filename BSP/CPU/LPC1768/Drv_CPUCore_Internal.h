/*******************************************************************************
*
* @file DrvCPUCoreInternal.h
*
* @author Murat Cakmak
*
* @brief CPU specific Internal Definitions
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
******************************************************************************/

#ifndef __DRV_CPUCORE_INTERNAL_H
#define __DRV_CPUCORE_INTERNAL_H

/********************************* INCLUDES ***********************************/
#include "Drv_CPUCore.h"
#include "postypes.h"

/***************************** MACRO DEFINITIONS ******************************/

/*
 * Super - Visor Call Types
 */
/* Start Context Switching using first task */
#define CPUCORE_SVCALL_START_CS				(0)
/* Task Yield */
#define CPUCORE_SVCALL_YIELD				(1)
/* Raise Privilege Mode */
#define CPUCORE_SVCALL_RAISE_PRIVILEGE		(2)
/***************************** TYPE DEFINITIONS *******************************/

/**************************** FUNCTION PROTOTYPES *****************************/

/******************************** VARIABLES ***********************************/

/*************************** FUNCTION PROTOTYPES *****************************/
/*
 * Starts Context Switching on CPU.
 *
 * [IMP] To be switched task (currentTCB) must be set before calling this
 * function.
 *
 * @param none
 *
 * @return none
 */
INTERNAL void StartContextSwitching(void);

/*
 * Initializes Task for Memory Protection
 *
 * @param tcb To be initialized TCB
 * @param stackBottom Bottom Of task stack. User stack is released for task
 *        itself so it can access its stack.
 * @param stackSize Size of user stack
 *
 * @return none
 */
INTERNAL void Drv_CPUCore_InitializeTaskMPU(TCB* tcb, reg32_t stackBottom, uint32_t stackSize);

#endif /* __DRV_CPUCORE_INTERNAL_H */
