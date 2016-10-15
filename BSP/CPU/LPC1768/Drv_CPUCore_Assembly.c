/*******************************************************************************
 *
 * @file DrvCPUCoreAssembly.c
 *
 * @author Murat Cakmak
 *
 * @brief CPU Core specific (Cortex-M3) driver implementation for assembly
 * 		  functions.
 *
 *        Includes two type of assembly code of same implementations
 *        	- ARMCC Assembly
 *        	- GCC (GNU C) Assembly
 *
 *
 *		  [ IMPORTANT NOTE! ]
 *        In that implementation we used FreeRTOS Context Switching
 *        implementation (except small modifications) and according to FreeRTOS
 *        license, P-OS guarantees that P-OS and FreeRTOS performances will not
 *        be compared.
 *        TODO : Implement own Context Switching according to P-OS Requirements.
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

/********************************* INCLUDES ***********************************/

#include "Drv_CPUCore_Internal.h"
#include "LPC17xx.h"
#include "postypes.h"

/***************************** MACRO DEFINITIONS ******************************/

/* Vector Table Offset Register */
#define REG_SCB_VTOR_ADDR					(0xE000ED08)

/* MPU Region Base Register Address */
#define REG_MPU_REGION_BASE_ADDR			(0xE000ED9C)

/* Fail */
#define LOAD_EXEC_RETURN_CODE 				(0xfffffffd)

/*  */
#define MAX_SYSCALL_INTERRUPT_PRIORITY 		(191) /* equivalent to 0xb0, or priority 11. */

/***************************** TYPE DEFINITIONS *******************************/

/**************************** FUNCTION PROTOTYPES *****************************/
ASSEMBLY_FUNCTION void SwitchToFirstTask(void);
/******************************** VARIABLES ***********************************/

/***************************** PRIVATE FUNCTIONS ******************************/

/***************************** PUBLIC FUNCTIONS *******************************/

#if defined(__ARMCC_VERSION) /* ARMCC Assembly Area */

/*
 * ISR for PendSV IRQ Handler
 *
 *  We use PendSV Interrupts for Context Switching
 *
 * Implementation copied from FreeRTOS
 */
ASSEMBLY_FUNCTION void POS_PendSV_Handler( void )
{
	extern currentTCB;
	extern SwitchContext;

	PRESERVE8

	mrs r0, psp
	isb

	/* Get the location of the current TCB. */
	ldr	r3, =currentTCB
	ldr	r2, [r3]

	/*
	 * Restore Task Registers
	 */
	mrs r1, control
	stmdb r0!, {r1, r4-r11}		/* Save the remaining registers. */
	str r0, [r2]				/* Save the new top of stack into the first member of the TCB. */

	/*
	 * Switch context
	 */
	stmdb sp!, {r3, r14}
	mov r0, #MAX_SYSCALL_INTERRUPT_PRIORITY
	msr basepri, r0
	dsb
	isb
	bl SwitchContext
	mov r0, #0
	msr basepri, r0
	ldmia sp!, {r3, r14}

	/*
	 * Memory Protection for Application Seperation
	 */
	ldr r1, [r3]
	ldr r0, [r1]						/* The first item in currentTCB is the task top of stack. */
	add r1, r1, #4						/* Move onto the second item in the TCB... */
	ldr r2, =REG_MPU_REGION_BASE_ADDR	/* Region Base Address register. */
	ldmia r1!, {r4-r11}					/* Read 4 sets of MPU registers. */
	stmia r2!, {r4-r11}					/* Write 4 sets of MPU registers. */

	/* Set new task registers */
	ldmia r0!, {r3, r4-r11}		/* Pop the registers */
	msr control, r3

	msr psp, r0
	isb

	/* Jump to next task */
	bx r14
}

/*
 * SVC (Super-Visor Call) IRQ Handler
 *
 *  We are using super-visor calls to handle context switching and system calls
 *  from unprivileged applications.
 *
 *   Please see
 *   http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0179b/ar01s02s07.html
 *   to see how to pass parameter to SVC Calls
 *
 */
ASSEMBLY_FUNCTION void POS_SVC_Handler(void)
{
	IMPORT SVCHandler

	TST lr, #4
	MRSEQ r0, MSP
	MRSNE r0, PSP

	/* Call SVCHandler function to process SVC Call */
	B SVCHandler
}

/*
 * SVC Request Handler
 *  Handles and processes specific Super-Visor Calls
 */
void SVCHandler(uint32_t * svc_args)
{
	uint32_t svc_number;

	/*
     * http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0179b/ar01s02s07.html
     * Stack contains:    * r0, r1, r2, r3, r12, r14, the return address and xPSR
	 * First argument (r0) is svc_args[0]
	 */
	svc_number = ((char *)svc_args[6])[-2];

	switch(svc_number)
	{
		case CPUCORE_SVCALL_START_CS:
			SwitchToFirstTask();
			break;
		case CPUCORE_SVCALL_YIELD:
			/* Set a PendSV to request a context switch. */
			SCB->ICSR = (reg32_t)SCB_ICSR_PENDSVSET_Msk;

			/*
			 * Barriers are normally not required but do ensure the code is completely
			 * within the specified behavior for the architecture. (Note From FreeRTOS)
			 */
			__DMB();
			break;
		case CPUCORE_SVCALL_RAISE_PRIVILEGE:
			/* Not defined yet */
			break;
		default:
			break;
	}
}

/*
 * ISR for SVC Exception
 *
 *  We use SVC Interrupt to start Context Switching
 *
 * Implementation copied from FreeRTOS
 */
ASSEMBLY_FUNCTION void SwitchToFirstTask( void )
{
	extern currentTCB;

	PRESERVE8

	/* Use the NVIC offset register to locate the stack. */
	ldr r0, =REG_SCB_VTOR_ADDR
	ldr r0, [r0]
	ldr r0, [r0]
	msr msp, r0							/* Set the msp back to the start of the stack. */
	ldr	r3, =currentTCB					/* Restore the context. */
	ldr r1, [r3]
	ldr r0, [r1]						/* The first item in the TCB is the task top of stack. */

	/*
	 * Memory Protection
	 */
	add r1, r1, #4						/* Move onto the second item in the TCB... */
	ldr r2, =REG_MPU_REGION_BASE_ADDR	/* Region Base Address register. */
	ldmia r1!, {r4-r11}					/* Read 4 sets of MPU registers. */
	stmia r2!, {r4-r11}					/* Write 4 sets of MPU registers. */
	ldmia r0!, {r3, r4-r11}				/* Pop the registers that are not automatically saved on exception entry. */

	/*
	 * Set control register for priviliged state
	 */
	msr control, r3
	msr psp, r0							/* Restore the task stack pointer. */
	mov r0, #0
	msr	basepri, r0
	ldr r14, =LOAD_EXEC_RETURN_CODE		/* Load exec return code. */

	/* Jump to first task*/
	bx r14
	nop
}

/*
 * Starts context switching by calling SVC interrupt
 *
 * @param none
 *
 * @return This function does not return, jumps to SVC ISR and SVC ISR continues
 *         with the Kernel (and user) defined tasks.
 *
 */
ASSEMBLY_FUNCTION void StartContextSwitching(void){
	PRESERVE8

	/* Use the NVIC offset register to locate the stack. */
	ldr r0, =REG_SCB_VTOR_ADDR
	ldr r0, [r0]
	ldr r0, [r0]

	/* Set the msp back to the start of the stack. */
	msr msp, r0

	/* Globally enable interrupts. */
	cpsie i
	cpsie f
	dsb
	isb

	/* Call SVC to start the first task. */
	svc #CPUCORE_SVCALL_START_CS
}

/*
 * Switches running task to provided new TCB
 *
 * @param newTCB to be switched TCB
 * @param none
 *
 */
void Drv_CPUCore_CSYieldTo(reg32_t* newTCB)
{
	extern reg32_t* nextTCB;

	/* Save TCB for task switching */
    nextTCB = newTCB;

	/*
	 * Make a service call to trigger
	 */
	__asm
	{
		svc #CPUCORE_SVCALL_YIELD
	}
}

#else /* GNU C - GCC Assembly Area */

/*
 * TODO : [IMP] Until we use assembly code, we will not test Assembly modules.
 * So compile assembly blocks only for "Project Builds"
 */
#if !defined(UNIT_TEST)

/*
 * Note: By default GNU ARM compiler stores and restore a Frame Pointer
 * using "r7" and do stack alignment when entering into and exiting functions.
 * To avoid such optimizations we need to declare our handlers as "naked".
 *
 * REF : http://www.thewireframecommunity.com/writing-a-basic-multitasking-os-for-arm-cortex-m3-processor
 *
 */
ASSEMBLY_FUNCTION void POS_PendSV_Handler(void)  	__attribute__ ((naked));
ASSEMBLY_FUNCTION void POS_SVC_Handler(void) 		__attribute__ ((naked));

/*
 * ISR for PendSV Exception
 *
 *  We use PendSV Interrupts for Context Switching
 *
 * Implementation copied from FreeRTOS
 */
ASSEMBLY_FUNCTION void POS_PendSV_Handler(void)
{
	__asm volatile
	(
		"	mrs r0, psp							\n"
		"	isb									\n"
		"										\n"
		"	ldr	r3, currentTCBConst				\n" /* Get the location of the current TCB. */
		"	ldr	r2, [r3]						\n"
		"										\n"
		"	stmdb r0!, {r4-r11}					\n" /* Save the remaining registers. */
		"	str r0, [r2]						\n" /* Save the new top of stack into the first member of the TCB. */
		"										\n"
		"	stmdb sp!, {r3, r14}				\n"
		"	mov r0, %0							\n"
		"	msr basepri, r0						\n"
		"	bl SwitchContext					\n"
		"	mov r0, #0							\n"
		"	msr basepri, r0						\n"
		"	ldmia sp!, {r3, r14}				\n"
		"										\n"	/* Restore the context, including the critical nesting count. */
		"	ldr r1, [r3]						\n"
		"	ldr r0, [r1]						\n" /* The first item in currentTCB is the task top of stack. */
		"	ldmia r0!, {r4-r11}					\n" /* Pop the registers. */
		"	msr psp, r0							\n"
		"	isb									\n"
		"	bx r14								\n"
		"										\n"
		"	.align 4							\n"
		"currentTCBConst: .word currentTCB		\n"
		::"i"(MAX_SYSCALL_INTERRUPT_PRIORITY)
	);
}

/*
 * ISR for SVC Exception
 *
 *  We use SVC Interrupt to start Context Switching
 *
 * Implementation copied from FreeRTOS
 */
ASSEMBLY_FUNCTION void POS_SVC_Handler(void)
{
	__asm volatile
	(
		"	ldr	r3, currentTCBConst2			\n" /* Restore the context. */
		"	ldr r1, [r3]						\n" /* Use currentTCBConst2 to get the currentTCB address. */
		"	ldr r0, [r1]						\n" /* The first item in currentTCB is the task top of stack. */
		"	ldmia r0!, {r4-r11}					\n" /* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */
		"	msr psp, r0							\n" /* Restore the task stack pointer. */
		"	isb									\n"
		"	mov r0, #0 							\n"
		"	msr	basepri, r0						\n"
		"	orr r14, #0xd						\n"
		"	bx r14								\n"
		"										\n"
		"	.align 4							\n"
		"currentTCBConst2: .word currentTCB		\n"
	);
}

/*
 * Starts context switching by calling SVC interrupt
 *
 * @param none
 *
 * @return This function does not return, jumps to SVC ISR and SVC ISR continues
 *         with the Kernel (and user) defined tasks.
 *
 */
ASSEMBLY_FUNCTION void StartContextSwitching(void)
{
	__asm volatile
	(
		" ldr r0, =0xE000ED08 	\n" /* Use the NVIC offset register to locate the stack. */
		" ldr r0, [r0] 			\n"
		" ldr r0, [r0] 			\n"
		" msr msp, r0			\n" /* Set the msp back to the start of the stack. */
		" cpsie i				\n" /* Globally enable interrupts. */
		" cpsie f				\n"
		" dsb					\n"
		" isb					\n"
		" svc 0					\n" /* System call to start first task. */
		" nop					\n"
		//::"i"(REG_SCB_VTOR_ADDR)
	);
}

#endif /* #if !defined(UNITY) */

#endif /* #if ARMCC | GCC */
