/*******************************************************************************
 *
 * @file Drv_CPUCore_MPU.c
 *
 * @author MC
 *
 * @brief Memory Protection Implementation
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

/********************************* INCLUDES ***********************************/

#include "Drv_CPUCore.h"
#include "LPC17xx.h"

#include "postypes.h"

/***************************** MACRO DEFINITIONS ******************************/
/* Defines to enable/disable MPU Regions */
#define MPU_REGION_DISABLE								(0)
#define MPU_REGION_ENABLE								(1)

/* Defines to mark regions as valid/invalid */
#define MPU_REGION_NOTVALID								(0)
#define MPU_REGION_VALID								(1)

/*
 * MPU Region Types
 *
 *  TODO need to be revisited after seperated user application support
 */
/* Common Code Area */
#define MPU_REGION_UNPRIVILEGED_FLASH					(0)
/* GPIO */
#define MPU_REGION_GPIO									(1)
/* Privileged Code Area */
#define MPU_REGION_PRIVILEGED_FLASH						(2)
/* Peripherals */
#define MPU_REGION_GENERAL_PERIPHERALS					(3)
/* User Application Stack */
#define MPU_REGION_USERAPP_STACK						(4)

/*
 * Details of MPU Regions
 */
/* Flash (Code) Area */
#define MPU_REGION_UNPRIVILEGED_FLASH_START				(0x0UL)
#define MPU_REGION_UNPRIVILEGED_FLASH_SIZE				(0x80000)
#define MPU_REGION_UNPRIVILEGED_FLASH_SIZE_VALUE		(18)

/* Device RAM */
#define MPU_REGION_SRAM_START							(0x10000000UL)
#define MPU_REGION_SRAM_SIZE							(0x8000)
#define MPU_REGION_SRAM_SIZE_VALUE						(14)

/* GPIO */
#define MPU_REGION_GPIO_START 							(LPC_GPIO_BASE)
#define MPU_REGION_GPIO_SIZE 							(0x3FFF)
#define MPU_REGION_GPIO_SIZE_VALUE						(13)

/* Peripherals */
#define MPU_REGION_PERIPHERALS_START					(0x40000000UL)
#define MPU_REGION_PERIPHERALS_END						(0x5FFFFFFFUL)
#define MPU_REGION_PERIPHERALS_SIZE_VALUE				(28)

/*
 * AP Encoding Types
 *  Specifies Read/Write Writes
 */
/* Read/Write  */
#define MPU_AP_ENCODING_RW								(3)
/* Read Only */
#define MPU_AP_ENCODING_RO								(6)
/* Privileged Read/Write */
#define MPU_AP_ENCODING_PRIVILEGED_RW					(1)
/* Privileged Read Only */
#define MPU_AP_ENCODING_PRIVILEGED_RO					(5)

/*
 * Access Types
 */
#define MPU_ACCESS_CACHEABLE_BUFFERABLE					(7)

/*
 * Aligns Region address for MPU
 *
 * Region address should be aligned with its size and address should be set
 * according to its address. See ADDR field of MPU RABR register for details.
 * While addr register needs only required address length (and rest of is
 * reserved bits), we are setting all address bits (including address + reserved)
 */
#define MPU_ALIGN_REGION_ADDR(addr)						((addr) >>MPU_RBAR_ADDR_Pos)

/*
 * Default Region Specific MPU Register Settings
 *
 *  TODO need to be revisited after seperated user application support
 *
 */
/* RBAR Settings for Code (Flash) Region */
#define MPU_FLASH_RBAR_VAL \
			(MPU_REGION_UNPRIVILEGED_FLASH) | \
			(MPU_RBAR_VALID_Msk) | \
			(MPU_ALIGN_REGION_ADDR(MPU_REGION_UNPRIVILEGED_FLASH_START) << MPU_RBAR_ADDR_Pos)

/* RASR Settings for Code (Flash) Region */
#define MPU_FLASH_RASR_VAL \
			(MPU_AP_ENCODING_RO << MPU_RASR_AP_Pos) | \
			(MPU_ACCESS_CACHEABLE_BUFFERABLE << MPU_RASR_B_Pos) | \
			(MPU_REGION_UNPRIVILEGED_FLASH_SIZE_VALUE << MPU_RASR_SIZE_Pos) | \
			(MPU_RASR_ENABLE_Msk)

/* RBAR Settings for GPIO Region */
#define MPU_GPIO_RBAR_VAL \
			(MPU_REGION_GPIO) | \
			(MPU_RBAR_VALID_Msk) | \
			(MPU_ALIGN_REGION_ADDR(MPU_REGION_GPIO_START) << MPU_RBAR_ADDR_Pos)

/* RASR Settings for GPIO Region */
#define MPU_GPIO_RASR_VAL \
			(MPU_AP_ENCODING_RW << MPU_RASR_AP_Pos) | \
			(MPU_ACCESS_CACHEABLE_BUFFERABLE << MPU_RASR_B_Pos) | \
			(MPU_REGION_GPIO_SIZE_VALUE << MPU_RASR_SIZE_Pos) | \
			(MPU_RASR_ENABLE_Msk)

/* RBAR Settings for Peripherals Region */
#define MPU_PERIPHERALS_RBAR_VAL \
			(MPU_REGION_GENERAL_PERIPHERALS) | \
			(MPU_RBAR_VALID_Msk) | \
			(MPU_ALIGN_REGION_ADDR(MPU_REGION_PERIPHERALS_START) << MPU_RBAR_ADDR_Pos)

/* RASR Settings for Peripherals Region */
#define MPU_PERIPHERALS_RASR_VAL \
			(MPU_AP_ENCODING_RW << MPU_RASR_AP_Pos) | \
			(MPU_RASR_XN_Msk) | \
			(MPU_REGION_PERIPHERALS_SIZE_VALUE << MPU_RASR_SIZE_Pos) | \
			(MPU_RASR_ENABLE_Msk)

/***************************** TYPE DEFINITIONS *******************************/
/*
 * Region Base Address Register (RBAR) Structure
 */
typedef struct
{
	/* Region No */
	reg32_t region : 4;
	/* Flag to mark valid */
	reg32_t valid : 1;
	/*
	 * Bit[31,N] : The value of N depends on the region size
	 *
	 * [IMP] addr field includes also reserved bits.
	 *
	 */
	reg32_t addr : 27;
} MPURegRBAR;

/*
 * Region Attribute and Size Register
 */
typedef struct
{
	reg32_t enable : 1;
	reg32_t size : 5;
	reg32_t __reserved : 2;
	reg32_t srd : 8;
	reg32_t bsc : 3;
	reg32_t tex : 3;
	reg32_t __reserved2 : 2;
	reg32_t ap : 3;
	reg32_t __reserved3 : 1;
	reg32_t xn : 1;
	reg32_t __reserved4 : 3;
} MPURegRASR;

/**************************** FUNCTION PROTOTYPES *****************************/

/******************************** VARIABLES ***********************************/

/***************************** PRIVATE FUNCTIONS ******************************/

/***************************** PUBLIC FUNCTIONS *******************************/

/*
 * Initializes MPU
 *
 *  NOTE This function initializes Flash, GPIO and Peripherals statically.
 *  On the other hand, a driver should provide a generic interface to support
 *  different needs of upper layers. Modify it for that purpose
 *
 */
void Drv_CPUCore_InitializeMPU(void)
{
	/* Enter Critical Section to ensure about integrity of MPU initialization */
	__disable_irq();

	/* FLASH */
	MPU->RBAR = MPU_FLASH_RBAR_VAL;
	MPU->RASR = MPU_FLASH_RASR_VAL;

	/* GPIO */
	MPU->RBAR = MPU_GPIO_RBAR_VAL;
	MPU->RASR = MPU_GPIO_RASR_VAL;

	/* PERIPHERALS */
	MPU->RBAR = MPU_PERIPHERALS_RBAR_VAL;
	MPU->RASR = MPU_PERIPHERALS_RASR_VAL;

	/* Enable the memory fault exception. */
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

	/* Enable the MPU with the background region configured. */
	MPU->CTRL |= ( MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk );

	/* Exit from critical section */
	__enable_irq();
}

/*
 * Initializes Task TCB according to its access needs.
 *
 *  This function initializes Task to give access to following regions
 *  	- Task Stack Section (Local Variables and required things for
 *        Context Switchings)
 * 		- Task Data Section (Global Variables)
 *
 *  TODO For now, all RAM section is opened for User Task until seperate User
 *  App is supported.
 *
 */
void Drv_CPUCore_InitializeTaskMPU(TCB* tcb, reg32_t stackBottom, uint32_t stackSize)
{
	/* Mapping RBAR and RASR on task specific info */
	MPURegRBAR* taskRBAR = (MPURegRBAR*)&tcb->mpuInfo[0].__internal;
	MPURegRASR* taskRASR = (MPURegRASR*)&tcb->mpuInfo[0].__internal2;
	int32_t regIndex;

	/*
	 * Give access to tasks own stack
	 *
	 * TODO for now give access of all RAM
	 */
	taskRBAR->region = MPU_REGION_USERAPP_STACK;
	taskRBAR->addr = MPU_ALIGN_REGION_ADDR(MPU_REGION_SRAM_START);
	taskRBAR->valid = MPU_REGION_VALID;

	taskRASR->ap = MPU_AP_ENCODING_RW;
	taskRASR->bsc = MPU_ACCESS_CACHEABLE_BUFFERABLE;
	taskRASR->size = MPU_REGION_SRAM_SIZE_VALUE;
	taskRASR->enable = MPU_REGION_ENABLE;

	for( regIndex = 1; regIndex <= 4; regIndex++ )
	{
		/* Invalidate the region. */
		tcb->mpuInfo[regIndex].__internal = ( MPU_REGION_USERAPP_STACK + regIndex ) | MPU_RBAR_VALID_Msk;
		tcb->mpuInfo[regIndex].__internal2 = 0UL;
	}
}
