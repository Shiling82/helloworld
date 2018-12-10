/*******************************************************************************
*   $FILE:  classb.c
*   Brief: Startup tests are defined in this file
*   Atmel Corporation:  http://www.atmel.com
******************************************************************************/
/*  License
*   Copyright (c) 2016, Atmel Corporation All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
*
*   1. Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*
*   2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
*   3. The name of ATMEL may not be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
*   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
*   SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
*   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
*   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
*   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*============================================================================
Include files
============================================================================*/
#if defined(__ICCAVR__)
#include <ioavr.h>
#include <intrinsics.h>
#include <inavr.h>
#elif defined(__GNUC__)
#include <avr/io.h>
#include <avr/interrupt.h>
#endif

#include "classB.h"

typedef unsigned char uint8_t;
typedef unsigned int  uint16_t;

#if defined(__GNUC__)
#define CMPBUF0 CMP0BUF
#define CMPBUF1 CMP1BUF
#define CMPBUF2 CMP2BUF
#define CC CCMP
#endif
/*
* the start up test functions are included in compilation
* only if this macro is defined in touch_config.h file
*/

/*============================================================================
Macro Definitions
============================================================================*/
#define __enable_interrupt() asm("sei")
#define __no_operation() asm("nop")
#define __disable_interrupt() asm("cli")

/*============================================================================
Funtion Prototypes
============================================================================*/
static uint8_t watchdog_test(void);
static uint8_t cpu_status_test(void);
static uint8_t sram_test(void);
// static uint8_t timer_tca_test(void);
// static uint8_t timer_tcb_test(void);
static uint8_t interrupt_test(void);

/*============================================================================
Global Variable Declarations
============================================================================*/

volatile uint8_t Interrupt_Test_Flag = 0;
register uint8_t Test_Result asm("r4");
volatile uint8_t count;
volatile uint8_t value;

/*============================================================================
Function Definitions
============================================================================*/
/*============================================================================
Name    :   startup_test
------------------------------------------------------------------------------
Purpose :	perform various startup test functions
Input   :   none
Output  :   none
Return	:	Test result TEST_FAIL or TEST_OK
Notes   :
============================================================================*/
void Startup_Test(void)
{
	Test_Result = TEST_OK;

	// perform watchdog functionality test
//	Test_Result &= watchdog_test();

//	__watchdog_reset();

	// Perform sram stuck on and March B test
	Test_Result &= sram_test();

	__watchdog_reset();

	// Perform stuck on test on Status register
//	Test_Result &= cpu_status_test();

//	__watchdog_reset();

// 	// Perform timer0 functionality test.
// 	Test_Result &= timer_tca_test();
// 
// 	__watchdog_reset();
// 
// 	// Perform timer1 functionality test.
// 	Test_Result &= timer_tcb_test();
// 
// 	__watchdog_reset();

	// Check the interrupt functionality
//	Test_Result &= interrupt_test();

//	__watchdog_reset();

	if (Test_Result == TEST_FAIL) {
		/* infinite loop if class B test has failed */
		while (1)
			;
	}
}

/*============================================================================
Name    :   watchdog_test
------------------------------------------------------------------------------
Purpose :	the purpose of this function is to test the Watchdog reset.
On power up watchdog is enabled for 1sec and wait until reset occurs
on watchdog reset this function returns TEST_OK
if watchdog reset didn't occur, then the control stays in while(1)
Input   :   none
Output  :   none
Return	:	returns TEST_OK if watchdog reset happens
============================================================================*/
static uint8_t watchdog_test(void)
{
	uint8_t result = TEST_FAIL;

	// check whether watchdog reset occurred on power up
	if ((RSTCTRL_RSTFR & 0x08) == 0x08) {
		// configure for system reset
		CCP           = 0xD8;
		WDT_CTRLA     = 0x08;
		RSTCTRL_RSTFR = 0x3F;
		// watchdog reset happens, test ok
		result = TEST_OK;
	} else // if reset occur not from WDT
	{

		RSTCTRL_RSTFR = 0x3F;
		CCP           = 0xD8;
		WDT_CTRLA     = 0x08;
		while (1)
			; // wait for watchdog reset
	}

	return result;
}

/*============================================================================
Name    :   cpu_status_test
------------------------------------------------------------------------------
Purpose :	the purpose of this function is to test the CPU Status register
Input   :   none
Output  :   none
Return	:	cpu_status_result TEST_FAIL or TEST_OK
Notes   :
============================================================================*/
static uint8_t cpu_status_test(void)
{
	uint8_t temp;
	uint8_t sav_sreg;
	uint8_t cpu_status_result = TEST_FAIL;

	// Assign 0x55 and read back
	sav_sreg = SREG;
	SREG     = 0x55;
	temp     = SREG;
	SREG     = sav_sreg;
	if (temp == 0x55) {
		sav_sreg = SREG;
		// General Interrupt will be tested in Interrupt test
		// Assign 0x2A and read back
		SREG = 0x2A;
		temp = SREG;
		SREG = sav_sreg;
		if (temp == 0x2A) {
			// Test is OK
			cpu_status_result = TEST_OK;
		}
	}

	return cpu_status_result;
}

/*============================================================================
Name    :   sram_test
------------------------------------------------------------------------------
Purpose :	the purpose of this function is to test the Internal SRAM
SRAM is filled with 0x55 and 0xAA to test if there is no stuck bit
March B test is implemented in two area to with test overlap between both area
Input   :   none
Output  :   none
Return	:	Test result TEST_FAIL or TEST_OK
Notes   :
============================================================================*/
// static uint8_t sram_test(void)__attribute__((optimize("-O0")));
static uint8_t sram_test(void)
{
	register volatile uint8_t *p_val;
	register uint8_t *         p_sav;
	register uint8_t           i;
	register uint8_t           sav;
	register uint8_t           val;
	register uint16_t          temp;
	register signed char       h;
	register uint8_t           test_status = 1;

	// test all SRAM locations with 0x55, 0xAA and complement of address value
	for (p_val = (uint8_t *)SRAM_START_ADR; p_val < ((uint8_t *)(SRAM_START_ADR + (SRAM_SIZE))); p_val++) {
		// write 0x55 and read back
		sav    = *p_val;
		*p_val = 0x55;
		i      = *p_val;
		if (i != 0x55) {
			return TEST_FAIL;
		}

		// write 0xAA and read back
		*p_val = 0xAA;
		i      = *p_val;
		if (i != 0xAA) {
			return TEST_FAIL;
		}

		// complement of address value
		val    = ~((unsigned short)p_val);
		*p_val = val;
		i      = *p_val;
		if (i != val) {
			return TEST_FAIL;
		}
		*p_val = sav;
	}

/*
* In GCC compiler, Stack is allocated at the end of the RAM and the
* Number of STACK locations are not fixed. Stack is updated from the end
* of the RAM
*
* In IAR compiler, stack is allocated at the start of the RAM and the
* number of stack locations are user defined. Stack is updated from the
* start of the RAM
*
* So, before performing test on stack area,
* stack content needs to be backed up. Also, stack content needs to be
* restored before returning from the function
*/

// SRAM is divided into two parts to perform March B test

// March B test is carried out at 1 SRAM portion at a time

//+++++++++++++++++++++TEST THE 2ND SRAM PORTION+++++++++++++++++++++

#if defined(__GNUC__)
	/* For GCC, the stack is allocated at the end of SRAM.
	* Assign 16bytes for SRAM overlapping for the test
	* assuming 0x40 bytes for stack backup
	*/

	// backup the stack content
	p_sav = (uint8_t *)(SRAM_START_ADR + SRAM_SIZE - 1);
	p_val = (uint8_t *)(SRAM_START_PAGE2 - 16 - 1);

	// copy the stack content from the 2nd portion of RAM to 1st portion
	while (p_sav >= (uint8_t *)(SRAM_START_ADR + SRAM_SIZE - 0x40)) {
		// copy the stack content
		*(p_val--) = *(p_sav--);
	}

	// relocate SP to the corresponding location at 1st portion
	temp = SP;
	temp = temp - SRAM_SIZE / 2 - 16;
	SP   = temp;

#endif //__GNUC__

#if defined(__ICCAVR__)
/*
* since stack is allocated at the start of RAM, no need to backup
* stack content
*/
#endif

	// Start of March B test for 2nd SRAM portion

	/*
	* test 1 on the 2nd portion of SRAM
	* erase all sram
	*/

	for (p_val = (uint8_t *)(SRAM_START_ADR + SRAM_SIZE - 1); p_val >= ((uint8_t *)SRAM_START_PAGE2 - 16); p_val--) {
		*p_val = 0;
	}

	// test 2 - 2nd portion
	for (p_val = (uint8_t *)SRAM_START_PAGE2 - 16; p_val < ((uint8_t *)(SRAM_START_ADR + SRAM_SIZE)); p_val++) {
		for (h = 0; h < 8; h++) {
			// read 0
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x00) {
				test_status = TEST_FAIL;
			}

			// write 1
			val    = *p_val | (1 << h);
			*p_val = val;

			// read 1
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x01) {
				test_status = TEST_FAIL;
			}

			// write 0
			val    = *p_val & ~(1 << h);
			*p_val = val;

			// read 0
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x00) {
				test_status = TEST_FAIL;
			}

			// write 1
			*p_val = (*p_val | (1 << h));
		}
	}

	// test 3 - 2nd portion
	for (p_val = (uint8_t *)SRAM_START_PAGE2 - 16; p_val < ((uint8_t *)(SRAM_START_ADR + SRAM_SIZE)); p_val++) {
		for (h = 0; h < 8; h++) {
			// read 1
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x01) {
				test_status = TEST_FAIL;
			}

			// write 0
			val    = *p_val & ~(1 << h);
			*p_val = val;

			// write 1
			*p_val = (*p_val | (1 << h));
		}
	}

	// test 4 - 2nd portion
	for (p_val = (uint8_t *)(SRAM_START_ADR + SRAM_SIZE - 1); p_val >= ((uint8_t *)SRAM_START_PAGE2 - 16); p_val--) {
		for (h = 7; h >= 0; h--) {
			// read 1
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x01) {
				test_status = TEST_FAIL;
			}

			// write 0
			val    = *p_val & ~(1 << h);
			*p_val = val;

			// write 1
			*p_val = (*p_val | (1 << h));

			// write 0
			val    = *p_val & ~(1 << h);
			*p_val = val;
		}
	}

	// test 5 - 2nd portion
	for (p_val = (uint8_t *)(SRAM_START_ADR + SRAM_SIZE - 1); p_val >= ((uint8_t *)SRAM_START_PAGE2 - 16); p_val--) {
		for (h = 7; h >= 0; h--) {
			// read 0
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x00) {
				test_status = TEST_FAIL;
			}

			// write 1
			*p_val = (*p_val | (1 << h));

			// write 0
			val    = *p_val & ~(1 << h);
			*p_val = val;
		}
	}

#if defined(__GNUC__)

	// Restore the stack content before returning in case of test fail

	// restore the stack content at the end of the RAM
	p_val = (uint8_t *)(SRAM_START_ADR + SRAM_SIZE - 1);
	p_sav = (uint8_t *)(SRAM_START_PAGE2 - 16 - 1);

	while (p_val >= (uint8_t *)(SRAM_START_ADR + SRAM_SIZE - 0x40)) {
		// copy the stack content to
		*(p_val--) = *(p_sav--);
	}

	// Restore SP content
	temp = SP;
	temp = temp + SRAM_SIZE / 2 + 16;
	SP   = temp;

#endif //__GNUC__

	// check for test fail
	// if test found any error in test, 1st portion
	// of SRAM is not required to test
	if (test_status == TEST_FAIL)
		return test_status;

	__watchdog_reset();

//+++++++++++++++++++++TEST THE 1ST SRAM PORTION+++++++++++++++++++++

#if defined(__GNUC__)
	/*
	* Back-up the SRAM content before performing test. the entire 1st portion
	* SRAM content cannot be backed up in the 2nd portion as it has stack
	* at the end. so only a portion can be backed up.
	* Here 0x40 bytes are for Stack.
	*
	* SRAM content are backed-up at the start of the 2nd portion
	*
	* during testing overlapping is done between 1st and 2nd portion.
	* number of overlapping location is fixed to 16.
	*/
	p_sav = (uint8_t *)SRAM_START_ADR;
	p_val = (uint8_t *)(SRAM_START_PAGE2 + 16);

	// backup part of 1st SRAM
	while (p_sav <= (uint8_t *)(SRAM_START_PAGE2 - 0x50 - 1)) {
		*(p_val++) = *(p_sav++);
	}

#endif
#if defined(__ICCAVR__)
	/*
	* backup the SRAM content in the 2nd portion of the RAM
	*
	* since STACK is allocated in the 1st portion, while backing up the
	* 1st portion, stack will also be backed-up. only the stack pointer needs
	* to be updated.
	*
	* due to overlapping between two portions of SRAM, entire 1st portion
	* cannot be backed up.
	*
	*/

	// back up the 1st portion.
	p_sav = (uint8_t *)SRAM_START_ADR;
	p_val = (uint8_t *)(SRAM_START_PAGE2 + 16);

	// copy the content to 2nd location. 16 is included considering overlapping
	while (p_sav <= (uint8_t *)(SRAM_START_PAGE2 - 0x10 - 1)) {
		*(p_val++) = *(p_sav++);
	}

	// update stack pointer
	temp = ((SPH << 8));
	temp = (temp | SPL);
	temp -= SRAM_START_ADR;
	temp += SRAM_START_PAGE2;
	temp += 16;

	SPL = ((uint8_t)(temp & 0x00FF));
	SPH = (temp & 0xFF00) >> 8;
#endif

	// test 1 - 1st portion
	for (p_val = (uint8_t *)(SRAM_START_PAGE2 + 16 - 1); p_val >= ((uint8_t *)SRAM_START_ADR); p_val--) {
		*p_val = 0;
	}

	// test 2 - 1st portion
	for (p_val = (uint8_t *)SRAM_START_ADR; p_val < ((uint8_t *)(SRAM_START_PAGE2 + 16)); p_val++) {
		for (h = 0; h < 8; h++) {
			// read 0
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x00) {
				return TEST_FAIL;
			}

			// write 1
			val    = *p_val | (1 << h);
			*p_val = val;

			// read 1
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x01) {
				return TEST_FAIL;
			}

			// write 0
			val    = *p_val & ~(1 << h);
			*p_val = val;

			// read 0
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x00) {
				return TEST_FAIL;
			}

			// write 1
			*p_val = (*p_val | (1 << h));
		}
	}

	// test 3 - 1st portion
	for (p_val = (uint8_t *)SRAM_START_ADR; p_val < ((uint8_t *)(SRAM_START_PAGE2 + 16)); p_val++) {
		for (h = 0; h < 8; h++) {
			// read 1
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x01) {
				return TEST_FAIL;
			}

			// write 0
			val    = *p_val & ~(1 << h);
			*p_val = val;

			// write 1
			*p_val = (*p_val | (1 << h));
		}
	}

	// test 4 - 1st portion
	for (p_val = (uint8_t *)(SRAM_START_PAGE2 + 16 - 1); p_val >= ((uint8_t *)SRAM_START_ADR); p_val--) {
		for (h = 7; h >= 0; h--) {
			// read 1
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x01) {
				return TEST_FAIL;
			}

			// write 0
			val    = *p_val & ~(1 << h);
			*p_val = val;

			// write 1
			*p_val = (*p_val | (1 << h));

			// write 0
			val    = *p_val & ~(1 << h);
			*p_val = val;
		}
	}

	// test 5 - 1st portion
	for (p_val = (uint8_t *)(SRAM_START_PAGE2 + 16 - 1); p_val >= ((uint8_t *)SRAM_START_ADR); p_val--) {
		for (h = 7; h >= 0; h--) {
			// read 0
			val = (((*p_val) >> h) & 0x01);
			if (val != 0x00) {
				return TEST_FAIL;
			}

			// write 1
			*p_val = (*p_val | (1 << h));

			// write 0
			val    = *p_val & ~(1 << h);
			*p_val = val;
		}
	}

#if defined(__GNUC__)
	// restore ram content
	p_val = (uint8_t *)SRAM_START_ADR;
	p_sav = (uint8_t *)(SRAM_START_PAGE2 + 16);

	// it is assumed that, till now, only 0x2f location are used in SRAM
	while (p_val <= (uint8_t *)(SRAM_START_PAGE2 - 0x50 - 1)) {
		*(p_val++) = *(p_sav++);
	}

#endif // __GNUC__

#if defined(__ICCAVR__)

	// restore ram content
	p_val = (uint8_t *)SRAM_START_ADR;
	p_sav = (uint8_t *)(SRAM_START_PAGE2 + 16);

	// copy the content to 2nd location. 16 is included considering overlapping
	while (p_val <= (uint8_t *)(SRAM_START_PAGE2 - 0x10 - 1)) {
		*(p_val++) = *(p_sav++);
	}

	// restore the stack pointer
	temp = (SPH << 8);
	temp = (temp | SPL);
	temp = (temp - (SRAM_START_PAGE2 - SRAM_START_ADR + 16));
	SPL  = ((uint8_t)(temp & 0x00FF));
	SPH  = (temp & 0xFF00) >> 8;

#endif // __ICCAVR__

	return TEST_OK;
}

/*============================================================================
Name    :   timer0_test
------------------------------------------------------------------------------
Purpose :	the purpose of this function is to test the timer 0
Input   :   none
Output  :   none
Return	:	Test result TEST_FAIL or TEST_OK
Notes   :
============================================================================*/
static uint8_t timer_tca_test(void)
{

	// write and read test
	TCA0.SINGLE.CTRLA = 0x05;
	if (TCA0.SINGLE.CTRLA != 0x05)
		return TEST_FAIL;
	TCA0.SINGLE.CTRLA = 0x0A;
	if (TCA0.SINGLE.CTRLA != 0x0A)
		return TEST_FAIL;
	TCA0.SINGLE.CTRLA = 0x00;

	TCA0.SINGLE.CTRLB = 0x55;
	if (TCA0.SINGLE.CTRLB != 0x55)
		return TEST_FAIL;
	TCA0.SINGLE.CTRLB = 0x2A;
	if (TCA0.SINGLE.CTRLB != 0x2A)
		return TEST_FAIL;
	TCA0.SINGLE.CTRLB = 0x00;

	TCA0.SINGLE.CTRLC = 0x05;
	if (TCA0.SINGLE.CTRLC != 0x05)
		return TEST_FAIL;
	TCA0.SINGLE.CTRLC = 0x02;
	if (TCA0.SINGLE.CTRLC != 0x02)
		return TEST_FAIL;
	TCA0.SINGLE.CTRLC = 0x00;

	TCA0.SINGLE.CTRLD = 0x01;
	if (TCA0.SINGLE.CTRLD != 0x01)
		return TEST_FAIL;
	TCA0.SINGLE.CTRLD = 0x00;
	if (TCA0.SINGLE.CTRLD != 0x00)
		return TEST_FAIL;

	TCA0.SINGLE.EVCTRL = 0x05;
	if (TCA0.SINGLE.EVCTRL != 0x05)
		return TEST_FAIL;
	TCA0.SINGLE.EVCTRL = 0x02;
	if (TCA0.SINGLE.EVCTRL != 0x02)
		return TEST_FAIL;

	TCA0.SINGLE.INTCTRL = 0x51;
	if (TCA0.SINGLE.INTCTRL != 0x51)
		return TEST_FAIL;
	TCA0.SINGLE.INTCTRL = 0x20;
	if (TCA0.SINGLE.INTCTRL != 0x20)
		return TEST_FAIL;
	TCA0.SINGLE.INTCTRL = 0x00;

	TCA0.SINGLE.TEMP = 0x55;
	if (TCA0.SINGLE.TEMP != 0x55)
		return TEST_FAIL;
	TCA0.SINGLE.TEMP = 0xAA;
	if (TCA0.SINGLE.TEMP != 0xAA)
		return TEST_FAIL;
	TCA0.SINGLE.TEMP = 0x00;

	TCA0.SINGLE.CNT = 0x5555;
	if (TCA0.SINGLE.CNT != 0x5555)
		return TEST_FAIL;
	TCA0.SINGLE.CNT = 0xAAAA;
	if (TCA0.SINGLE.CNT != 0xAAAA)
		return TEST_FAIL;
	TCA0.SINGLE.CNT = 0x0000;

	TCA0.SINGLE.PER = 0x5555;
	if (TCA0.SINGLE.PER != 0x5555)
		return TEST_FAIL;
	TCA0.SINGLE.PER = 0xAAAA;
	if (TCA0.SINGLE.PER != 0xAAAA)
		return TEST_FAIL;
	TCA0.SINGLE.PER = 0xFFFF;

	TCA0.SINGLE.PERBUF = 0x5555;
	if (TCA0.SINGLE.PERBUF != 0x5555)
		return TEST_FAIL;
	TCA0.SINGLE.PERBUF = 0xAAAA;
	if (TCA0.SINGLE.PERBUF != 0xAAAA)
		return TEST_FAIL;
	TCA0.SINGLE.PERBUF = 0x0000;

	TCA0.SINGLE.CMP0 = 0x5555;
	if (TCA0.SINGLE.CMP0 != 0x5555)
		return TEST_FAIL;
	TCA0.SINGLE.CMP0 = 0xAAAA;
	if (TCA0.SINGLE.CMP0 != 0xAAAA)
		return TEST_FAIL;
	TCA0.SINGLE.CMP0 = 0x0000;

	TCA0.SINGLE.CMP1 = 0x5555;
	if (TCA0.SINGLE.CMP1 != 0x5555)
		return TEST_FAIL;
	TCA0.SINGLE.CMP1 = 0xAAAA;
	if (TCA0.SINGLE.CMP1 != 0xAAAA)
		return TEST_FAIL;
	TCA0.SINGLE.CMP1 = 0x0000;

	TCA0.SINGLE.CMP2 = 0x5555;
	if (TCA0.SINGLE.CMP2 != 0x5555)
		return TEST_FAIL;
	TCA0.SINGLE.CMP2 = 0xAAAA;
	if (TCA0.SINGLE.CMP2 != 0xAAAA)
		return TEST_FAIL;
	TCA0.SINGLE.CMP2 = 0x0000;

	TCA0.SINGLE.CMP0BUF = 0x5555;
	if (TCA0.SINGLE.CMPBUF0 != 0x5555)
		return TEST_FAIL;
	TCA0.SINGLE.CMPBUF0 = 0xAAAA;
	if (TCA0.SINGLE.CMPBUF0 != 0xAAAA)
		return TEST_FAIL;
	TCA0.SINGLE.CMPBUF0 = 0x0000;

	TCA0.SINGLE.CMPBUF1 = 0x5555;
	if (TCA0.SINGLE.CMPBUF1 != 0x5555)
		return TEST_FAIL;
	TCA0.SINGLE.CMPBUF1 = 0xAAAA;
	if (TCA0.SINGLE.CMPBUF1 != 0xAAAA)
		return TEST_FAIL;
	TCA0.SINGLE.CMPBUF1 = 0x0000;

	TCA0.SINGLE.CMPBUF2 = 0x5555;
	if (TCA0.SINGLE.CMPBUF2 != 0x5555)
		return TEST_FAIL;
	TCA0.SINGLE.CMPBUF2 = 0xAAAA;
	if (TCA0.SINGLE.CMPBUF2 != 0xAAAA)
		return TEST_FAIL;
	TCA0.SINGLE.CMPBUF2 = 0x0000;

	TCA0.SINGLE.INTFLAGS |= 0x71;
	TCA0.SINGLE.CNT  = 0xFF64;
	TCA0.SINGLE.CMP0 = 0xFF96;
	TCA0.SINGLE.CTRLA |= 0x01;

	count = 0;
	__watchdog_reset();
	while (count < 200) // features a time-out
	{
		value = (TCA0.SINGLE.INTFLAGS);
		value = value & 0x11;

		switch (value) {
		case 0x00:
			// initial state
			break;

		case 0x01:
			// OVF occurred
			break;

		case 0x10:
			// CMP 0 occurred
			break;

		case 0x11:
			// OVF & CMP0 occurred
			count = 250; // generate an ok signal
			break;

		default:
			// all others cases are fault
			count = 210; // generate an error signal
		}
		count++;
	}

	if (count < 220)
		return TEST_FAIL;

	TCA0.SINGLE.CTRLA &= 0xFE;

	return TEST_OK;
}

/*============================================================================
Name    :   timer1_test
------------------------------------------------------------------------------
Purpose :	the purpose of this function is to test the timer 1
Input   :   none
Output  :   none
Return	:	Test result TEST_FAIL or TEST_OK
Notes   :
============================================================================*/
static uint8_t timer_tcb_test(void)
{

	TCB0.CTRLA = 0x55;
	if (TCB0.CTRLA != 0x55)
		return TEST_FAIL;
	TCB0.CTRLA = 0x02;
	if (TCB0.CTRLA != 0x02)
		return TEST_FAIL;
	TCB0.CTRLA = 0x00;

	TCB0.CTRLB = 0x55;
	if (TCB0.CTRLB != 0x55)
		return TEST_FAIL;
	TCB0.CTRLB = 0x22;
	if (TCB0.CTRLB != 0x22)
		return TEST_FAIL;
	TCB0.CTRLB = 0x00;

	TCB0.EVCTRL = 0x51;
	if (TCB0.EVCTRL != 0x51)
		return TEST_FAIL;
	TCB0.EVCTRL = 0x00;

	TCB0.INTCTRL = 0x01;
	if (TCB0.INTCTRL != 0x01)
		return TEST_FAIL;
	TCB0.INTCTRL = 0x00;

	TCB0.TEMP = 0x55;
	if (TCB0.TEMP != 0x55)
		return TEST_FAIL;
	TCB0.TEMP = 0xAA;
	if (TCB0.TEMP != 0xAA)
		return TEST_FAIL;
	TCB0.TEMP = 0x00;

	TCB0.CNT = 0x5555;
	if (TCB0.CNT != 0x5555)
		return TEST_FAIL;
	TCB0.CNT = 0xAAAA;
	if (TCB0.CNT != 0xAAAA)
		return TEST_FAIL;
	TCB0.CNT = 0x00;

	TCB0.CC = 0x5555;
	if (TCB0.CC != 0x5555)
		return TEST_FAIL;
	TCB0.CC = 0xAAAA;
	if (TCB0.CC != 0xAAAA)
		return TEST_FAIL;
	TCB0.CC = 0x00;
	__watchdog_reset();
	TCB0.CC = 0x0096;
	TCB0.CTRLA |= 0x01;

	count = 0;

	if ((TCB0.INTFLAGS & 0x01) != 0)
		return TEST_FAIL;

	while (count < 200) // features a time-out
	{

		value = (TCB0.INTFLAGS);
		value = value & 0x01;

		switch (value) {
		case 0x00:
			// stay in initial state
			break;

		case 0x01:
			// OVF occurred
			count = 250; // generate an ok signal
			break;

		default:
			// all others cases are fault
			count = 210; // generate an error signal
		}
		count++;
	}

	if (count < 220)
		return TEST_FAIL;

	TCB0.CTRLA &= 0xFE;

	return TEST_OK;
}

/*============================================================================
Name    :   interrupt_test
------------------------------------------------------------------------------
Purpose :	the purpose of this function is to test the interrupt controller
Input   :   none
Output  :   none
Return	:	Test result TEST_FAIL or TEST_OK
Notes   :	Timer 0 is used to test the interrupt controller
============================================================================*/
static uint8_t interrupt_test(void)
{

	TCA0.SINGLE.EVCTRL = 0x00;
	TCA0.SINGLE.INTCTRL |= 0x01;
	Interrupt_Test_Flag = 0;
	TCA0.SINGLE.PER     = 0xFFFD;
	TCA0.SINGLE.CMP0    = 0xFFFD;
	TCA0.SINGLE.CNT     = 0xFFFB;
	TCA0.SINGLE.CTRLA |= 0x01;

	__enable_interrupt();

	__no_operation();
	__no_operation();
	__no_operation();
	__no_operation();
	__no_operation();

	__disable_interrupt();

	TCA0.SINGLE.INTCTRL &= 0xFE;
	TCA0.SINGLE.CTRLA &= 0xFE;

	// if interrupt occurred
	if (Interrupt_Test_Flag == 1)
		return TEST_OK;

	return TEST_FAIL;
}

/*============================================================================
Name    :   ISR(TIMER0_OVF_vect)
------------------------------------------------------------------------------
Purpose :	Timer0 overflow interrupt
Input   :   none
Output  :   none
Return	:	none
Notes   : 	brief Interrupt controller test
Use an interruption which is not used by the application
pre Configuration the dedicated interruption (initialization)
post Set the Interrupt_Test_Flag
============================================================================*/
#if defined(__GNUC__)
ISR(TCA0_OVF_vect)
#elif defined(__ICCAVR__)
#pragma vector = TCA0_OVF_LUNF_vect
__interrupt void unused_interrupt(void)
#endif
{
	///* set the flag when timer interrupt is occurred
	//* this flag will be tested in interrupt test function
	//*/
	Interrupt_Test_Flag = (unsigned char)1;
}

/*============================================================================
END OF FILE
============================================================================*/
