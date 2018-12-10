/*******************************************************************************
*   $FILE:  classB.h
*   Brief: contains funtion prototype and macro definitions for classb.c file
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
#if defined(__GNUC__)
#include <avr/io.h>
#include <avr/common.h>
#elif defined(__ICCAVR__)
#include <ioavr.h>
#include <intrinsics.h>
#include <inavr.h>
#endif

/*============================================================================
Macro Definitions
============================================================================*/
// define RAMSTART if not defined
#if defined(__ICCAVR__)
#define RAMSTART 0x3E00
#endif

#ifndef RAMSTART
#error "assign RAM start address"
#endif

#ifdef RAMSTART
#define SRAM_START_ADR (RAMSTART)
#endif
#define SRAM_SIZE 256

//#define SRAM_END_ADR    RAMEND
//#define SRAM_SIZE       (RAMEND - RAMSTART + 1)
#define SRAM_END_ADR SRAM_START_ADR + SRAM_SIZE - 1

/* start address of SRAM second locaiton used by SRAM test */
/* SRAM_START_ADR + (SRAM_SIZE/2) */
#define SRAM_START_PAGE2 SRAM_START_ADR + (SRAM_SIZE / 2) //;//>>1)

/* stack pointer used during SRAM test */
/* SRAM_START_ADR + (SRAM_SIZE/4) - 1 */
#define NEW_STACK_POINTER SRAM_START_ADR + (SRAM_SIZE / 2) - 1

/* Test result returns*/
#define TEST_OK 1
#define TEST_FAIL 0

// Defined reset reasons.
#define RESET_POWER_ON 0
#define RESET_BROWN_OUT 1
#define RESET_RESET_PIN 2
#define RESET_WATCH_DOG 3
#define RESET_SOFTWARE 4
#define RESET_PDI 5

#define __watchdog_reset() asm("wdr")

/*============================================================================
Function Prototypes
============================================================================*/
void Startup_Test(void);

/*============================================================================
  END OF FILE
============================================================================*/
