/**
 * \file
 *
 * \brief Driver ISR.
 *
 (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/*
 * Code generated by START.
 *
 * This file will be overwritten when reconfiguring your START project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include <driver_init.h>
#include <compiler.h>
#include "main.h"

ISR(RTC_CNT_vect)
{

	/* Insert your RTC Overflow interrupt handling code */
 	//RTC.CNT = RTC_START_CNTS;
	RTC_proc();
	//BSP_timing_proc();
	/* Overflow interrupt flag has to be cleared manually */
	RTC.INTFLAGS = RTC_OVF_bm;
}

ISR(ADC0_RESRDY_vect)
{
	/* Insert your ADC result ready interrupt handling code here */
	ADC_get_result();	
	/* The interrupt flag has to be cleared manually */
	ADC0.INTFLAGS = ADC_RESRDY_bm;
}

ISR(AC0_AC_vect)
{
	/* Insert your AC interrupt handling code here */
	if(AC0.STATUS &AC_STATE_bm !=0)
	{
		AComparator_change();
	}
	/* The interrupt flag has to be cleared manually */
	AC0.STATUS = AC_CMP_bm;
}

ISR(PORTA_PORT_vect)
{
	/* Overflow interrupt flag has to be cleared manually */
	PORTA.INTFLAGS = PORT_INT1_bm; 
	//PORTB.INTFLAGS = PORT_INT0_bm;  	
	ZeroCross_proc();
}