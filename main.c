#include <atmel_start.h>
#include "classB.h"
#include "main.h"
/*System variable*/
uint8_t                   wdata[] = {0, 1, 2, 3};
uint8_t                   rdata[6];
uint8_t                   expected[6] = {0xff, 0, 1, 2, 3, 0xff};
volatile nvmctrl_status_t status;
volatile uint8_t          rb;

volatile int8_t		c_tmr_100us = 0 ;
volatile int8_t		c_tmr_500us = 0 ;
volatile uint16_t	c_system_flag = 0;	
volatile uint16_t	c_system_state = SM_STATE_INIT;	
/*end  System variable......*/
/*ADC Result data*/
volatile uint8_t sendflag   = 0, sample_cnt = MAX_SAMPLE_READY;
volatile uint8_t adc_channel   = ADC_MUXPOS_AIN2_gc;
volatile uint8_t adc_result = 0;
volatile  uint16_t		uc_adc_CalBuf[2][MAX_SAMPLE_CNT] = {0};
volatile  uint16_t		uc_ac_speed_coil = 0;
/*ADC Result data*/
volatile uint8_t c_zcross_tmr_buf =0;
volatile uint8_t c_zcross_half_cnt =0;
volatile uint8_t c_zcross_fire_trig =0;
void verify_error()
{
	//printf("Verify Error\r\n");
	while (1)
	;
};

void cmd_error()
{
//	printf("Command Error\r\n");
	while (1)
	;
};
void system_reset(void)
{
    uc_ac_speed_coil =0;
	sendflag   = 0, 
	sample_cnt = 0;
	adc_channel = ADC_MUXPOS_AIN2_gc;
	c_system_flag = 0;
	
}

void AComparator_change(void)
{
	uc_ac_speed_coil++;

}
// ADC functions
void ADC_start_conversion()
{
	
//	ADC0_COMMAND = ADC_STCONV_bm; // ADC0 starts conversion
	if((c_system_flag&SYSTEM_FLAG_ADC_START)==SYSTEM_FLAG_ADC_START)
	{
		 if(sendflag == 0)
		 {
			 sendflag = 1;	 
			 ADC_0_start_conversion(adc_channel);			
			 
		 }
	}
}



void ADC_get_result()
{
	//adc_result = ADC0.RESL;
	
	if (ADC_0_is_conversion_done())
	{
		adc_result =   ADC_0_get_conversion_result();//ADC0.RES;	
		if(sendflag == 1)
		{ 		
			//Triac_toggle_level();   //***test purpuse, if for demo should delete it			
			if(sample_cnt < MAX_SAMPLE_CNT)
			{
					if(adc_channel==ADC_MUXPOS_AIN2_gc)
					{	
						uc_adc_CalBuf[ADCRST_RC3][sample_cnt] = adc_result;
						adc_channel = ADC_MUXPOS_AIN5_gc ;	}
					else if(adc_channel==ADC_MUXPOS_AIN5_gc)
					{	
						uc_adc_CalBuf[ADCRST_VR][sample_cnt] = adc_result;
						adc_channel = ADC_MUXPOS_AIN11_gc ;	}
					else if(adc_channel==ADC_MUXPOS_AIN11_gc)
					{	
						uc_adc_CalBuf[ADCRST_ISYNC][sample_cnt++] = adc_result;
						adc_channel = ADC_MUXPOS_AIN2_gc ;
						c_system_flag &= SYSTEM_FLAG_ADC_START_CLR;
					}					   	 			
			}
			if(sample_cnt >= MAX_SAMPLE_CNT)
			{				
				sample_cnt = 0;
				c_system_flag |= SYSTEM_FLAG_ADC_READY;	// Set the flag, that indicate the AD is already read
				//	c_flag_indacitor = c_flag_indacitor | PID_IND;				
			}
			sendflag = 0;
		}
	}
}
void RTC_proc(void)
{
	c_tmr_100us++;
	/**/
	if(c_zcross_half_cnt > TIMING_ZCROSS_DETECT_LIMIT)
	{
		c_system_flag &= SYSTEM_FLAG_ZCROSS_READY_CLR;			 
		c_system_state = SM_STATE_STANDBY;	
	}else
	{
		c_zcross_half_cnt ++;		
	}
  	ADC_start_conversion();  // if the zero cross is in high status	  
	Fire_cal_timing(); 
	
}

void ZeroCross_proc(void)
{
	//if((c_system_flag&SYSTEM_FLAG_ZCROSS_READY)==0)
	{
		RTC.CNT = 0;		// after the zero cross signal is detected, and it reset the timer,		// which is used to make stable output waveform.
		RTC.INTFLAGS = RTC_OVF_bm;					// Reset the overflow interrupt flag
	}
	c_zcross_tmr_buf = c_zcross_half_cnt;					// load the time value to the zero-cross time buffer
	c_zcross_half_cnt = 0;
	c_system_flag |= SYSTEM_FLAG_ZCROSS_READY;
	c_system_flag &= SYSTEM_FLAG_FIRE_ON_CLR;
	c_system_flag &= SYSTEM_FLAG_TRIAC_RETRY_CLR ;
	if(Zcross_control_get_level() == 1 )					// when the zero-cross is in high status, and it start to read the NTC level (AD)
	{
	
		Set_CurrentSpeed(uc_ac_speed_coil);
		uc_ac_speed_coil = 0; 
		
	}else
	{
	
	}
}

void test_write_eeprom_byte()
{
	uint16_t adr = 35; // Write to eeprom address 35
	// Test write of one byte to eeprom: Write 0xba
//	printf("Test WE1\r\n"); 
	status = FLASH_0_write_eeprom_byte(adr, 0xca);
	if (status != NVM_OK) {
		cmd_error();
	}

	// Read back and check result
	rb = FLASH_0_read_eeprom_byte(adr);
	if (rb != 0xca) {
		verify_error();
	}
//	printf("--> Passed\r\n");
}

void test_write_eeprom_block()
{
	uint16_t adr = 89;
	uint8_t                   i;
	// Test write of many bytes to eeprom.
//	printf("Test WE2\r\n");

	// Write 4 bytes to this eeprom address
	status = FLASH_0_write_eeprom_block(adr, wdata, 4);
	if (status != NVM_OK) {
		cmd_error();
	}

	// Read back and check result
	FLASH_0_read_eeprom_block(adr, rdata, 4);

	for (i = 0; i < 4; i++)
	if (rdata[i] != expected[i + 1]) {
		verify_error();
	}

//	printf("--> Passed\r\n");
}


int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init(); 
	sei();
	Startup_Test(); /* Class B Test */
	
	// Read value of BOOTEND from FUSE section of memory map
	uint8_t bootend = FUSE.BOOTEND;
	if (bootend < 8) {		 
		cmd_error();
	}

	// Test illegal write, BOOT should not be allowed to write to BOOT
	status = FLASH_0_write_flash_byte(0, NULL, 1);
	if (status != NVM_ERROR) { 
	 
		cmd_error();
	}
	test_write_eeprom_byte();
	test_write_eeprom_block();
	system_reset();
	ADC_Data_Init();
	c_system_state = SM_STATE_INIT;	
	c_system_flag |= SYSTEM_FLAG_ADC_START;
	/* Replace with your application code */
	while (1) {
		ADC_Data_process();		
		ZCross_process();
        mainloop_OP();
		__watchdog_reset();
		
	}
}
