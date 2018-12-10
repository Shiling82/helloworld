/*
 * main.h
 *
 * Created: 11/27/2018 11:56:08
 *  Author: xlliu
 */ 


#ifndef MAIN_H_
#define MAIN_H_

typedef enum SYSTEM_STATE{
	SM_STATE_INIT = 0,
	SM_STATE_REFERENCE,
	SM_STATE_NORMAL,
	SM_STATE_STANDBY,
	SM_STATE_TEST,
}SYSTEM_STATE;
#define  SYSTEM_FLAG_ADC_READY					0x0001
#define  SYSTEM_FLAG_ADC_READY_CLR						0xFFFE
#define  SYSTEM_FLAG_ZCROSS_READY				0x0002
#define  SYSTEM_FLAG_ZCROSS_READY_CLR					0xFFFD

#define  SYSTEM_FLAG_TRIAC_Ready					0x0004
#define  SYSTEM_FLAG_TRIAC_Ready_CLR						0xFFFB
#define  SYSTEM_FLAG_TRIAC_RETRY				0x0008
#define  SYSTEM_FLAG_TRIAC_RETRY_CLR					0xFFF7

#define  SYSTEM_FLAG_ErP_ON						0x0010
#define  SYSTEM_FLAG_ErP_ON_CLR							0xFFEF
#define  SYSTEM_FLAG_RELAY_ON					0x0020
#define  SYSTEM_FLAG_RELAY_ON_CLR						0xFFDF
#define  SYSTEM_FLAG_ADC_START					0x0040
#define  SYSTEM_FLAG_ADC_START_CLR						0xFFBF
#define  SYSTEM_FLAG_VR_CHANGED					0x0080
#define  SYSTEM_FLAG_VR_CHANGED_CLR						0xFF7F
#define  SYSTEM_FLAG_FIRE_ON					0x0100
#define  SYSTEM_FLAG_FIRE_ON_CLR						0xFEFF


#define  TIMING_ZCROSS_DETECT_LIMIT				253

#define  MAX_SAMPLE_CNT				3
#define  MAX_SAMPLE_READY          0xEF
typedef enum ADC_ARRAY{
	ADCRST_RC3 = 0,
	ADCRST_VR,
	ADCRST_ISYNC,
	ADCRST_HALL,
	ADCRST_NTC,
}ADC_ARRAY;

extern volatile  unsigned int		uc_adc_CalBuf[2][MAX_SAMPLE_CNT];
extern volatile unsigned int	c_system_flag;
extern  volatile unsigned int	c_system_state;
extern volatile unsigned char c_zcross_tmr_buf;
#endif /* MAIN_H_ */