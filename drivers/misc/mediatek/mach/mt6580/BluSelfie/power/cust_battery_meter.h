#ifndef _CUST_BATTERY_METER_H
#define _CUST_BATTERY_METER_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
//#define SOC_BY_AUXADC
//#define SOC_BY_HW_FG
#define SOC_BY_SW_FG
//#define HW_FG_FORCE_USE_SW_OCV

//#define CONFIG_DIS_CHECK_BATTERY
//#define FIXED_TBAT_25

/* ADC Channel Number */
#if 0
#define CUST_TABT_NUMBER 17
#define VBAT_CHANNEL_NUMBER      7
#define ISENSE_CHANNEL_NUMBER	 6
#define VCHARGER_CHANNEL_NUMBER  4
#define VBATTEMP_CHANNEL_NUMBER  5
#endif
/* ADC resistor  */
#define R_BAT_SENSE 4					
#define R_I_SENSE 4						
#define R_CHARGER_1 330
#define R_CHARGER_2 39

#define TEMPERATURE_T0             110
#define TEMPERATURE_T1             0
#define TEMPERATURE_T2             25
#define TEMPERATURE_T3             50
#define TEMPERATURE_T              255 // This should be fixed, never change the value

#define FG_METER_RESISTANCE 	0

/* Qmax for battery  */
#if 1///lc mike_zhu 20150526 for ch
#define Q_MAX_POS_50	2028///    2007 2014 2035 2054
#define Q_MAX_POS_25	1980///1996 1997 1961 1967
#define Q_MAX_POS_0	1360//	1358
#define Q_MAX_NEG_10	1122//1120

#define Q_MAX_POS_50_H_CURRENT	2007///  1988 1990 2024 2027 
#define Q_MAX_POS_25_H_CURRENT	1940//1958 1956 1922 1925
#define Q_MAX_POS_0_H_CURRENT	 1314//1304
#define Q_MAX_NEG_10_H_CURRENT	840//830
#else
#define Q_MAX_POS_50	1482
#define Q_MAX_POS_25	1438
#define Q_MAX_POS_0		953
#define Q_MAX_NEG_10	641

#define Q_MAX_POS_50_H_CURRENT	1466
#define Q_MAX_POS_25_H_CURRENT	1416
#define Q_MAX_POS_0_H_CURRENT	  506
#define Q_MAX_NEG_10_H_CURRENT	446
#endif

/* Discharge Percentage */

#define OAM_D5		 1		//  1 : D5,   0: D2 
/* battery meter parameter */
#define CHANGE_TRACKING_POINT
#define CUST_TRACKING_POINT  1
#define CUST_R_SENSE         200
#define CUST_HW_CC 		    0
#define AGING_TUNING_VALUE   103
#define CUST_R_FG_OFFSET    0

#define OCV_BOARD_COMPESATE	0 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
#define CAR_TUNE_VALUE		100 //1.00


/* HW Fuel gague  */
#define CURRENT_DETECT_R_FG	10  //1mA
#define MinErrorOffset       1000
#define FG_VBAT_AVERAGE_SIZE 18
#define R_FG_VALUE 			10 // mOhm, base is 20

#define CUST_POWERON_DELTA_CAPACITY_TOLRANCE	30///lc mike_zhu 20150728    ///40
#define CUST_POWERON_LOW_CAPACITY_TOLRANCE	5
#define CUST_POWERON_MAX_VBAT_TOLRANCE			90
#define CUST_POWERON_DELTA_VBAT_TOLRANCE		5///30
#define CUST_POWERON_DELTA_HW_SW_OCV_CAPACITY_TOLRANCE 10

/* Disable Battery check for HQA */
#ifdef CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
#define FIXED_TBAT_25
#endif

/* Dynamic change wake up period of battery thread when suspend*/
#define VBAT_NORMAL_WAKEUP		3600		//3.6V
#define VBAT_LOW_POWER_WAKEUP		3500		//3.5v
#define NORMAL_WAKEUP_PERIOD		5400 		//90 * 60 = 90 min
#define LOW_POWER_WAKEUP_PERIOD		300		//5 * 60 = 5 min
#define CLOSE_POWEROFF_WAKEUP_PERIOD	30	//30 s

#endif	//#ifndef _CUST_BATTERY_METER_H
