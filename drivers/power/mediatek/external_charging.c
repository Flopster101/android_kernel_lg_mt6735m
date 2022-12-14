/*****************************************************************************
 *
 * Filename:
 * ---------
 *    external_charging.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   08 Apr 2014 07:47:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/kernel.h>
#include <mach/battery_common.h>
#include <mach/charging.h>
#include "cust_charging.h"
#include <mach/mt_boot.h>
#include <mach/battery_meter.h>
#ifdef CONFIG_LGE_PM_BATTERY_ID
#include <mach/lge_battery_id.h>
#endif
#ifdef CONFIG_MACH_LGE
#include <mach/board_lge.h>
#endif

// ============================================================ //
//define
// ============================================================ //
//cut off to full
#define POST_CHARGING_TIME	30 * 60 // 30mins

// ============================================================ //
//global variable
// ============================================================ //
kal_uint32 g_bcct_flag=0;
kal_uint32 g_bcct_value=0;
CHR_CURRENT_ENUM g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_thermal_CC_value = CHARGE_CURRENT_0_00_MA;
kal_uint32 g_usb_state = USB_UNCONFIGURED;
static bool usb_unlimited=false;
BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_350000_V;

// ============================================================ //
// function prototype
// ============================================================ //

// ============================================================ //
//extern variable
// ============================================================ //
extern int g_platform_boot_mode;
#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
extern int get_AtCmdChargingModeOff(void);
#endif


// ============================================================ //
//extern function
// ============================================================ //

// ============================================================ //
void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	battery_log(BAT_LOG_CRTI, "[BATTERY_SetUSBState] in FPGA/EVB, no service\n");
#else
	if ((usb_state_value < USB_SUSPEND) || (usb_state_value > USB_CONFIGURED)) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Fail! Restore to default value\n");
		usb_state_value = USB_UNCONFIGURED;
	} else {
		battery_log(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Success! Set %d\n", usb_state_value);
		g_usb_state = usb_state_value;
	}
#endif
}

kal_uint32 get_charging_setting_current(void)
{
	return g_temp_CC_value;
}

bool get_usb_current_unlimited(void)
{
	if (BMT_status.charger_type == STANDARD_HOST || BMT_status.charger_type == CHARGING_HOST)
		return usb_unlimited;
	else
		return false;
}

void set_usb_current_unlimited(bool enable)
{
	usb_unlimited = enable;
}

void select_charging_current_bcct(void)
{
	if (g_temp_thermal_CC_value < g_temp_input_CC_value) {
		g_temp_input_CC_value = g_temp_thermal_CC_value;
		battery_log(BAT_LOG_CRTI, "[BATTERY] bcct input current limit = %dmA\n",
			g_temp_input_CC_value / 100);

	}
}

static void pchr_turn_on_charging (void);
kal_uint32 set_bat_charging_current_limit(int current_limit)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] set_bat_charging_current_limit (%d)\n", current_limit);

	if (current_limit != -1) {
		g_bcct_flag=1;
		g_bcct_value = current_limit;
		g_temp_thermal_CC_value = current_limit * 100;
	} else {
		//change to default current setting
		g_bcct_flag=0;
		g_temp_thermal_CC_value = CHARGE_CURRENT_2000_00_MA;
	}

	//wake_up_bat();
	pchr_turn_on_charging();

	return g_bcct_flag;
}

void select_charging_current(void)
{
#ifdef CONFIG_MACH_LGE
	/* Set MAX Charging current when factory cable connected */
	if (lge_get_board_cable() == LT_CABLE_56K ||
		lge_get_board_cable() == LT_CABLE_130K ||
		lge_get_board_cable() == LT_CABLE_910K) {
		g_temp_input_CC_value = CHARGE_CURRENT_2000_00_MA;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		return;
	}
#endif

	switch (BMT_status.charger_type) {
	case CHARGER_UNKNOWN:
		g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
		g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
		break;
	case NONSTANDARD_CHARGER:
		g_temp_input_CC_value = NON_STD_AC_CHARGER_CURRENT;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		break;
	case STANDARD_CHARGER:
		g_temp_input_CC_value = AC_CHARGER_CURRENT;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		break;
	default:
		g_temp_input_CC_value = USB_CHARGER_CURRENT;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		break;
	}

#ifdef CONFIG_LGE_PM_CHARGING_SCENARIO
	/* Decrease charging current when battery is too hot */
	if (BMT_status.bat_charging_state == CHR_HOLD) {
		g_temp_CC_value = CHARGE_CURRENT_400_00_MA;
		battery_log(BAT_LOG_CRTI, "[BATTERY] lcs current limit = %dmA\n",
			g_temp_CC_value / 100);

	}
#endif

#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	/* To avoid power-off in ATS test, increase charging current */
	if (BMT_status.usb_current_max_enabled) {
		if (g_temp_input_CC_value < CHARGE_CURRENT_700_00_MA) {
			g_temp_input_CC_value = CHARGE_CURRENT_700_00_MA;
			battery_log(BAT_LOG_CRTI, "[BATTERY] USB current MAX Mode enabled.\n");
		}
	}
#endif
	if (g_bcct_flag) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] select_charging_current_bcct\n");
		select_charging_current_bcct();
	}
}

static int recharging_check(void)
{
	if (BMT_status.bat_charging_state != CHR_BATFULL)
		return KAL_FALSE;

	if (BMT_status.SOC < 100)
		return KAL_TRUE;

	if (BMT_status.bat_vol <= RECHARGING_VOLTAGE)
		return KAL_TRUE;

	return KAL_FALSE;
}

static int eoc_check(void)
{
	int data = BMT_status.bat_vol;
	int eoc = KAL_FALSE;

	if (!BMT_status.bat_exist)
		return KAL_FALSE;

	if (BMT_status.bat_vol <= RECHARGING_VOLTAGE)
		return KAL_FALSE;

	battery_charging_control(CHARGING_CMD_GET_CHARGING_STATUS, &data);
	eoc = data;

	battery_log(BAT_LOG_CRTI, "[BATTERY] EOC = %d\n", eoc);
	if (eoc == KAL_TRUE)
		return KAL_TRUE;

	return KAL_FALSE;
}

static void pchr_turn_on_charging (void)
{
	kal_uint32 charging_enable = KAL_TRUE;

	if (BMT_status.bat_charging_state == CHR_ERROR) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Charger Error, turn OFF charging\n");
		charging_enable = KAL_FALSE;
	} else if ((g_platform_boot_mode==META_BOOT) || (g_platform_boot_mode==ADVMETA_BOOT)) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] In meta or advanced meta mode, disable charging.\n");
		charging_enable = KAL_FALSE;
	} else if (BMT_status.bat_charging_state == CHR_BATFULL) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Full, turn OFF charging\n");
		charging_enable = KAL_FALSE;
	} else {
		/*HW initialization*/
		battery_charging_control(CHARGING_CMD_INIT, NULL);

		battery_log(BAT_LOG_FULL, "charging_hw_init\n" );

		/* Set Charging Current */
		select_charging_current();

		if (g_temp_CC_value == CHARGE_CURRENT_0_00_MA || g_temp_input_CC_value == CHARGE_CURRENT_0_00_MA) {
			charging_enable = KAL_FALSE;

			battery_log(BAT_LOG_CRTI, "[BATTERY] charging current is set 0mA, turn off charging\n");
		} else {
			cv_voltage = V_CC2TOPOFF_THRES * 1000;

			battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT, &g_temp_input_CC_value);
			battery_charging_control(CHARGING_CMD_SET_CURRENT, &g_temp_CC_value);
			battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);
		}
	}

	/* enable/disable charging */
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	battery_log(BAT_LOG_FULL, "[BATTERY] pchr_turn_on_charging(), enable=%d\n", charging_enable);
}

PMU_STATUS BAT_PreChargeModeAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] Pre-CC mode charge, timer=%u on %u\n", BMT_status.PRE_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time += BAT_TASK_PERIOD;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	if ( BMT_status.bat_vol > V_PRE2CC_THRES ) {
		BMT_status.bat_charging_state = CHR_CC;
	}

	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] CC mode charge, timer=%u on %u !!\n", BMT_status.CC_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time += BAT_TASK_PERIOD;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	if (eoc_check() == KAL_TRUE) {
		if (BMT_status.SOC >= 100) {
			BMT_status.bat_charging_state = CHR_BATFULL;
		}
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	}

	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_BatteryFullAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] Battery full !!\n");

	BMT_status.bat_full = KAL_TRUE;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.bat_in_recharging_state = KAL_FALSE;

	if (recharging_check() == KAL_TRUE) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Re-charging !!\n");

		BMT_status.bat_in_recharging_state = KAL_TRUE;
		BMT_status.bat_full = KAL_FALSE;
		BMT_status.bat_charging_state = CHR_CC;
	}

	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_BatteryHoldAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] Hold mode !!\n");

	/* Enable charger */
	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_BatteryStatusFailAction(void)
{
	kal_uint32 charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] BAD Battery status... Charging Stop !!\n");

	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	return PMU_STATUS_OK;
}

void mt_battery_charging_algorithm()
{
	battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);

#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
	if (get_AtCmdChargingModeOff()) {
		if (BMT_status.bat_charging_state != CHR_ERROR) {
			BMT_status.bat_charging_state = CHR_ERROR;
			pchr_turn_on_charging();
		}

		battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
		return;
	}
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID
	if (BMT_status.bat_exist && lge_get_battery_id() == BATT_ID_UNKNOWN) {
		/* Invalid battery inserted. Stop charging */
		BMT_status.bat_charging_state = CHR_ERROR;
	}
#endif

	battery_log(BAT_LOG_CRTI, "[BATTERY] Charging State = 0x%x\n", BMT_status.bat_charging_state);
	switch (BMT_status.bat_charging_state) {
	case CHR_PRE :
		/* Default State */
		BAT_PreChargeModeAction();
		break;
	case CHR_CC :
		/* Normal Charging */
		BAT_ConstantCurrentModeAction();
		break;
	case CHR_BATFULL:
		/* End of Charging */
		BAT_BatteryFullAction();
		break;
	case CHR_HOLD:
		/* Current decreased by OTP */
		BAT_BatteryHoldAction();
		break;
	case CHR_ERROR:
		/* Charging Stop by OTP */
		BAT_BatteryStatusFailAction();
		break;
	default:
		battery_log(BAT_LOG_CRTI, "[BATTERY] Should not be in here. Check the code.\n");
		BMT_status.bat_charging_state = CHR_PRE;
		break;
	}

	battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
}

