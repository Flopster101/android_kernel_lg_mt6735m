/***************************************************************************
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *    File  : lgtp_model_config_misc.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[CONFIG]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_4/lgtp_common.h>
#include <linux/input/unified_driver_4/lgtp_model_config_misc.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_power.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/


/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Variables
****************************************************************************/

#if defined(TOUCH_MODEL_E1)
	extern TouchDeviceControlFunction Sn280h_Func;
	extern TouchDeviceControlFunction Ft6x36_Func;
#else
	#error "Model should be defined"
#endif


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/

/****************************************************************************
* Global Functions
****************************************************************************/
/* this function is for platform api so do not use it in other module */
TouchDeviceControlFunction * TouchGetDeviceControlFunction(int index)
{
	TouchDeviceControlFunction *pControlFunction = NULL;
	#if defined(TOUCH_MODEL_E1)
	if (index == FIRST_MODULE)
		pControlFunction = &Sn280h_Func;
	else if (index == SECOND_MODULE)
		pControlFunction = &Ft6x36_Func;
	#else
	#error "Model should be defined"
	#endif

	return pControlFunction;
}

void TouchGetModelConfig(TouchDriverData *pDriverData)
{
	TouchModelConfig *pConfig = &pDriverData->mConfig;
    #if defined(TOUCH_MODEL_E1)
	pConfig->button_support = 0;
	pConfig->number_of_button = 0;
	pConfig->button_name[0] = 0;
	pConfig->button_name[1] = 0;
	pConfig->button_name[2] = 0;
	pConfig->button_name[3] = 0;
	pConfig->max_x = 480;
	pConfig->max_y = 854;
	pConfig->max_pressure = 0xff;
	pConfig->max_width = 15;
	pConfig->max_orientation = 1;
	pConfig->max_id = 10;

	pConfig->protocol_type = MT_PROTOCOL_B;
    
	#else
	#error "Model should be defined"
	#endif

	TOUCH_LOG("======== Model Configuration ( Begin ) ========\n");
	TOUCH_LOG("button_support=%d\n", pConfig->button_support);
	TOUCH_LOG("number_of_button=%d\n", pConfig->number_of_button);
	TOUCH_LOG("button_name[0]=%d\n", pConfig->button_name[0]);
	TOUCH_LOG("button_name[1]=%d\n", pConfig->button_name[1]);
	TOUCH_LOG("button_name[2]=%d\n", pConfig->button_name[2]);
	TOUCH_LOG("button_name[3]=%d\n", pConfig->button_name[3]);
	TOUCH_LOG("max_x=%d\n", pConfig->max_x);
	TOUCH_LOG("max_y=%d\n", pConfig->max_y);
	TOUCH_LOG("max_pressure=%d\n", pConfig->max_pressure);
	TOUCH_LOG("max_width=%d\n", pConfig->max_width);
	TOUCH_LOG("max_orientation=%d\n", pConfig->max_orientation);
	TOUCH_LOG("max_id=%d\n", pConfig->max_id);
	TOUCH_LOG("protocol_type=%s", (pConfig->protocol_type == MT_PROTOCOL_A) ?\
			 "MT_PROTOCOL_A\n" : "MT_PROTOCOL_B\n");
	TOUCH_LOG("======== Model Configuration ( End ) ========\n");

	return;

}

void TouchVddPowerModel(int isOn)
{
	#if defined(TOUCH_MODEL_E1)
		TouchPowerPMIC(isOn,MT6328_POWER_LDO_VGP1, VOL_3000);
	#else
		#error "Model should be defined"
	#endif
}

void TouchVioPowerModel(int isOn)
{
    #if defined(TOUCH_MODEL_E1)
 	    /* there is no power control */
	#else
		#error "Model should be defined"
    #endif
}


/* End Of File */

