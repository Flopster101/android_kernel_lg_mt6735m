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
 *    File  : lgtp_model_config_i2c.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[CONFIG]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_4/lgtp_common.h>
#include <linux/input/unified_driver_4/lgtp_model_config_i2c.h>



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

#if defined ( TOUCH_MODEL_E1 )
static struct of_device_id Sn280h_MatchTable[] = {
    { .compatible = "unified_driver4,sn280h", },
	{ },
};

static struct of_device_id Ft6x36_MatchTable[] = {
    { .compatible = "unified_driver4,fx6x36", },
	{ },
};

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
#if defined ( TOUCH_PLATFORM_MTK )
int TouchGetDeviceSlaveAddress(int index)
{
	int slave_addr = 0;
	
	#if defined ( TOUCH_MODEL_E1 )
	if (index == FIRST_MODULE)
		slave_addr = TOUCH_SN280H_I2C_SLAVE_ADDR;
	else if(index == SECOND_MODULE)
		slave_addr = TOUCH_FT6X36_I2C_SLAVE_ADDR;
	#else
	#error "Model should be defined"
	#endif

	return slave_addr;
}
#endif

struct of_device_id * TouchGetDeviceMatchTable(int index)
{
	struct of_device_id * match_table = NULL;
	
	#if defined ( TOUCH_MODEL_E1 )
	if (index == FIRST_MODULE)
		match_table = Sn280h_MatchTable;
	else if (index == SECOND_MODULE)
		match_table = Ft6x36_MatchTable;
	#else
	#error "Model should be defined"
	#endif

	return match_table;
	
}


/* End Of File */

