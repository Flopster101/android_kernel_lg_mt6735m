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
 *    File  : lgtp_model_config_i2c.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined(_LGTP_MODEL_CONFIG_I2C_H_)
#define _LGTP_MODEL_CONFIG_I2C_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/
#if defined ( TOUCH_PLATFORM_MTK )

#if defined ( TOUCH_MODEL_E1 )
#define TOUCH_I2C_USE
#define TOUCH_I2C_BUS_NUM 			1
#define TOUCH_LU202X_I2C_SLAVE_ADDR 0x0E
#define TOUCH_SN280H_I2C_SLAVE_ADDR 0x3C
#define TOUCH_FT6X36_I2C_SLAVE_ADDR 0x38
#define TOUCH_I2C_ADDRESS_8BIT
#else
#error "Model should be defined"
#endif

#endif



/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/


/****************************************************************************
* Global Function Prototypes
****************************************************************************/
#if defined ( TOUCH_PLATFORM_MTK )
int TouchGetDeviceSlaveAddress(int index);
#endif

#if defined ( TOUCH_PLATFORM_MTK )
struct of_device_id * TouchGetDeviceMatchTable(int index);
#endif


#endif /* _LGTP_MODEL_CONFIG_I2C_H_ */

/* End Of File */

