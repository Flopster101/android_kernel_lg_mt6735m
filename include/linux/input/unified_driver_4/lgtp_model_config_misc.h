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
 *    File  : lgtp_model_config_misc.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined(_LGTP_MODEL_CONFIG_MISC_H_)
#define _LGTP_MODEL_CONFIG_MISC_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/

/* Hardware(Board) Configuration */
#if defined(TOUCH_MODEL_E1)

#define GPIO_TOUCH_RESET            (GPIO62 | 0x80000000)
#define GPIO_TOUCH_RESET_M_GPIO  	GPIO_MODE_00
//#define TOUCH_GPIO_INTERRUPT		10
//#define TOUCH_GPIO_MAKER_ID			55
#define TOUCH_IRQ_FLAGS (IRQF_ONESHOT)
#define USE_EARLY_FB_EVENT_BLANK
#define USE_EARLY_FB_EVENT_UNBLANK

#else
#error "Model should be defined"
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
void TouchVddPowerModel(int isOn);
void TouchVioPowerModel(int isOn);
void TouchAssertResetModel(void);
TouchDeviceControlFunction * TouchGetDeviceControlFunction(int index);
void TouchGetModelConfig(TouchDriverData *pDriverData);


#endif /* _LGTP_MODEL_CONFIG_MISC_H_ */

/* End Of File */

