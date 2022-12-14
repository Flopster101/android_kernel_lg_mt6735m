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
 *    File  : lgtp_project_setting.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined(_LGTP_PROJECT_SETTING_H_)
#define _LGTP_PROJECT_SETTING_H_


/****************************************************************************
* Project Setting ( Model )
****************************************************************************/
#if defined(CONFIG_ARCH_MT6735M)

#if defined(CONFIG_TOUCHSCREEN_SN280H)
#define TOUCH_MODEL_E1
#endif

#endif

/****************************************************************************
* Available Feature supported by Unified Driver
* If you want to use it, define it inside of model feature
****************************************************************************/
/* #define ENABLE_HOVER_DETECTION */
/* #define ENABLE_TOUCH_AT_OFF_CHARGING */

/****************************************************************************
* Project Setting ( AP Solution / AP Chipset / Touch Device )
****************************************************************************/
#if defined(TOUCH_MODEL_E1)

/* AP Solution */
#define TOUCH_PLATFORM_MTK
/* AP Chipset */
#define TOUCH_PLATFORM_MT6735P
/* Touch Device */
#define TOUCH_DEVICE_SN280H
#define TOUCH_DEVICE_FT6X36
/* Driver Feature */
#define ENABLE_HOVER_DETECTION
/* IC Type */
#define TOUCH_TYPE_ONCELL

#else
#error "Model should be defined"
#endif

#endif /* _LGTP_PROJECT_SETTING_H_ */

/* End Of File */

