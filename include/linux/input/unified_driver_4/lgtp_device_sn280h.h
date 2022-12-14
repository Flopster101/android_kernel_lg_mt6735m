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
 *    File  	: lgtp_device_s3320.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_SN280H_H_ )
#define _LGTP_DEVICE_SN280H_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/
#include <linux/input/unified_driver_4/lgtp_common.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <TestLimits_sn280h.h>

/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/
#define TPD_I2C_ADDRESS				0x3C
#define I2C_DEVICE_ADDRESS_LEN		2
#define MAX_TRANSACTION_LENGTH		8
#define MAX_I2C_TRANSFER_SIZE		(MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)


/* Knock On/Code */
#define KNOCK_ON_STATUS		        0x0082
#define KNOCK_TAP_COUNT	    	    0x0083
#define KNOCK_STATUS	    	    0x00C0
#define KNOCK_TAP_THON		        0x00C1
#define KNOCK_EXCEPT_PALM_ONCH	    0x00C5
#define KNOCK_WAKEUP_INTERVAL	    0x00C9
#define KNOCK_TAPOFF_TIMEOUT	    0x00D2
#define KNOCK_ON_TAP_COUNT	        0x00D4
#define KNOCK_ON_REPORT_DELAY       0x00D5
#define KNOCK_CODE_TAPOFF_TIMEOUT	0x00DB
#define KNOCK_CODE_TAP_COUNT		0x00DD


/* Touch Event Type */
#define TYPE_PRESS		            0x01
#define TYPE_MOVE		            0x02
#define TYPE_RELEASE		        0x03

/* Key Event Type */
#define KEY_PRESSED		            1
#define KEY_RELEASED		        0
#define CANCEL_KEY		            0xFF

#define SCREEN_MAX_X    	        1280
#define SCREEN_MAX_Y    	        800
#define PRESS_MAX       	        255

#define EVENT_NONE		            0x00
#define EVENT_ABS		            0x01
#define EVENT_KEY	    	        0x02
#define EVENT_KNOCK_ON              0x03
#define EVENT_KNOCK_CODE            0x04
#define EVENT_KNOCK_OVER            0x05
#define EVENT_HOVERING_NEAR         0x06
#define EVENT_HOVERING_FAR          0x07
#define EVENT_GEST		            0x04
#define EVENT_MOUSE		            0x08

#define FWSTATUS_NORMAL		        0x00
#define FWSTATUS_INITREQ	        0xFF
#define FWSTATUS_CHFAIL	    	    0xfe
#define FWSTATUS_CALFAIL	        0xfd

#define I2C_DEVICE_ADDRESS_LEN	    2
#define MAX_TRANSACTION_LENGTH	    8

#define FW_STATUS                   0x0000
#define INT_INFORM                  0x0001
#define TOUCH_VALID                 0x0002
#define TOUCH_KEY                   0x0003
#define TOUCH_FINGER                0x0005 

#define FW_VERSION_REG		        0x0080

/* Define flash type for firmware update */
#define REG_ISP_VAL_EEPROM      0xBABA
#define REG_ISP_VAL_ERROR       0xDEAD

/* Common register address for firmware update */
#define REG_ISP_MODE			0xF102	// ISP mode control
#define REG_ISP_MODE_BUS		0xF104	// ISP mode bus functions 
#define REG_ISP_MODE_ENABLE		0xF108	// ISP mode enable 
#define REG_ISP_MEM_TYPE		0xF100	// MEM TYPE: EEPROM/EFLASH

/* EFLASH register address for firmware update */
#define REG_CMD_FLASH_AUTH		0xF400	// 0x0100 : get eFlash approach authority
#define REG_CMD_FLASH_CON_EN	0xF402	// 0x0000 : enable eFlash controller
#define REG_CMD_FLASH_COMMAND	0xF404	// 0x0200 : erase eFlash, 0x0000 : write eFlash
#define REG_CMD_FLASH_BUSY		0xF408	// [15] bit is busy flag for eflash operation.

/* EEPROM register address for firmware update */
#define REG_CMD_EER_XPROT		0xF400	// 0x0000 : get EEPROM approach authority
#define REG_CMD_EER_PDOWN		0xF402	// 0x0000 : enable EEPROM controller
#define REG_CMD_EER_RESET		0xF404	// 0x0000 : set EEPROM state to ACTIVE 
#define REG_CMD_EER_MODE		0xF406	// 0x0007 : Enable EEPROM erase function 
										// 0x0008 : Enable EEPROM write function
#define REG_CMD_EER_XEN			0xF408	// 0x0001 : EEPROM Excution Enable 
#define REG_CMD_EER_EXTEND		0xF40E	// 0x0001 : EEPROM chip select control 
#define REG_CMD_EER_CSCON		0xF410	// 0x0001 : EEPROM chip select control 
#define REG_CMD_EER_STATE		0xF412	// [2] bit is busy flag for EEPROM operation. 
                                        // the value 0 of [2] bit means EEPROM busy 

/* Firmware update information */
#define TSC_EEPROM_PAGE_SIZE	64 
#define FW_UPGRADE_RETRY_COUNT	2
#define BUSY_CHECK_RETRY_COUNT	20


/* Touch status & data register address */
#define	REG_TS_STATUS			0xE000
#define REG_TS_GEST_STATUS		0xE040
#define	REG_TS_DATA_BASE		0xE002
//#define	REG_TS_DATA(x)			(((x * 6) >> 8) + REG_TS_DATA_BASE)
#define REG_TS_DATA(x)          (REG_TS_DATA_BASE+(6*x))
#define REG_KNOCK_DATA_BASE     0x4006
#define REG_KNOCK_DATA(x)       ((x * 4) + REG_KNOCK_DATA_BASE)


/* Touch Firmware version Field*/
#define REG_FIRMWARE_VERSION        0x3EE0
#define REG_IC_CODE                 0x3FC0      //8Byte SN310
#define TSC_FLASH_FW_VER_POS	    0x3FCA	    //2Byte
#define TSC_PANEL_TEST_VER          0x3FCC      //2Byte 0x0001
#define REG_DEVIATION_CODE          0x3FD0      //2Byte 0x000A
#define REG_PROJECT_CODE            0x3FCE      //2Byte 0x0073
#define REG_MP_FIELD                0x3FD2      //2Byte 0x0001 Test : 0, MP : 1

/* Touch event about action */
#define	TS_EVENT_UNKNOWN            0x00
#define	TS_EVENT_PRESS              0x01
#define	TS_EVENT_MOVE               0x02
#define	TS_EVENT_RELEASE            0x03

//#define MAX_BUTTONS               4
#define MAX_FINGER_NUM              2
//#define MAX_CHANNEL 34
#define TOUCH_TX_CHANNEL              4
#define TOUCH_RX_CHANNEL              8

/* Touch value about Gesture Action*/
#define GESTURE_ACTION              0x8000
#define HOVER_STATUS_REG            0x4002
#define LPWG_STATUS_REG             0x4004
#define TAP_DATA_REG                0x4006
#define HOVER_FAR                   0x0101
#define HOVER_NEAR                  0x0201   

/* Touch raw data */
#define CAP_REFERENCE               0X4110
#define CAP_DELTA                   0x40A0

/****************************************************************************
* Type Definitions
****************************************************************************/


struct semisense_ts_data {
	struct i2c_client	*client;
	TouchState currState;
	LpwgSetting lpwgSetting;
};

typedef struct status_reg__t {
    u32 ts_cnt      :4;
    u32 proximity   :4;
    u32 button      :5;
    u32 reserved2   :2;
    u32 gesture     :1;
}__attribute__ ((packed)) status_reg_t;

typedef union status_reg__u {
    u16 uint;
    status_reg_t bits;
}__attribute__ ((packed)) status_reg_u;

typedef struct data_reg__t {
    u16 packet0;
    u16 packet1;
    u16 packet2;
}__attribute__ ((packed)) data_reg_t;

typedef struct {
    u32 status[MAX_FINGER_NUM]; /*press, release, unknown*/
    u32 id[MAX_FINGER_NUM];
    u32 x[MAX_FINGER_NUM];
    u32 y[MAX_FINGER_NUM];  
    u32 area[MAX_FINGER_NUM];
    u32 pressure[MAX_FINGER_NUM];
    /* Not support */
    //u8 key_index[MAX_BUTTONS];
    //u8 key_status[MAX_BUTTONS];
} touch_info;

#if 0
typedef struct finger__t {
    u32 status;     // true : ts data updated, false : no update data
    u32 event;      // ts event type
    u32 id;         // ts received id
    u32 x;          // ts data x
    u32 y;          // ts data y
    u32 area;       // ts finger area
    u32 pressure;       // ts finger pressure
}__attribute__ ((packed)) finger_t;
#endif
typedef union data_reg__u {
    u32             uint;
    data_reg_t      bits;
}__attribute__ ((packed)) data_reg_u;

/****************************************************************************
* Enum
****************************************************************************/

/* ISP flash memory type.*/
enum 
{
    E_MEM_TYPE_EFLASH = 0,
    E_MEM_TYPE_EEPROM = 1,
    E_MEM_TYPE_MAX,
};

/* Operation mode define for firmware upgrade. */
enum {
	E_FLASH_OPMODE_READ = 0,
	E_FLASH_OPMODE_WRITE = 1,
	E_FLASH_OPMODE_ERASE = 2,
	E_FLASH_OPMODE_MAX
};

/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/
#define SWAP_16BITS(p) ((((p) & 0xFF00) >> 8) | (((p) & 0x00FF) << 8))


/****************************************************************************
* Global Function Prototypes
****************************************************************************/

#endif /* _LGTP_DEVICE_SN280H_H_ */

/* End Of File */

