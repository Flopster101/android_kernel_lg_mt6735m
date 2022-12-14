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
 *    File  	: mediatek\custom\common\kernel\alsps\apds9130.c
 *    Author(s)   :  Kang Jun Mo < junmo.kang@lge.com >
 *    Description :
 *
 ***************************************************************************/

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#define MT6735

#ifdef MT6516
#include <mach/mt6516_devs.h>
#include <mach/mt6516_typedefs.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_pll.h>
#endif

#ifdef MT6573
#include <mach/mt6573_devs.h>
#include <mach/mt6573_typedefs.h>
#include <mach/mt6573_gpio.h>
#include <mach/mt6573_pll.h>
#endif


#ifdef MT6582
#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#ifdef MT6735
#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#ifdef MT6516
#define POWER_NONE_MACRO MT6516_POWER_NONE
#endif

#ifdef MT6573
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6582
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6735
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "apds9130.h"

#include <cust_gpio_usage.h>

#include <cust_eint.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define APDS9130_DEV_NAME     "APDS9130"

#define APDS9130_ENABLE_REG 0x00
#define APDS9130_PTIME_REG  0x02
#define APDS9130_WTIME_REG  0x03
#define APDS9130_PILTL_REG  0x08
#define APDS9130_PILTH_REG  0x09
#define APDS9130_PIHTL_REG  0x0A
#define APDS9130_PIHTH_REG  0x0B
#define APDS9130_PERS_REG 0x0C
#define APDS9130_CONFIG_REG 0x0D
#define APDS9130_PPCOUNT_REG  0x0E
#define APDS9130_CONTROL_REG  0x0F
#define APDS9130_REV_REG  0x11
#define APDS9130_ID_REG   0x12
#define APDS9130_STATUS_REG 0x13
#define APDS9130_PDATAL_REG 0x18
#define APDS9130_PDATAH_REG 0x19

#define CMD_BYTE  0x80
#define CMD_WORD  0xA0
#define CMD_SPECIAL 0xE0

#define CMD_CLR_PS_INT  0xE5
#define CMD_CLR_ALS_INT 0xE6
#define CMD_CLR_PS_ALS_INT  0xE7

#define APDS9130_PINT 0x20
#define APDS9130_PVALID 0x02

#define APDS9130_PSAT 0x40  /* PS saturation bit check */

/****************************************************************************
 * Macros
 ****************************************************************************/
#define SENSOR_TAG                  "[LGE_Proximity]"
//#define DEBUG 1
	
#ifdef DEBUG
#define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
#define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt, __FUNCTION__,##args)
#define SENSOR_DBG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[D]""%s : "fmt, __FUNCTION__,##args)
#else
#define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
#define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt, __FUNCTION__,##args)
#define SENSOR_DBG(fmt, args...)    NULL
#endif
#define CONFIG_OF_DT

#ifdef CONFIG_OF_DT
static const struct of_device_id psensor_of_match[] = {
	{ .compatible = "mediatek,als_ps", },
	{},
};
#endif

/****************************************************************************
* Type Definitions
****************************************************************************/
typedef enum
{
	PS_NEAR = 0,
	PS_FAR = 1,
	PS_UNKNOWN = 2
} PS_STATUS;

struct apds9130_priv
{
	struct i2c_client *client;
	struct work_struct eint_work;

	unsigned int activate; /* 1 = activate, 0 = deactivate */

	/* variables to store APDS9130 register value - begin */
	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;
	/* variables to store APDS9130 register value - end */

	/* CAUTION : in case of strong sunlight, ps_state is not same as ps_th_status. */
	unsigned int ps_status; /* current status of poximity detection : 0 = near, 1 = far */
	unsigned int ps_th_status; /* current threshold status of poximity detection : 0 = near, 1 = far */

	/* threshold value to detect "near-to-far" event */
	unsigned int ps_th_near_low;
	unsigned int ps_th_near_high;

	/* threshold value to detect "far-to-near" event */
	unsigned int ps_th_far_low;
	unsigned int ps_th_far_high;

	unsigned int ps_cross_talk; /* a result value of calibration. it will be used to compensate threshold value. */

	#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
	#endif
};


/****************************************************************************
* Variables
****************************************************************************/
static struct i2c_client *apds9130_i2c_client = NULL; /* for general file I/O service. will be init on apds9130_i2c_probe() */
static struct apds9130_priv *g_apds9130_ptr = NULL; /* for interrupt service call. will be init on apds9130_i2c_probe() */
static struct platform_driver apds9130_alsps_driver;

#if defined(TARGET_MT6582_Y70)
static int Target_Pdata = 250; /* parameter for taget pdata setting */
static int NearToFar = 90; /* parameter for far detection */
#elif defined(TARGET_MT6582_B2L)
static int Target_Pdata = 300; /* parameter for taget pdata setting */
static int NearToFar = 100; /* parameter for far detection */
#else
static int Target_Pdata = 150; /* parameter for taget pdata setting */
static int NearToFar = 50; /* parameter for far detection */
#endif


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
extern void mt_eint_unmask ( unsigned int line );
extern void mt_eint_mask ( unsigned int line );
extern void mt_eint_set_polarity ( unsigned int eint_num, unsigned int pol );
extern void mt_eint_set_hw_debounce ( unsigned int eint_num, unsigned int ms );
extern unsigned int mt_eint_set_sens ( unsigned int eint_num, unsigned int sens );
//extern void mt_eint_registration ( unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void ( EINT_FUNC_PTR ) ( void ),
	//								   unsigned int is_auto_umask );
void mt_eint_registration(unsigned int eint_num, unsigned int flag,
              void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);


#ifdef MT6516
extern void MT6516_EINTIRQUnmask ( unsigned int line );
extern void MT6516_EINTIRQMask ( unsigned int line );
extern void MT6516_EINT_Set_Polarity ( kal_uint8 eintno, kal_bool ACT_Polarity );
extern void MT6516_EINT_Set_HW_Debounce ( kal_uint8 eintno, kal_uint32 ms );
extern kal_uint32 MT6516_EINT_Set_Sensitivity ( kal_uint8 eintno, kal_bool sens );
extern void MT6516_EINT_Registration ( kal_uint8 eintno, kal_bool Dbounce_En, kal_bool ACT_Polarity, void ( EINT_FUNC_PTR ) ( void ),
									   kal_bool auto_umask );
#endif

#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS) //for auto detect
static int apds9130_local_init(void);
static int apds9130_local_remove(void);
static int apds9130_init_flag = -1;//0<==>OK, -1<==>fail
static struct sensor_init_info apds9130_init_info = {
	.name = "APDS9130",
	.init = apds9130_local_init,
	.uninit = apds9130_local_remove,
};
#endif

/****************************************************************************
* Local Function Prototypes
****************************************************************************/
void apds9130_eint_func ( void );
/*
#define GPIO_PROXIMITY_INT         (GPIO82 | 0x80000000)//GPIO82
#define GPIO_PROXIMITY_INT_M_GPIO   GPIO_MODE_00
#define GPIO_PROXIMITY_INT_M_EINT   GPIO_PROXIMITY_INT_M_GPIO
*/
#define CUST_EINT_PROXIMITY_NUM              82
#define CUST_EINT_PROXIMITY_DEBOUNCE_CN      0
#define CUST_EINT_PROXIMITY_TYPE			CUST_EINTF_TRIGGER_FALLING
#define CUST_EINT_PROXIMITY_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE

/****************************************************************************
* Local Functions
****************************************************************************/

//==========================================================
// Platform(AP) dependent functions
//==========================================================
static void apds9130_setup_eint ( void )
{
	SENSOR_FUN ();

#if 0 //not use
	/* Configure GPIO settings for external interrupt pin  */
	mt_set_gpio_dir ( GPIO_ALS_EINT_PIN, GPIO_DIR_IN );
	mt_set_gpio_mode ( GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT );
	mt_set_gpio_pull_enable ( GPIO_ALS_EINT_PIN, TRUE );
	mt_set_gpio_pull_select ( GPIO_ALS_EINT_PIN, GPIO_PULL_UP );

	/* Configure external interrupt settings for external interrupt pin */
	mt_eint_set_sens ( CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE );
	mt_eint_set_polarity ( CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY );
	mt_eint_set_hw_debounce ( CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN );

	mt_eint_registration ( CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, apds9130_eint_func, 0);

	/* Mask external interrupt to avoid un-wanted interrupt. Unmask it after initialization of APDS9130 */
	mt_eint_mask ( CUST_EINT_ALS_NUM );
#else
	/* Configure GPIO settings for external interrupt pin  */
	mt_set_gpio_mode(GPIO_PROXIMITY_INT, GPIO_PROXIMITY_INT_M_EINT);
	mt_set_gpio_dir(GPIO_PROXIMITY_INT, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_PROXIMITY_INT, GPIO_PULL_DISABLE);

	/* Configure external interrupt settings for external interrupt pin */
	mt_eint_set_hw_debounce(CUST_EINT_PROXIMITY_NUM, CUST_EINT_PROXIMITY_DEBOUNCE_EN);
	mt_eint_registration(CUST_EINT_PROXIMITY_NUM, EINTF_TRIGGER_FALLING, apds9130_eint_func, 0);

	/* Mask external interrupt to avoid un-wanted interrupt. Unmask it after initialization of APDS9130 */
	mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );
#endif
}
extern unsigned int system_rev;

#if 0 //not use
static void apds9130_main_power ( struct alsps_hw *hw, unsigned int on )
{
	static unsigned int main_power_on = 0xFF;

	SENSOR_FUN ();

	if ( main_power_on != on )
	{
		if ( on )
		{
			if ( !hwPowerOn ( hw->power_id, hw->power_vol, "APDS9130" ) )
			{
				SENSOR_ERR ( "failed to power on ( VCAM_AF )\n" );
				goto EXIT_ERR;
			}

			SENSOR_LOG( "turned on the power ( VCAM_AF )" );
		}
		else
		{
			if ( !hwPowerDown ( hw->power_id, "APDS9130" ) )
			{
				SENSOR_ERR ( "failed to power down ( VCAM_AF )\n" );
				goto EXIT_ERR;
			}

			SENSOR_LOG( "turned off the power ( VCAM_AF )" );
		}

		main_power_on = on;
	}

	EXIT_ERR:
		return;
}

#endif

#if 0 //not use
static void apds9130_led_power ( unsigned int on )
{
	static unsigned int led_power_on = 0xFF;

	SENSOR_FUN ();

	if ( led_power_on != on )
	{
		if ( on )
		{
		#ifdef MT6575
			if ( !hwPowerOn ( MT65XX_POWER_LDO_VCAMA, VOL_2800, "APDS9130LEDA" ) )
				{
					SENSOR_ERR ( "failed to power on ( VCAMA )\n" );
		#else
		if ( !hwPowerOn ( MT65XX_POWER_LDO_VCAM_AF, VOL_3000, "APDS9130LEDA" ) )
				{
					SENSOR_ERR ( "failed to power on ( VCAM_AF )\n" );
		#endif

					goto EXIT_ERR;
				}
		#ifdef MT6575
			SENSOR_LOG("turned on the power ( VCAMA )");
		#else
			SENSOR_LOG("turned on the power ( VCAM_AF )");
		#endif

		}
		else
		{
		#ifdef MT6575
			if ( !hwPowerDown ( MT65XX_POWER_LDO_VCAMA, "APDS9130LEDA" ) )
				{
					SENSOR_ERR ( "failed to power down ( VCAMA )\n" );
		#else
		if ( !hwPowerDown ( MT65XX_POWER_LDO_VCAM_AF, "APDS9130LEDA" ) )
				{
					SENSOR_ERR ( "failed to power down ( VCAM_AF )\n" );
		#endif
					goto EXIT_ERR;
				}
		#ifdef MT6575
			SENSOR_LOG("turned off the power ( VCAMA )\n");
		#else
			SENSOR_LOG("turned off the power ( VCAM_AF )\n");
		#endif
		}

		led_power_on = on;
	}

	EXIT_ERR:
		return;
}
#endif
//==========================================================
// APDS9130 Register Read / Write Funtions
//==========================================================
static int apds9130_write_cmd ( struct i2c_client *client, u8 val )
{
	int res = 0;

	res = i2c_master_send ( client, &val, 1 );
	if ( res == 1 )
	{
		SENSOR_DBG ( "I2C write ( val=0x%02x )\n", val );
		return APDS9130_SUCCESS;
	}
	else
	{
		SENSOR_ERR ( "failed to write to APDS9130 ( err=%d, cmd=0x%02x )\n", res, val );
		return APDS9130_ERR_I2C;
	}
}

static int apds9130_write_byte ( struct i2c_client *client, u8 reg, u8 val )
{
	int res = 0;
	u8 pBuf[2] = { 0 };

	pBuf[0] = reg;
	pBuf[1] = val;

	res = i2c_master_send ( client, pBuf, 2 );
	if ( res == 2 )
	{
		SENSOR_DBG ( "I2C write ( reg=0x%02x, val=0x%02x )\n", reg, val );
		return APDS9130_SUCCESS;
	}
	else
	{
		SENSOR_ERR ( "failed to write to APDS9130 ( err=%d, reg=0x%02x, val=0x%02x )\n", res, reg, val );
		return APDS9130_ERR_I2C;
	}

}

static int apds9130_write_word ( struct i2c_client *client, u8 reg, u16 val )
{
	int res = 0;
	u8 pBuf[3] = { 0 };

	pBuf[0] = reg ;
	pBuf[1] = val & 0xFF ;
	pBuf[2] = ( val >> 8 ) & 0xFF ;

	res = i2c_master_send ( client, pBuf, 3 );
	if ( res == 3 )
	{
		SENSOR_DBG ( "I2C write ( reg=0x%02x, val=0x%04x )\n", reg, val );
		return APDS9130_SUCCESS;
	}
	else
	{
		SENSOR_ERR ( "failed to write to APDS9130 ( err=%d, reg=0x%02x, val=0x%04x )\n", res, reg, val );
		return APDS9130_ERR_I2C;
	}

}

static int apds9130_read_byte ( struct i2c_client *client, u8 reg, u8 *pVal )
{
	int res = 0;

	if ( pVal == NULL )
	{
		SENSOR_ERR ( "invalid input ( pVal=NULL )" );
		goto EXIT_ERR;
	}

	res = i2c_master_send ( client, &reg, 1 );
	if ( res != 1 )
	{
		SENSOR_ERR ( "apds9130_read_byte error i2c_master_send (1)....\n" );
		goto EXIT_ERR;
	}

	res = i2c_master_recv ( client, pVal, 1 );
	if ( res != 1 )
	{
		SENSOR_ERR ( "apds9130_read_byte error i2c_master_recv (2)....\n" );
		goto EXIT_ERR;
	}

	SENSOR_DBG ( "I2C read ( reg=0x%02x, val=0x%02x )\n", reg, *pVal );
	return APDS9130_SUCCESS;

	EXIT_ERR:
	SENSOR_ERR ( "failed to read from APDS9130 ( err=%d, reg=0x%02x )\n", res, reg );
	return APDS9130_ERR_I2C;
}

static int apds9130_read_word ( struct i2c_client *client, u8 reg, u16 *pVal )
{
	int res = 0;
	u8 pBuf[2] = { 0 };

	if ( pVal == NULL )
	{
		SENSOR_ERR ( "invalid input ( pVal=NULL )" );
		goto EXIT_ERR;
	}

	res = i2c_master_send ( client, &reg, 1 );
	if ( res != 1 )
	{
		goto EXIT_ERR;
	}

	res = i2c_master_recv ( client, pBuf, 2 );
	if ( res != 2 )
	{
		goto EXIT_ERR;
	}

	*pVal = ( ( u16 ) pBuf[1] << 8 ) | pBuf[0] ;

	SENSOR_DBG ( "I2C read ( reg=0x%02x, val=0x%04x )\n", reg, *pVal );
	return APDS9130_SUCCESS;

	EXIT_ERR:
	SENSOR_ERR ( "failed to read from APDS9130 ( err=%d, reg=0x%02x )\n", res, reg );
	return APDS9130_ERR_I2C;

}

//==========================================================
// APDS9130 Basic Read / Write Funtions
//==========================================================
static int apds9130_set_enable ( struct i2c_client *client, int enable )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9130_write_byte ( client, CMD_BYTE | APDS9130_ENABLE_REG, ( u8 ) enable );
	if ( res == APDS9130_SUCCESS )
	{
		obj->enable = enable;
	}

	return res;
}

static int apds9130_set_ptime ( struct i2c_client *client, int ptime )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9130_write_byte ( client, CMD_BYTE | APDS9130_PTIME_REG, ( u8 ) ptime );
	if ( res == APDS9130_SUCCESS )
	{
		obj->ptime = ptime;
	}

	return res;
}

static int apds9130_set_wtime ( struct i2c_client *client, int wtime )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9130_write_byte ( client, CMD_BYTE | APDS9130_WTIME_REG, ( u8 ) wtime );
	if ( res == APDS9130_SUCCESS )
	{
		obj->wtime = wtime;
	}

	return res;
}

static int apds9130_set_pilt ( struct i2c_client *client, int threshold )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9130_write_word ( client, CMD_WORD | APDS9130_PILTL_REG, ( u16 ) threshold );
	if ( res == APDS9130_SUCCESS )
	{
		obj->pilt = threshold;
	}

	return res;
}

static int apds9130_set_piht ( struct i2c_client *client, int threshold )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9130_write_word ( client, CMD_WORD | APDS9130_PIHTL_REG, ( u16 ) threshold );
	if ( res == APDS9130_SUCCESS )
	{
		obj->piht = threshold;
	}

	return res;
}

static int apds9130_set_pers ( struct i2c_client *client, int pers )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9130_write_byte ( client, CMD_BYTE | APDS9130_PERS_REG, ( u8 ) pers );
	if ( res == APDS9130_SUCCESS )
	{
		obj->pers = pers;
	}

	return res;
}

static int apds9130_set_config ( struct i2c_client *client, int config )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9130_write_byte ( client, CMD_BYTE | APDS9130_CONFIG_REG, ( u8 ) config );
	if ( res == APDS9130_SUCCESS )
	{
		obj->config = config;
	}

	return res;
}

static int apds9130_set_ppcount ( struct i2c_client *client, int ppcount )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9130_write_byte ( client, CMD_BYTE | APDS9130_PPCOUNT_REG, ( u8 ) ppcount );
	if ( res == APDS9130_SUCCESS )
	{
		obj->ppcount = ppcount;
	}

	return res;
}

static int apds9130_set_control ( struct i2c_client *client, int control )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9130_write_byte ( client, CMD_BYTE | APDS9130_CONTROL_REG, ( u8 ) control );
	if ( res == APDS9130_SUCCESS )
	{
		obj->control = control;
	}

	return res;
}

static int apds9130_get_status ( struct i2c_client *client, int *pData )
{
	int res = 0;

	res = apds9130_read_byte ( client, CMD_BYTE | APDS9130_STATUS_REG, ( u8 * ) pData );
	if ( res == APDS9130_SUCCESS )
	{
		SENSOR_LOG ( "STATUS=0x%02x\n", ( u8 ) * pData );
	}

	return res;
}

static int apds9130_get_pdata ( struct i2c_client *client, int *pData )
{
	int res = 0;

	res = apds9130_read_word ( client, CMD_WORD | APDS9130_PDATAL_REG, ( u16 * ) pData );
	if ( res == APDS9130_SUCCESS )
	{
		SENSOR_DBG ( "PDATA=0x%04x\n", ( u16 ) * pData );
	}

	return res;
}

//LGE_CHANGE_S 2013-06-28 chulho.park@lge.com add at%proximity=3
static int apds9130_get_deivceid( struct i2c_client *client, int *pData )
{
	int res = 0;

	res = apds9130_read_byte ( client, CMD_BYTE | APDS9130_ID_REG, ( u8* ) pData );
	if ( res == APDS9130_SUCCESS )
	{
		SENSOR_DBG ( "DEVICEID=0x%02x\n", ( u8 ) * pData );
	}

	return res;
}
//LGE_CHANGE_E 2013-06-28 chulho.park@lge.com add at%proximity=3


static int apds9130_clear_interrupt ( struct i2c_client *client )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9130_write_cmd ( client, ( u8 ) CMD_CLR_PS_ALS_INT );
	if ( res == APDS9130_SUCCESS )
	{
		SENSOR_DBG ( "APDS9130 interrupte was cleared\n" );
	}

	return res;
}

//==========================================================
// APDS9130 Data Processign Funtions
//==========================================================
static int apds9130_decide_ps_state ( struct i2c_client *client, int pdata, int int_status )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int ps_status = obj->ps_status;

	if ( obj->ps_status == PS_FAR )
	{
	    /* Even saturation bit is set by apds9130, pdata is correctly update. So Do not check saturation bit */
		if ( ( pdata >= obj->ps_th_far_high ) && ( ( int_status & APDS9130_PSAT ) != APDS9130_PSAT ) )
		{
			ps_status = PS_NEAR;
			SENSOR_LOG ( "PS = NEAR\n" );
		}
		else
		{
			SENSOR_ERR ( "Unknown Event State\n" );
		}
	}
	else
	{
		if ( pdata <= obj->ps_th_near_low )
		{
			ps_status = PS_FAR;
			SENSOR_LOG ( "PS = FAR\n" );
		}
		else
		{
			SENSOR_ERR ( "Unknown Event State\n" );
		}
	}

	return ps_status;

}

static long apds9130_initialize ( struct i2c_client *client  )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
	int id = 0;

	res = apds9130_read_byte ( client, CMD_BYTE | APDS9130_ID_REG, ( u8 * ) &id );
	if ( res != APDS9130_SUCCESS )
	{
		SENSOR_ERR ( "failed to read Device ID : %d and it means I2C error happened\n", ( u8 ) id );
		return res;
	}

	SENSOR_LOG ( "APDS9130 Device ID = 0x%02x\n", ( u8 ) id );

	obj->activate = 0;

	/* disable proximity */
	apds9130_set_enable ( client, 0 );

	/* initialize registers of proximity */
	apds9130_set_ptime ( client, 0xFF ); /* 2.72ms Prox integration time */
	apds9130_set_wtime ( client, 0xDC ); /* 100ms Wait time */
	apds9130_set_pers ( client, 0x20 ); /* 2 consecutive Interrupt persistence */
	apds9130_set_config ( client, 0 );

#if defined(TARGET_MT6582_B2L)
    apds9130_set_ppcount ( client, 0x0c ); /* 10-Pulse for proximity */
	apds9130_set_control ( client, 0x24 ); /* 50mA, IR-diode, 2X PGAIN */
#else
	apds9130_set_ppcount ( client, 0x0a ); /* 10-Pulse for proximity */
	apds9130_set_control ( client, 0x24 ); /* 100mA, IR-diode, 2X PGAIN */
#endif


	/* crosstalk value shall be set by LGP Server using I/O so init here to 150 */
    obj->ps_cross_talk = 150;

	/* initialize threshold value of PS */
	if ( obj->ps_cross_talk > 870 )
	{
		obj->ps_cross_talk = 870;
	}
	if ( obj->ps_cross_talk < 0 )
	{
		obj->ps_cross_talk = 0;
	}

	obj->ps_th_far_low = 0;
	obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
	obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;
	obj->ps_th_near_high = 1023;

	return res;

}


static long apds9130_enable ( struct i2c_client *client  )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	hwm_sensor_data sensor_data;
	int res = 0;
	int status = 0;
	int pdata = 0;

	/* enable ADC but block interrupt */
	apds9130_set_enable ( client, 0x0D );

	mdelay ( 100 );

	apds9130_get_status( client, &status);

	if ( ( status & APDS9130_PVALID ) == APDS9130_PVALID )
	{
		/* read sensor data */
		apds9130_get_pdata ( client, &pdata );

		/* decide current PS threshold state and set PS thershold to proper range */
		if ( pdata >= obj->ps_th_far_high )
		{
			obj->ps_th_status = PS_NEAR;
			apds9130_set_pilt ( client, obj->ps_th_near_low );
			apds9130_set_piht ( client, obj->ps_th_near_high );
			SENSOR_LOG ( "PS_TH=NEAR\n" );
		}
		else
		{
			obj->ps_th_status = PS_FAR;
			apds9130_set_pilt ( client, obj->ps_th_far_low );
			apds9130_set_piht ( client, obj->ps_th_far_high );
			SENSOR_LOG ( "PS_TH=FAR\n" );
		}

		/* decide current PS status */
		if ( ( pdata >= obj->ps_th_far_high ) && ( (status&0x40) != 0x40 ) )
		{
			obj->ps_status = PS_NEAR;
			SENSOR_LOG ( "Enable PS Status=NEAR\n" );
		}
		else
		{
			obj->ps_status = PS_FAR;
			SENSOR_LOG ( "Enable PS Status=FAR\n" );
		}

	}
	else
	{
		SENSOR_ERR("ADC value is invalid so set to PS_FAR\n");

		obj->ps_th_status = PS_FAR;
		obj->ps_status = PS_FAR;
		apds9130_set_pilt ( client, obj->ps_th_far_low );
		apds9130_set_piht ( client, obj->ps_th_far_high );
	}

	/* inform to upper layer ( hwmsen ) */
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	sensor_data.value_divide = 1;
	sensor_data.values[0] = obj->ps_status;
	if ( ( res = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data ) ) )
	{
		SENSOR_ERR ( "failed to send inform ( err = %d )\n", res );
	}

	/* enable APDS9130 */
	res = apds9130_set_enable ( client, 0x2D );
	if ( res == APDS9130_SUCCESS )
	{
		/* unmask external interrupt */
		mt_eint_unmask ( CUST_EINT_PROXIMITY_NUM );
		SENSOR_LOG ( "APDS9130 was enabled\n" );

	}
	else
	{
		SENSOR_ERR ( "failed to enable APDS9130\n" );
	}

	return res;

}

static long apds9130_disable ( struct i2c_client *client  )
{
	int res = 0;

	/* mask external interrupt */
	mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );

	/* disable APDS9130 */
	res = apds9130_set_enable ( client, 0 );
	if ( res == APDS9130_SUCCESS )
	{
		SENSOR_LOG ( "APDS9130 was disabled\n" );
	}
	else
	{
		SENSOR_ERR ( "failed to disable APDS9130\n" );
	}

	return res;

}

void apds9130_swap(int *x, int *y)
{
     int temp = *x;
     *x = *y;
     *y = temp;
}

static int apds9130_do_calibration ( struct i2c_client *client, int *value )
{
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	unsigned int sum_of_pdata = 0;
	int temp_pdata[20] = {0};
	int temp_state[20] = {0};
	unsigned int i=0;
	unsigned int j=0;
	unsigned int ArySize = 20;
	unsigned int cal_check_flag = 0;
	unsigned int old_enable = 0;

	//apds9130_led_power ( 1 ); //not use
	old_enable = obj->enable;

RE_CALIBRATION:
	sum_of_pdata = 0;

	/* Enable PS and Mask interrupt */
	apds9130_set_enable ( client, 0x0D );

	mdelay ( 50 );

	/* Read pdata */
	for ( i = 0 ; i < 20 ; i++ )
	{
		apds9130_get_status ( client, &( temp_state[i] ) );
		apds9130_get_pdata ( client, &( temp_pdata[i] ) );
		mdelay ( 6 );
	}

	#if defined ( APS_DEBUG )
	SENSOR_LOG ( "State Value = " );
	for ( i = 0 ; i < 20 ; i++ )
	{
		SENSOR_LOG ( "%d ", temp_state[i] );
	}
	SENSOR_LOG ( "\n" );
	SENSOR_LOG ( "Read Value = " );
	for ( i = 0 ; i < 20 ; i++ )
	{
		SENSOR_LOG ( "%d ", temp_pdata[i] );
	}
	SENSOR_LOG ( "\n" );
	#endif

	/* sort pdata */
	for ( i = 0 ; i < ArySize - 1 ; i++ )
	{
		for ( j = i + 1 ; j < ArySize ; j++ )
		{
			if ( temp_pdata[i] > temp_pdata[j] )
			{
				apds9130_swap ( temp_pdata+i, temp_pdata+j );
			}
		}
	}

	#if defined(APS_DEBUG)
#if 1 /* branden.you@lge.com_20120924 */
	SENSOR_LOG ( "Read Value = " );
	for ( i = 0 ; i < 20 ; i++ )
	{
		SENSOR_LOG ( "%d ", temp_pdata[i] );
	}
	SENSOR_LOG ( "\n" );
#else
	SENSOR_DBG ( "Read Value = " );
	for ( i = 0 ; i < 20 ; i++ )
	{
		SENSOR_DBG ( "%d ", temp_pdata[i] );
	}
	SENSOR_DBG ( "\n" );
#endif
	#endif

	/* take ten middle data only */
	for ( i = 5 ; i < 15 ; i++ )
	{
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	/* calculate average */
	obj->ps_cross_talk = sum_of_pdata / 10;
	SENSOR_LOG ( "New calibrated cross talk = %d\n", obj->ps_cross_talk );

	/* check if average is acceptable */
	if ( obj->ps_cross_talk > 870 )
	{
		if ( cal_check_flag == 0 )
		{
			cal_check_flag = 1;
			goto RE_CALIBRATION;
		}
		else
		{
			SENSOR_ERR ( "failed to calibrate cross talk/n" );
			apds9130_set_enable ( client, 0x00 );
			apds9130_set_enable ( client, old_enable );
			*value = obj->ps_cross_talk;
			return -1;
		}
	}

	apds9130_set_enable ( client, 0x00 ); /* Power Off */
	apds9130_set_enable ( client, old_enable );
	//apds9130_led_power ( 0 );  //not use

	obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
	obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;

	/* we should store it to storage ( it should be free from factory reset ) but ATCI Demon will store it through LGP Demon */

	*value = obj->ps_cross_talk;
	return 0;
}

//==========================================================
// APDS9130 General Control Funtions
//==========================================================
static long apds9130_activate ( struct i2c_client *client, int enable )
{
	SENSOR_FUN ();

	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	long res = 0;

	if ( obj->activate != enable )
	{
		if ( enable )
		{
			//apds9130_led_power ( 1 );  //not use

			res = apds9130_enable ( client );
			if ( res == APDS9130_SUCCESS )
			{
				SENSOR_LOG ( "APDS9130 was enabled\n" );
			}
			else
			{
				SENSOR_ERR ( "failed to enable APDS9130\n" );
			}
		}
		else
		{
			res = apds9130_disable ( client );
			if ( res == APDS9130_SUCCESS )
			{
				SENSOR_LOG ( "APDS9130 was disabled\n" );
				//apds9130_led_power ( 0 ); //not use
			}
			else
			{
				SENSOR_ERR ( "failed to disable APDS9130\n" );
			}
		}

		if ( res == APDS9130_SUCCESS )
		{
			obj->activate = enable;
		}

	}

	return res;

}

//==========================================================
// APDS9130 Interrupt Service Routines
//==========================================================
void apds9130_eint_func ( void )
{
	SENSOR_FUN ();
	struct apds9130_priv *obj = g_apds9130_ptr;
	if ( !obj )
	{
		return;
	}
	schedule_work ( &obj->eint_work );
}

static void apds9130_eint_work ( struct work_struct *work )
{
	SENSOR_FUN ();
	struct apds9130_priv *obj = ( struct apds9130_priv * ) container_of ( work, struct apds9130_priv, eint_work );
	struct i2c_client *client = obj->client;
	hwm_sensor_data sensor_data;

	int err;

	int int_status = 0;
	int pdata = 0;
	int new_ps_status = 0;

	SENSOR_LOG ( "External interrupt happened\n" );

	/* read status register */
	apds9130_get_status ( client, &int_status );

	if ( ( int_status & APDS9130_PVALID ) != ( APDS9130_PVALID ) )
	{
		SENSOR_ERR("ADC value is not valid so just skip this interrupt");
		goto CLEAR_INTERRUPT;
	}

	/* disable ADC first */
	apds9130_set_enable ( client, 0x01 );

	/* read sensor data */
	apds9130_get_pdata ( client, &pdata );

   /* check data saturation and valid */
#if 0
	if (( int_status & APDS9130_PSAT ) == APDS9130_PSAT)
		{
          if( (pdata==0) || (pdata==0x03ff))
          	{
		     SENSOR_ERR("saturation with wrong pdata=0x%x, so just skip this interrupt\n",pdata);
		     goto CLEAR_INTERRUPT;
          	}
		}
#endif

	/* process PS interrupt */
	if ( ( int_status & APDS9130_PINT ) == APDS9130_PINT )
	{
		SENSOR_LOG ( "PS interrupt happened\n" );

		/* change threshold to avoid frequent interrupt */
		if ( obj->ps_th_status == PS_FAR )
		{
			if ( pdata >= obj->ps_th_far_high )
			{
				SENSOR_LOG ( "PS_TH = NEAR\n" );
				obj->ps_th_status = PS_NEAR;
				apds9130_set_pilt ( client, obj->ps_th_near_low );
				apds9130_set_piht ( client, obj->ps_th_near_high );
			}
		}
		else
		{
			if ( pdata <= obj->ps_th_near_low )
			{
				SENSOR_LOG ( "PS_TH = FAR\n" );
				obj->ps_th_status = PS_FAR;
				apds9130_set_pilt ( client, obj->ps_th_far_low );
				apds9130_set_piht ( client, obj->ps_th_far_high );
			}
		}

		/* make a decision if it is near or far */
		new_ps_status = apds9130_decide_ps_state( client, pdata, int_status );

		/* inform to upper layer ( hwmsen ), if status was changed */
		if ( new_ps_status != obj->ps_status )
		{
			obj->ps_status = new_ps_status;

			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			sensor_data.value_divide = 1;
			sensor_data.values[0] = obj->ps_status;
			if ( ( err = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data ) ) )
			{
				SENSOR_ERR ( "failed to send inform ( err = %d )\n", err );
			}
		}

	}

CLEAR_INTERRUPT:
	/* clear interrupt of proximity */
	apds9130_clear_interrupt ( client );

	/* unmask external interrupt */
   	mt_eint_unmask ( CUST_EINT_PROXIMITY_NUM );

	/* activate proximity */
	apds9130_set_enable ( client, 0x2D );

}

//==========================================================
// APDS9130 ADB Shell command function
//==========================================================
static ssize_t apds9130_show_cali_value ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%u\n", data->ps_cross_talk );
}

static ssize_t apds9130_store_cali_value ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9130_i2c_client;
	int ret;
	int data;

	ret = apds9130_do_calibration ( client, &data );

	return count;
}

static ssize_t apds9130_show_ptime ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->ptime );
}

static ssize_t apds9130_store_ptime ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9130_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9130_set_ptime ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9130_show_wtime ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->wtime );
}

static ssize_t apds9130_store_wtime ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9130_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9130_set_wtime ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9130_show_pilt ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->pilt );
}

static ssize_t apds9130_store_pilt ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9130_set_pilt ( client, val );

	if ( ret < 0 )
		return ret;

	obj->ps_th_near_low = val;

	return count;
}

static ssize_t apds9130_show_piht ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->piht );
}

static ssize_t apds9130_store_piht ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9130_set_piht ( client, val );

	if ( ret < 0 )
		return ret;

	obj->ps_th_far_high = val;

	return count;
}

static ssize_t apds9130_show_pers ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->pers );
}

static ssize_t apds9130_store_pers ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9130_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9130_set_pers ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9130_show_config ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->config );
}

static ssize_t apds9130_store_config ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9130_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9130_set_config ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9130_show_ppcount ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->ppcount );
}

static ssize_t apds9130_store_ppcount ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9130_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9130_set_ppcount ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9130_show_control ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->control );
}

static ssize_t apds9130_store_control ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9130_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9130_set_control ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9130_show_status ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = apds9130_i2c_client;
    int status = 0;

    apds9130_get_status ( client, &status );

    return sprintf ( buf, "0x%02x\n", status );
}

static ssize_t apds9130_show_pdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = apds9130_i2c_client;
    int data = 0;

    apds9130_get_pdata ( client, &data );

    return sprintf ( buf, "%d\n", data );
}

//LGE_CHANGE_S 2013-06-28 chulho.park@lge.com add at%proximity=3
static ssize_t apds9130_show_deviceid ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = apds9130_i2c_client;
    int data = 0;

    apds9130_get_deivceid ( client, &data );

    return sprintf ( buf, "%02x\n", ( u8 )data );
}
//LGE_CHANGE_E 2013-06-28 chulho.park@lge.com add at%proximity=3

static ssize_t apds9130_show_target_pdata ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", Target_Pdata);
}

static ssize_t apds9130_store_target_pdata ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	Target_Pdata = val;
	obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
	obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;

	ret = apds9130_set_piht ( client, obj->ps_th_far_high );
	ret = apds9130_set_pilt ( client, obj->ps_th_near_low );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9130_show_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9130_i2c_client;
	struct apds9130_priv *data = i2c_get_clientdata ( client );

    switch(data->activate) {
          case 0:
         	   return sprintf ( buf, "%s\n", "Proximity Disabled");
          case 1:
               return sprintf ( buf, "%s\n", "Proximity Enabled" );

           default:
               return sprintf ( buf, "%s\n", "Proximity Error" );
     	}

}

static ssize_t apds9130_store_enable ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9130_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret=0;

    switch(val) {
          case 0:
            ret = apds9130_activate ( client, 0 );
            break;
          case 1:
            ret = apds9130_activate ( client, 1 );
            break;

           default:
           	break;
     	}

	if ( ret < 0 )
		return ret;

	return count;
}


static DRIVER_ATTR ( cali, S_IWUSR | S_IRUGO, apds9130_show_cali_value, apds9130_store_cali_value );
static DRIVER_ATTR ( ptime, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_ptime, apds9130_store_ptime );
static DRIVER_ATTR ( wtime, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_wtime, apds9130_store_wtime );
static DRIVER_ATTR ( pilt, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_pilt, apds9130_store_pilt );
static DRIVER_ATTR ( piht, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_piht, apds9130_store_piht );
static DRIVER_ATTR ( pers, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_pers, apds9130_store_pers );
static DRIVER_ATTR ( config, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_config, apds9130_store_config );
static DRIVER_ATTR ( ppcount, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_ppcount, apds9130_store_ppcount );
static DRIVER_ATTR ( control, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_control, apds9130_store_control );
static DRIVER_ATTR ( status, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_status, NULL );
static DRIVER_ATTR ( pdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_pdata, NULL );
//LGE_CHANGE_S 2013-06-28 chulho.park@lge.com add at%proximity=3
static DRIVER_ATTR ( deviceid, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_deviceid, NULL );  //chulho.park
//LGE_CHANGE_E 2013-06-28 chulho.park@lge.com add at%proximity=3
static DRIVER_ATTR ( target_pdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_target_pdata, apds9130_store_target_pdata );
static DRIVER_ATTR ( enable, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9130_show_enable, apds9130_store_enable );


static struct driver_attribute *apds9130_attr_list[] = {
	&driver_attr_cali,		   /*show calibration data*/
	&driver_attr_ptime,
	&driver_attr_wtime,
	&driver_attr_pilt,
	&driver_attr_piht,
	&driver_attr_pers,
	&driver_attr_config,
	&driver_attr_ppcount,
	&driver_attr_control,
	&driver_attr_status,
	&driver_attr_pdata,
//LGE_CHANGE_S 2013-06-28 chulho.park@lge.com add at%proximity=3
	&driver_attr_deviceid,
//LGE_CHANGE_E 2013-06-28 chulho.park@lge.com add at%proximity=3
	&driver_attr_target_pdata,
	&driver_attr_enable,
};

static int apds9130_create_attr ( struct device_driver *driver )
{
	int idx;
	int err = 0;
	int num = ( int ) ( sizeof ( apds9130_attr_list ) / sizeof ( apds9130_attr_list[0] ) );

	if ( driver == NULL )
	{
		return -EINVAL;
	}

	for ( idx = 0 ; idx < num ; idx++ )
	{
		if ( err = driver_create_file ( driver, apds9130_attr_list[idx] ) )
		{
			SENSOR_ERR ( "driver_create_file (%s) = %d\n", apds9130_attr_list[idx]->attr.name, err );
			break;
		}
	}

	return err;
}

static int apds9130_delete_attr ( struct device_driver *driver )
{
	int idx;
	int err = 0;
	int num = ( int ) ( sizeof ( apds9130_attr_list ) / sizeof ( apds9130_attr_list[0] ) );

	if ( driver == NULL )
	{
		return -EINVAL;
	}

	for ( idx = 0 ; idx < num ; idx++ )
	{
		driver_remove_file ( driver, apds9130_attr_list[idx] );
	}

	return err;
}

//==========================================================
// APDS9130 Service APIs ( based on File I/O )
//==========================================================
static int apds9130_open ( struct inode *inode, struct file *file )
{
	SENSOR_FUN ();
	file->private_data = apds9130_i2c_client;

	if ( !file->private_data )
	{
		SENSOR_ERR ( "Invalid input paramerter\n" );
		return -EINVAL;
	}

	return nonseekable_open ( inode, file );
}

static int apds9130_release ( struct inode *inode, struct file *file )
{
	SENSOR_FUN ();
	file->private_data = NULL;
	return 0;
}

static long apds9130_unlocked_ioctl ( struct file *file, unsigned int cmd, unsigned long arg )
{
	SENSOR_FUN ();
	struct i2c_client *client = ( struct i2c_client * ) file->private_data;
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	long err = 0;
	void __user *ptr = ( void __user * ) arg;
	int dat;
	uint32_t enable;
	uint32_t crosstalk = 0;

	switch ( cmd )
	{
		case ALSPS_SET_PS_MODE:
			SENSOR_LOG ( "CMD = ALSPS_SET_PS_MODE\n" );
			if ( copy_from_user ( &enable, ptr, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			if ( enable )
			{
				if ( ( err = apds9130_activate ( obj->client, 1 ) ) )
				{
					SENSOR_ERR ( "failed to activate APDS9130 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			else
			{
				if ( ( err = apds9130_activate ( obj->client, 0 ) ) )
				{
					SENSOR_ERR ( "failed to deactivate APDS9130 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			break;

		case ALSPS_GET_PS_MODE:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_MODE\n" );
			enable = obj->activate;
			if ( copy_to_user ( ptr, &enable, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_DATA\n" );
			dat = obj->ps_status;
			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_RAW_DATA\n" );
			if ( err = apds9130_get_pdata ( obj->client, &dat ) )
			{
				goto err_out;
			}

			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		/*case ALSPS_GET_CALI:
			SENSOR_LOG ( "CMD = ALSPS_GET_CALI\n" );
			err = apds9130_do_calibration ( obj->client, &dat );
			if ( err == 0 )
			{
				if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
				{
					err = -EFAULT;
					goto err_out;
				}
			}
			break;

		case ALSPS_SET_CALI:
            SENSOR_LOG ( "CMD = ALSPS_SET_CALI\n" );
            if ( copy_from_user ( &crosstalk, ptr, sizeof ( crosstalk ) ) )
            {
                err = -EFAULT;
                goto err_out;
            }

			if ( ( crosstalk == 0x0000FFFF ) || ( crosstalk == 0 ) )
			{
				obj->ps_cross_talk = 150;
			}
			else
			{
				obj->ps_cross_talk = crosstalk;
			}

			obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
		    obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;
            break;

//LGE_CHANGE_S 2013-06-28 chulho.park@lge.com add at%proximity=3
		case ALSPS_GET_DEVICEID:
            SENSOR_LOG ( "CMD = ALSPS_GET_DEVICEID\n" );
			if ( err = apds9130_get_deivceid ( obj->client, &dat ) )
			{
				goto err_out;
			}

			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
//LGE_CHANGE_E 2013-06-28 chulho.park@lge.com add at%proximity=3*/
		default:
			SENSOR_ERR ( "Invalid Command = 0x%04x\n", cmd );
			err = -ENOIOCTLCMD;
			break;
	}

	err_out : return err;
}

static struct file_operations apds9130_fops = { .owner = THIS_MODULE, .open = apds9130_open, .release = apds9130_release,
												.unlocked_ioctl = apds9130_unlocked_ioctl,  };

static struct miscdevice apds9130_device = { .minor = MISC_DYNAMIC_MINOR, .name = "als_ps", .fops = &apds9130_fops,  };

//==========================================================
// APDS9130 Service APIs ( based on hwmsen Interface )
//==========================================================
static int apds9130_ps_operate ( void *self, uint32_t command, void *buff_in, int size_in, void *buff_out, int size_out, int *actualout )
{
	SENSOR_FUN ();
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data = NULL;
	struct apds9130_priv *obj = ( struct apds9130_priv * ) self;

	switch ( command )
	{
		case SENSOR_DELAY:
			SENSOR_LOG ( "CMD = SENSOR_DELAY\n" );
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				value = *( int * ) buff_in;
				if ( value )
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Enable )\n" );
					if ( err = apds9130_activate ( obj->client, 1 ) )
					{
						SENSOR_ERR ( "failed to activate APDS9130 ( err = %d )\n", err );
						return -1;
					}
				}
				else
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Disable )\n" );
					if ( err = apds9130_activate ( obj->client, 0 ) )
					{
						SENSOR_ERR ( "failed to deactivate APDS9130 ( err = %d )\n", err );
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			SENSOR_LOG ( "CMD = SENSOR_GET_DATA\n" );
			if ( ( buff_out == NULL ) || ( size_out < sizeof ( hwm_sensor_data ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				sensor_data->values[0] = obj->ps_status;
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;

		default:
			SENSOR_ERR ( "Invalid Command = %d\n", command );
			err = -1;
			break;
	}

	return err;
}

//==========================================================
// APDS9130 Initialization related Routines
//==========================================================
static int apds9130_init_client ( struct i2c_client *client )
{
	SENSOR_FUN ();
	struct apds9130_priv *obj = i2c_get_clientdata ( client );
	int err = 0;

	err = apds9130_initialize ( client );
	if ( err != APDS9130_SUCCESS )
	{
		SENSOR_ERR ( "failed to init APDS9130\n" );
	}

	return err;
}

/****************************************************************************
* I2C BUS Related Functions
****************************************************************************/

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void apds9130_early_suspend ( struct early_suspend *h )
{
	SENSOR_FUN ();
}

static void apds9130_late_resume ( struct early_suspend *h )
{
	SENSOR_FUN ();
}
#endif

static int apds9130_i2c_probe ( struct i2c_client *client, const struct i2c_device_id *id )
{
	SENSOR_FUN ();
	struct apds9130_priv *obj;
	struct hwmsen_object obj_ps;
	struct alsps_hw *hw = get_cust_alsps_hw ();
	int err = 0;

#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
	if(apds9130_init_flag==0)
	{
		SENSOR_LOG("Proximity Sensor already probed...just skip\n");
		return err;
	}	
	apds9130_setup_eint ();
#endif

	if ( !( obj = kzalloc ( sizeof ( *obj ), GFP_KERNEL ) ) )
	{
		err = -ENOMEM;
		goto exit;
	}
	memset ( obj, 0, sizeof ( *obj ) );

	obj->client = client;
	i2c_set_clientdata ( client, obj );

	g_apds9130_ptr = obj;
	apds9130_i2c_client = client;

	INIT_WORK ( &obj->eint_work, apds9130_eint_work );

	#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend = apds9130_early_suspend,
	obj->early_drv.resume = apds9130_late_resume,
	register_early_suspend ( &obj->early_drv );
	#endif

	/* Initialize APDS9130 */
	if ( err = apds9130_init_client ( client ) )
	{
		SENSOR_ERR ( "failed to init APDS9130 ( err = %d )\n", err );
		goto exit_init_failed;
	}

	/* Register APDS9130 as a misc device for general I/O interface */
	if ( err = misc_register ( &apds9130_device ) )
	{
		SENSOR_ERR ( "failed to register misc device ( err = %d )\n", err );
		goto exit_misc_device_register_failed;
	}

#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
	if(err = apds9130_create_attr(&(apds9130_init_info.platform_diver_addr->driver)))
	{
		SENSOR_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
#else
	if ( err = apds9130_create_attr ( &apds9130_alsps_driver.driver ) )
	{
		SENSOR_ERR ( "create attribute err = %d\n", err );
		goto exit_create_attr_failed;
	}
#endif
	/* Register APDS9130 as a member device of hwmsen */
	obj_ps.self = obj;
	obj_ps.polling = hw->polling_mode_ps;
	obj_ps.sensor_operate = apds9130_ps_operate;
	if ( err = hwmsen_attach ( ID_PROXIMITY, &obj_ps ) )
	{
		SENSOR_ERR ( "failed to attach to hwmsen ( err = %d )\n", err );
		goto exit_create_attr_failed;
	}
	SENSOR_LOG("%s: OK\n",__func__);
#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
	apds9130_init_flag=0;
#endif
	return 0;

	exit_create_attr_failed:
	misc_deregister ( &apds9130_device );
	exit_misc_device_register_failed:
	exit_init_failed:
	unregister_early_suspend ( &obj->early_drv );
	kfree ( obj );
	exit:
	apds9130_i2c_client = NULL;
	SENSOR_ERR ( "Err = %d\n", err );
#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
	apds9130_init_flag=-1;
#endif
	return err;
}

static int apds9130_i2c_remove ( struct i2c_client *client )
{
	SENSOR_FUN ();
	int err;
#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
	if(err = apds9130_delete_attr(&(apds9130_init_info.platform_diver_addr->driver)))
	{
		SENSOR_ERR("apds9130_delete_attr fail: %d\n", err);
	}
#else
	if ( err = apds9130_delete_attr ( &apds9130_alsps_driver.driver ) )
	{
		SENSOR_ERR ( "apds9130_delete_attr fail: %d\n", err );
	}
#endif
	if ( err = misc_deregister ( &apds9130_device ) )
	{
		SENSOR_ERR ( "failed to deregister misc driver : %d\n", err );
	}

	apds9130_i2c_client = NULL;
	i2c_unregister_device ( client );
	kfree ( i2c_get_clientdata ( client ) );

	return 0;
}

static int apds9130_i2c_suspend ( struct i2c_client *client, pm_message_t msg )
{
	SENSOR_FUN();

	return 0;
}

static int apds9130_i2c_resume ( struct i2c_client *client )
{
	SENSOR_FUN();

	return 0;
}

static int apds9130_i2c_detect ( struct i2c_client *client, struct i2c_board_info *info )
{
	SENSOR_FUN ();
	strcpy ( info->type, APDS9130_DEV_NAME );
	return 0;
}

static const struct i2c_device_id apds9130_i2c_id[] = { { APDS9130_DEV_NAME, 0 }, {} };

static struct i2c_driver apds9130_i2c_driver = { .probe = apds9130_i2c_probe, .remove = apds9130_i2c_remove, .suspend = apds9130_i2c_suspend,
												 .resume = apds9130_i2c_resume, .detect = apds9130_i2c_detect, .id_table = apds9130_i2c_id,
												 .driver = { .name = APDS9130_DEV_NAME, }, };

/****************************************************************************
* Linux Device Driver Related Functions
****************************************************************************/
static int apds9130_probe ( struct platform_device *pdev )
{
	SENSOR_FUN ();
	struct alsps_hw *hw = get_cust_alsps_hw ();

	/* Configure external ( GPIO ) interrupt */
	apds9130_setup_eint ();

	/* Turn on the power for APDS9130 */
	//apds9130_main_power ( hw, 1 );
	//msleep ( 9 );

	/* Add APDS9130 as I2C driver */
	if ( i2c_add_driver ( &apds9130_i2c_driver ) )
	{
		SENSOR_ERR ( "failed to add i2c driver\n" );
		return -1;
	}

	return 0;
}

static int apds9130_remove ( struct platform_device *pdev )
{
	SENSOR_FUN ();
	struct alsps_hw *hw = get_cust_alsps_hw ();

	/* Turn off the power for APDS9130 */
	//apds9130_main_power ( hw, 0 );

	i2c_del_driver ( &apds9130_i2c_driver );

	return 0;
}
static struct i2c_board_info __initdata i2c_APDS9130 = { I2C_BOARD_INFO ( "APDS9130", APDS9130_I2C_ADDR ) };

static struct platform_driver apds9130_alsps_driver = {
	.probe = apds9130_probe,
	.remove = apds9130_remove,
	.driver = {
		.name = "als_ps",
#ifdef CONFIG_OF_DT
		.of_match_table = psensor_of_match,
#endif
	}

};

#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
static int  apds9130_local_init(void)
{
   struct alsps_hw *hw = get_cust_alsps_hw();
   
	SENSOR_FUN();

	if(i2c_add_driver(&apds9130_i2c_driver))
	{
		SENSOR_ERR("add driver error\n");
		return -1;
	}
	if(-1 == apds9130_init_flag)
	{
	   return -1;
	}

	return 0;
}

static int  apds9130_local_remove(void)
{
    struct alsps_hw *hw = get_cust_alsps_hw();

    SENSOR_FUN();
    i2c_del_driver(&apds9130_i2c_driver);
    return 0;
}
#endif

static int __init apds9130_init ( void )
{
    struct alsps_hw *hw = get_cust_alsps_hw();
	SENSOR_FUN ();

	i2c_register_board_info ( hw->i2c_num, &i2c_APDS9130, 1 );
#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
	hwmsen_alsps_sensor_add(&apds9130_init_info);
#else
	if ( platform_driver_register ( &apds9130_alsps_driver ) )
	{
		SENSOR_ERR ( "failed to register platform driver\n" );
		return -ENODEV;
	}
#endif
	return 0;
}

static void __exit apds9130_exit ( void )
{
	SENSOR_FUN ();
	platform_driver_unregister ( &apds9130_alsps_driver );
}


module_init ( apds9130_init );
module_exit ( apds9130_exit );

MODULE_AUTHOR ( "Kang Jun Mo" );
MODULE_DESCRIPTION ( "apds9130 driver" );
MODULE_LICENSE ( "GPL" );

/* End Of File */

