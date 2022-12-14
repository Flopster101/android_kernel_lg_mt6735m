/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file rt8542.c
   brief This file contains all function implementations for the rt8542 in linux
   this source file refer to MT6572 platform 
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/kernel.h>
#include <linux/delay.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <mach/mt_pm_ldo.h>

#include <linux/platform_device.h>
#include <cust_acc.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>

#include <linux/leds.h>

#define LCD_LED_MAX 0x7F
#define LCD_LED_MIN 0

#define DEFAULT_BRIGHTNESS 0x73 //for 20mA
#define RT8542_MIN_VALUE_SETTINGS 10 /* value leds_brightness_set*/
#define RT8542_MAX_VALUE_SETTINGS 255 /* value leds_brightness_set*/
#define MIN_MAX_SCALE(x) (((x)<RT8542_MIN_VALUE_SETTINGS) ? RT8542_MIN_VALUE_SETTINGS : (((x)>RT8542_MAX_VALUE_SETTINGS) ? RT8542_MAX_VALUE_SETTINGS:(x)))

#define BACKLIHGT_NAME "chargepump"

#define RT8542_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define RT8542_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

#define RT8542_DEV_NAME "rt8542"

#define CPD_TAG                  "[ChargePump] "
#define CPD_FUN(f)               printk(CPD_TAG"%s\n", __FUNCTION__)
#define CPD_ERR(fmt, args...)    printk(CPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define CPD_LOG(fmt, args...)    printk(CPD_TAG fmt, ##args)

// I2C variable
static struct i2c_client *new_client = NULL;
static const struct i2c_device_id rt8542_i2c_id[] = {{RT8542_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_rt8542={ I2C_BOARD_INFO(RT8542_DEV_NAME, 0x39)};

static int rt8542_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_OF
static const struct of_device_id rt8542_of_match[] = {
	{ .compatible = "rt8542", },
	{},
};

MODULE_DEVICE_TABLE(of, rt8542_of_match);
#endif

static struct i2c_driver rt8542_driver = {
    .driver = {
    .name    = "rt8542",
#ifdef CONFIG_OF 
    .of_match_table = rt8542_of_match,
#endif
    },
    .probe       = rt8542_driver_probe,
    .id_table    = rt8542_i2c_id,
};

// Flash control
unsigned char strobe_ctrl;
unsigned char flash_ctrl=0; //[LGE_UPDATE][yonghwan.lym@lge.com][2014-07-12] flash_en register(0x0A) setting position change.
#define GPIO_LCD_BL_EN GPIO_LCM_BL_EN
#define GPIO_LCD_BL_EN_M_GPIO GPIO_LCM_BL_EN_M_GPIO
// Gamma 2.2 Table
unsigned char bright_arr[] = {
	1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,3,3,3,3,4,
	4,4,4,5,5,6,6,6,7,7,8,8,8,9,9,10,10,10,11,11,
	12,12,13,14,15,16,17,18,19,20,21,21,22,23,24,25,26,27,28,29,
	30,31,33,34,35,37,38,39,40,42,43,44,46,47,48,50,51,52,54,55,
	57,58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,94,96};

static unsigned char current_brightness = 0;
static unsigned char is_suspend = 0;
static unsigned char lcd_backlight_is_setted = 1;//LGE_CHANGE: [2014-12-02] yonghwan.lym@lge.com, rt8542 is disable, when push the power key.
static bool rt8542_power_on = 1; //LGE_CHANGE: [2014-12-02] yonghwan.lym@lge.com, rt8542 is disable, when push the power key.

struct semaphore rt8542_lock;

/* generic */
#define RT8542_MAX_RETRY_I2C_XFER (100)
#define RT8542_I2C_WRITE_DELAY_TIME 1

typedef struct
{
    kal_bool       bat_exist;
    kal_bool       bat_full;
    kal_bool       bat_low;
    UINT32      bat_charging_state;
    UINT32      bat_vol;
    kal_bool     charger_exist;
    UINT32      pre_charging_current;
    UINT32      charging_current;
    INT32      charger_vol;
    UINT32       charger_protect_status;
    UINT32      ISENSE;
    UINT32      ICharging;
    INT32       temperature;
    UINT32      total_charging_time;
    UINT32      PRE_charging_time;
    UINT32      CC_charging_time;
    UINT32      TOPOFF_charging_time;
    UINT32      POSTFULL_charging_time;
    UINT32       charger_type;
    UINT32       PWR_SRC;
    UINT32       SOC;
    UINT32       ADC_BAT_SENSE;
    UINT32       ADC_I_SENSE;
} PMU_ChargerStruct;

extern PMU_ChargerStruct BMT_status;

/* i2c read routine for API*/
static char rt8542_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMA_USE_BASIC_I2C_FUNC
	s32 dummy;
	if (NULL == client)
		return -1;

	while (0 != len--) {
#ifdef BMA_SMBUS
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			CPD_ERR("i2c bus read error");
			return -1;
		}
		*data = (u8)(dummy & 0xff);
#else
		dummy = i2c_master_send(client, (char *)&reg_addr, 1);
		if (dummy < 0)
		{
   CPD_ERR("send dummy is %d", dummy);
			return -1;
		}

		dummy = i2c_master_recv(client, (char *)data, 1);
		if (dummy < 0)
		{
   CPD_ERR("recv dummy is %d", dummy);
			return -1;
		}
#endif
		reg_addr++;
		data++;
	}
	return 0;
#else
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < RT8542_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(RT8542_I2C_WRITE_DELAY_TIME);
	}

	if (RT8542_MAX_RETRY_I2C_XFER <= retry) {
		CPD_ERR("I2C xfer error");
		return -EIO;
	}

	return 0;
#endif
}

/* i2c write routine for */
static char rt8542_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMA_USE_BASIC_I2C_FUNC
	s32 dummy;
#ifndef BMA_SMBUS
	u8 buffer[2];
#endif

	if (NULL == client)
		return -1;

	while (0 != len--) {
		#if 1
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(client, (char *)buffer, 2);
		#endif

		reg_addr++;
		data++;
		if (dummy < 0) {
			return -1;
		}
	}

#else
	u8 buffer[2];
	int retry;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buffer,
		 },
	};

	while (0 != len--) {
		buffer[0] = reg_addr;
		buffer[1] = *data;
		for (retry = 0; retry < RT8542_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, msg,
						ARRAY_SIZE(msg)) > 0) {
				break;
			} else {
				mdelay(RT8542_I2C_WRITE_DELAY_TIME);
			}
		}
		if (RT8542_MAX_RETRY_I2C_XFER <= retry) {
			return -EIO;
		}
		reg_addr++;
		data++;
	}
#endif
//	printk("\n [RT8542] rt8542_i2c_write \n");
	return 0;
}

static int rt8542_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	return rt8542_i2c_read(client,reg_addr,data,1);
}

static int rt8542_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	int ret_val = 0;
	int i = 0;

	ret_val = rt8542_i2c_write(client,reg_addr,data,1);

	for ( i = 0; i < 5; i++)
	{
		if (ret_val != 0)
			rt8542_i2c_write(client,reg_addr,data,1);
		else
			return ret_val;
	}
	return ret_val;
}

static int rt8542_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	return rt8542_i2c_read(client,reg_addr,data,len);
}

bool check_charger_pump_vendor()
{
  int err = 0;
  unsigned char data = 0;

  err = rt8542_smbus_read_byte(new_client,0x01,&data);

  if(err < 0)
   CPD_ERR("read charge-pump vendor id fail\n");

//    CPD_ERR("vendor is 0x%x\n");

  if((data&0x03) == 0x03) //Richtek
      return FALSE;
  else
      return TRUE;
}
	
int chargepump_set_backlight_level(unsigned int level)
{
	unsigned char data = 0;
	unsigned char data1 = 0;
	unsigned int bright_per = 0;

	if (level == 0) 
	{
		printk( "------	 backlight_level suspend-----\n");
		down_interruptible(&rt8542_lock);
		data1 = 0x00; //backlight2 brightness 0
		rt8542_smbus_write_byte(new_client, 0x05, &data1);
		
        rt8542_smbus_read_byte(new_client, 0x0A, &data1);
        data1 &= 0x66;

		rt8542_smbus_write_byte(new_client, 0x0A, &data1);
		up(&rt8542_lock); 
			printk( "[RT8542] flash_ctrl = %d\n", flash_ctrl);
			if( flash_ctrl == 0 ){
			mt_set_gpio_out(GPIO_LCD_BL_EN,GPIO_OUT_ZERO);
				rt8542_power_on = 0;
		}
		is_suspend = 1;
	}
	else 
	{
		level = MIN_MAX_SCALE(level);
		printk(KERN_ERR "[RT8542] level = %d", level);
		
		//Gamma 2.2 Table adapted
		bright_per = (level - (unsigned int)10) *(unsigned int)100 / (unsigned int)245;
		data = bright_arr[bright_per];

		printk("[Backlight] %s bright_per = %d, data = %d\n", __func__, bright_per, data);//<==Open by Minrui for backlight debug
//		data = 0x70;//force the backlight on if level > 0 <==Add by Minrui for backlight debug

		if (is_suspend == 1) 
		{
			//printk( "------	 backlight_level resume-----\n");
			is_suspend = 0;
			mt_set_gpio_out(GPIO_LCD_BL_EN,GPIO_OUT_ONE);
			rt8542_power_on = 1; //LGE_CHANGE: [2014-12-02] yonghwan.lym@lge.com, rt8542 is disable, when push the power key.
			mdelay(10);
			down_interruptible(&rt8542_lock); 
			if(check_charger_pump_vendor() == FALSE)
			{
				data1 = 0x54;//0x37; 
				rt8542_smbus_write_byte(new_client, 0x02, &data1);
				CPD_LOG("[ChargePump]-Richtek\n");
			}
			else
			{
				data1 = 0x54;//0x57; 
				rt8542_smbus_write_byte(new_client, 0x02, &data1);
				CPD_LOG("[RT8542]-TI\n");
			}

			rt8542_smbus_write_byte(new_client, 0x05, &data);
			CPD_LOG("[RT8542]-backlight brightness Setting[reg0x05][value:0x%x]\n",data);
			
   rt8542_smbus_read_byte(new_client, 0x0A, &data1);
			data1 |= 0x19;
			if(flash_ctrl==0)
				data1 &= 0x99; // flashlight off

			rt8542_smbus_write_byte(new_client, 0x0A, &data1);
			up(&rt8542_lock); 
		}

		down_interruptible(&rt8542_lock); 
  {
   unsigned char read_data = 0;

   rt8542_smbus_read_byte(new_client, 0x02, &read_data);
   CPD_LOG("[RT8542]-OVP[0x%x]\n",read_data);
  }

		CPD_LOG("[RT8542]-backlight Seting[reg0x05][value:0x%x]\n",data);
		rt8542_smbus_write_byte(new_client, 0x05, &data);
		up(&rt8542_lock);
	}
	return 0;
}

static int rt8542_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err=0; 

    CPD_FUN();

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }    
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;    
	
    return 0;                                                                                       

exit:
    return err;
}

static int rt8542_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{   
	CPD_FUN();

	if (client == NULL)
	{
		printk("%s client is NULL\n", __func__);
		return -1;
	}

	new_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CPD_LOG("i2c_check_functionality error\n");
		return -1;
	}

//    sema_init(&rt8542_lock, 1);

	printk("%s %p %x %x\n", __func__, client->adapter, client->addr, client->flags);

	return 0;
}


static int rt8542_i2c_remove(struct i2c_client *client)
{
 CPD_FUN();
 new_client = NULL;
	return 0;
}


static int __attribute__ ((unused)) rt8542_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{ 
 CPD_FUN();
	return 0;
}

static struct i2c_driver rt8542_i2c_driver = {
	.driver.name = RT8542_DEV_NAME,
	.probe = rt8542_i2c_probe,
	.remove = rt8542_i2c_remove,
	.id_table	= rt8542_i2c_id,
};

static int rt8542_pd_probe(struct platform_device *pdev) 
{
 CPD_FUN();

	//i2c number 1(0~2) control
	i2c_register_board_info(2, &i2c_rt8542, 1); 

	mt_set_gpio_mode(GPIO_LCD_BL_EN, GPIO_LCD_BL_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_LCD_BL_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCD_BL_EN, GPIO_DIR_OUT);

//	i2c_add_driver(&rt8542_i2c_driver);
 if(i2c_add_driver(&rt8542_driver)!=0)
 {
   CPD_ERR("Failed to register rt8542 driver");
 }
	return 0;
}

static int __attribute__ ((unused)) rt8542_pd_remove(struct platform_device *pdev)
{
    CPD_FUN();
    i2c_del_driver(&rt8542_i2c_driver);
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void rt8542_early_suspend(struct early_suspend *h)
{
	int err = 0;
	unsigned char data;

 CPD_FUN();

	down_interruptible(&rt8542_lock); 
	data = 0x00; //backlight2 brightness 0
	err = rt8542_smbus_write_byte(new_client, 0x05, &data);

    err = rt8542_smbus_read_byte(new_client, 0x0A, &data);
    data &= 0x66;

	err = rt8542_smbus_write_byte(new_client, 0x0A, &data);
	up(&rt8542_lock); 
	CPD_LOG("[RT8542] rt8542_early_suspend  [%d]\n",data);
}
//LGE_CHANGE_S: [2014-12-02] yonghwan.lym@lge.com, rt8542 is disable, when push the power key.
void rt8542_flash_ctrl_during_lcd_off()
{
    printk("rt8542_flash_ctrl_during_lcd_off, flash_ctrl:%d\n",flash_ctrl);
    int data1;

    if(flash_ctrl == 2)
    {
        mt_set_gpio_out(GPIO_LCD_BL_EN,GPIO_OUT_ONE);

        down_interruptible(&rt8542_lock);
        rt8542_smbus_read_byte(new_client, 0x06, &data1);
        data1 &= 0x0F;
        data1 |= 0x50;
        rt8542_smbus_write_byte(new_client, 0x06, &data1);

        data1 = 0x40 | 0x1f;
        rt8542_smbus_write_byte(new_client, 0x07, &data1);

        rt8542_smbus_read_byte(new_client, 0x09, &data1);
        data1 &= 0xF3;
        data1 |= (0x20 | 0x10);
        rt8542_smbus_write_byte(new_client, 0x09, &data1);

        rt8542_smbus_read_byte(new_client, 0x0A, &data1);
        data1 &= 0x99;
        data1 |= 0x62;
        rt8542_smbus_write_byte(new_client, 0x0A, &data1);
        up(&rt8542_lock);
    }
    else
    {
        mt_set_gpio_out(GPIO_LCD_BL_EN,GPIO_OUT_ZERO);
    }
}

void rt8542_flash_strobe_prepare(char OnOff,char ActiveHigh)
{
	int err = 0;

 CPD_FUN();

 down_interruptible(&rt8542_lock);

 err = rt8542_smbus_read_byte(new_client, 0x09, &strobe_ctrl);        

 err = rt8542_smbus_read_byte(new_client, 0x0A, &flash_ctrl);

 strobe_ctrl &= 0xF3;
	flash_ctrl = OnOff;
    
	if(ActiveHigh)
	{
		strobe_ctrl |= 0x20;
	}
	else
	{
		strobe_ctrl &= 0xDF;
	}

	if(OnOff == 1)
	{
		CPD_LOG("Strobe mode On\n");
		strobe_ctrl |= 0x10;		
  }
  else if(OnOff == 2)
  {
 		CPD_LOG("Torch mode On\n");
 		strobe_ctrl |= 0x10;
  }
 	else
 	{
 		CPD_LOG("Flash Off\n");
 		strobe_ctrl &= 0xEF;
 	}

	err = rt8542_smbus_write_byte(new_client, 0x09, &strobe_ctrl);
 
 up(&rt8542_lock);
}

//strobe enable
void rt8542_flash_strobe_en()
{
	int err = 0;
    int flash_OnOff=0;

    //LGE_CHANGE_S: [2014-12-02] yonghwan.lym@lge.com, rt8542 is disable, when push the power key.
    if(rt8542_power_on == 0)
    {
        rt8542_flash_ctrl_during_lcd_off();
        return;
    }
    //LGE_CHANGE_E: [2014-12-02] yonghwan.lym@lge.com, rt8542 is disable, when push the power key.

    down_interruptible(&rt8542_lock);
    err = rt8542_smbus_read_byte(new_client, 0x0A, &flash_OnOff);
    if(flash_ctrl == 1)
	{
        flash_OnOff |= 0x66;
    }
	else if(flash_ctrl == 2)
	{
        flash_OnOff |= 0x62;
    }
	else
        flash_OnOff &= 0x99;
    err = rt8542_smbus_write_byte(new_client, 0x0A, &flash_OnOff);
    up(&rt8542_lock); 
}

//strobe level
void rt8542_flash_strobe_level(char level)
{
	int err = 0;
	unsigned char data1=0;
 unsigned char data2=0;
 unsigned char torch_level;
 unsigned char strobe_timeout = 0x1F;

 CPD_FUN();

 down_interruptible(&rt8542_lock); 
#if 0
    if( level == 1) 
    {
        torch_level = 0x20;
    }
    else
    {
        torch_level = 0x50;
    }
        
    err = rt8542_smbus_read_byte(new_client, 0x06, &data1);

	if(31 < level)
    {    
		data1= torch_level | 0x0A;
        strobe_timeout = 0x0F;
    }
    else if(level < 0)
    {
		data1= torch_level ;
    }
    else
    {
		data1= torch_level | level; 
    }

#else
    torch_level = 0x30; // 225/2 = 112.5mA

    err = rt8542_smbus_read_byte(new_client, 0x06, &data1);

    strobe_timeout = 0x1F;
    if(level < 0)
        data1= torch_level;
    else if(level == 1)
        data1= torch_level | 0x02;
    else if(level == 2)
        data1= torch_level | 0x04;
    else if(level == 3)
        data1= torch_level | 0x06;
    else if(level == 4)
        data1= torch_level | 0x09;
    else
        data1= torch_level | level;
#endif

/*
    if(0)
    {
	    CPD_LOG("Batt temp=%d\n", BMT_status.temperature );
        
        torch_level = 0xF0 & data1;
        level = 0x0F & data1;
        torch_level = 0xF0 & (torch_level >> 2);
        level = 0x0F & (level >> 2);
        
        data1 = torch_level | level;
    }
*/
	CPD_LOG("Flash Level =0x%x\n", data1);
 err = rt8542_smbus_write_byte(new_client, 0x06, &data1);
 
 data2 = 0x40 | strobe_timeout;
 CPD_LOG("Storbe Timeout =0x%x\n", data2);
 err |= rt8542_smbus_write_byte(new_client, 0x07, &data2);
 up(&rt8542_lock); 
}
//LGE_CHANGE_S: [2015-10-06] kyhunghun.oh@lge.com, forced flash off when camera on with volume key
void release_rt8542_forced_flash_off()
{
	lcd_backlight_is_setted = 0;
	return;
}

void rt8542_forced_flash_off()
{
	unsigned char data1;
	int err;
	if(lcd_backlight_is_setted == 0 && is_suspend == 1)
	{
		mt_set_gpio_out(GPIO_LCD_BL_EN,GPIO_OUT_ONE);
		rt8542_power_on = 1;
		down_interruptible(&rt8542_lock);
		data1 = 0x00; // flashlight off
		flash_ctrl = 2;
		err = rt8542_smbus_write_byte(new_client, 0x0A, &data1);

		up(&rt8542_lock);
	}
	return;

}
//LGE_CHANGE_E: [2015-10-06] kyhunghun.oh@lge.com, forced flash off when camera on with volume key
static void rt8542_late_resume(struct early_suspend *h)
{
	int err = 0;
	unsigned char data1;
	
 CPD_FUN();

	mt_set_gpio_out(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
	rt8542_power_on = 1; //LGE_CHANGE: [2014-12-02] yonghwan.lym@lge.com, rt8542 is disable, when push the power key.
	mdelay(50);
	down_interruptible(&rt8542_lock); 
	err = rt8542_smbus_write_byte(new_client, 0x05, &current_brightness);
	
    err = rt8542_smbus_read_byte(new_client, 0x0A, &data1);
	data1 |= 0x19;//backlight enable
	if(flash_ctrl==0)
		data1 &= 0x99; // flashlight off

	err = rt8542_smbus_write_byte(new_client, 0x0A, &data1);
	up(&rt8542_lock); 
	CPD_LOG("[RT8542] rt8542_late_resume  [%d]\n",data1);
}

static struct early_suspend __attribute__ ((unused)) rt8542_early_suspend_desc = {
	.level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend	= rt8542_early_suspend,
	.resume		= rt8542_late_resume,
};
#endif

static struct platform_driver rt8542_backlight_driver = {
	.remove = rt8542_pd_remove,
	.probe      = rt8542_pd_probe,
	.driver     = {
		   .name = BACKLIHGT_NAME,
		   },
};      

#ifdef CONFIG_OF
static struct platform_device mtk_backlight_dev = {
	.name = BACKLIHGT_NAME,
	.id   = -1,};
#endif

static int __init rt8542_init(void)
{
	CPD_FUN();
	sema_init(&rt8542_lock, 1);

#if 0
#ifdef CONFIG_OF
	if (platform_device_register(&mtk_backlight_dev))
	{
		CPD_ERR("failed to register device");
		return -1;
	}
#endif

	#ifndef	CONFIG_MTK_LEDS
	register_early_suspend(&rt8542_early_suspend_desc);
	#endif	

	if(platform_driver_register(&rt8542_backlight_driver))
	{
		CPD_ERR("failed to register driver");
		return -1;
	}
#else

	//i2c number 1(0~2) control
	i2c_register_board_info(2, &i2c_rt8542, 1); 

	mt_set_gpio_mode(GPIO_LCD_BL_EN, GPIO_LCD_BL_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_LCD_BL_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCD_BL_EN, GPIO_DIR_OUT);

//	i2c_add_driver(&rt8542_i2c_driver);
 if(i2c_add_driver(&rt8542_driver)!=0)
 {
   CPD_ERR("Failed to register rt8542 driver");
 }
#endif

	return 0;
}

static void __exit rt8542_exit(void)
{
	platform_driver_unregister(&rt8542_backlight_driver);
}

EXPORT_SYMBOL(rt8542_flash_strobe_en);
EXPORT_SYMBOL(rt8542_flash_strobe_prepare);
EXPORT_SYMBOL(rt8542_flash_strobe_level);
MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("rt8542 driver");
MODULE_LICENSE("GPL");

late_initcall(rt8542_init);
module_exit(rt8542_exit);

