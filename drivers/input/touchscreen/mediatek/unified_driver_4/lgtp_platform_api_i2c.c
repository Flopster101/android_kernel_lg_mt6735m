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
 *    File  	: lgtp_platform_i2c.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description : 
 *
 ***************************************************************************/
#define LGTP_MODULE "[I2C]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_4/lgtp_common.h>
#include <linux/input/unified_driver_4/lgtp_model_config_i2c.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_misc.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define LGE_TOUCH_NAME "touch_i2c"

#if defined ( TOUCH_PLATFORM_MTK )
#define MAX_I2C_TRANSFER_SIZE 255
#endif

/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/

/****************************************************************************
* Variables
****************************************************************************/
#if defined ( TOUCH_PLATFORM_MTK )
static u8* I2CDMABuf_va = NULL;
static u32 I2CDMABuf_pa = NULL;
#endif

static struct i2c_client *pClient = NULL;

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
#if defined ( TOUCH_PLATFORM_MTK )
static int dma_allocation(void)
{
	I2CDMABuf_va = (u8*)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
	if( I2CDMABuf_va == NULL ) {
		TOUCH_ERR("fail to allocate DMA\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int i2c_dma_write(struct i2c_client *client, const uint8_t *buf, int len)
{
	int i = 0;

	for ( i = 0 ; i < len ; i++ ) {
		I2CDMABuf_va[i] = buf[i];
	}

	if (len < 8) {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_send(client, buf, len);
	}
	else {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_send(client, I2CDMABuf_pa, len);
	}
}

static int i2c_dma_read(struct i2c_client *client, uint8_t *buf, int len)
{
	int i = 0;
	int ret = 0;
#if 0
	if(len < 8) {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_recv(client, buf, len);
	}
	else {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		ret = i2c_master_recv(client, I2CDMABuf_pa, len);
		if ( ret < 0 )
			return ret;

		for ( i = 0 ; i < len ; i++ ) {
			buf[i] = I2CDMABuf_va[i];
		}
	}

	return ret;
#else
	client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
	client->timing = 200;
	ret = i2c_master_recv(client, I2CDMABuf_pa, len);
	if ( ret < 0 )
		return ret;

	for ( i = 0 ; i < len ; i++ ) {
		buf[i] = I2CDMABuf_va[i];
	}
	return ret;
#endif
}

static int i2c_msg_transfer(struct i2c_client *client, struct i2c_msg *msgs, int count)
{
	int i = 0;
	int ret = 0;

	for (i = 0 ; i < count ; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = i2c_dma_read(client, msgs[i].buf, msgs[i].len);
		else
			ret = i2c_dma_write(client, msgs[i].buf, msgs[i].len);

		if ( ret < 0 )
			return ret;
	}

	return 0;
}
#endif

static int i2c_read(u8 *reg, int regLen, u8 *buf, int dataLen)
{
#if defined ( TOUCH_PLATFORM_QCT )

	int result = TOUCH_SUCCESS;
	int ret = 0;
	
	struct i2c_msg msgs[2] = {
		{ .addr = pClient->addr, .flags = 0, .len = regLen, .buf = reg, },
		{ .addr = pClient->addr, .flags = I2C_M_RD, .len = dataLen, .buf = buf, },
	};

	ret = i2c_transfer(pClient->adapter, msgs, 2);
	if (ret < 0)
	{
		result = TOUCH_FAIL;
	}

	return result;
	
#elif defined ( TOUCH_PLATFORM_MTK )

	if( dataLen <= MAX_I2C_TRANSFER_SIZE )
	{
		int result = TOUCH_SUCCESS;
		int ret = 0;

		struct i2c_msg msgs[2] = {
			{ .addr = pClient->addr, .flags = 0, .len = regLen, .buf = reg, },
			{ .addr = pClient->addr, .flags = I2C_M_RD, .len = dataLen, .buf = buf, },
		};
		
		ret = i2c_msg_transfer(pClient, msgs, 2);
		if (ret < 0) {
			result = TOUCH_FAIL;
		}
		
		return result;
	}
	else
	{
		int result = TOUCH_SUCCESS;
		int ret = 0;
		int i = 0;
		
		int msgCount = 0;
		int remainedDataLen = 0;
		
		struct i2c_msg* msgs = NULL;
	
		remainedDataLen = dataLen%MAX_I2C_TRANSFER_SIZE;
		
		msgCount = 1; /* msg for register */
		msgCount += (int)(dataLen/MAX_I2C_TRANSFER_SIZE); /* add msgs for data read */
		if( remainedDataLen > 0 ) {
			msgCount += 1; /* add msg for remained data */
		}

		msgs = (struct i2c_msg *)kcalloc(msgCount, sizeof(struct i2c_msg), GFP_KERNEL);
		if(msgs != NULL) {
			memset(msgs, 0x00, sizeof(struct i2c_msg));
		} else {
			return TOUCH_FAIL;
		}

		msgs[0].addr = pClient->addr;
		msgs[0].flags = 0;
		msgs[0].len = regLen;
		msgs[0].buf = reg;

		for (i = 1 ; i < msgCount ; i++) {
			msgs[i].addr = pClient->addr;
			msgs[i].flags = I2C_M_RD;
			msgs[i].len = MAX_I2C_TRANSFER_SIZE;
			msgs[i].buf = buf + MAX_I2C_TRANSFER_SIZE *(i-1);
		}

		if( remainedDataLen > 0 ) {
			msgs[msgCount-1].len = remainedDataLen;
		}
		
		ret = i2c_msg_transfer(pClient, msgs, msgCount);
		if (ret < 0) {
			result = TOUCH_FAIL;
		}

		kfree(msgs);
		
		return result;
		
	}
	
	
#else
	#error "Platform should be defined"
#endif
	
}

static int i2c_write(u8 *reg, int regLen, u8 *buf, int dataLen)
{
	int result = TOUCH_SUCCESS;
	int ret = 0;

	struct i2c_msg msg = {
		.addr = pClient->addr, .flags = pClient->flags, .len = (regLen+dataLen), .buf = NULL,
	};

	#if defined ( TOUCH_PLATFORM_MTK )
	if( dataLen > MAX_I2C_TRANSFER_SIZE ) {
		TOUCH_ERR("data length to write is exceed the limit ( length = %d, limit = %d )\n", dataLen, MAX_I2C_TRANSFER_SIZE);
		TOUCH_ERR("You should implement to overcome this problem like read\n");
		return TOUCH_FAIL;
	}
	#endif	

	u8 *pTmpBuf = NULL;

	 pTmpBuf = (u8 *)kcalloc(1, regLen+dataLen, GFP_KERNEL);
	 if(pTmpBuf != NULL) {
		 memset(pTmpBuf, 0x00, regLen+dataLen);
	 } else {
		 return TOUCH_FAIL;
	 }

	memcpy(pTmpBuf, reg, regLen);
	memcpy((pTmpBuf+regLen), buf, dataLen);

	msg.buf = pTmpBuf;
	
	#if defined ( TOUCH_PLATFORM_QCT )
	ret = i2c_transfer(pClient->adapter, &msg, 1);
	#elif defined ( TOUCH_PLATFORM_MTK )
	ret = i2c_msg_transfer(pClient, &msg, 1);
	#else
	#error "Platform should be defined"
	#endif

	if (ret < 0) {
		result = TOUCH_FAIL;
	}

	kfree(pTmpBuf);
	
	return result;
	
}


/****************************************************************************
* Global Functions
****************************************************************************/
struct i2c_client *Touch_Get_I2C_Handle(void)
{
	return pClient;
}

#define ADDR_SIZE 2

int Semisense_I2C_Read(u8 *cmd, u8 *data, u32 cmd_len, u32 len)
{
	u8 regValue[ADDR_SIZE] = {cmd[0], cmd[1]};
	int ret = 0;

	ret = i2c_read(regValue, cmd_len, data, len);
	if (ret == TOUCH_FAIL) {
		if (printk_ratelimit())
			TOUCH_ERR("failed to read i2c ( reg = 0x%02X%02X )\n",
					cmd[0], cmd[1]);

		return TOUCH_FAIL;
	}

	return 	len;
}

int Semisense_I2C_Write(u8 *cmd, u8 *data, u32 cmd_len, u32 len)
{
	u8 regValue[ADDR_SIZE] = {cmd[0], cmd[1]};
	int ret = 0;

	ret = i2c_write(regValue, cmd_len, data, len);
	if (ret == TOUCH_FAIL) {
		if (printk_ratelimit())
			TOUCH_ERR("failed to write i2c ( reg = 0x%02X%02X )\n",
					cmd[0], cmd[1]);

		return TOUCH_FAIL;
	}

	return len;
}

int Mit300_I2C_Read ( struct i2c_client *client, u8 *addr, u8 addrLen, u8 *rxbuf, int len )
{
	int ret = 0;

	ret = i2c_read( addr, addrLen, rxbuf, len);
	if( ret == TOUCH_FAIL ) {
		if (printk_ratelimit()) {
			TOUCH_ERR("failed to read i2c ( reg = %d )\n", (u16)((addr[0]<<7)|addr[1]));
		}
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

int Mit300_I2C_Write ( struct i2c_client *client, u8 *writeBuf, u32 write_len )
{
	int ret = 0;

	ret = i2c_write( writeBuf, 2, writeBuf+2, write_len-2);
	if( ret == TOUCH_FAIL ) {
		if (printk_ratelimit()) {
			TOUCH_ERR("failed to write i2c ( reg = %d )\n", (u16)((writeBuf[0]<<7)|writeBuf[1]));
		}
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

int Touch_I2C_Read ( u16 addr, u8 *rxbuf, int len )
{
	int ret = 0;
    
    #if defined(TOUCH_I2C_ADDRESS_8BIT)
       u8 regValue = (u8)addr;
       ret = i2c_read(&regValue, 1, rxbuf, len);
    #elif defined(TOUCH_I2C_ADDRESS_16BIT)
       u8 regValue[2] = { ( (addr>>8) & 0xFF ), ( addr & 0xFF ) }; 
       ret = i2c_read(regValue, 2, rxbuf, len);
    #endif
	
	if( ret == TOUCH_FAIL ) {
		if (printk_ratelimit()) {
			TOUCH_ERR("failed to read i2c ( reg = %d )\n", addr);
		}
		return TOUCH_FAIL;
	}
	
	return TOUCH_SUCCESS;	
}

int Touch_I2C_Write ( u16 addr, u8 *txbuf, int len )
{
	int ret = 0;
	
    #if defined(TOUCH_I2C_ADDRESS_8BIT)
        u8 regValue = (u8)addr;
        ret = i2c_write(&regValue, 1, txbuf, len);
    #elif defined(TOUCH_I2C_ADDRESS_16BIT)
        u8 regValue[2] = { ( (addr>>8) & 0xFF ), ( addr & 0xFF ) };
        ret = i2c_write(regValue, 2, txbuf, len);
    #endif
	if( ret == TOUCH_FAIL ) {
		if (printk_ratelimit()) {
			TOUCH_ERR("failed to write i2c ( reg = %d )\n", addr);
		}
		return TOUCH_FAIL;
	}
	
	return TOUCH_SUCCESS;	
}

static int touch_i2c_pm_suspend(struct device *dev)
{
	TOUCH_FUNC();

	return TOUCH_SUCCESS;
}

static int touch_i2c_pm_resume(struct device *dev)
{
	TOUCH_FUNC();
	
	return TOUCH_SUCCESS;
}

static int touch_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	TOUCH_FUNC();

	pClient = client;

	return TOUCH_SUCCESS;
}

static int touch_i2c_remove(struct i2c_client *client)
{
	pClient = NULL;

	TOUCH_FUNC();

	return TOUCH_SUCCESS;
}

#if defined ( TOUCH_PLATFORM_MTK )
static struct i2c_board_info __initdata i2c_tpd = {
	I2C_BOARD_INFO(LGE_TOUCH_NAME, 0)
};
#endif

static struct i2c_device_id touch_i2c[] = {
	{LGE_TOUCH_NAME, 0},
};

static struct dev_pm_ops touch_i2c_pm_ops = {
	.suspend = touch_i2c_pm_suspend,
	.resume = touch_i2c_pm_resume,
};

static struct i2c_driver lge_touch_driver = {
	.probe = touch_i2c_probe,
	.remove = touch_i2c_remove,
	.id_table = touch_i2c,
	.driver = {
		.name = LGE_TOUCH_NAME,
		.owner = THIS_MODULE,
		.pm = &touch_i2c_pm_ops,
	},
};

static int __init touch_i2c_init(void)
{
	
#if defined(TOUCH_I2C_USE)
    int idx = 0;

	TOUCH_FUNC();

	#if defined ( TOUCH_PLATFORM_MTK )
	{
		int ret = 0;
		ret = dma_allocation();
		if( ret == TOUCH_FAIL ) {
			return TOUCH_FAIL;
		}
	}
	#endif
	
	idx = TouchGetModuleIndex();
	
	#if defined ( TOUCH_PLATFORM_MTK )
	//i2c_tpd.addr = TouchGetDeviceSlaveAddress(idx);
	//i2c_register_board_info(TOUCH_I2C_BUS_NUM, &i2c_tpd, 1);
	lge_touch_driver.driver.of_match_table = TouchGetDeviceMatchTable(idx);
	#endif

	#if defined ( TOUCH_PLATFORM_QCT )
	lge_touch_driver.driver.of_match_table = TouchGetDeviceMatchTable(idx);
	#endif

	if( i2c_add_driver ( &lge_touch_driver ) ) {
		TOUCH_ERR("failed at i2c_add_driver()\n" );
		return -ENODEV;
	}
#else
    TOUCH_LOG("TOUCH_I2C_USE is not defined in this model\n" );
#endif
	return TOUCH_SUCCESS;
	
}

static void __exit touch_i2c_exit(void)
{
	TOUCH_FUNC();

	i2c_del_driver(&lge_touch_driver);
}

module_init(touch_i2c_init);
module_exit(touch_i2c_exit);

MODULE_AUTHOR("D3 BSP Touch Team");
MODULE_DESCRIPTION("LGE Touch Unified Driver");
MODULE_LICENSE("GPL");

/* End Of File */

