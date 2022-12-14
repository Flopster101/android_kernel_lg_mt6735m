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
 *    File  	: lgtp_device_dummy.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[SN280H]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_4/lgtp_common.h>

#include <linux/input/unified_driver_4/lgtp_common_driver.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_i2c.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_misc.h>
#include <linux/input/unified_driver_4/lgtp_device_sn280h.h>


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
#if defined( TOUCH_MODEL_E1)
static const char defaultFirmware[] = "semisense/e1/S212_E1_LTE_LGE_4P5_HS_FW_V03_151007.bin";
#endif

static struct semisense_ts_data *ts = NULL;

static int temp_hover_status = 0;
static int knock_flag = 0;
static TouchDriverData *gTouchDriverData;
static touch_info info;
static int tap_count = 0;

int g_miscInitialize = 0;


#if defined(ENABLE_SWIPE_MODE)  
static int get_swipe_mode = 1;
/*
static int wakeup_by_swipe = 0;
*/
extern int lockscreen_stat;
#endif


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static void Sn280h_Reset(void);


/****************************************************************************
* Local Functions
****************************************************************************/
static ssize_t show_Model_Info(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;	
    
    WRITE_SYSBUF(buf, ret, "======== Model info ========\n");
	#if 0
	if( TouchReadGpioMakerId() == 0 )
    {
        TOUCH_LOG("Touch IC : LeadingUI\n");
        WRITE_SYSBUF(buf, ret, "Maker ID PIN: 0\n");
        WRITE_SYSBUF(buf, ret, "Module Product : SUNTEL\n");
        WRITE_SYSBUF(buf, ret, "Touch IC : LeadingUI\n");
        //return TOUCH_SUCCESS;
    }
    else if(TouchReadGpioMakerId() == 1 )
    {
        WRITE_SYSBUF(buf, ret, "Maker ID PIN: 1\n");
        WRITE_SYSBUF(buf, ret, "Module Product : LGIT\n");
        WRITE_SYSBUF(buf, ret, "Touch IC : Focaltech\n");
    }   
    #endif
	return ret;
}
static LGE_TOUCH_ATTR(Model_Info, S_IRUGO | S_IWUSR, show_Model_Info, NULL);

static ssize_t show_version_read(TouchDriverData *pDriverData, char *buf)
{
    int result = 0;
    int ret = 0;
    u16 addr = 0;
    u16 rdata = 0;
    u8 ic_code[8] = {0,};
    /* Factory data & firmware version check 0x1e --> 30 -->*/
    addr = REG_FIRMWARE_VERSION;
    result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata));
        
    if(result < 0)
    {
            goto i2c_fail;
    }
    else
    {
        WRITE_SYSBUF(buf, ret,"[TEST]Firmware version = 0x%x\n", rdata); 
        TOUCH_LOG("[TEST]Firmware version = 0x%x\n", rdata); 
    }

    addr = REG_IC_CODE;
    result = Semisense_I2C_Read((u8 *)&addr, (u8 *)ic_code, sizeof(addr), sizeof(ic_code));
    if(result < 0)
    {
        goto i2c_fail;    
    }
    else
    {
        WRITE_SYSBUF(buf, ret, "[TEST]IC CODE = %d,%d,%d,%d,%d,%d,%d,%d \n",
                ic_code[0],ic_code[1],ic_code[2],ic_code[3],
                ic_code[4],ic_code[5],ic_code[6],ic_code[7]);
        TOUCH_LOG("[TEST]IC CODE = %d,%d,%d,%d,%d,%d,%d,%d \n",
                ic_code[0],ic_code[1],ic_code[2],ic_code[3],
                ic_code[4],ic_code[5],ic_code[6],ic_code[7]);
    }

    addr = TSC_FLASH_FW_VER_POS;
    result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata));
    if(result < 0)
    {
        goto i2c_fail;    
    }
    else
    {
        WRITE_SYSBUF(buf, ret,"[TEST]TSC_FLASH_FW_VER_POS = 0x%x\n", rdata); 
        TOUCH_LOG("[TEST]TSC_FLASH_FW_VER_POS = 0x%x\n", rdata); 
    }


    addr = TSC_PANEL_TEST_VER;
    result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata));
    if(result < 0)
    {
        goto i2c_fail;    
    }
    else
    {
        WRITE_SYSBUF(buf, ret,"[TEST]TSC_PANEL_TEST_VER = 0x%x\n", rdata); 
        TOUCH_LOG("[TEST]TSC_PANEL_TEST_VER = 0x%x\n", rdata); 
    }


    addr = REG_DEVIATION_CODE;
    result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata));
    if(result < 0)
    {    
        goto i2c_fail;
    }
    else
    {
        WRITE_SYSBUF(buf, ret,"[TEST]REG_DEVIATION_CODE  = 0x%x\n", rdata); 
        TOUCH_LOG("[TEST]REG_DEVIATION_CODE  = 0x%x\n", rdata); 
    }


    addr = REG_PROJECT_CODE;
    result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata));
    if(result < 0)
    {
        goto i2c_fail;    
    }
    else
    {
        WRITE_SYSBUF(buf, ret,"[TEST]REG_PROJECT_CODE  = 0x%x\n", rdata); 
        TOUCH_LOG("[TEST]REG_PROJECT_CODE  = 0x%x\n", rdata); 
    }

    addr = REG_MP_FIELD;
    result = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata));
    if(result < 0)
    {
        goto i2c_fail;    
    }
    else
    {
        WRITE_SYSBUF(buf, ret,"[TEST]REG_MP_FIELD  = 0x%x\n", rdata); 
        TOUCH_LOG("[TEST]REG_MP_FIELD  = 0x%x\n", rdata); 
    }

    return ret;
i2c_fail:
    WRITE_SYSBUF(buf, ret, "I2C fail with addr(0x%x) \n", addr);
    TOUCH_ERR("I2C fail with addr(0x%x)\n", addr );
    return ret;
}
static LGE_TOUCH_ATTR(version_read, S_IRUGO | S_IWUSR, show_version_read, NULL);

static ssize_t show_Pincheck(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;
    int gpioState = 0;
    gpioState = mt_get_gpio_in(GPIO_TOUCH_INT); /* TBD */
    TOUCH_LOG("INTERRUPT PIN = %d\n", gpioState);

    gpioState = mt_get_gpio_in(GPIO_TOUCH_RESET); /* TBD */
    TOUCH_LOG("RESET PIN = %d\n", gpioState);

    return ret;
}
static LGE_TOUCH_ATTR(Pincheck, S_IRUGO | S_IWUSR, show_Pincheck, NULL);

static ssize_t show_knockon(TouchDriverData *pDriverData, char *buf)
{
    int ret = TOUCH_SUCCESS;
    unsigned short Addr = LPWG_STATUS_REG;
    unsigned short wData = 0;
    unsigned short rData = 0;
    knock_flag = 1;
    wData = 0x0021;
    Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData, sizeof(Addr), sizeof(wData));
    TOUCH_LOG("[NSM]SN280h was changed to STATE_KNOCK_ON_ONLY\n");
    return ret;
}
static LGE_TOUCH_ATTR(knockon, S_IRUGO | S_IWUSR, show_knockon, NULL);

static ssize_t show_Touch_Reset(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;
    Sn280h_Reset();
    WRITE_SYSBUF(buf, ret, "[touch reset]\n");
    return ret;
}
static LGE_TOUCH_ATTR(Touch_Reset, S_IRUGO | S_IWUSR, show_Touch_Reset, NULL);


static ssize_t show_Set_vp(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;
    WRITE_SYSBUF(buf, ret, "[%s] \n", temp_hover_status ? "1" : "0");
	
    return ret;
}


static ssize_t store_Set_vp(TouchDriverData *pDriverData, const char *buf, size_t count)
{
    u16 Addr = HOVER_STATUS_REG;
    u16 wData = 0;
    int cmd = 0;
    int ret = 0;
    
    sscanf(buf, "%d", &cmd);
    if(cmd == 1)
    {
        wData = 0x0404;
        ret = Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData, sizeof(Addr), sizeof(wData));
        if(ret < 0)
        {
            goto i2c_fail;    
        }
        TOUCH_LOG("Set hover on mode.\n");
        temp_hover_status = 1;
    }
    else if(cmd == 0)
    {
        wData = 0x0505;
        ret = Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData, sizeof(Addr), sizeof(wData));
        if(ret < 0)
        {
            goto i2c_fail;    
        }
        TOUCH_LOG("Set hover off mode. \n");
        temp_hover_status = 0;
    }
    else
    {
        TOUCH_LOG("Invalid HOVER command\n");
    }
    return count;
i2c_fail:    
    TOUCH_LOG("I2C fail with addr(0x%x), write data(0x%x)\n", Addr, wData);
    return count;
}
static LGE_TOUCH_ATTR(Set_vp, S_IRUGO | S_IWUSR, show_Set_vp, store_Set_vp);

int print_channel(char *buf, int ret, int tx)
{
    int rx = 0;
    int i = 0;
    for(i = 0; i<2; i++)
    {
        WRITE_SYSBUF(buf, ret,"[D%d] |   ",
            (tx == 0 || tx == 1) ? (i == 0) ? 0 : 3 : (i == 1) ? 2 : 1);
        rx = (tx % 2 == 0) ? 8 : 7;
        for ( ; rx > 0; rx -= 2) 
        {
            WRITE_SYSBUF(buf, ret, "[S%d]  ", rx - 1);
        }
        if (i == 0)
            WRITE_SYSBUF(buf, ret, "\t| ");
    }
    WRITE_SYSBUF(buf, ret, "\n");
    return ret;
}

int read_channel_data(char *buf, u16 Addr)
{
    u16 rData[TOUCH_TX_CHANNEL * TOUCH_RX_CHANNEL] = {0,};
    int rx = 0, tx = 0;
    int ret = 0;

    ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)rData,
            sizeof(Addr), sizeof(rData));
    if (ret < 0) {
        goto i2c_fail;
    }
    ret = 0;

    WRITE_SYSBUF(buf, ret, "=====================================================\n");
    WRITE_SYSBUF(buf, ret, "               %s\n", (Addr == CAP_DELTA) ?\
                        "DELTA VALUE" : "RAW DATA VALUE");
    WRITE_SYSBUF(buf, ret, "=====================================================\n");
    for (tx = 0; tx < TOUCH_TX_CHANNEL; tx++) {
        if (tx % 2 == 0)
            ret = print_channel(buf, ret, tx);
        WRITE_SYSBUF(buf, ret, "\t");

        for (rx = 0; rx < TOUCH_RX_CHANNEL; rx++) {
            if (rx == 4) {
                WRITE_SYSBUF(buf, ret, "\t|\t  ");
            }
            WRITE_SYSBUF(buf, ret, "%05d ", rData[tx * TOUCH_RX_CHANNEL + rx]);
        }
        WRITE_SYSBUF(buf, ret, "\n");

        if (tx % 2 == 1)
            ret = print_channel(buf, ret, tx);
    }

    return ret;

i2c_fail:
    TOUCH_LOG("I2C fail with addr(0x%x)-(%x)\n", Addr, ret);
    return TOUCH_FAIL;
}

static ssize_t show_delta(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;

    ret = read_channel_data(buf, CAP_DELTA);
    if(ret == TOUCH_FAIL)
    {
        TOUCH_LOG("I2C fail with addr(0x%x)\n", CAP_DELTA);
    }
    return ret;
}
static LGE_TOUCH_ATTR(delta, S_IRUGO | S_IWUSR, show_delta, NULL);

static ssize_t show_raw_data(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;

    ret = read_channel_data(buf, CAP_REFERENCE);
    if(ret == TOUCH_FAIL)
    {
        TOUCH_LOG("I2C fail with addr(0x%x)\n", CAP_REFERENCE);
    }
    return ret;
}
static LGE_TOUCH_ATTR(raw_data, S_IRUGO, show_raw_data, NULL);

static ssize_t store_i2c_control(TouchDriverData *pDriverData, const char *buf, size_t count)
{
    u16 cmd[3] ={0,};
    u16 Addr = 0;
    u16 rData = 0;
    u16 wData = 0;
    int ret = 0;

    sscanf(buf, "%x %x %x",
        (unsigned int*)&cmd[0],
        (unsigned int*)&cmd[1],
        (unsigned int*)&cmd[2]);
    TOUCH_LOG("cmd : 0x%x 0x%x 0x%x\n", cmd[0], cmd[1], cmd[2]);

    Addr = cmd[1];
    wData = cmd[2];

    TouchDisableIrq();
    switch (cmd[0]) {
    case 1 :
        ret = Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData,
                sizeof(Addr), sizeof(wData));
        if (ret < 0)
            goto i2c_fail;

        TOUCH_LOG("Success write : Addr[0x%x] Data[0x%x]\n", Addr, wData);
        break;

    case 2 :
        ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)&rData,
                sizeof(Addr), sizeof(rData));
        if (ret < 0)
            goto i2c_fail;

        TOUCH_LOG("Success read : Addr[0x%x] Data[0x%x]\n", Addr, rData);
        break;

    default :
        TOUCH_LOG("===============================================\n");
        TOUCH_LOG(" if you want to use this sysfs as follows \n");
        TOUCH_LOG(" echo [1(Write)/2(Read)] [Address] [Write Value] \n");
        TOUCH_LOG("===============================================\n");
        break;
    }
    TouchEnableIrq();

    return count;

i2c_fail:
    TouchEnableIrq();
    TOUCH_LOG("I2C fail with addr(0x%x)\n", Addr);
    return count;
}

static ssize_t show_i2c_control(TouchDriverData *pDriverData, char *buf)
{
    int ret = 0;
    u16 Addr = 0;
    u16 rData = 0;
    u16 wData = 0;

    ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)&rData,
                sizeof(Addr), sizeof(rData));
    if (ret < 0)
        goto i2c_fail;

    WRITE_SYSBUF(buf, ret, "Address[0x%x] Value[0x%x]\n", Addr, rData);

i2c_fail:
    TOUCH_LOG("I2C fail with addr(0x%x)\n", Addr);
    return ret;
}
static LGE_TOUCH_ATTR(i2c_control, S_IRUGO | S_IWUSR, show_i2c_control, store_i2c_control);


static struct attribute *Sn280h_attribute_list[] = {
	&lge_touch_attr_Model_Info.attr,
    &lge_touch_attr_version_read.attr,
    &lge_touch_attr_Pincheck.attr,
    &lge_touch_attr_Touch_Reset.attr,
    &lge_touch_attr_Set_vp.attr,
    &lge_touch_attr_delta.attr,
    &lge_touch_attr_raw_data.attr,
    &lge_touch_attr_i2c_control.attr,
    &lge_touch_attr_knockon.attr,
    NULL,
};

static int Sn280h_Initialize(TouchDriverData *pDriverData)
{
    struct i2c_client *client = Touch_Get_I2C_Handle();
    TOUCH_FUNC();
    ts = devm_kzalloc(&client->dev, sizeof(struct semisense_ts_data), GFP_KERNEL);
    if (ts == NULL)
    {
        TOUCH_ERR("failed to allocate memory for device driver data\n");
        return TOUCH_FAIL;
    }
    gTouchDriverData = pDriverData;
    ts->client = client;
    return TOUCH_SUCCESS;
}

static void Sn280h_Reset(void)
{
    TouchSetGpioReset(0);
    mdelay(1);
    TouchSetGpioReset(1);
    mdelay(2);
    TouchSetGpioReset(0);
    mdelay(1);
    TouchSetGpioReset(1);
    mdelay(20);
    temp_hover_status = 0;
    ts->currState = STATE_NORMAL;
    knock_flag = 0;
    TOUCH_LOG("Device was reset\n");
}

static int Sn280h_InitRegister(void)
{
    /* IMPLEMENT : Register initialization after reset */
    TOUCH_FUNC();
    return TOUCH_SUCCESS;
}

static int get_lpwg_data(TouchReadData *pData, u8 tap_count)
{
    int ret = TOUCH_SUCCESS;
    u16 rData[2] = {0,};
    u16 Addr = 0;
    int i = 0;

    for (i = 0; i < tap_count; i++) {
        Addr = REG_KNOCK_DATA(i);
        ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)rData,
                sizeof(Addr), sizeof(rData));
        if (ret < 0) {
            TOUCH_LOG("KNOCK ON/CODE(%x) Data read Fail!\n", Addr);
            goto i2c_fail;
        }

        pData->knockData[i].x = rData[0];
        pData->knockData[i].y = rData[1];
        TOUCH_LOG("TAB DATA TEST = [%d %d]\n", rData[0], rData[1]);
    }

    return ret;

i2c_fail:
    ret = TOUCH_FAIL;
    return  ret;
}

static int Sn280h_InterruptHandler(TouchReadData *pData)
{
    struct i2c_client *client = Touch_Get_I2C_Handle();

    //TouchDriverData *pDriverData = platform_get_drvdata(client);

    TouchFingerData *pFingerData = NULL;
    status_reg_u ts_status;
    data_reg_t ts_data;
    int result = 0;
    int ret = 0;
    u16 Addr = 0;
    u16 rData = 0;
    u8 cnt = 0;
    u8 index_temp = 0;
    
    u32 reportedFinger = gTouchDriverData->reportData.finger;   
    u32 newFinger = 0;
    u32 changedFinger = 0;
    u32 pressedFinger = 0;
    u32 releasedFinger = 0;
    int i = 0;

    memset(&info, 0x0, sizeof(touch_info));
    memset(&ts_data, 0x00, sizeof(data_reg_t));
    pData->type = DATA_UNKNOWN;
    pData->count = 0;
    Addr = REG_TS_STATUS;
    
#if 0
    /* Semisense Specific Driver */
    if(g_miscInitialize == 0)   {
        if(sys_chmod((const char __user *)"/dev/sn310m_dist", 666) < 0) 
        {
            TOUCH_LOG("failed to change the \"/dev/sn310m_dist\" permission.\n");
        }
        else 
        {
            TOUCH_LOG("succeeded to change the \"/dev/sn310m_dist\" permission.\n");
            g_miscInitialize = 1;
        }
    }
#endif
    Addr = REG_TS_STATUS;
    ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)&ts_status.uint, sizeof(Addr), sizeof(status_reg_u));
    if(ret < 0)
    {
        goto i2c_fail;
    }
        /* Touch Point Data */
    if(ts_status.bits.ts_cnt <= MAX_FINGER_NUM && !knock_flag)
    {
        pData->type = DATA_FINGER;
        for(cnt = 0; cnt < ts_status.bits.ts_cnt; cnt++)
        {
            u32 area, pressure;
            Addr = REG_TS_DATA(cnt);
            if(Semisense_I2C_Read((u8 *)&Addr, (u8 *)&ts_data.packet0, sizeof(Addr), sizeof(data_reg_t)) < 0)
            {
                goto i2c_fail;
            }
            info.id[cnt]= ts_data.packet0 >> 12;
            info.x[cnt]= ts_data.packet0 & 0xfff;
            info.y[cnt]= ts_data.packet1 & 0xfff;
            area = ts_data.packet2 & 0xfff;
            pressure = ((ts_data.packet1 >> 8) & 0x00f0) + (ts_data.packet2 >> 12);
            pFingerData = &pData->fingerData[pData->count];
            pFingerData->status = FINGER_PRESSED;
            pFingerData->id = info.id[cnt];
            pFingerData->x = info.x[cnt];
            pFingerData->y = info.y[cnt];
            pFingerData->width_major = 15;
            pFingerData->width_minor = 10;
            pFingerData->orientation = 1;
            pFingerData->pressure = pressure;
            pData->count++;
        }
    }
    /* Knock on/code data - knock on,code */
    else if(knock_flag)
    {
        Addr = LPWG_STATUS_REG;
        ret = Semisense_I2C_Read((u8 *)&Addr, (u8 *)&rData, sizeof(Addr), sizeof(rData));
        if(ret < 0)
        {
            goto i2c_fail;
        }
        TOUCH_LOG("rData = %x\n", rData);
        if(rData == 0x0100)
        {
            pData->type = DATA_KNOCK_ON;
            result = get_lpwg_data(pData, 2);
            TOUCH_LOG("Knock on Occur\n");
            return TOUCH_SUCCESS;
        }
        else if(rData == 0x0200)
        {
            pData->type = DATA_KNOCK_CODE;
            result = get_lpwg_data(pData, tap_count);
            pData->count = tap_count;
            TOUCH_LOG("Knock code Occur\n");
            return TOUCH_SUCCESS;
        }
        else
        {
            TOUCH_LOG("Invalid lpwg status\n");
        }
    }
#if 0
        /* Hovering Data */
        if(ts_status.uint == HOVER_NEAR)
        {
            pData->type = DATA_HOVER_NEAR; 
            pData->hoverState = 0;
            TOUCH_LOG("DEVICE [HOVERING NEAR]\n");
        }
        else if(ts_status.uint == HOVER_FAR)
        {
            pData->type = DATA_HOVER_FAR;
            pData->hoverState = 1;
            TOUCH_LOG("DEVICE [HOVERING FAR]\n");
        }
#endif
    return TOUCH_SUCCESS;
i2c_fail:
    TOUCH_LOG("I2C fail with addr(0x%x)\n", Addr);
    result = TOUCH_FAIL;
    return result;
}

static int Sn280h_ReadIcFirmwareInfo( TouchFirmwareInfo *pFwInfo)
{
    int ret = TOUCH_SUCCESS;
    u16 addr = 0;
    u16 rData = 0;
    TOUCH_FUNC();

    addr = TSC_FLASH_FW_VER_POS;
    ret = Semisense_I2C_Read((u8 *)&addr, (u8 *)&rData, sizeof(addr), sizeof(rData));
    if(ret < 0)
    {
        goto i2c_fail;    
    }
    TOUCH_LOG("IC FIRMWARE VERSION %x\n", rData);

    pFwInfo->isOfficial = 1;
    if (0xa0 <= rData && rData <= 0xFF)
    {
        pFwInfo->isOfficial = 1;
    }
    pFwInfo->version = rData;

    /* Not use data
    pFwInfo->modelID = 0; ex)????
    pFwInfo->moduleMakerID = 0; ex) suntel, tovis, lgit
    pFwInfo->moduleVersion = 0; ex) product module version
    */

    TOUCH_LOG("IC Firmware Official = %d\n", pFwInfo->isOfficial);
    TOUCH_LOG("IC Firmware Version = 0x%04X\n", pFwInfo->version);

    return ret;
i2c_fail:
    TOUCH_LOG("I2C fail with addr(0x%x):data(0x%x)\n", addr, rData);
    ret = TOUCH_FAIL;
    return ret;
}

static int Sn280h_GetBinFirmwareInfo( char *pFilename, TouchFirmwareInfo *pFwInfo)
{
    const struct firmware *fw = NULL;
    int ret = TOUCH_SUCCESS;
    char *pFwFilename = NULL;
    u8 *pFw = NULL;

    TOUCH_FUNC();

    if( pFilename == NULL )
    {
        pFwFilename = (char *)defaultFirmware;
    }
    else
    {
        pFwFilename = pFilename;
    }

    TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

    /* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &(ts->client->dev)/*&client->dev*/);
    if(ret < 0)
    {
        TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", ret);
        ret = TOUCH_FAIL;
        goto earlyReturn;
    }
    pFw = (u8 *)(fw->data);
    pFwInfo->isOfficial = 1;
    pFwInfo->version = pFw[TSC_FLASH_FW_VER_POS];
    
    TOUCH_LOG("BIN Firmware Official = %d\n", pFwInfo->isOfficial);
    TOUCH_LOG("BIN Firmware Version = 0x%04X\n", pFwInfo->version);

	/* Free firmware image buffer */
	release_firmware(fw);
	
earlyReturn:
	return ret;	
 }
static int FlashStartSequnce(u8 flashMemType, u8 OpMode)
{
    int result = TOUCH_SUCCESS;
    u16 addr = 0;
    u16 wdata = 0;

    if(flashMemType == E_MEM_TYPE_EEPROM)
    {
        addr = REG_CMD_EER_PDOWN;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_RESET;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_CSCON;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
    }
    else
    {
        addr = REG_CMD_FLASH_CON_EN;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
    }

    if(flashMemType == E_MEM_TYPE_EEPROM)
    {
        if(OpMode == E_FLASH_OPMODE_READ)
        {
            addr = REG_CMD_EER_MODE;
            wdata = 0x0000;
            if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
                goto fail;
        }
    }
    else
    {
        if(OpMode != E_FLASH_OPMODE_READ)
        {
            addr = REG_CMD_FLASH_AUTH;
            wdata = 0x0000;
            if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
                goto fail;
        }
    }
    return result;
fail:
    TOUCH_LOG("I2C fail with addr(0x%x):data(0x%x)\n", addr, wdata);
    result = TOUCH_FAIL;
    return result;
}
static int FlashEndSequence(u8 flashType)
{
    u16 addr = 0;
    u16 wdata = 0;
    if(flashType == E_MEM_TYPE_EEPROM)
    {
        addr = REG_CMD_EER_CSCON;
        wdata = 0x0000;
    }
    else
    {
        addr = REG_CMD_FLASH_AUTH;
        wdata = 0x0001;
    }
    if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
        return TOUCH_FAIL;

    return TOUCH_SUCCESS;
}

/* To do : function rename */
static int SetISPMode(void)
{
    int result = TOUCH_SUCCESS;
    u16 addr = 0;
    u16 wdata = 0;

    addr = REG_ISP_MODE;
    wdata = 0x0001;
    if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
        goto fail;

    addr = REG_ISP_MODE;
    wdata = 0x0002;
    if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0) 
        goto fail;

    addr = REG_ISP_MODE_BUS;
    wdata = 0x0000;
    if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
        goto fail;

    addr = REG_ISP_MODE_ENABLE;
    wdata = 0xFFFF;
    if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
        goto fail;

    return TOUCH_SUCCESS;
fail:
    TOUCH_LOG("I2C fail with addr(0x%x):data(0x%x)\n", addr, wdata);
    result = TOUCH_FAIL;
    return result;
}

static int FirmwareUpgradeCheck(u8 *flashType)
{
    int result = TOUCH_SUCCESS;
    u16 rdata = 0;
    u16 flashAddr = 0;
    u8 flashMemType = E_MEM_TYPE_EFLASH;

 
    /* Set ISP mode */
    if(SetISPMode() != TOUCH_SUCCESS)
    {
        TOUCH_LOG("Set ISP Mode fail. \n");
         goto fail;
    }
    
    /* Check flash type. */
    flashAddr = REG_ISP_MEM_TYPE;
    if( Semisense_I2C_Read((u8 *)&flashAddr, (u8 *)&rdata, sizeof(flashAddr), sizeof(rdata)) < 0)
    {
        TOUCH_LOG("I2C read fail (flashAddr 0x%x)\n", flashAddr);
        goto fail;
    }
    
    if( rdata == REG_ISP_VAL_ERROR )
    {
        TOUCH_LOG("ISP memory type error(0xDEAD)\n");
        goto fail;
    }
    else
    {
        flashMemType = (rdata == REG_ISP_VAL_EEPROM) ? E_MEM_TYPE_EEPROM : E_MEM_TYPE_EFLASH;
        TOUCH_LOG("ISP memory type [%s]\n", (flashMemType == E_MEM_TYPE_EEPROM) ? "EEPROM" : "EFLASH");
        *flashType = flashMemType;
    }

    if(FlashStartSequnce(flashMemType, E_FLASH_OPMODE_READ) != TOUCH_SUCCESS)
    {
        TOUCH_LOG("Flash sequence fail(operation mode = E_FLASH_OPMODE_READ)\n");
        goto fail;
    }
    /* Read operation. */
    
    return result;
fail : 
    result = TOUCH_FAIL;
    //Sn280h_Reset();
    return result;
}

static int FirmwareErase(u8 flashType)
{
    int result = TOUCH_SUCCESS;
    u16 addr = 0;
    u16 rdata = 0;
    u16 wdata = 0;
    u8 retry = 0;

    /* Start of Erase Operation */
    TOUCH_LOG("[ERASE] START. \n");
    if(FlashStartSequnce(flashType, E_FLASH_OPMODE_ERASE) != TOUCH_SUCCESS)
    {
        TOUCH_LOG("Flash start sequnce fail.\n");
        return TOUCH_FAIL;
    }
    
    if(flashType == E_MEM_TYPE_EFLASH)
    {
        addr = REG_CMD_FLASH_COMMAND;
        wdata = 0x0002;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
            
        addr = 0x0000;
        wdata = 0xFFFF;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
        
        addr = REG_CMD_FLASH_BUSY;
        wdata = 0x8000;
        retry = 0;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
        do
        {
            mdelay(10);
            if(Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata)) < 0)
                TOUCH_LOG("Busy check I2C read fail (retry = %d)\n", retry);
            if(retry)
                TOUCH_LOG("[retry_%d] Busy flag (read data_0x%x) at (addr_0x%x)\n", retry, rdata, addr);
            retry++;
        }while((rdata & 0x8000) && (retry < BUSY_CHECK_RETRY_COUNT ));
    }
    else
    {
    TOUCH_LOG("[ERASE] START eeprom. \n");
        addr = REG_CMD_EER_XPROT;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
        
        addr = REG_CMD_EER_MODE;
        wdata = 0x0007;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = 0x0000;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_XEN;
        wdata = 0x0001;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = 0x0000;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_STATE;
        wdata = 0x0000;
        retry = 0;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        do
        {
            mdelay(10);
            if(Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata)) < 0)
                TOUCH_LOG("Busy check I2C read fail (retry = %d)\n", retry);
            if(retry)
                TOUCH_LOG("[retry_%d] Busy flag (read data_0x%x) at (addr_0x%x)\n", retry, rdata, addr);
            retry++;
        }while(((rdata & 0x0004) == 0) && (retry < BUSY_CHECK_RETRY_COUNT ));
    
        addr = REG_CMD_EER_XEN;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_XPROT;
        wdata = 0x0001;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;

        addr = REG_CMD_EER_MODE;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
    TOUCH_LOG("[ERASE] eeprom. \n");
    }

    if(retry >= BUSY_CHECK_RETRY_COUNT)
    {
        TOUCH_LOG("FW upgrade erase busy check fail.\n");
        goto fail;
    }
    
    if(FlashEndSequence(flashType) != TOUCH_SUCCESS) 
    {
        TOUCH_LOG("Flash end sequnce fail.\n");
        return TOUCH_FAIL;
    }
    /* End of Erase Operation */
    TOUCH_LOG("[ERASE] END. \n");
    
    return TOUCH_SUCCESS;
fail:
    TOUCH_LOG("I2C fail with addr(0x%x):data(0x%x)\n", addr, wdata);
    result = TOUCH_FAIL;
    return result;
}

static int FirmwareProgram(const u8 *fwData, int fwSize, u8 flashType)
{
    int result = TOUCH_SUCCESS;
    u16 wdata = 0;
    u16 rdata = 0;
    u16 addr = 0;
    u16 retry = 0;
    u16 flashAddr = 0;

    /* Start of Program Operation */
    TOUCH_LOG("[PROGRAM] Start.\n");

    
    if(FlashStartSequnce(flashType, E_FLASH_OPMODE_WRITE) != TOUCH_SUCCESS)
    {
        TOUCH_LOG("START Flash sequence fail(operation mode = E_FLASH_OPMODE_READ)\n");
        goto fail;
    }

    if(flashType == E_MEM_TYPE_EFLASH)
    {
        addr = REG_CMD_FLASH_COMMAND;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto fail;
        for(flashAddr = 0; flashAddr < fwSize; flashAddr += 2)
        {
            u16 wAddr = flashAddr;
            if(Semisense_I2C_Write((u8 *)&wAddr, (u8 *)&fwData[flashAddr], sizeof(wAddr), 2) < 0)
                goto i2c_fail;
            addr = REG_CMD_FLASH_BUSY;
            rdata = 0x8000;
            retry = 0;
            do 
            {
                /* if it failed once then, waiting time will be increased */
                if(retry)
                    mdelay(30);
                if(Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata)) < 0)
                    TOUCH_LOG("Busy check I2C read fail retry(%d)\n", retry);
                if(retry)
                    TOUCH_LOG("[%d] Busy flag (0x%x) at (0x%x)\n", retry, rdata, addr);    
                retry++;
            }while((rdata & 0x8000) && (retry < BUSY_CHECK_RETRY_COUNT));

            if(retry >= BUSY_CHECK_RETRY_COUNT)
            {
                TOUCH_LOG("FW Upgrade Program Busy Check Fail.\n");
                goto fail;
            }
        }
    }
    else if(flashType == E_MEM_TYPE_EEPROM)
    {
        u16 pageIndex = 0;
        u16 pageOffset = 0;
        u16 numOfPage = (fwSize + TSC_EEPROM_PAGE_SIZE - 1) / TSC_EEPROM_PAGE_SIZE;
        u16 targetAddr = 0;
        u16 wAddr = 0;

        addr = REG_CMD_EER_XPROT;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto i2c_fail;

        addr = REG_CMD_EER_XEN;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto i2c_fail;

        addr = REG_CMD_EER_MODE;
        wdata = 0x0008;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto i2c_fail;

        addr = REG_CMD_EER_EXTEND;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto i2c_fail;

        for(pageIndex = 0; pageIndex < numOfPage; pageIndex++)
        {
            u16 wLen = (pageIndex == numOfPage - 1) ? (fwSize - (numOfPage - 1) * TSC_EEPROM_PAGE_SIZE) : TSC_EEPROM_PAGE_SIZE;
            for(pageOffset = 0; pageOffset < wLen; pageOffset += 2)
            {
                targetAddr = pageIndex * TSC_EEPROM_PAGE_SIZE + pageOffset;
                wAddr = targetAddr;
                if(Semisense_I2C_Write((u8 *)&wAddr, (u8 *)&fwData[targetAddr],sizeof(wAddr), 2) < 0) 
                {
                    goto i2c_fail;
                }
            }

            addr = REG_CMD_EER_XEN;
            wdata = 0x0001;
            if(Semisense_I2C_Write((u8 *)&addr,(u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0) 
                goto i2c_fail;

            addr = (pageIndex * TSC_EEPROM_PAGE_SIZE);
            wdata = 0x0000; 
            if(Semisense_I2C_Write((u8 *)&addr,(u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0) 
                goto i2c_fail;
            

            addr = REG_CMD_EER_STATE;
            rdata = 0x0000;
            retry = 0;
            do 
            {
                /* if it failed once then, waiting time will be increased */ 
                if(retry) 
                    udelay(100);
#if 0
                if(Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata)) < 0)
                    TOUCH_LOG("Busy check I2C read fail retry(%d)\n", retry);

                if(retry) 
                    TOUCH_LOG("[%d] Busy flag (0x%x) at (0x%x)\n", retry, rdata, addr);
#else

                /* May be log bad effect to firmware updatae. */
                Semisense_I2C_Read((u8 *)&addr, (u8 *)&rdata, sizeof(addr), sizeof(rdata));
                    //TOUCH_LOG("Busy check I2C read fail retry(%d)\n", retry);

//                if(retry) 
//                   TOUCH_LOG("[%d] Busy flag (0x%x) at (0x%x)\n", retry, rdata, addr);
#endif
                retry++;
            } while(((rdata & 0x0004) == 0) && (retry < BUSY_CHECK_RETRY_COUNT)); /* check busy flag & retry count */

            if(retry >= BUSY_CHECK_RETRY_COUNT)
            {
                TOUCH_LOG("FW Upgrade Program Busy Check Fail.\n");
                goto fail;
            }

            addr = REG_CMD_EER_XEN;
            wdata = 0x0000;
            if(Semisense_I2C_Write((u8 *)&addr,(u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
                goto i2c_fail;
            if(pageIndex % 64 == 0 | pageIndex == numOfPage -1)
            {
                TOUCH_LOG("FW Upgrade Write Page(%d) of Total(%d) OK!\n", pageIndex, numOfPage);        
            }
        }

        addr = REG_CMD_EER_XPROT;
        wdata = 0x0001;
        if(Semisense_I2C_Write((u8 *)&addr,(u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0)
            goto i2c_fail;

        addr = REG_CMD_EER_MODE;
        wdata = 0x0000;
        if(Semisense_I2C_Write((u8 *)&addr,(u8 *)&wdata, sizeof(addr), sizeof(wdata)) < 0) 
            goto i2c_fail;
    }

    if(FlashEndSequence(flashType) != TOUCH_SUCCESS)
    {
        TOUCH_LOG("START Flash sequence fail\n");
        goto fail;
    }
    /* Start of Program Operation */
    TOUCH_LOG("[PROGRAM] END.\n");
    return result;
i2c_fail:
    TOUCH_LOG("I2C fail with addr(0x%x):data(0x%x)\n", addr, wdata);
fail:
    result = TOUCH_FAIL;
    return result;
}

static int DoUpgrade(const struct firmware *fw_img, u8 flashType)
{
    int result = TOUCH_SUCCESS;
    int fwSize = 0;
    u16 addr = 0;
    u16 flashAddr = 0;
    u16 rdata = 0;
    u16 wdata = 0;
    u8 *fwData = NULL;

    fwData = (u8 *)(fw_img->data);
    fwSize = fw_img->size;

    if(SetISPMode() != TOUCH_SUCCESS)
    {
        TOUCH_ERR("ISP set fail.\n");
        goto fail;
    }

    if(FirmwareErase(flashType) != TOUCH_SUCCESS)
    {
        TOUCH_ERR("ERASE fail.\n");
        goto fail;
    }

    if(FirmwareProgram(fwData, fwSize, flashType) != TOUCH_SUCCESS)
    {
        TOUCH_ERR("Programming fail.\n");
        goto fail;
    }

    /* Start of Verify Operation */
    TOUCH_LOG("[VERIFY] Start.\n");  

    if(FlashStartSequnce(flashType, E_FLASH_OPMODE_READ) != TOUCH_SUCCESS)
    {
        TOUCH_LOG("Flash start sequence fail(operation mode = E_FLASH_OPMODE_READ)\n");
        goto fail;
    }
    
    for(flashAddr = 0; flashAddr < fwSize; flashAddr += 2)
    {
        u16 wAddr = flashAddr;
        u16 data = ((fwData[flashAddr]) | (fwData[flashAddr + 1] << 8));

        if(Semisense_I2C_Read((u8 *)&wAddr, (u8 *)&rdata, sizeof(wAddr), sizeof(rdata)) < 0)
            goto fail;
        if(data != rdata)
        {
            TOUCH_ERR("[VERIFY] Fw upgrade verify fail. data[0x%0x] at Addr [%d] \n", data, flashAddr);
            goto fail;    
        }
    }

    if(FlashEndSequence(flashType) != TOUCH_SUCCESS)
    {
        TOUCH_LOG("Flash end sequence fail(operation mode = E_FLASH_OPMODE_READ)\n");
        goto fail;
    }

    addr = REG_ISP_MODE;
    wdata = 0x0001;
    if(Semisense_I2C_Write((u8 *)&addr, (u8 *)&wdata, sizeof(addr), sizeof(wdata)) <0)
    {
        TOUCH_ERR("Fw Upgrade Final  Step I2C Fail\n");
        goto fail;
    }
    TOUCH_LOG("[VERIFY] END.\n");  
    return result;
fail : 
    result = TOUCH_FAIL;
    //Add reset
    return result;
}

static int FirmwareUpgrade(const struct firmware *fw_img)
{
    int result = TOUCH_SUCCESS;
    u8 flashType = 0;   
    if(FirmwareUpgradeCheck(&flashType) == TOUCH_SUCCESS)
    {
        TOUCH_LOG("==== [START] upgrading TSC F/W ... ====\n");
        if(DoUpgrade(fw_img, flashType) == TOUCH_SUCCESS)
        {
            TOUCH_LOG("==== [DONE] upgrading TSC F/W ... ====\n");
        }
        else
        {
            result = TOUCH_FAIL;
        }
    }
    else
    {
        result = TOUCH_FAIL;
    }
    return result;
}

static int Sn280h_UpdateFirmware( char *pFilename)
{
    const struct firmware *fw = NULL;
    int result = TOUCH_SUCCESS;
    char *pFwFilename = NULL;
    u8 *pBin = NULL;
    TOUCH_FUNC();

    /* Select firmware */
    if( pFilename == NULL )
    {
        pFwFilename = (char *)defaultFirmware;
    }
    else
    {
        pFwFilename = pFilename;
    }

    TOUCH_LOG("Firmware file name = %s \n", pFwFilename);

    /* Get firmware image buffer pointer from file*/
    result = request_firmware(&fw, pFwFilename, &(ts->client->dev));
    if( result )
    {
        TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", result);
        result = TOUCH_FAIL;
        return result;
    }
    pBin = (u8 *)(fw->data);

    /* Do firmware upgrade */
    result = FirmwareUpgrade(fw);
    if(result != TOUCH_SUCCESS)
    {
        TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", result);
        result = TOUCH_FAIL;
    }

    /* Free firmware image buffer */
    release_firmware(fw);
    return result;
}

static int Sn280h_SetLpwgMode( TouchState newState, LpwgSetting  *pLpwgSetting)
{
    //TouchDriverData *pDriverData = i2c_get_clientdata(client);
    int result = TOUCH_SUCCESS;
    unsigned short Addr = LPWG_STATUS_REG;
    unsigned short wData = 0;
    unsigned short rData = 0;
        
    TOUCH_FUNC();
    ts->currState = newState;
    TOUCH_LOG("ts->currState = %d \n",ts->currState);
    TouchDisableIrq();
    switch(ts->currState)
    {
        case STATE_NORMAL:
            //Sn280h_Reset();
            TOUCH_LOG("SN280h was changed to NORMAL\n");
            break;
        case STATE_KNOCK_ON_ONLY:
            knock_flag = 1;
            wData = 0x0021;
            result = Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData, sizeof(Addr), sizeof(wData));
            if(result < 0)
                goto i2c_fail;
            TOUCH_LOG("SN280h was changed to STATE_KNOCK_ON_ONLY\n");
            break;
        case STATE_KNOCK_ON_CODE:
            knock_flag = 1;
            tap_count = pLpwgSetting->tapCount;
            wData = (tap_count << 4) | 0x0003;
            result = Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData, sizeof(Addr), sizeof(wData));
            if(result < 0)
                goto i2c_fail;
            TOUCH_LOG("SN280h was changed to STATE_KNOCK_ON_CODE \n");
            break;
        case STATE_NORMAL_HOVER:
            #if 0
            wData = 0x0404;
            Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData, sizeof(Addr), sizeof(wData));
            TOUCH_LOG("Set hover on mode.\n");
            #endif
            break;
        case STATE_HOVER:
            #if 0
            wData = 0x0404;
            Semisense_I2C_Write((u8 *)&Addr, (u8 *)&wData, sizeof(Addr), sizeof(wData));
            TOUCH_LOG("Set hover on mode.\n");
            #endif
            break;
        case STATE_OFF:
            TouchSetGpioReset(0);
            TOUCH_LOG("SN280h was changed to STATE_OFF\n");
            break;
        default:
            TOUCH_ERR("Unknown State %d", newState);
            break;
    }
    TouchEnableIrq();
    return result;
 i2c_fail:
    TOUCH_LOG("I2C fail with addr(0x%x)\n", Addr);
    result = TOUCH_FAIL;
    return result;

}

static void Sn280h_sd_write(char *data, int time)
{
    int fd;
    char *fname = "/mnt/sdcard/touch_self_test.txt";
    char time_string[64] = {0};
    struct timespec my_time;
    struct tm my_date;

    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);

    fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND|O_SYNC, 0644);

    if (fd >= 0)
    {
        if (time > 0)
        {
            my_time = __current_kernel_time();
            time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
            snprintf(time_string, 64, "\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
            my_date.tm_mon + 1,my_date.tm_mday, my_date.tm_hour,
            my_date.tm_min, my_date.tm_sec,
            (unsigned long) my_time.tv_nsec / 1000000);
            sys_write(fd, time_string, strlen(time_string));
        }
        sys_write(fd, data, strlen(data));
        sys_close(fd);
    }

    set_fs(old_fs);
}

static int Sn280h_DoSelfDiagnosis(int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
    int offset = 0;
    *pRawStatus = TOUCH_SUCCESS;
    *pChannelStatus = TOUCH_SUCCESS;
    
    Sn280h_sd_write(pBuf, 1);
    msleep(30);

    offset += read_channel_data(pBuf, CAP_REFERENCE);
    Sn280h_sd_write(pBuf, 0);   
    msleep(30);
    memset(pBuf, 0, bufSize);

    /* Additional Test */
    WRITE_SYSBUF(pBuf, offset, "======ADDITIONAL======\n");
    Sn280h_sd_write(pBuf, 0);
    msleep(30);
    memset(pBuf, 0, bufSize);

    *pDataLen = 0;
    return TOUCH_SUCCESS;
}

static int Sn280h_DoSelfDiagnosis_Lpwg(int *pLpwgStauts, char* pBuf, int bufSize, int* pDataLen)
{
    return TOUCH_SUCCESS;
}

static void Sn280h_PowerOn(int isOn)
{
    #if 1
    mt_set_gpio_mode ( GPIO_TOUCH_RESET, GPIO_TOUCH_RESET_M_GPIO );
    mt_set_gpio_dir ( GPIO_TOUCH_RESET, GPIO_DIR_OUT );

    hwPowerOn ( MT6328_POWER_LDO_VGP1, VOL_3000, "TP" );
    TouchSetGpioReset(0);
    mdelay(1);
    TouchSetGpioReset(1);
    msleep(2);
    TouchSetGpioReset(0);
    msleep(1);
    TouchSetGpioReset(1);
    TOUCH_LOG ( "turned on the power ( VGP1 )\n" );
    //Sn280h_Reset();
    msleep ( 300 );//
    #endif
}

static void Sn280h_ClearInterrupt(void)
{

}

static void Sn280h_NotifyHandler(TouchNotify notify, int data)
{

}

TouchDeviceControlFunction Sn280h_Func = {
    .Power                  = Sn280h_PowerOn,
	.Initialize 			= Sn280h_Initialize,
	.Reset 					= Sn280h_Reset,
	.InitRegister 			= Sn280h_InitRegister,
	.ClearInterrupt         = Sn280h_ClearInterrupt,
	.InterruptHandler 		= Sn280h_InterruptHandler,
	.ReadIcFirmwareInfo 	= Sn280h_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo 	= Sn280h_GetBinFirmwareInfo,
	.UpdateFirmware 		= Sn280h_UpdateFirmware,
	.SetLpwgMode 			= Sn280h_SetLpwgMode,
	.DoSelfDiagnosis 		= Sn280h_DoSelfDiagnosis,
	.DoSelfDiagnosis_Lpwg	= Sn280h_DoSelfDiagnosis_Lpwg,
	.device_attribute_list 	= Sn280h_attribute_list,
	.NotifyHandler          = Sn280h_NotifyHandler,
};


/* End Of File */


