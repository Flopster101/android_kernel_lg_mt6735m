#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/types.h>

#ifdef CONFIG_MTK_SMART_BATTERY
#include <mach/charging.h>
#include <mach/battery_common.h>
#include <mach/battery_meter.h>
#include "cust_charging.h"
#endif

#ifdef CONFIG_MACH_LGE
#include <mach/board_lge.h>
#endif

#ifdef CONFIG_MTK_SMART_BATTERY
extern PMU_ChargerStruct BMT_status;
extern void wake_up_bat(void);
#endif

static int g_AtCmdChargeMode = 0;		//for at%charge
static int g_AtCmdUsbId = 0;     		//for at%usbiadc
static int g_AtCmdUsbAdc = 0;    		//for at%usbiadc
static int g_AtCmdChargingModeOff = 0;

/* Internal APIs */
extern int IMM_auxadc_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern unsigned int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount, int trimd);

#define AUXADC_USB_ID_CHANNEL 12
static int get_usb_id_voltage(void)
{
#ifdef AUXADC_USB_ID_CHANNEL
	int data[4] = {0, 0, 0, 0};
	int rawvalue    = 0;
	int adc_voltage   = 0;

	if (IMM_auxadc_GetOneChannelValue(AUXADC_USB_ID_CHANNEL, data, &rawvalue) < 0) {
		printk(KERN_INFO "[LGBM_AtCmd] Failed to read AuxADC\n");
		return -1;
	}

	adc_voltage = g_AtCmdUsbAdc = data[0] * 100 + data[1];	//g_AtCmdUsbAdc for at%usbiadc

	printk(KERN_INFO "[LGBM_AtCmd] USB_Cable ADC : %d\n", adc_voltage);

	return adc_voltage;
#endif

#ifdef PMIC_AUXADC_USB_ID_CHANNEL
#error not implemented.
#endif
}

typedef struct {
	int adc_min;
	int adc_max;
	int cable_type;
} usb_cable_adc_type;

static usb_cable_adc_type cable_adc_table[]    =   {
/*                  adc_min     adc_max     cable_type  */
/* _56K     */  {   13,         34,         LT_CABLE_56K},
/* _130K    */  {   35,         55,         LT_CABLE_130K},
/* _910K    */  {   87,         108,        LT_CABLE_910K},
/* _USER    */  {   110,        180,        USB_CABLE_400MA},
};

int get_usb_type(void)
{
	int cable_enum = 0;
	int voltage = 0;
	int usb_cable_type_num = NO_INIT_CABLE;

	voltage = get_usb_id_voltage();
	if (voltage < 0)
		return NO_INIT_CABLE;

	for (cable_enum = 0; cable_enum < ARRAY_SIZE(cable_adc_table); cable_enum++) {
		if ((cable_adc_table[cable_enum].adc_min <= voltage) &&
			(voltage <= cable_adc_table[cable_enum].adc_max))
		break;
	}

	if (cable_enum == ARRAY_SIZE(cable_adc_table))
		usb_cable_type_num = ABNORMAL_USB_CABLE_400MA;
	else
		usb_cable_type_num = cable_adc_table[cable_enum].cable_type;

	switch (usb_cable_type_num) {
	case LT_CABLE_56K:
		g_AtCmdUsbId = 56;
		break;
	case LT_CABLE_130K:
		g_AtCmdUsbId = 130;
		break;
	case LT_CABLE_910K:
		g_AtCmdUsbId = 910;
		break;
	default:
		g_AtCmdUsbId = 0;
		break;
	}

	return usb_cable_type_num;
}

#define PMIC_AUXADC_PCB_REV_CHANNEL	7
static int get_pcb_revision_voltage(void)
{
#ifdef AUXADC_PCB_REV_CHANNEL
#error not implemented.
#endif

#ifdef PMIC_AUXADC_PCB_REV_CHANNEL
	int adc_voltage = 0;

	adc_voltage = PMIC_IMM_GetOneChannelValue(PMIC_AUXADC_PCB_REV_CHANNEL, 5, 0);

	printk(KERN_INFO "[LGBM_AtCmd] PCB_Revision ADC : %d\n", adc_voltage);

	return adc_voltage;
#endif
}

/* External APIs */
int get_AtCmdChargingModeOff(void)
{
	return g_AtCmdChargingModeOff;
}
EXPORT_SYMBOL(get_AtCmdChargingModeOff);

/* sysfs */
static ssize_t show_LGBM_AtCmdUsbidadc(struct device *dev, struct device_attribute *attr, char *buf)
{
	get_usb_type();
	return sprintf(buf, "%d,%d",g_AtCmdUsbAdc,g_AtCmdUsbId);
}
static DEVICE_ATTR(LGBM_AtCmdUsbidadc, 0444, show_LGBM_AtCmdUsbidadc, NULL);

static ssize_t show_LGBM_AtCmdCharge(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* return factory charge mode */
	return sprintf(buf, "%d\n", g_AtCmdChargeMode);
}
static ssize_t store_LGBM_AtCmdCharge(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	/* set factory charge mode */
	if(buf != NULL && size != 0) {
		if(buf[0] == '1')
			g_AtCmdChargeMode = 1;
		else
			g_AtCmdChargeMode = 0;
	}

#ifdef CONFIG_MTK_SMART_BATTERY
	wake_up_bat();
#endif

	return size;
}
static DEVICE_ATTR(LGBM_AtCmdCharge, 0664, show_LGBM_AtCmdCharge, store_LGBM_AtCmdCharge);

static ssize_t show_LGBM_AtCmdChcomp(struct device *dev, struct device_attribute *attr, char *buf)
{
	int isChargeComplete = 0;
	int voltage = battery_meter_get_battery_voltage(KAL_TRUE);

	if (voltage > RECHARGING_VOLTAGE) {
		isChargeComplete = 1;
	}

	return sprintf(buf, "%d\n", isChargeComplete);
}
static DEVICE_ATTR(LGBM_AtCmdChcomp, 0444, show_LGBM_AtCmdChcomp, NULL);

static ssize_t show_LGBM_AtCmdChargingModeOff(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_AtCmdChargingModeOff);
}
static ssize_t store_LGBM_AtCmdChargingModeOff(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	if (buf != NULL && size != 0) {
		if(buf[0] == '1')
			g_AtCmdChargingModeOff = 1;
	}

#ifdef CONFIG_MTK_SMART_BATTERY
	wake_up_bat();
#endif

	return size;
}
static DEVICE_ATTR(LGBM_AtCmdChargingModeOff, 0664, show_LGBM_AtCmdChargingModeOff, store_LGBM_AtCmdChargingModeOff);

static ssize_t show_LGBM_AtCmdBattExist(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", BMT_status.charger_exist);
}
static DEVICE_ATTR(LGBM_AtCmdBattExist, 0444, show_LGBM_AtCmdBattExist, NULL);

static ssize_t show_LGBM_AtCmdBatl(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* return battery voltage */

	int batVolt = 0;

	if( g_AtCmdChargeMode == 1 )
		batVolt = battery_meter_get_battery_voltage(KAL_TRUE);
	else
		batVolt = BMT_status.bat_vol;

	return sprintf(buf, "%d\n", batVolt);
}
static DEVICE_ATTR(LGBM_AtCmdBatl, 0444, show_LGBM_AtCmdBatl, NULL);

static ssize_t show_LGBM_AtCmdBatmp(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* return battery temperature */

	kal_int32 batTemp = 0;

	if( g_AtCmdChargeMode == 1 )
		batTemp = battery_meter_get_battery_temperature();
	else
		batTemp = BMT_status.temperature;

	return sprintf(buf, "%d\n", batTemp);
}
static DEVICE_ATTR(LGBM_AtCmdBatmp, 0444, show_LGBM_AtCmdBatmp, NULL);

static ssize_t show_LGBM_AtCmdPcbrev(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* return pcb revision voltage */

	int pcb_rev_voltage;

	pcb_rev_voltage = get_pcb_revision_voltage();

	return sprintf(buf, "%d\n", pcb_rev_voltage);
}
static DEVICE_ATTR(LGBM_AtCmdPcbrev, 0444, show_LGBM_AtCmdPcbrev, NULL);

/* should be called in battery_probe() */
int LGBM_AtCmd_create_files(struct platform_device *dev)
{
	int ret_device_file = 0;

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdCharge);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdChcomp);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdUsbidadc);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdChargingModeOff);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBattExist);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBatl);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBatmp);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdPcbrev);

	return 0;
}
EXPORT_SYMBOL(LGBM_AtCmd_create_files);

int LGBM_AtCmd_init(struct platform_device *dev)
{
	return 0;
}
EXPORT_SYMBOL(LGBM_AtCmd_init);
