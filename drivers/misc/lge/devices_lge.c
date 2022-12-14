#include <linux/kernel.h>
#include <asm/setup.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/module.h>
#include <mach/board_lge.h>

#include "mt_auxadc_sw.h"

static hw_rev_type lge_bd_rev;
EXPORT_SYMBOL(lge_bd_rev);

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
static bool is_mac_os;
#endif

/* CAUTION: These strings are come from LK. */
char *rev_str[] = {
	"evb1",
	"evb2",
	"rev_a",
	"rev_b",
	"rev_c",
	"rev_d",
	"rev_e",
	"rev_f",
	"rev_g",
	"rev_h",
	"rev_i",
	"rev_10",
	"rev_11",
	"rev_12",
	"rev_13",
	"reserved",
};

static int __init board_revno_setup(char *rev_info)
{
	int i;
	lge_bd_rev = HW_REV_EVB1;
	for (i = 0; i < HW_REV_MAX; i++) {
		if (!strncmp(rev_info, rev_str[i], (strlen(rev_info) > strlen(rev_str[i])) ? strlen(rev_info) : strlen(rev_str[i]))) {
			lge_bd_rev = (hw_rev_type) i;
			/* it is defined externally in <asm/system_info.h> */
			// system_rev = lge_bd_rev;
			break;
		}
	}

	printk(KERN_ALERT "unified LK bootcmd lge.rev setup: %s\n", rev_str[lge_bd_rev]);
	return 1;
}
__setup("lge.rev=", board_revno_setup);

hw_rev_type lge_get_board_revno(void)
{
	return lge_bd_rev;
}


static usb_cable_type lge_cable;
EXPORT_SYMBOL(lge_cable);

/* CAUTION: These strings are come from LK. */
char *cable_str[] = {
	" "," "," "," ", " ", " ",
	"LT_56K",
	"LT_130K",
	"400MA",
	"DTC_500MA",
	"Abnormal_400MA",
	"LT_910K",
	"NO_INIT",
};

static int __init board_cable_setup(char *cable_info)
{
	int i;
	lge_cable = NO_INIT_CABLE;
	for (i = LT_CABLE_56K; i < NO_INIT_CABLE; i++) {
		if (!strncmp(cable_info, cable_str[i], (strlen(cable_info) > strlen(cable_str[i])) ? strlen(cable_info) : strlen(cable_str[i]))) {
			lge_cable = (usb_cable_type) i;
			/* it is defined externally in <asm/system_info.h> */
			// system_rev = lge_bd_rev;
			break;
		}
	}

	printk(KERN_ALERT "unified LK bootcmd lge.cable: %s\n", cable_str[lge_cable]);

	return 1;
}
__setup("lge.cable=", board_cable_setup);

usb_cable_type lge_get_board_cable(void)
{
	return lge_cable;
}

static enum lge_laf_mode_type lge_laf_mode = LGE_LAF_MODE_NORMAL;

int __init lge_laf_mode_init(char *s)
{
    if (strcmp(s, "") && strcmp(s, "MID"))
        lge_laf_mode = LGE_LAF_MODE_LAF;

    return 1;
}
__setup("androidboot.laf=", lge_laf_mode_init);

enum lge_laf_mode_type lge_get_laf_mode(void)
{
    return lge_laf_mode;
}

#if defined(CONFIG_LGE_LUT_KCAL)
int g_kcal_r = 255;
int g_kcal_g = 255;
int g_kcal_b = 255;

EXPORT_SYMBOL(g_kcal_r);
EXPORT_SYMBOL(g_kcal_g);
EXPORT_SYMBOL(g_kcal_b);

static int __init lcd_kcal(char *lcd_kcal)
{
        char valid_k = 0;

        sscanf(lcd_kcal, "%d|%d|%d|%c", &g_kcal_r, &g_kcal_g, &g_kcal_b, &valid_k);
        printk("lcd_kcal is %d|%d|%d|%c\n", g_kcal_r, g_kcal_g, g_kcal_b, valid_k);

        if(valid_k != 'K')
        {
            printk("kcal not calibrated yet : %d\n", valid_k);
            g_kcal_r = g_kcal_g = g_kcal_b = 255;
            printk("set to default : %d\n", g_kcal_r);
        }
        return 0;
}
__setup("lge.kcal=", lcd_kcal);
#endif

unsigned int touch_module;
EXPORT_SYMBOL(touch_module);

static int __init touch_module_check(char *touch)
{
	if(!strcmp(touch,"PRIMARY_MODULE")){
		touch_module = 0;
		printk("touch_module 0\n");
	}else if(!strcmp(touch,"SECONDARY_MODULE")){
		touch_module = 1;
		printk("touch_module 1\n");
	}else if(!strcmp(touch,"TERITARY_MODULE")){
		touch_module = 2;
	}else if(!strcmp(touch,"QUATENARY_MODULE")){
		touch_module = 3;
	}
	printk("[TOUCH][BOOTCMD]kernel touch module check : %s\n", touch);
	return 0;
}
early_param("lge.touchModule", touch_module_check);

unsigned char g_qem_check;
EXPORT_SYMBOL(g_qem_check);

static int __init qem_check(char *qem)
{
	g_qem_check = *qem;
	printk("kernel qem check : %c\n", g_qem_check);
	return 0;
}
early_param("qem", qem_check);

LGBmCableId g_lgbmBootUsbCableId = 0;
EXPORT_SYMBOL(g_lgbmBootUsbCableId);

static int __init usb_id(char *usbid)
{
	sscanf(usbid, "%u", &g_lgbmBootUsbCableId);
	printk("kernel g_lgbmBootUsbCableId : %d\n", g_lgbmBootUsbCableId);
	return 0;
}
early_param("usbid", usb_id);

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
void lgeusb_set_host_os(u16 w_length)
{
        switch (w_length) {
        case MAC_OS_TYPE:
                is_mac_os = true;
                break;
        case WIN_LINUX_TYPE:
                is_mac_os = false;
                break;
        default:
                break;
        }
}

bool lgeusb_get_host_os(void)
{
        return is_mac_os;
}
#endif

/* get boot mode information from cmdline.
 * If any boot mode is not specified,
 * boot mode is normal type.
 */
static enum lge_boot_mode_type lge_boot_mode = LGE_BOOT_MODE_NORMAL;
static int __init lge_boot_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGER;
	else if (!strcmp(s, "chargerlogo"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
	else if (!strcmp(s, "qem_56k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_56K;
	else if (!strcmp(s, "qem_130k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_130K;
	else if (!strcmp(s, "qem_910k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_910K;
	else if (!strcmp(s, "pif_56k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_56K;
	else if (!strcmp(s, "pif_130k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_130K;
	else if (!strcmp(s, "pif_910k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_910K;
	/* LGE_UPDATE_S for MINIOS2.0 */
	else if (!strcmp(s, "miniOS"))
		lge_boot_mode = LGE_BOOT_MODE_MINIOS;
	pr_info("ANDROID BOOT MODE : %d %s\n", lge_boot_mode, s);
	/* LGE_UPDATE_E for MINIOS2.0 */

	return 1;
}
__setup("androidboot.mode=", lge_boot_mode_init);

enum lge_boot_mode_type lge_get_boot_mode(void)
{
	return lge_boot_mode;
}

int lge_get_factory_boot(void)
{
	int res;

	/*   if boot mode is factory,
	 *   cable must be factory cable.
	 */
	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_QEM_56K:
	case LGE_BOOT_MODE_QEM_130K:
	case LGE_BOOT_MODE_QEM_910K:
	case LGE_BOOT_MODE_PIF_56K:
	case LGE_BOOT_MODE_PIF_130K:
	case LGE_BOOT_MODE_PIF_910K:
	case LGE_BOOT_MODE_MINIOS:
		res = 1;
		break;
	default:
		res = 0;
		break;
	}
	return res;
}

#if defined(CONFIG_LGE_KSWITCH)
static int kswitch_status;
static int atoi(const char *name)
{
	int val = 0;

	for (;; name++) {
		switch (*name) {
		case '0' ... '9':
			val = 10*val+(*name-'0');
			break;
		default:
			return val;
		}
	}
}

static int __init kswitch_setup(char *value)
{
	kswitch_status = atoi(value);

	if (kswitch_status < 0)
		kswitch_status = 0;

	printk(KERN_INFO "[KSwitch] %d \n", kswitch_status);
	return 1;
}
__setup("kswitch=", kswitch_setup);

int lge_get_kswitch_status(void)
{
    return kswitch_status;
}
#endif

