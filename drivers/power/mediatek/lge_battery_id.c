#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/module.h>

#include <mach/lge_battery_id.h>

char *battery_id_str[] = {
	"Unknown",
	"DS2704_N",
	"DS2704_L",
	"DS2704_C",
	"ISL6296_N",
	"ISL6296_L",
	"ISL6296_C",
	"RA4301_VC0",
	"RA4301_VC1",
	"RA4301_VC1",
	"SW3800_VC0",
	"SW3800_VC1",
	"SW3800_VC2",
};
BATT_ID battery_id = BATT_ID_UNKNOWN;

BATT_ID lge_get_battery_id(void)
{
	return battery_id;
}
EXPORT_SYMBOL(lge_get_battery_id);

char* lge_get_battery_id_str(void)
{
	if (battery_id < BATT_ID_MAX)
		return battery_id_str[battery_id];
	return battery_id_str[BATT_ID_UNKNOWN];
}
EXPORT_SYMBOL(lge_get_battery_id_str);

static int __init batt_info(char *batt_id)
{
	int i;

	for (i = 0; i < BATT_ID_MAX; i++) {
		if (strcmp(batt_id, battery_id_str[i]) == 0)
			break;
	}
	if (i < BATT_ID_MAX) {
		battery_id = i;
	}

	return 0;
}
early_param("lge.battid", batt_info);