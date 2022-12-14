#ifndef __LGE_BATTERY_PROFILE_H__
#define __LGE_BATTERY_PROFILE_H__

/* used in cust_battery_meter_table.h */
#define INIT_BATTERY_PROFILE(maker) \
static int Q_MAX_POS_50 = Q_MAX_POS_50_##maker; \
static int Q_MAX_POS_25 = Q_MAX_POS_25_##maker; \
static int Q_MAX_POS_0 = Q_MAX_POS_0_##maker; \
static int Q_MAX_NEG_10 = Q_MAX_NEG_10_##maker; \
static int Q_MAX_POS_50_H_CURRENT = Q_MAX_POS_50_H_CURRENT_##maker; \
static int Q_MAX_POS_25_H_CURRENT = Q_MAX_POS_25_H_CURRENT_##maker; \
static int Q_MAX_POS_0_H_CURRENT = Q_MAX_POS_0_H_CURRENT_##maker; \
static int Q_MAX_NEG_10_H_CURRENT = Q_MAX_NEG_10_H_CURRENT_##maker; \
static int battery_profile_size = BATTERY_PROFILE_SIZE_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t0 = battery_profile_t0_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t1 = battery_profile_t1_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t2 = battery_profile_t2_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t3 = battery_profile_t3_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_temperature = battery_profile_temperature_##maker; \
static int r_profile_size = R_PROFILE_SIZE_##maker; \
static R_PROFILE_STRUC *r_profile_t0 = r_profile_t0_##maker; \
static R_PROFILE_STRUC *r_profile_t1 = r_profile_t1_##maker; \
static R_PROFILE_STRUC *r_profile_t2 = r_profile_t2_##maker; \
static R_PROFILE_STRUC *r_profile_t3 = r_profile_t3_##maker; \
static R_PROFILE_STRUC *r_profile_temperature = r_profile_temperature_##maker;

#define DECLARE_PROFILE(id, maker) \
static int Q_MAX_POS_50_##id = Q_MAX_POS_50_##maker; \
static int Q_MAX_POS_25_##id = Q_MAX_POS_25_##maker; \
static int Q_MAX_POS_0_##id = Q_MAX_POS_0_##maker; \
static int Q_MAX_NEG_10_##id = Q_MAX_NEG_10_##maker; \
static int Q_MAX_POS_50_H_CURRENT_##id = Q_MAX_POS_50_H_CURRENT_##maker; \
static int Q_MAX_POS_25_H_CURRENT_##id = Q_MAX_POS_25_H_CURRENT_##maker; \
static int Q_MAX_POS_0_H_CURRENT_##id = Q_MAX_POS_0_H_CURRENT_##maker; \
static int Q_MAX_NEG_10_H_CURRENT_##id = Q_MAX_NEG_10_H_CURRENT_##maker; \
static int battery_profile_size_##id = BATTERY_PROFILE_SIZE_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t0_##id = battery_profile_t0_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t1_##id = battery_profile_t1_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t2_##id = battery_profile_t2_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_t3_##id = battery_profile_t3_##maker; \
static BATTERY_PROFILE_STRUC *battery_profile_temperature_##id = battery_profile_temperature_##maker; \
static int r_profile_size_##id = R_PROFILE_SIZE_##maker; \
static R_PROFILE_STRUC *r_profile_t0_##id = r_profile_t0_##maker; \
static R_PROFILE_STRUC *r_profile_t1_##id = r_profile_t1_##maker; \
static R_PROFILE_STRUC *r_profile_t2_##id = r_profile_t2_##maker; \
static R_PROFILE_STRUC *r_profile_t3_##id = r_profile_t3_##maker; \
static R_PROFILE_STRUC *r_profile_temperature_##id = r_profile_temperature_##maker;

/* used in battery_meter.c */
#define SET_BATTERY_PROFILE(id) \
{ \
	Q_MAX_POS_50 = Q_MAX_POS_50_##id; \
	Q_MAX_POS_25 = Q_MAX_POS_25_##id; \
	Q_MAX_POS_0 = Q_MAX_POS_0_##id; \
	Q_MAX_NEG_10 = Q_MAX_NEG_10_##id; \
	Q_MAX_POS_50_H_CURRENT = Q_MAX_POS_50_H_CURRENT_##id; \
	Q_MAX_POS_25_H_CURRENT = Q_MAX_POS_25_H_CURRENT_##id; \
	Q_MAX_POS_0_H_CURRENT = Q_MAX_POS_0_H_CURRENT_##id; \
	Q_MAX_NEG_10_H_CURRENT = Q_MAX_NEG_10_H_CURRENT_##id; \
	battery_profile_size = battery_profile_size_##id; \
	battery_profile_t0 = battery_profile_t0_##id; \
	battery_profile_t1 = battery_profile_t1_##id; \
	battery_profile_t2 = battery_profile_t2_##id; \
	battery_profile_t3 = battery_profile_t3_##id; \
	battery_profile_temperature = battery_profile_temperature_##id; \
	r_profile_size = r_profile_size_##id; \
	r_profile_t0 = r_profile_t0_##id; \
	r_profile_t1 = r_profile_t1_##id; \
	r_profile_t2 = r_profile_t2_##id; \
	r_profile_t3 = r_profile_t3_##id; \
	r_profile_temperature = r_profile_temperature_##id; \
}
#endif
