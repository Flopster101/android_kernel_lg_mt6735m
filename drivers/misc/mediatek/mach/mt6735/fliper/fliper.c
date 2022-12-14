#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <asm/uaccess.h>

#include <linux/platform_device.h>
#include <linux/suspend.h>

#include <linux/timer.h>
#include <linux/jiffies.h>

#include <linux/earlysuspend.h>

#include <mach/fliper.h>
#include <mach/mt_vcore_dvfs.h>
#define TAG "[FLIPER]"
#define SEQ_printf(m, x...)	    \
	do {			    \
		if (m)		    \
		seq_printf(m, x);	\
		else		    \
		printk(x);	    \
	} while (0)

#define X_ms 100
#define Y_steps (2000/X_ms)
#define Z_steps (3000/X_ms)
#define BW_THRESHOLD 1500
#define PERFORMANCE 200
#define JUST_MAKE 1500
#define BW_THRESHOLD_MAX 9000
#define BW_THRESHOLD_MIN 100

#define DISABLE_IN_EARLY_SUSPEND
enum {
	Default = 0,
	Low_Power_Mode = 1,
	Just_Make_Mode = 2,
	Performance_Mode = 3,
};
static int bw_threshold = 0;
static int fliper_enabled = 0;
static void enable_fliper(void);
static void disable_fliper(void);
static int vcore_high(void);
static int vcore_low(void);
extern unsigned int get_ddr_type(void)__attribute__((weak));
static int pp_index1, pp_index2;
/* define supported DRAM types */
enum
{
	LPDDR2 = 0,
	DDR3_16,
	DDR3_32,
	LPDDR3,
	mDDR,
};
static int performance_mode = 0;
static int fliper_debug = 0;
static ssize_t mt_fliper_write(struct file *filp, const char *ubuf,
		size_t cnt, loff_t *data)
{
	char buf[64];
	int ret;
	unsigned long arg1, arg2;
	char option[64], arg[10];
	int i, j;

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = '\0';

	pr_emerg(TAG"option buffer is %s\n", buf);

	/* get option */
	for (i = 0; i < cnt && buf[i] != ' '; i++)
		option[i] = buf[i];
	option[i] = '\0';
	pr_emerg(TAG"option string is %s\n", option);

	/* get arg1 */
	for (; i < cnt && buf[i] == ' '; i++)
		;
	for (j = 0; i < cnt && buf[i] != ' '; i++, j++)
		arg[j] = buf[i];
	arg[j] = '\0';
	pr_emerg(TAG"arg1 %s end", arg);
	if (j > 0) {
		ret = kstrtoul(arg, 0, (unsigned long *)&arg1);
		if (ret < 0) {
			pr_emerg(TAG"1 ret of kstrtoul is broke\n");
			return ret;
		}
	}

	/* get arg2 */
	for (; i < cnt && buf[i] == ' '; i++)
		;
	for (j = 0; i < cnt && buf[i] != ' '; i++, j++)
		arg[j] = buf[i];
	arg[j] = '\0';
	if (j > 0) {
		ret = kstrtoul(arg, 0, (unsigned long *)&arg2);
		if (ret < 0) {
			pr_emerg(TAG"1 ret of kstrtoul is broke\n");
			return ret;
		}
	}

	if(strncmp(option, "ENABLE", 6) == 0){
		fliper_enabled = arg1;
		if (fliper_enabled)
			enable_fliper();
		else
			disable_fliper();
	}else if(strncmp(option, "DEBUG", 5) == 0){
		fliper_debug ^= arg1;
	}else if(strncmp(option, "POWER_MODE", 10) == 0){
		if (arg1 == Default) {
			enable_fliper();
			fliper_enabled = 1;
			fliper_set_bw(BW_THRESHOLD);
		} else if (arg1 == Low_Power_Mode) {
			disable_fliper();
			fliper_enabled = 0;
		} else if (arg1 == Just_Make_Mode) {
			enable_fliper();
			fliper_enabled = 1;
			fliper_set_bw(JUST_MAKE);
		} else if (arg1 == Performance_Mode) {
			enable_fliper();
			fliper_enabled = 1;
			performance_mode = 1;
			pp_index2 = 0;
			fliper_set_bw(PERFORMANCE);
		}
	}else if(strncmp(option, "RESTORE", 7) == 0){
		fliper_restore_bw();
	}
	return cnt;

}

static int mt_fliper_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "----------------------------------------\n");
	SEQ_printf(m, "Fliper Enabled:%d, bw threshold:%d MB/s\n", fliper_enabled, bw_threshold);
	SEQ_printf(m, "----------------------------------------\n");
	return 0;
}
/*** Seq operation of mtprof ****/
static int mt_fliper_open(struct inode *inode, struct file *file) 
{ 
	return single_open(file, mt_fliper_show, inode->i_private);
} 

static const struct file_operations mt_fliper_fops = { 
	.open = mt_fliper_open,
	.write = mt_fliper_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/******* POWER PERF TRANSFORMER *********/
#include <asm/div64.h>
//Cache info
#include <mach/mt_cpufreq.h>

static void mt_power_pef_transfer(void);
static DEFINE_TIMER(mt_pp_transfer_timer, (void *)mt_power_pef_transfer, 0, 0);

static void mt_power_pef_transfer_work(void);
static DECLARE_WORK(mt_pp_work,(void *) mt_power_pef_transfer_work);
//VOCRE

static int vcore_high()
{
	int ret = 0;
	ret = vcorefs_request_dvfs_opp(KIR_EMIBW, OPPI_PERF);
	return ret;
}
static int vcore_low()
{
	int ret = 0;
	ret = vcorefs_request_dvfs_opp(KIR_EMIBW, OPPI_UNREQ);
	return ret;
}
//EMI
extern unsigned long long get_mem_bw(void);
static void mt_power_pef_transfer_work()
{
	unsigned long long emi_bw = 0;
	int perf_mode = -1;
	int ret;
	unsigned long long t1, t2;
	t1 = 0; t2 = 0;
	/*  Get EMI*/
	if(fliper_debug == 1){
		t1 = sched_clock();
		emi_bw = get_mem_bw();
		t2 = sched_clock();
	}else
		emi_bw = get_mem_bw();

	if (!performance_mode) {
		if(emi_bw > bw_threshold)
			perf_mode = 1;

		if(perf_mode == 1){
			if(pp_index1 == 0){
				ret = vcore_high();
				printk(KERN_CRIT"\n<<SOC DVFS FLIPER>> flip to S(%d), %llu\n", ret, emi_bw);
			}
			pp_index1 = 1 << Y_steps;
		}else{
			if(pp_index1 == 1){
				ret = vcore_low();
				printk(KERN_CRIT"\n<<SOC DVFS FLIPER>> flip to E(%d), %llu\n", ret, emi_bw);
			}
			pp_index1 = pp_index1 >> 1;
		}
	} else {
		if (pp_index2 == 0){
			ret = vcore_high();
			printk(KERN_CRIT"\n<<SOC DVFS FLIPER>> perfservice(PERFORMANCE) to S(%d)\n", ret);
		}
		if (pp_index2 >= Z_steps) {
			ret = vcore_low();
			performance_mode = 0;
			printk(KERN_CRIT"\n<<SOC DVFS FLIPER>> perfservice(PERFORMANCE) to E(%d) (timeout)\n", ret);
		}
		pp_index2++;
	}
	if(fliper_debug == 1)
		printk(KERN_CRIT"EMI:Rate:count1:count2:mode %6llu:%4d:%4d:%llu ns\n", emi_bw, pp_index1, pp_index2, t2-t1);
}
int fliper_set_bw(int bw)
{
	if(bw <= BW_THRESHOLD_MAX && bw >= BW_THRESHOLD_MIN){
		printk(KERN_CRIT"\n<<SOC DVFS FLIPER>> Set bdw threshold %d -> %d\n", bw_threshold, bw);
		bw_threshold = bw;
	}else{
		printk(KERN_CRIT"\n<<SOC DVFS FLIPER>> Set bdw threshold Error: %d (MAX:%d, MIN:%d)\n",bw, BW_THRESHOLD_MAX, BW_THRESHOLD_MIN );
	}
	return 0;
}
int fliper_restore_bw()
{
	printk(KERN_CRIT"\n<<SOC DVFS FLIPER>> Restore bdw threshold %d -> %d\n", bw_threshold, BW_THRESHOLD);
	bw_threshold = BW_THRESHOLD;
	return 0;
}
static void enable_fliper()
{
	printk(KERN_CRIT"fliper enable +++\n");
	mod_timer(&mt_pp_transfer_timer, jiffies + msecs_to_jiffies(X_ms));
}
static void disable_fliper()
{
	printk(KERN_CRIT"fliper disable ---\n");
	del_timer(&mt_pp_transfer_timer);
}
static void mt_power_pef_transfer()
{
	mod_timer(&mt_pp_transfer_timer, jiffies + msecs_to_jiffies(X_ms));
	schedule_work(&mt_pp_work);
}
#if 0
/*-------------FLIPER DEVICE/DRIVER--------------*/
static int fliper_probe(struct platform_device *dev)
{
	printk("[%s] enter...\n", __func__);
	return 0;
}

static int fliper_remove(struct platform_device *dev)
{
	printk("[%s] enter...\n", __func__);
	return 0;
}
static int fliper_pm_suspend(struct device *device)
{
	int ret;
	ret = vcorefs_request_dvfs_opp(KR_EMI_MON, OPPI_UNREQ);
	printk(KERN_EMERG"\n<<SOC DVFS FLIPER>> Suspend and flip to E(%d)\n", ret);

	return 0;
}
struct dev_pm_ops fliper_pm_ops = {
	.suspend = fliper_pm_suspend,
};

struct platform_device fliper_device = {
	.name = "fliper",
	.id = -1,
	.dev = {},
};

static struct platform_driver fliper_driver = {
	.driver = {
		.name = "fliper",
#ifdef CONFIG_PM
		.pm = &fliper_pm_ops
#endif
		.owner = THIS_MODULE,
	},
	.probe = fliper_probe,
	.remove = fliper_remove,
};
#endif
	static int
fliper_pm_callback(struct notifier_block *nb,
		unsigned long action, void *ptr)
{
	int ret;
	switch (action) {

		case PM_SUSPEND_PREPARE:
			ret = vcore_low();
			printk(KERN_CRIT"\n<<SOC DVFS FLIPER>> Suspend and flip to E(%d)\n", ret);
			pp_index1 = 0;
			pp_index2 = 0;
			disable_fliper();
		case PM_HIBERNATION_PREPARE:
			break;

		case PM_POST_SUSPEND:
			if(fliper_enabled == 1)
				enable_fliper();
			else
				printk(KERN_CRIT"\n<<SOC DVFS FLIPER>> Resume enable flipper but flipper is disabled\n");

		case PM_POST_HIBERNATION:
			break;

		default:
			return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}
static void fliper_early_suspend(struct early_suspend *h)
{
	int ret;
	ret = vcore_low();
	printk(KERN_EMERG"\n<<SOC DVFS FLIPER>> Early Suspend and flip to E(%d)\n", ret);
	pp_index1 = 0;
	pp_index2 = 0;
	disable_fliper();
}
static void fliper_late_resume(struct early_suspend *h)
{
	printk(KERN_EMERG"\n<<SOC DVFS FLIPER>> Late Resume\n" );
	enable_fliper();
}
static struct early_suspend fliper_early_suspend_handler =
{
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
	.suspend = fliper_early_suspend,
	.resume = fliper_late_resume,
};
/*-----------------------------------------------*/

#define TIME_5SEC_IN_MS 5000
static int __init init_fliper(void)
{
	struct proc_dir_entry *pe;
	pe = proc_create("fliper", 0644, NULL, &mt_fliper_fops);
	if (!pe)
		return -ENOMEM;
	bw_threshold = BW_THRESHOLD;
	printk("prepare mt pp transfer: jiffies:%lu-->%lu\n",jiffies, jiffies + msecs_to_jiffies(TIME_5SEC_IN_MS));
	printk("-  next jiffies:%lu >>> %lu\n",jiffies, jiffies + msecs_to_jiffies(X_ms));
	mod_timer(&mt_pp_transfer_timer, jiffies + msecs_to_jiffies(TIME_5SEC_IN_MS));
	fliper_enabled = 1;
#ifdef DISABLE_IN_EARLY_SUSPEND
	register_early_suspend(&fliper_early_suspend_handler);
#else
	pm_notifier(fliper_pm_callback, 0);
#endif

#if 0
	/*-------------FLIPER DEVICE/DRIVER--------------*/
	int ret;
	ret = platform_device_register(&fliper_device);
	if (ret) {
		printk("fliper_device register fail(%d)\n", ret);
		return ret;
	}

	ret = platform_driver_register(&fliper_driver);
	if (ret) {
		printk("fliper_driver register fail(%d)\n", ret);
		return ret;
	}
#endif

	return 0;
}
__initcall(init_fliper);

//MODULE_LICENSE("GPL");
//MODULE_AUTHOR("MTK");
//MODULE_DESCRIPTION("The fliper function");
