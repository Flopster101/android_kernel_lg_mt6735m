
 /****************************************************************************
 * Debugging Macros
 ****************************************************************************/
#define TPD_TAG                  "[LU2010] "
#define TPD_FUN(f)               printk(KERN_ERR TPD_TAG"%s\n", __FUNCTION__)
#define TPD_ERR(fmt, args...)    printk(KERN_ERR TPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define TPD_LOG(fmt, args...)    printk(KERN_ERR TPD_TAG fmt, ##args)
