/* BMA255 motion sensor driver
 *
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#ifndef __K2HH_H__
#define  __K2HH_H__

#include <linux/ioctl.h>

#define SAD0L            0x00
#define SAD0H            0x01
#define K2HH_ACC_I2C_SADROOT    0x0F
#define K2HH_ACC_I2C_SAD_L    ((K2HH_ACC_I2C_SADROOT<<1)|SAD0L)
#define K2HH_ACC_I2C_SAD_H    ((K2HH_ACC_I2C_SADROOT<<1)|SAD0H)
#define    K2HH_ACC_DEV_NAME    "K2HH_acc_misc"

/************************************************/
/*     Accelerometer defines section         */
/************************************************/

/* Accelerometer Sensor Full Scale */
#define K2HH_ACC_FS_MASK        0x30
#define K2HH_ACC_G_2G               0x00
#define K2HH_ACC_G_4G               0x10
#define K2HH_ACC_G_8G               0x20
#define K2HH_ACC_G_16G              0x30


/* Accelerometer Sensor Operating Mode */
#define K2HH_ACC_ENABLE        0x01
#define K2HH_ACC_DISABLE        0x00

#define K2HH_SUCCESS                                0
#define K2HH_ERR_I2C                                -1
#define K2HH_ERR_STATUS                         -3
#define K2HH_ERR_SETUP_FAILURE          -4
#define K2HH_ERR_GETGSENSORDATA     -5
#define K2HH_ERR_IDENTIFICATION         -6

#define K2HH_BUFSIZE                    256
#endif
