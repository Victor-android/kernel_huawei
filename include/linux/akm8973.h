/*
 * Definitions for akm8973 compass chip.
 */
#ifndef AKM8973_H
#define AKM8973_H

#include <linux/ioctl.h>

#define AKM8973_I2C_NAME "akm8973"

/* Compass device dependent definition */
#define AKECS_MODE_MEASURE	0x00	/* Starts measurement. Please use AKECS_MODE_MEASURE_SNG */
					/* or AKECS_MODE_MEASURE_SEQ instead of this. */
#define AKECS_MODE_E2P_READ	0x02	/* E2P access mode (read). */
#define AKECS_MODE_POWERDOWN	0x03	/* Power down mode */

#define RBUFF_SIZE		4	/* Rx buffer size */

/* huawei liujinggang 20101201 begin */
/* AK8973 register address */
#define AKECS_REG_ST			0xC0      /*Status Register*/
#define AKECS_REG_TMPS			0xC1      /*Temperature sensor data register*/
#define AKECS_REG_H1X			0xC2	/*Magnetic sensor data register*/
#define AKECS_REG_H1Y			0xC3       /*Magnetic sensor data register*/
#define AKECS_REG_H1Z			0xC4      /*Magnetic sensor data register*/

#define AKECS_REG_MS1			0xE0    /*Mode setting  register*/
#define AKECS_REG_HXDA			0xE1    /*Magnetic sensor X-axis DAC setting register*/
#define AKECS_REG_HYDA			0xE2    /*Magnetic sensor Y-axis DAC setting register*/
#define AKECS_REG_HZDA			0xE3    /*Magnetic sensor Z-axis DAC setting register*/
#define AKECS_REG_HXGA			0xE4   /*Magnetic sensor X-axis DAC gain register*/
#define AKECS_REG_HYGA			0xE5   /*Magnetic sensor Y-axis DAC gain register*/
#define AKECS_REG_HZGA			0xE6   /*Magnetic sensor Z-axis DAC gain register*/
#define AKECS_REG_EHXGA			0x66   /*Magnetic sensor X-axis DAC E2PROME gain register*/
#define AKECS_REG_EHYGA			0x67   /*Magnetic sensor Y-axis DAC E2PROME gain register*/
#define AKECS_REG_EHZGA			0x68   /*Magnetic sensor Z-axis DACE2PROME  gain register*/
/*BK4D03309, update firmware for compass, dingxifeng dKF14049,2009-7-30 start*/ 

#define  AKECS_REG_ETS				0x62	/* Offset adjustment for temperature sensor*/
#define  AKECS_REG_EVIR				0x63	/* VREF &IREF adjustment value*/
#define  AKECS_REG_EIHE				0x64	/*HE drive power supply correction value*/
#define  AKECS_REG_ETST				0x65	/*for testing the value is 0xC7*/

/*BK4D03309, update firmware for compass, dingxifeng dKF14049,2009-7-30 end*/ 
#define AKMIO				0xA1

/* IOCTLs for AKM library */
#define ECS_IOCTL_INIT                  _IO(AKMIO, 0x01)
#define ECS_IOCTL_WRITE                 _IOW(AKMIO, 0x02, char[5])
#define ECS_IOCTL_READ                  _IOWR(AKMIO, 0x03, char[5])
#define ECS_IOCTL_RESET      	        _IO(AKMIO, 0x04)
#define ECS_IOCTL_SET_MODE              _IOW(AKMIO, 0x07, short)
#define ECS_IOCTL_GETDATA               _IOR(AKMIO, 0x08, char[RBUFF_SIZE+1])
#define ECS_IOCTL_GET_NUMFRQ            _IOR(AKMIO, 0x09, char[2])
#define ECS_IOCTL_SET_YPR               _IOW(AKMIO, 0x0C, short[12])
#define ECS_IOCTL_GET_OPEN_STATUS       _IOR(AKMIO, 0x0D, int)
#define ECS_IOCTL_GET_CLOSE_STATUS      _IOR(AKMIO, 0x0E, int)
#define ECS_IOCTL_GET_DELAY             _IOR(AKMIO, 0x30, short)
#define ECS_IOCTL_GET_PROJECT_NAME      _IOR(AKMIO, 0x0D, char[64])
#define ECS_IOCTL_GET_MATRIX            _IOR(AKMIO, 0x0E, short [4][3][3])

/* IOCTLs for APPs */
#define ECS_IOCTL_APP_SET_MODE		_IOW(AKMIO, 0x10, short)
#define ECS_IOCTL_APP_SET_MFLAG		_IOW(AKMIO, 0x11, short)
#define ECS_IOCTL_APP_GET_MFLAG		_IOW(AKMIO, 0x12, short)
#define ECS_IOCTL_APP_SET_AFLAG		_IOW(AKMIO, 0x13, short)
#define ECS_IOCTL_APP_GET_AFLAG		_IOR(AKMIO, 0x14, short)
#define ECS_IOCTL_APP_SET_TFLAG		_IOR(AKMIO, 0x15, short)
#define ECS_IOCTL_APP_GET_TFLAG		_IOR(AKMIO, 0x16, short)
#define ECS_IOCTL_APP_RESET_PEDOMETER   _IO(AKMIO, 0x17)
#define ECS_IOCTL_APP_SET_DELAY		_IOW(AKMIO, 0x18, short)
#define ECS_IOCTL_APP_GET_DELAY		ECS_IOCTL_GET_DELAY
#define ECS_IOCTL_APP_SET_MVFLAG	_IOW(AKMIO, 0x19, short)	/* Set raw magnetic vector flag */
#define ECS_IOCTL_APP_GET_MVFLAG	_IOR(AKMIO, 0x1A, short)	/* Get raw magnetic vector flag */
/* huawei liujinggang 20101201 end */

/* < BU5D05131 gaohuajiang 20100318 begin */
#define ECS_IOCTL_APP_GET_LFLAG		_IOR(AKMIO, 0x1B, short)
#define ECS_IOCTL_APP_SET_LFLAG		_IOW(AKMIO, 0x1C, short)
#define ECS_IOCTL_APP_GET_PFLAG		_IOR(AKMIO, 0x1D, short)
#define ECS_IOCTL_APP_SET_PFLAG		_IOW(AKMIO, 0x1E, short)
/* BU5D05131 gaohuajiang 20100318 end > */
/* < DTS2011042703449  liujinggang 20110427 begin */
#define ECS_IOCTL_APP_GET_DEVID 	_IOR(AKMIO, 0x1F, char[20])
/* DTS2011042703449  liujinggang 20110427 end > */
struct akm8973_platform_data {
	short layouts[4][3][3];
	char project_name[64];
	int reset;
	int intr;
};

#endif

