/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 Rockchip Electronics Co., Ltd.
 */
#ifndef __IAM20680_H
#define __IAM20680_H

#include <linux/ioctl.h>
/**add***/
#define IAM20680_PRECISION		16
#define IAM20680_RANGE			2000000

#define IAM20680_SMPLRT_DIV		0x19
#define IAM20680_CONFIG			0x1A
#define IAM20680_GYRO_CONFIG		0x1B
#define IAM20680_ACCEL_CONFIG	0x1C
#define IAM20680_ACCEL_CONFIG2	0x1D
#define IAM20680_LP_ACCEL_ODR   0x1E
#define IAM20680_WOM_THRESH	0x1F
#define IAM20680_FIFO_EN			0x23
#define IAM20680_INT_PIN_CFG		0x37
#define IAM20680_INT_ENABLE		0x38
#define IAM20680_DMP_INT_STATUS	0x39
#define IAM20680_INT_STATUS		0x3A
#define IAM20680_ACCEL_XOUT_H	0x3B
#define IAM20680_TEMP_OUT_H	0x41
#define IAM20680_GYRO_XOUT_H	0x43
#define IAM20680_ACCEL_INTEL_CTRL 0x69
#define IAM20680_USER_CTRL		0x6A
#define IAM20680_PWR_MGMT_1		0x6B
#define IAM20680_PWR_MGMT_2		0x6C
#define IAM20680_PRGM_STRT_ADDRH     0x70
#define IAM20680_FIFO_COUNTH		0x72
#define IAM20680_FIFO_R_W		0x74
#define IAM20680_WHOAMI			0x75

#define IAM20680_DEVICE_ID		0xA9
/*
 * IAM20680_CONFIG
 */
#define DLPF_CFG_250HZ		0x00
#define DLPF_CFG_184HZ		0x01
#define DLPF_CFG_98HZ		0x02
#define DLPF_CFG_41HZ		0x03
#define DLPF_CFG_20HZ		0x04
#define DLPF_CFG_10HZ		0x05
#define DLPF_CFG_5HZ		0x06
#define DLPF_CFG_3600HZ		0x07
#define EXT_SYNC_SET_TEMP	0x08
#define EXT_SYNC_SET_GYRO_X	0x10
#define EXT_SYNC_SET_GYRO_Y	0x18
#define EXT_SYNC_SET_GYRO_Z	0x20
#define EXT_SYNC_SET_ACCEL_X	0x28
#define EXT_SYNC_SET_ACCEL_Y	0x30
#define EXT_SYNC_SET_ACCEL_Z	0x38


/*
 * IAM20680_GYRO_CONFIG
 */
#define GFSR_250DPS			(0<<3)
#define GFSR_500DPS			(1<<3)
#define GFSR_1000DPS		(2<<3)
#define GFSR_2000DPS		(3<<3)

/*
 * IAM20680_ACCEL_CONFIG
 */
#define AFSR_2G		(0<<3)
#define AFSR_4G		(1<<3)
#define AFSR_8G		(2<<3)
#define AFSR_16G	(3<<3)


/*
 * IAM20680_ACCEL_CONFIG2
 */
#define A_DLPF_CFG_460HZ		0x00
#define A_DLPF_CFG_184HZ		0x01
#define A_DLPF_CFG_92HZ		0x02
#define A_DLPF_CFG_41HZ		0x03
#define A_DLPF_CFG_20HZ		0x04
#define A_DLPF_CFG_10HZ		0x05
#define A_DLPF_CFG_5HZ			0x06
//#define A_DLPF_CFG_460HZ      0x07
#define BIT_FIFO_SIZE_1K                 0x40
#define BIT_ACCEL_FCHOICE_B		0x08


/*
 * IAM20680_LP_ACCEL_ODR
 */
#define LPA_CLK_P24HZ	0x0
#define LPA_CLK_P49HZ	0x1
#define LPA_CLK_P98HZ	0x2
#define LPA_CLK_1P95HZ	0x3
#define LPA_CLK_3P91HZ	0x4
#define LPA_CLK_7P81HZ	0x5
#define LPA_CLK_15P63HZ	0x6
#define LPA_CLK_31P25HZ	0x7
#define LPA_CLK_62P50HZ	0x8
#define LPA_CLK_125HZ	0x9
#define LPA_CLK_250HZ	0xa
#define LPA_CLK_500HZ	0xb


/*
 * IAM20680_PWR_MGMT_1
 */
#define BIT_H_RESET			(1<<7)
#define BIT_SLEEP			(1<<6)
#define BIT_CYCLE			(1<<5)
#define BIT_GYRO_STANDBY	(1<<4)
#define BIT_PD_PTAT			(1<<3)
#define BIT_CLKSEL			(1<<0)

#define CLKSEL_INTERNAL		0
#define CLKSEL_PLL			1

/*
 * IAM20680_PWR_MGMT_2
 */
#define BIT_ACCEL_STBY              0x38
#define BIT_GYRO_STBY               0x07
#define BITS_LPA_WAKE_CTRL 0xC0
#define BITS_LPA_WAKE_1HZ 0x00
#define BITS_LPA_WAKE_2HZ 0x40
#define BITS_LPA_WAKE_20HZ 0x80

#define IAM20680_PWRM1_SLEEP				0x40
#define IAM20680_PWRM1_GYRO_STANDBY		0x10
#define IAM20680_PWRM2_ACCEL_DISABLE		0x38
#define IAM20680_PWRM2_GYRO_DISABLE		0x07

/*
 * IAM20680_ACCEL_INTEL_CTRL
 */
#define BIT_ACCEL_INTEL_EN	0x80
#define BIT_ACCEL_INTEL_MODE	0x40


/*
 * IAM20680_USER_CTRL
 */
#define BIT_FIFO_RST                    0x04
#define BIT_DMP_RST                     0x08
#define BIT_I2C_MST_EN                  0x20
#define BIT_FIFO_EN                     0x40
#define BIT_DMP_EN                      0x80


/*
 * IAM20680_FIFO_EN
 */
#define BIT_ACCEL_OUT           0x08
#define BITS_GYRO_OUT           0x70


/*
 * IAM20680_INT_PIN_CFG
 */
#define BIT_BYPASS_EN           0x2

/*
 * IAM20680_INT_EN/INT_STATUS
 */
#define BIT_FIFO_OVERLOW	0x80
#define BIT_MOT_INT				0x40
#define BIT_MPU_RDY                 0x04
#define BIT_DMP_INT                 0x02
#define BIT_RAW_RDY                 0x01


#define DMP_START_ADDR           0x400



#define AXIS_NUM 3
#define AXIS_ADC_BYTE 2
#define SENSOR_PACKET (AXIS_NUM * AXIS_ADC_BYTE)





/*
 * self-test parameter
 */

#define DEF_ST_PRECISION            1000
#define DEF_ST_IAM20680_ACCEL_LPF        2
#define DEF_STABLE_TIME_ST 50
#define DEF_SELFTEST_GYRO_FS            (0 << 3)
#define DEF_SELFTEST_ACCEL_FS           (2 << 3)
#define DEF_SELFTEST_6500_ACCEL_FS      (0 << 3)
#define DEF_SW_SELFTEST_GYRO_FS	GFSR_2000DPS
#define DEF_SW_SELFTEST_SENSITIVITY		((2000*DEF_ST_PRECISION)/32768)

#define DEF_SW_SELFTEST_SAMPLE_COUNT 75
#define DEF_SW_SELFTEST_SAMPLE_TIME 75
#define DEF_SW_ACCEL_CAL_SAMPLE_TIME 50
#define DEF_SW_SKIP_COUNT 10

#define DEF_ST_6500_STABLE_TIME         20
#define BYTES_PER_SENSOR        (6)
#define DEF_SELFTEST_SAMPLE_RATE             0
#define DEF_GYRO_WAIT_TIME          50
#define THREE_AXIS              (3)
#define INIT_ST_SAMPLES          200
#define FIFO_COUNT_BYTE         (2)
#define DEF_ST_TRY_TIMES            2
#define REG_6500_XG_ST_DATA     0x0
#define REG_6500_XA_ST_DATA     0xD
#define BITS_SELF_TEST_EN		0xE0

#define DEF_ST_SCALE                    (1L << 15)

/*---- IAM20680 Self Test Pass/Fail Criteria ----*/
/* Gyro Offset Max Value (dps) */
#define DEF_GYRO_OFFSET_MAX             20
/* Gyro Self Test Absolute Limits ST_AL (dps) */
#define DEF_GYRO_ST_AL                  60
/* Accel Self Test Absolute Limits ST_AL (mg) */
#define DEF_ACCEL_ST_AL_MIN             225
#define DEF_ACCEL_ST_AL_MAX             675
#define DEF_6500_ACCEL_ST_SHIFT_DELTA   500
#define DEF_6500_GYRO_CT_SHIFT_DELTA    500
#define DEF_ST_IAM20680_ACCEL_LPF        2
#define DEF_ST_6500_ACCEL_FS_MG         2000UL
#define DEF_SELFTEST_6500_ACCEL_FS      (0 << 3)

#define DEF_SELFTEST_GYRO_SENS          (32768 / 250)


#define  GSENSOR_DEV_PATH    "/dev/mma8452_daemon"

#endif

