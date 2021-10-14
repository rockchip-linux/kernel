/* SPDX-License-Identifier: GPL-2.0 */
#ifndef ICM40605_H_
#define ICM40605_H_

#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/mutex.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/regmap.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
extern const struct regmap_config icm40605_regmap_config;
extern const struct dev_pm_ops inv_icm42600_pm_ops;
#define ICM40605_POWER_UP_TIME 100

/** BANK0 */
#define MPUREG_CHIP_CONFIG_REG 0x11
	#define BIT_SPI_MODE 0x10
	#define BIT_SOFT_RESET_CHIP_CONFIG 0x01

#define MPUREG_DRIVE_CONFIG_REG 0x13
	#define BIT_PADS_SLEW_TRIM_D2A 0x07
	#define BIT_SPI_SPEED_5M 0x03
	#define BIT_SPI_SPEED_17M 0x05

#define MPUREG_INT_CONFIG_REG 0x14
	#define BIT_INT2_MODE 0x20
	#define BIT_INT2_DRIVE_CIRCUIT 0x10
	#define BIT_INT2_POLARITY 0x08
	#define BIT_INT1_MODE 0x04
	#define BIT_INT1_DRIVE_CIRCUIT 0x02
	#define BIT_INT1_POLARITY 0x01
	#define BIT_ONLY_INT1_ACTIVE_HIGH 0x1B
	#define BIT_ONLY_INT1_ACTIVE_LOW 0x1A

#define MPUREG_FIFO_CONFIG_REG 0x16
	#define BIT_FIFO_MODE_CTRL_MASK ((0x03)<<6)
	#define BIT_FIFO_MODE_CTRL_BYPASS ((0x00)<<6)
	#define BIT_FIFO_MODE_CTRL_STREAM ((0x01)<<6)
	#define BIT_FIFO_MODE_CTRL_SNAPSHOT ((0x02)<<6)

#define MPUREG_TEMP_DATA0_UI 0x1D

#define MPUREG_ACCEL_DATA_X0_UI 0x1F

#define MPUREG_GYRO_DATA_X0_UI 0x25

#define MPUREG_TMST_FSYNC1 0x2B

#define MPUREG_INT_STATUS 0x2D
	#define BIT_STATUS_UI_FSYNC 0x40
	#define BIT_STATUS_PLL_RDY 0x20
	#define BIT_STATUS_RESET_DONE 0x10
	#define BIT_STATUS_DRDY 0x08
	#define BIT_STATUS_FIFO_THS 0x04
	#define BIT_STATUS_FIFO_FULL 0x02
	#define BIT_STATUS_AGC_RDY 0x01

#define MPUREG_FIFO_BYTE_COUNT1_REG 0x2E
#define MPUREG_FIFO_BYTE_COUNT2_REG 0x2F
#define MPUREG_FIFO_DATA_REG 0x30

#define MPUREG_SIGNAL_PATH_RESET_REG 0x4B
	#define BIT_ABORT_AND_RESET 0x08
	#define BIT_TMST_STROBE 0x04
	#define BIT_FIFO_FLUSH 0x02
	#define BIT_TEMP_RST 0x01

#define MPUREG_INTF_CONFIG0_REG 0x4C
	#define BIT_FIFO_SREG_INVALID_IND_DIS 0x80
	#define BIT_FIFO_COUNT_REC 0x40
	#define BIT_FIFO_COUNT_ENDIAN 0x20
	#define BIT_SENSOR_DATA_ENDIAN 0x10
	#define BIT_SPI_MODE_OIS2 0x08
	#define BIT_SPI_MODE_OIS1 0x04
	#define BIT_SPI_I2C_SEL_MASK (0x03) // follow 3 not defined in ref driver, but in datasheet
	#define BIT_SEL_SPI_DISABLE 0x02
	#define BIT_SEL_I2C_DISABLE 0x03

#define MPUREG_INTF_CONFIG1_REG 0x4D
	#define BITS_GYRO_AFSR_MODE_MASK (0xC0)
	#define BITS_ACCEL_AFSR_MODE_MASK (0x30)
	#define BITS_ACCEL_LP_CLK_SEL 0x08
	#define BITS_RTC_MODE 0x04
	#define BITS_CLKSEL_MASK (0x03)

#define MPUREG_PWR_MGMT_0_REG 0x4E
	#define BIT_TEMP_DIS 0x20
	#define BIT_IDLE 0x10
	#define BIT_GYRO_MODE_MASK ((0x03)<<2)
		#define BIT_GYRO_MODE_OFF ((0x00)<<2)
		#define BIT_GYRO_MODE_STANDBY ((0x01)<<2)
		#define BIT_GYRO_MODE_LP ((0x02)<<2)
		#define BIT_GYRO_MODE_LN ((0x03)<<2)
	#define BIT_ACCEL_MODE_MASK ((0x03)<<0)
		#define BIT_ACCEL_MODE_OFF 0x00
		#define BIT_ACCEL_MODE_LP 0x02
		#define BIT_ACCEL_MODE_LN 0x03

#define SET_LPM 0
#define SET_LNM 1

#define MPUREG_GYRO_CONFIG0_REG 0x4F
	#define BIT_GYRO_UI_FS_SEL_SHIFT 5
	#define BIT_GYRO_UI_FS_SEL_MASK ((0x07)<<BIT_GYRO_UI_FS_SEL_SHIFT)
	#define ICM40605_GYRO_FSR_2000DPS	    ((0x00)<<BIT_GYRO_UI_FS_SEL_SHIFT)
	#define ICM40605_GYRO_FSR_1000DPS	    ((0x01)<<BIT_GYRO_UI_FS_SEL_SHIFT)
	#define ICM40605_GYRO_FSR_500DPS	    ((0x02)<<BIT_GYRO_UI_FS_SEL_SHIFT)
	#define ICM40605_GYRO_FSR_250DPS	    ((0x03)<<BIT_GYRO_UI_FS_SEL_SHIFT)
	#define ICM40605_GYRO_FSR_125DPS	    ((0x04)<<BIT_GYRO_UI_FS_SEL_SHIFT)
	#define ICM40605_GYRO_FSR_62_5DPS	    ((0x05)<<BIT_GYRO_UI_FS_SEL_SHIFT)
	#define ICM40605_GYRO_FSR_31_25DPS	    ((0x06)<<BIT_GYRO_UI_FS_SEL_SHIFT)
	#define ICM40605_GYRO_FSR_15_625DPS	    ((0x07)<<BIT_GYRO_UI_FS_SEL_SHIFT)
	#define BIT_GYRO_ODR_NONFLAME_MASK ((0x0F)<<0)
	#define ICM40605_GYRO_ODR_8KHZ     0x3
	#define ICM40605_GYRO_ODR_4KHZ     0x4
	#define ICM40605_GYRO_ODR_2KHZ     0x5
	#define ICM40605_GYRO_ODR_1KHZ     0x6
	#define ICM40605_GYRO_ODR_200HZ    0x7
	#define ICM40605_GYRO_ODR_100HZ    0x8
	#define ICM40605_GYRO_ODR_50HZ     0x9
	#define ICM40605_GYRO_ODR_25HZ     0xA

#define MPUREG_ACCEL_CONFIG0_REG 0x50
	#define BIT_ACCEL_UI_FS_SEL_SHIFT 5
	#define BIT_ACCEL_UI_FS_SEL_MASK ((0x07)<<BIT_ACCEL_UI_FS_SEL_SHIFT)
	#define ICM40605_ACCEL_FSR_2G	 ((0x03)<<BIT_ACCEL_UI_FS_SEL_SHIFT)
	#define ICM40605_ACCEL_FSR_4G	 ((0x02)<<BIT_ACCEL_UI_FS_SEL_SHIFT)
	#define ICM40605_ACCEL_FSR_8G  ((0x01)<<BIT_ACCEL_UI_FS_SEL_SHIFT)
	#define ICM40605_ACCEL_FSR_16G ((0x00)<<BIT_ACCEL_UI_FS_SEL_SHIFT)
	#define BIT_ACCEL_ODR_NONFLAME_MASK ((0x0F)<<0)
	#define ICM40605_ACCEL_ODR_8KHZ     0x3
	#define ICM40605_ACCEL_ODR_4KHZ     0x4
	#define ICM40605_ACCEL_ODR_2KHZ     0x5
	#define ICM40605_ACCEL_ODR_1KHZ     0x6
	#define ICM40605_ACCEL_ODR_200HZ    0x7
	#define ICM40605_ACCEL_ODR_100HZ    0x8
	#define ICM40605_ACCEL_ODR_50HZ     0x9
	#define ICM40605_ACCEL_ODR_25HZ     0xA

#define MPUREG_GYRO_CONFIG1_REG 0x51
	#define BIT_TEMP_FILT_BW_SHIFT 5
	#define BIT_TEMP_FILT_BW_MASK ((0x07)<<BIT_TEMP_FILT_BW_SHIFT)
	#define BIT_GYRO_AVG_FILT_RATE 0x10
	#define BIT_GYRO_UI_FILT_ORD_IND_SHIFT 2
	#define BIT_GYRO_UI_FILT_ORD_IND_MASK ((0x03)<<BIT_GYRO_UI_FILT_ORD_IND_SHIFT)
	#define BIT_GYRO_DEC2_M2_ORD_MASK (0x03)

#define MPUREG_ACCEL_GYRO_CONFIG0_REG 0x52
	#define BIT_ACCEL_UI_FILT_BW_IND_SHIFT 4
	#define BIT_ACCEL_UI_FILT_BW_IND_MASK ((0x0F)<<BIT_ACCEL_UI_FILT_BW_IND_SHIFT)
	#define BIT_GYRO_UI_FILT_BW_IND_MASK (0x0F)

#define MPUREG_ACCEL_CONFIG1_REG 0x53
	#define BIT_ACCEL_AVG_FILT_RATE 0x01
	#define BIT_ACCEL_UI_FILT_ORD_IND_SHIFT 3
	#define BIT_ACCEL_UI_FILT_ORD_IND_MASK ((0x03)<<BIT_ACCEL_UI_FILT_ORD_IND_SHIFT)
	#define BIT_ACCEL_DEC2_M2_ORD_MASK ((0x03)<<1)

#define MPUREG_ACCEL_WOM_X_THR_REG 0x54
#define MPUREG_ACCEL_WOM_Y_THR_REG 0x55
#define MPUREG_ACCEL_WOM_Z_THR_REG 0x56

#define MPUREG_SMD_CONFIG_REG 0x57
	#define BIT_WOM_INT_MODE_AND  ((0x01)<<3)
	#define BIT_WOM_MODE_CMP_PREV ((0x01)<<2)
	#define BIT_SMD_MODE_SMD_LONG  0x03
	#define BIT_SMD_MODE_SMD_SHORT 0x02
	#define BIT_SMD_MODE_WOM       0x01
	#define BIT_SMD_MODE_DISABLE   0x00

#define MPUREG_INT_RAW_REG 0x58

#define MPUREG_INT_STATUS2_REG 0x59
    #define BIT_SMD_INT        0x08
    #define BIT_WOM_Z_INT      0x04
    #define BIT_WOM_Y_INT      0x02
    #define BIT_WOM_X_INT      0x01

#define MPUREG_TMST_CONFIG_REG 0x5A
	#define BIT_FIFO_RAM_ISO_ENA 0x40
	#define BIT_EN_DREG_FIFO_D2A 0x20
	#define BIT_TMST_TO_REGS_EN 0x10
	#define BIT_TMST_RESOL 0x08
	#define BIT_TMST_DELTA_EN 0x04
	#define BIT_TMST_FSYNC_EN 0x02
	#define BIT_TMST_EN 0x01

#define MPUREG_FIFO_CONFIG1_REG 0x5F
	#define BIT_FIFO_RESUME_PARTIAL_RD 0x40
	#define BIT_FIFO_WM_GT_TH 0x20
	#define BIT_FIFO_HIRES_EN 0x10
	#define BIT_FIFO_TMST_FSYNC_EN 0x08
	#define BIT_FIFO_TEMP_EN 0x04
	#define BIT_FIFO_GYRO_EN 0x02
	#define BIT_FIFO_ACCEL_EN 0x01

#define MPUREG_FIFO_CONFIG2_REG 0x60
	#define BIT_FIFO_WM5 0x10
	#define INT_FIFO_WM5_NUM 16

#define MPUREG_FSYNC_CONFIG_REG 0x62
	#define BIT_FSYNC_UI_SEL_MASK ((0x07)<<4)
		#define BIT_FSYNC_UI_SEL_TAG_TEMP ((0x01)<<4)
	#define BIT_FSYNC_UI_FLAG_CLEAR_SEL 0x02

#define MPUREG_INT_CONFIG0_REG 0x63
#define MPUREG_INT_CONFIG1_REG 0x64
	#define BIT_INT_ASY_RST_DISABLE 0x10

#define MPUREG_INT_SOURCE0_REG 0x65
	#define BIT_INT_UI_FSYNC_INT1_EN 0x40
	#define BIT_INT_PLL_RDY_INT1_EN 0x20
	#define BIT_INT_RESET_DONE_INT1_EN 0x10
	#define BIT_INT_UI_DRDY_INT1_EN 0x08
	#define BIT_INT_FIFO_THS_INT1_EN 0x04
	#define BIT_INT_FIFO_FULL_INT1_EN 0x02
	#define BIT_INT_UI_AGC_RDY_INT1_EN 0x01

#define MPUREG_INT_SOURCE1_REG 0x66
	#define BIT_INT_SMD_INT1_EN   0x08
	#define BIT_INT_WOM_Z_INT1_EN 0x04
	#define BIT_INT_WOM_Y_INT1_EN 0x02
	#define BIT_INT_WOM_X_INT1_EN 0x01

#define MPUREG_INT_SOURCE2_REG 0x67
#define MPUREG_INT_SOURCE3_REG 0x68
	#define BIT_INT_UI_FSYNC_INT2_EN 0x40
	#define BIT_INT_PLL_RDY_INT2_EN 0x20
	#define BIT_INT_RESET_DONE_INT2_EN 0x10
	#define BIT_INT_UI_DRDY_INT2_EN 0x08
	#define BIT_INT_FIFO_THS_INT2_EN 0x04
	#define BIT_INT_FIFO_FULL_INT2_EN 0x02
	#define BIT_INT_UI_AGC_RDY_INT2_EN 0x01
#define MPUREG_INT_SOURCE4_REG 0x69
#define MPUREG_INT_SOURCE5_REG 0x6A

#define MPUREG_SENSOR_SELFTEST_REG 0x6B
	#define BIT_ACCEL_ST_RESULT 0x08
	#define BIT_GYRO_ST_RESULT 0x04
	#define BIT_ACCEL_ST_STATUS 0x02
	#define BIT_GYRO_ST_STATUS 0x01

#define MPUREG_FIFO_LOST_PKT0_REG 0x6C

#define MPUREG_AFSR_CONFIG0_REG 0x6E
#define MPUREG_AFSR_CONFIG1_REG 0x6F

#define MPUREG_SELF_TEST_CONFIG_REG 0x70
	#define BIT_ST_REGULATOR_EN 0x40
	#define BIT_ACCEL_Z_ST_EN 0x20
	#define BIT_ACCEL_Y_ST_EN 0x10
	#define BIT_ACCEL_X_ST_EN 0x08
	#define BIT_GYRO_Z_ST_EN 0x04
	#define BIT_GYRO_Y_ST_EN 0x02
	#define BIT_GYRO_X_ST_EN 0x01

#define MPUREG_SCAN0_REG 0x71

#define MPUREG_MEM_BANK_SEL 0x72
#define MPUREG_MEM_START_ADDR 0x73
#define MPUREG_FIFO_R_W 0x74

#define MPUREG_WHO_AM_I 0x75
	#define BIT_I_AM_ICM40605 0x33

#define MPUREG_REG_BANK_SEL 0x76

#define MPUREG_GOS_USER_0_REG 0x77

/** BANK1 */
#define MPUREG_SENSOR_CONFIG1_B1_REG 0x04
	#define BIT_PAD_SCENARIO_MASK ((0x0F)<<4)
		#define BIT_PAD_SCENARIO_4 ((0x04)<<4)
		#define BIT_PAD_SCENARIO_10 ((0x0A)<<4)
#define MPUREG_SENSOR_CONFIG2_B1_REG 0x05
	#define BIT_OIS_MODE_MASK ((0x03)<<4)
		#define BIT_OIS_MODE_OFF ((0x00)<<4)
		#define BIT_OIS_MODE_8k ((0x01)<<4)
		#define BIT_OIS_MODE_32k ((0x02)<<4)
		#define BIT_OIS_MODE_64k ((0x03)<<4)
	#define BIT_GYRO_4000DPS_FS_MASK ((0x01)<<1)
	#define BIT_ACCEL_32G_FS_MASK ((0x01)<<0)
#define MPUREG_GYRO_CONFIG_STATIC0_B1_REG 0x09
#define MPUREG_GYRO_CONFIG_STATIC1_B1_REG 0x0A
#define MPUREG_INTF_CONFIG4_B1_REG 0x7A

/** BANK2 */
#define MPUREG_ACCEL_CONFIG_STATIC0_B2_REG 0x39
#define MPUREG_ACCEL_CONFIG_STATIC1_B2_REG 0x3A
#define MPUREG_OIS1_CONFIG1_REG 0x44
	#define BIT_OIS1_MASK   (0x07<<2)
		#define BIT_OIS1_DEC_1  (0x00<<2)
		#define BIT_OIS1_DEC_2  (0x01<<2)
		#define BIT_OIS1_DEC_4  (0x02<<2)
		#define BIT_OIS1_DEC_8  (0x03<<2)
		#define BIT_OIS1_DEC_16 (0x04<<2)
		#define BIT_OIS1_DEC_32 (0x05<<2)
	#define BIT_GYRO_OIS1_EN 0x02
	#define BIT_FSYNC_OIS_SEL_TAG_FSYNC_GYRO_XOUT ((0x02)<<5)
#define MPUREG_OIS1_CONFIG2_REG 0x45
	#define BIT_GYRO_OIS_FS_SEL_MASK ((0x03)<<3)
		#define BIT_GYRO_OIS_FS_SEL_2000DPS ((0x00)<<3)
		#define BIT_GYRO_OIS_FS_SEL_1000DPS ((0x01)<<3)
		#define BIT_GYRO_OIS_FS_SEL_500DPS  ((0x02)<<3)
		#define BIT_GYRO_OIS_FS_SEL_250DPS  ((0x03)<<3)
		#define BIT_GYRO_OIS_FS_SEL_125DPS  ((0x04)<<3)
		#define BIT_GYRO_OIS_FS_SEL_62_5DPS ((0x05)<<3)
		#define BIT_GYRO_OIS_FS_SEL_31_25DPS ((0x06)<<3)
		#define BIT_GYRO_OIS_FS_SEL_15_6DPS ((0x07)<<3)
#define MPUREG_GYRO_DATA_X0_OIS1_B2_REG 0x4F
/* Only accessible from AUX1 */
#define MPUREG_INT_STATUS_OIS1_B2_REG 0x57
	#define BIT_STATUS_OIS_DRDY 0x02
/* End of Only accessible from AUX1 */
#define MPUREG_OIS2_CONFIG1_REG 0x59
	#define BIT_GYRO_OIS2_EN 0x02
#define MPUREG_GYRO_DATA_X0_OIS2_B2_REG 0x64

/** BANK3 */
#define MPUREG_AMP_GX_TRIM1_B3_REG 0x31
#define MPUREG_AMP_GX_TRIM2_B3_REG 0x32
#define MPUREG_AMP_GY_TRIM1_B3_REG 0x36
#define MPUREG_AMP_GY_TRIM2_B3_REG 0x37
#define MPUREG_AMP_GZ_TRIM1_B3_REG 0x3B
#define MPUREG_AMP_GZ_TRIM2_B3_REG 0x3C
#define MPUREG_ACCEL_XY_TRIM4_B3_REG 0x47
#define MPUREG_ACCEL_X_TRIM3_B3_REG 0x4B
#define MPUREG_ACCEL_Y_TRIM3_B3_REG 0x4F
#define MPUREG_ACCEL_Z_TRIM1_B3_REG 0x51
#define MPUREG_ACCEL_Z_TRIM5_B3_REG 0x55

/** BANK4 */
#define MUPREG_DRV_GYR_CFG0_REG 0x10
	#define GYRO_DRV_TEST_FSMFORCE_D2A_LINEAR_START_MODE         0x0D
	#define GYRO_DRV_TEST_FSMFORCE_D2A_STEADY_STATE_AGC_REG_MODE 0x2A
#define MUPREG_DRV_GYR_CFG1_REG 0x11
#define MUPREG_DRV_GYR_CFG2_REG 0x12
	#define GYRO_DRV_SPARE2_D2A_EN 0x1

/** FIFO CONTENT DEFINITION */
#define HEADER_SIZE        1
#define ACCEL_DATA_SIZE    6
#define GYRO_DATA_SIZE     6
#define TEMP_DATA_SIZE     1
#define TS_FSYNC_SIZE      2

enum ICM406xx_fio_format {
	FIFO_20_BYTE,
	FIFO_ACCEL_ONLY,
	FIFO_GYRO_ONLY,
	FIFO_16_BYTE,
};
#define FIFO_ACCEL_EN           0x40
#define FIFO_GYRO_EN            0x20
#define FIFO_TS_MASK            0x0C
#define FIFO_FSYNC_BITS         0x0C
#define HAVANA_MAX_PACKET_SIZE      20
#define ICM40605_FIFO_COUNT_LIMIT         60

// BANK SEL
enum icm40605_bank_index {
	ICM40605_BANK0 = 0x00,
	ICM40605_BANK1 = 0x01,
	ICM40605_BANK2 = 0x10,
	ICM40605_BANK3 = 0x11,
	ICM40605_BANK4 = 0x100,
};

// ODR
enum icm40605_odr_index {
	ICM40605_ODR_RESERVED0 = 0,
	ICM40605_ODR_RESERVED1,
	ICM40605_ODR_RESERVED2,
	ICM40605_ODR_8KHZ,
	ICM40605_ODR_4KHZ,
	ICM40605_ODR_2KHZ,
	ICM40605_ODR_1KHZ,
	ICM40605_ODR_200HZ,
	ICM40605_ODR_100HZ,
	ICM40605_ODR_50HZ,
	ICM40605_ODR_25HZ,
	ICM40605_NUM_ODRS, /* must be last */
};

struct icm40605_chip_config {
	unsigned int fsr:2;
	unsigned int lpf:3;
	unsigned int accl_fs:2;
	int gyro_odr:10;
	int accel_odr:10;
	unsigned int accl_fifo_enable:1;
	unsigned int gyro_fifo_enable:1;
	unsigned int temp_fifo_enable:1;
	unsigned int time_fifo_enable:1;
	u8 divider;
	u8 user_ctrl;
};

struct icm40605_data {
	struct mutex lock;
	struct regmap *regmap;
	struct iio_trigger  *trig;
	struct device_node	*node;
	int int1_gpio;
	struct regulator *vdd_supply;
	struct regulator *vddio_supply;
	u16  accel_frequency;
	u16  gyro_frequency;
	u16  accel_frequency_buff;
	u16  gyro_frequency_buff;
	int irq;
	u8 irq_mask;
	int chip_type; // not used
	unsigned int powerup_count;
	struct icm40605_chip_config chip_config;
	int skip_samples;
	s64 it_timestamp;
	s64 data_timestamp;
	s64 standard_period;
	s64 interrupt_period;
	s64 period_min;
	s64 period_max;
	int period_divider;
	int interrupt_regval;
	u8  data_buff[ICM40605_FIFO_COUNT_LIMIT];
};

/* scan indexes follow DATA register order */
enum icm40605_scan_axis {
	// ICM40605_SCAN_HEADER = 0,
	ICM40605_SCAN_ACCEL_X = 0,
	ICM40605_SCAN_ACCEL_Y,
	ICM40605_SCAN_ACCEL_Z,
	ICM40605_SCAN_GYRO_X,
	ICM40605_SCAN_GYRO_Y,
	ICM40605_SCAN_GYRO_Z,
	ICM40605_SCAN_TEMP,
	// ICM40605_SCAN_INNER_TIME,
	ICM40605_SCAN_TIMESTAMP,
};

enum icm40605_sensor_type {
	ICM40605_ACCEL = 0,
	ICM40605_GYRO,
	ICM40605_TEMP,
	ICM40605_TIMESTAMP,
	ICM40605_NUM_SENSORS /* must be last */
};

#define IIO_TRIGGER 1
#define ICM40605_RESET_FLAG 1
#define ICM40605_DEBUG_FLAG 0

// fifo
#define ICM40605_OUTPUT_DATA_SIZE 24 // align 8, last 8 for timestamp
#define ICM40605_OUTPUT_DATA_SIZE_PULS_ONE 25
#define ICM40605_FIFO_DATUM               16
#define ICM40605_BYTES_PER_3AXIS_SENSOR   6
#define ICM40605_FIFO_COUNT_BYTE          2
#define ICM40605_BYTE_FIFO_TEMP           1
#define ICM40605_FIFO_SIZE                1024
#define INV_MPU6050_TS_PERIOD_JITTER      4

irqreturn_t icm40605_read_fifo(int irq, void *p);
int icm40605_set_mode(struct icm40605_data *data, enum icm40605_sensor_type t, bool mode);
int icm40605_reset_fifo(struct iio_dev *indio_dev);
int icm40605_core_probe(struct regmap *regmap, int irq, const char *name,
						int chip_type, bool use_spi);
void icm40605_core_remove(struct device *dev);
int icm40605_probe_trigger(struct iio_dev *indio_dev, int irq_type);
int icm40605_set_enable(struct iio_dev *indio_dev, bool enable);
#endif  /* ICM40605_H_ */
