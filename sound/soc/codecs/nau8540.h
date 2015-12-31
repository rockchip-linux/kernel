#ifndef _NAU8540_H
#define _NAU8540_H

#define REG0x00_SW_RESET                     0x00
#define REG0x01_POWER_MANAGEMENT             0x01
#define REG0x02_CLOCK_CTRL                   0x02
#define REG0x03_CLOCK_SRC                    0x03
#define REG0x04_FLL1                         0x04
#define REG0x05_FLL2                         0x05
#define REG0x06_FLL3                         0x06
#define REG0x07_FLL4                         0x07
#define REG0x08_FLL5                         0x08
#define REG0x09_FLL6                         0x09
#define REG0x0A_FLL_VCO_RSV                  0x0A
#define REG0x10_PCM_CTRL0                    0x10
#define REG0x11_PCM_CTRL1                    0x11
#define REG0x12_PCM_CTRL2                    0x12
#define REG0x13_PCM_CTRL3                    0x13
#define REG0x14_PCM_CTRL4                    0x14
#define REG0x20_ALC_CONTROL_1                0x20
#define REG0x21_ALC_CONTROL_2                0x21
#define REG0x22_ALC_CONTROL_3                0x22
#define REG0x23_ALC_CONTROL_4                0x23
#define REG0x24_ALC_CONTROL_5                0x24
#define REG0x2D_ALC_GAIN_CH12                0x2D
#define REG0x2E_ALC_GAIN_CH34                0x2E
#define REG0x2F_ALC_STATUS                   0x2F
#define REG0x30_NOTCH_FIL1_CH1               0x30
#define REG0x31_NOTCH_FIL2_CH1               0x31
#define REG0x32_NOTCH_FIL1_CH2               0x32
#define REG0x33_NOTCH_FIL2_CH2               0x33
#define REG0x34_NOTCH_FIL1_CH3               0x34
#define REG0x35_NOTCH_FIL2_CH3               0x35
#define REG0x36_NOTCH_FIL1_CH4               0x36
#define REG0x37_NOTCH_FIL2_CH4               0x37
#define REG0x38_HPF_FILTER_CH12              0x38
#define REG0x39_HPF_FILTER_CH34              0x39
#define REG0x3A_ADC_SAMPLE_RATE              0x3A
#define REG0x40_DIGITAL_GAIN_CH1             0x40
#define REG0x41_DIGITAL_GAIN_CH2             0x41
#define REG0x42_DIGITAL_GAIN_CH3             0x42
#define REG0x43_DIGITAL_GAIN_CH4             0x43
#define REG0x44_DIGITAL_MUX                  0x44
#define REG0x48_P2P_CH1                      0x48
#define REG0x49_P2P_CH2                      0x49
#define REG0x4A_P2P_CH3                      0x4A
#define REG0x4B_P2P_CH4                      0x4B
#define REG0x4C_PEAK_CH1                     0x4C
#define REG0x4D_PEAK_CH2                     0x4D
#define REG0x4E_PEAK_CH3                     0x4E
#define REG0x4F_PEAK_CH4                     0x4F
#define REG0x50_GPIO_CTRL                    0x50
#define REG0x51_MISC_CTRL                    0x51
#define REG0x52_I2C_CTRL                     0x52
#define REG0x58_I2C_DEVICE_ID                0x58
#define REG0x5A_RST                          0x5A
#define REG0x60_VMID_CTRL                    0x60
#define REG0x61_MUTE                         0x61
#define REG0x64_ANALOG_ADC1                  0x64
#define REG0x65_ANALOG_ADC2                  0x65
#define REG0x66_ANALOG_PWR                   0x66
#define REG0x67_MIC_BIAS                     0x67
#define REG0x68_REFERENCE                    0x68
#define REG0x69_FEPGA1                       0x69
#define REG0x6A_FEPGA2                       0x6A
#define REG0x6B_FEPGA3                       0x6B
#define REG0x6C_FEPGA4                       0x6C
#define REG0x6D_PWR                          0x6D

#define NAU8540_MAX_REGISTER                 0x6D

/* codec private data */
struct nau8540_priv {
	struct device *dev;
	struct regmap *regmap;
	void *control_data;
	unsigned int mclk;
};

#endif

