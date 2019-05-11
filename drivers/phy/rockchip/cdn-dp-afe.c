#define AFE_LINK_RATE_1_6	0x6
#define AFE_LINK_RATE_2_7	0x0a
#define AFE_LINK_RATE_5_4	0x14

struct {
	char* item;
	int swing;
	int pe;
} configs[] = {
	{ "s0p0", 0x2a, 0x00 },
	{ "s0p1", 0x1f, 0x15 },
	{ "s0p2", 0x14, 0x22 },
	{ "s0p3", 0x02, 0x2b },
	{ "s1p0", 0x21, 0x00 },
	{ "s1p1", 0x12, 0x15 },
	{ "s1p2", 0x02, 0x22 },
	{ "s2p0", 0x15, 0x00 },
	{ "s2p1", 0x00, 0x15 },
	{ "s3p0", 0x00, 0x00 },
	{ "boost", 0x700, 0}, //10
	{ "scale", 0x13c, 0}, //11
};

static void afe_config_lane(void __iomem *reg, int link_rate, u8 voltage_swing,
			    u8 pre_emphasis, u8 lane)
{
	writel(0, reg + PHY_DP_TX_CTL);

	switch (voltage_swing) {
	case 0:
		switch (pre_emphasis) {
		case 0:
			writel(configs[0].swing, reg + TX_TXCC_MGNFS_MULT_000(lane));
			writel(configs[0].pe, reg + TX_TXCC_CPOST_MULT_00(lane));
			break;
		case 1:
			writel(configs[1].swing, reg + TX_TXCC_MGNFS_MULT_000(lane));
			writel(configs[1].pe, reg + TX_TXCC_CPOST_MULT_00(lane));
			break;
		case 2:
			writel(configs[2].swing, reg + TX_TXCC_MGNFS_MULT_000(lane));
			writel(configs[2].pe, reg + TX_TXCC_CPOST_MULT_00(lane));
			break;
		case 3:
			writel(configs[3].swing, reg + TX_TXCC_MGNFS_MULT_000(lane));
			writel(configs[3].pe, reg + TX_TXCC_CPOST_MULT_00(lane));
			break;
		}
		break;
	case 1:
		switch (pre_emphasis) {
		case 0:
			writel(configs[4].swing, reg + TX_TXCC_MGNFS_MULT_000(lane));
			writel(configs[4].pe, reg + TX_TXCC_CPOST_MULT_00(lane));
			break;
		case 1:
			writel(configs[5].swing, reg + TX_TXCC_MGNFS_MULT_000(lane));
			writel(configs[5].pe, reg + TX_TXCC_CPOST_MULT_00(lane));
			break;
		case 2:
			writel(configs[6].swing, reg + TX_TXCC_MGNFS_MULT_000(lane));
			writel(configs[6].pe, reg + TX_TXCC_CPOST_MULT_00(lane));
			break;
		case 3:
			break;
		}
		break;
	case 2:
		switch (pre_emphasis) {
		case 0:
			writel(configs[7].swing, reg + TX_TXCC_MGNFS_MULT_000(lane));
			writel(configs[7].pe, reg + TX_TXCC_CPOST_MULT_00(lane));

			/* C1 lane2 need increase boost for eye diagram */
		//	if (lane == 2) {
			if (link_rate != AFE_LINK_RATE_5_4) {
				writel(configs[10].swing, reg + TX_DIAG_TX_DRV(lane));
				writel(configs[11].swing, reg + TX_TXCC_CAL_SCLR_MULT(lane));
			}
			break;
		case 1:
			writel(configs[8].swing, reg + TX_TXCC_MGNFS_MULT_000(lane));
			writel(configs[8].pe, reg + TX_TXCC_CPOST_MULT_00(lane));
			break;
		case 2:
		case 3:
			break;
		}
		break;
	case 3:
		switch (pre_emphasis) {
		case 0:
			writel(configs[9].swing, reg + TX_TXCC_MGNFS_MULT_000(lane));
			writel(configs[9].pe, reg + TX_TXCC_CPOST_MULT_00(lane));
			break;
		case 1:
		case 2:
		case 3:
			break;
		}
		break;
	}
}

static void afe_pre_config(void __iomem *reg)
{
	u32 val;

	/*
	 * Write PHY_DP_MODE_CTL[3:0] with 0b1000.
	 * (Place the PHY lanes in the A3 power state.)
	 * Wait for PHY_DP_MODE_CTL[7:4] == 0b1000
	 */
	val = readl(reg + DP_MODE_CTL);
	val = val & 0xFFF0;
	val = val | 8;
	writel(val, reg +  DP_MODE_CTL);
	do {
		val = readl(reg + DP_MODE_CTL);
		val = val & 0x00F0;
		val = val >> 4;
	} while (val != 8);

	/*
	 * Clear PHY_DP_CLK_CTL[2]. Gate the PLL clocks from PMA.
	 * Wait for PHY_DP_CLK_CTL[3] == 0.
	 */
	val = readl(reg + DP_CLK_CTL);
	val = val & 0xFFFB;
	writel(val, reg +  DP_CLK_CTL);
	do {
		val = readl(reg + DP_CLK_CTL);
		val = val & 8;
		val = val >> 3;
	} while (val);

	/*
	 * Clear PHY_DP_CLK_CTL[0]. Disable the PLL.
	 * Wait for PHY_DP_CLK_CTL[1] == 0.
	 */
	val = readl(reg + DP_CLK_CTL);
	val = val & 0xFFFE;
	writel(val, reg +  DP_CLK_CTL);
	do {
		val = readl(reg + DP_CLK_CTL);
		val = val & 2;
		val = val >> 1;
	} while (val);
}

static void afe_post_config(void __iomem *reg)
{
	u32 val;

	/*
	 * Set PHY_DP_CLK_CTL[0]. Enable the PLL.
	 * Wait for PHY_DP_CLK_CTL[1] == 1.
	 */
	val = readl(reg + DP_CLK_CTL);
	val = val | 1;
	writel(val, reg +  DP_CLK_CTL);
	do {
		val = readl(reg + DP_CLK_CTL);
		val = val & 2;
		val = val >> 1;
	} while (val != 1);

	/*
	 * Set PHY_DP_CLK_CTL[2]. Enable PMA PLL clocks.
	 * Wait for PHY_DP_CLK_CTL[3] == 1.
	 */
	val = readl(reg + DP_CLK_CTL);
	val = val | 4;
	writel(val, reg +  DP_CLK_CTL);
	do {
		val = readl(reg + DP_CLK_CTL);
		val = val & 8;
		val = val >> 3;
	} while (val != 1);

	/*
	 * Write PHY_DP_MODE_CTL[3:0] with 0b0100 (A2 power state).
	 * The PMA must go through the A2 power state upon a data rate change.
	 * Wait for PHY_DP_MODE_CTL[7:4] == 0b0100.
	 */
	val = readl(reg + DP_MODE_CTL);
	val = val & 0xFF00;
	val = val | 4;
	writel(val, reg +  DP_MODE_CTL);
	do {
		val = readl(reg + DP_MODE_CTL);
		val = val & 0x00F0;
		val = val >> 4;
	} while (val != 4);

	/*
	 * As required, change the PHY power state to A0
	 * Wait for PHY_DP_MODE_CTL[7:4] == 0b0001.
	 */
	val = readl(reg + DP_MODE_CTL);
	val = val & 0xFF00;
	val = val | 1;
	writel(val, reg +  DP_MODE_CTL);
	do {
		val = readl(reg + DP_MODE_CTL);
		val = val & 0x00F0;
		val = val >> 4;
	} while (val != 1);
}

static void afe_enable_lanes(void __iomem *reg)
{
	u32 val;

	afe_pre_config(reg);

	val = readl(reg + DP_MODE_CTL);
	/* set 4 lanes */
	val = val | 0xF000;
	val &= ~(15 << 12);
	writel(val, reg +  DP_MODE_CTL);

	afe_post_config(reg);
}

static void afe_set_link_rate(void __iomem *reg, int link_rate, int ssc)
{
	u32 val;
	int i;

	afe_pre_config(reg);

	/*
	 * Re-configure PMA registers for the new data rate
	 * (as defined in the programmer?s guide).
	 */
	val = readl(reg + DP_CLK_CTL);
	val = val & 0x00FF;
	switch (link_rate) {
	case AFE_LINK_RATE_1_6:
	case AFE_LINK_RATE_2_7:
		val = val | (0x24 << 8);
		break;

	case AFE_LINK_RATE_5_4:
		val = val | (0x12 << 8);
		break;
	}
	writel(val, reg +  DP_CLK_CTL);

	switch (link_rate) {
	case AFE_LINK_RATE_1_6:
		if (ssc)
			val = 0x86;
		else

			val = 0x87;
		break;
	case AFE_LINK_RATE_5_4:
	case AFE_LINK_RATE_2_7:
		if (ssc)
			val = 0xe0;
		else
			val = 0xe1;
		break;
	}
	writel(val, reg +  CMN_PLL1_INTDIV);

	switch (link_rate) {
	case AFE_LINK_RATE_1_6:
		writel(0x30b9, reg + CMN_PLL1_VCOCAL_START);
		writel(0x0000, reg + CMN_PLLSM1_USER_DEF_CTRL);
		writel(0x0006, reg + CMN_DIAG_PLL1_V2I_TUNE);
		writel(0x0100, reg + CMN_DIAG_PLL1_PTATIS_TUNE1);
		writel(0x0007, reg + CMN_DIAG_PLL1_PTATIS_TUNE2);
		writel(0x22, reg + CMN_PLL1_HIGH_THR);
		writel(0x0001, reg + CMN_DIAG_PLL1_INCLK_CTRL);
		if (ssc) {
			writel(0xf915, reg + CMN_PLL1_FRACDIV);
			writel(0x140, reg + CMN_PLL1_SS_CTRL1);
			writel(0x7f03, reg + CMN_PLL1_SS_CTRL2);
		} else {
			writel(0x0000, reg + CMN_PLL1_FRACDIV);
			writel(0x8000, reg + CMN_PLL1_SS_CTRL1);
			writel(0x0000, reg + CMN_PLL1_SS_CTRL2);
		}
		break;
	case AFE_LINK_RATE_2_7:
		writel(0x30b4, reg + CMN_PLL1_VCOCAL_START);
		writel(0x1000, reg + CMN_PLLSM1_USER_DEF_CTRL);
		writel(0x0007, reg + CMN_DIAG_PLL1_V2I_TUNE);
		writel(0x001, reg + CMN_DIAG_PLL1_PTATIS_TUNE1);
		writel(0x0001, reg + CMN_DIAG_PLL1_PTATIS_TUNE2);
		writel(0x0001, reg + CMN_DIAG_PLL1_INCLK_CTRL);
		writel(0x5, reg + CMN_PLL1_HIGH_THR);
		if (ssc) {
			writel(0xf479, reg + CMN_PLL1_FRACDIV);
			writel(0x204, reg + CMN_PLL1_SS_CTRL1);
			writel(0x7f03, reg + CMN_PLL1_SS_CTRL2);
		} else {
			writel(0x0000, reg + CMN_PLL1_FRACDIV);
			writel(0x8000, reg + CMN_PLL1_SS_CTRL1);
			writel(0x0000, reg + CMN_PLL1_SS_CTRL2);
		}
		break;
	case AFE_LINK_RATE_5_4:
		writel(0x30b4, reg + CMN_PLL1_VCOCAL_START);
		writel(0x1000, reg + CMN_PLLSM1_USER_DEF_CTRL);
		writel(0x0007, reg + CMN_DIAG_PLL1_V2I_TUNE);
		writel(0x001, reg + CMN_DIAG_PLL1_PTATIS_TUNE1);
		writel(0x0001, reg + CMN_DIAG_PLL1_PTATIS_TUNE2);
		writel(0x0001, reg + CMN_DIAG_PLL1_INCLK_CTRL);
		writel(0x5, reg + CMN_PLL1_HIGH_THR);
		if (ssc) {
			writel(0xf479, reg + CMN_PLL1_FRACDIV);
			writel(0x204, reg + CMN_PLL1_SS_CTRL1);
			writel(0x7f03, reg + CMN_PLL1_SS_CTRL2);
		} else {
			writel(0x0000, reg + CMN_PLL1_FRACDIV);
			writel(0x8000, reg + CMN_PLL1_SS_CTRL1);
			writel(0x0000, reg + CMN_PLL1_SS_CTRL2);
		}
		break;
	}

	val = readl(reg + CMN_DIAG_HSCLK_SEL);
	val = val & 0xFFCF;
	switch (link_rate) {
	case AFE_LINK_RATE_1_6:
	case AFE_LINK_RATE_2_7:
		val = val | (3 << 4);
		break;
	case AFE_LINK_RATE_5_4:
		val = val | (2 << 4);
		break;
	}
	writel(val, reg +  CMN_DIAG_HSCLK_SEL);

	for (i = 0; i < 4; i++) {
		val = readl(reg + XCVR_DIAG_PLLDRC_CTRL(i));
		val = val & 0x8FFF;
		switch (link_rate) {
		case AFE_LINK_RATE_1_6:
		case AFE_LINK_RATE_2_7:
			val = val | (6 << 12);
			break;
		case AFE_LINK_RATE_5_4:
			val = val | (4 << 12);
			break;
		}
		writel(val, reg +  XCVR_DIAG_PLLDRC_CTRL(i));
	}

	afe_post_config(reg);
}

static void afe_config(void __iomem *reg,  int link_rate, u8 voltage_swing,
		       u8 pre_emphasis, bool ssc)
{
	u8 lane;

	printk("%s, %d, %d, %d, %d\n", __func__, link_rate, voltage_swing,
	       pre_emphasis, ssc);
	afe_enable_lanes(reg);
	afe_set_link_rate(reg, link_rate, ssc);

	for (lane = 0; lane < 4; lane++)
		afe_config_lane(reg, link_rate, voltage_swing, pre_emphasis, lane);
}

static void afe_init(void __iomem *reg)
{
	int i;

	for (i = 0; i < 4; i++) {
		writel(0, reg + TX_TXCC_MGNFS_MULT_000(i));
		writel(0, reg + TX_TXCC_MGNFS_MULT_001(i));
		writel(0, reg + TX_TXCC_MGNFS_MULT_010(i));
		writel(0, reg + TX_TXCC_MGNFS_MULT_011(i));
		writel(0, reg + TX_TXCC_MGNFS_MULT_100(i));
		writel(0, reg + TX_TXCC_MGNFS_MULT_101(i));
		writel(0, reg + TX_TXCC_MGNFS_MULT_110(i));
		writel(0, reg + TX_TXCC_MGNFS_MULT_111(i));
		writel(0, reg + TX_TXCC_CPOST_MULT_10(i));
		writel(0, reg + TX_TXCC_CPOST_MULT_01(i));
		writel(0, reg + TX_TXCC_CPOST_MULT_00(i));
		writel(0, reg + TX_TXCC_CPOST_MULT_11(i));
		writel(0x128, reg + TX_TXCC_CAL_SCLR_MULT(i));
		writel(0x0400, reg + TX_DIAG_TX_DRV(i));
	}

	writel(0, reg + PHY_DP_TX_CTL);
}

/*
 * set dp link rate:
 * 0: disabled afe test
 * 0x6: RBR;
 * 0x0a: HBR;
 * 0x14: HBR2;
 */
void tcphy_afe_config(struct phy *phy, u8 rate, u8 voltage_swing,
		      u8 pre_emphasis, bool ssc)
{
	struct rockchip_typec_phy *tcphy = phy_get_drvdata(phy);

	afe_init(tcphy->base);
	afe_config(tcphy->base, rate, voltage_swing, pre_emphasis, ssc);
}
EXPORT_SYMBOL_GPL(tcphy_afe_config);

static ssize_t phy_config_store(struct device *kdev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret, i, sw = 0, pe = 0;
	char *item, *token, *sptr;
	char mem[50];

	strcpy(mem, buf);
	sptr = mem;

	item = strsep(&sptr, " ");
	if (!item)
		return count;

	token = strsep(&sptr, " ");
	if(token) {
		ret = kstrtou32(token, 0, &sw);
		if (ret)
			return ret;
	}

	token = strsep(&sptr, " ");
	if(token) {
		ret = kstrtou32(token, 0, &pe);
		if (ret)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(configs); i++) {
		if (!strcmp(item, configs[i].item)) {
			configs[i].swing = sw;
			configs[i].pe = pe;
			printk("setting %s: swing = 0x%X, pre_epmhasis = 0x%X \n", item, sw, pe);
		}
	}

	return count;
}

static ssize_t phy_config_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t count = 0;

	count += snprintf(&buf[count], (PAGE_SIZE - count - 2),
			  "item swing pre-emphasis\n");

	for (i = 0; i < ARRAY_SIZE(configs); i++)
		count += snprintf(&buf[count], (PAGE_SIZE - count - 2),
				  "%s 0x%X 0x%X\n", configs[i].item, configs[i].swing, configs[i].pe);

	return count;
}
static DEVICE_ATTR(phy_config,  S_IRUGO | S_IWUSR, phy_config_show, phy_config_store);

void phy_parameter_init(struct kobject *kobj)
{

	int ret;

	ret = sysfs_create_file(kobj, &dev_attr_phy_config.attr);
}
