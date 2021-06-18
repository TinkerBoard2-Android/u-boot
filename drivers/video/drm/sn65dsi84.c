// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2008-2018 Fuzhou Rockchip Electronics Co., Ltd
 */

#include <common.h>
#include <i2c.h>
#include <errno.h>
#include <dm.h>
#include <dm/uclass.h>
#include <dm/uclass-id.h>

#include "sn65dsi84.h"
#include <asm/gpio.h>

struct sn65dsi84_data *g_sn65dsi84 = NULL;
static bool switch_to_lvds = false;

void sn65dsi84_i2c_reg_write(struct udevice *dev, uint offset, uint value)
{
	#define SN65DSI84_I2C_WRITE_RETRY_COUNT (6)
	int ret = 0;
	int i = 0;

	do {
		ret = dm_i2c_reg_write(dev, offset, value);
		if (ret < 0) {
			printf("sn65dsi84_i2c_reg_write, reg = %x value = %x  i = %d ret = %d\n", offset, value, i, ret);
			mdelay(20);
		}
	} while ((++i <= SN65DSI84_I2C_WRITE_RETRY_COUNT) && (ret < 0));
}

int  sn65dsi84_i2c_reg_read(struct udevice *dev, uint offset)
{
	#define SN65DSI84_I2C_READ_RETRY_COUNT (3)
	int ret = 0;
	int i = 0;

	do {
		ret = dm_i2c_reg_read(dev, offset);
		if (ret < 0) {
			printf("sn65dsi84_i2c_reg_read fail, i = %d  reg = %x ret = %d\n", i, offset, ret);
			mdelay(20);
		} else
			return ret;
	} while ((++i <= SN65DSI84_I2C_READ_RETRY_COUNT) && (ret < 0));

	return ret;
}

void ConvertBoard_power_on(struct sn65dsi84_data *sn65dsi84)
{
	printf("%s \n", __func__);
	if (dm_gpio_is_valid(&sn65dsi84->pwr_source_gpio)) {
		dm_gpio_set_value(&sn65dsi84->pwr_source_gpio, 1);
		mdelay(20);
	}
}

void ConvertBoard_power_off(struct sn65dsi84_data *sn65dsi84)
{
	printf("%s \n", __func__);
	if (dm_gpio_is_valid(&sn65dsi84->pwr_source_gpio)) {
		dm_gpio_set_value(&sn65dsi84->pwr_source_gpio, 0);
	}
}

void lvds_power_on(struct sn65dsi84_data *sn65dsi84)
{
	printf("%s \n", __func__);
	if (dm_gpio_is_valid(&sn65dsi84->lvds_vdd_en_gpio)) {
		dm_gpio_set_value(&sn65dsi84->lvds_vdd_en_gpio, 1);
		//msleep(20);//T2: 0.01ms ~50ms
	}
}

void lvds_power_off(struct sn65dsi84_data *sn65dsi84)
{
	printf("%s \n", __func__);
	if (dm_gpio_is_valid(&sn65dsi84->lvds_vdd_en_gpio)) {
		//msleep(10);//T5: 0.01ms ~50ms
		dm_gpio_set_value(&sn65dsi84->lvds_vdd_en_gpio, 0);
		//msleep(1000);//T7: 1000ms
	}
}

bool sn65dsi84_is_connected(void)
{
	printf("%s  sn65dsi84 connect = %d\n", __func__, switch_to_lvds);
	return switch_to_lvds ;
}

void sn65dsi84_chip_enable(struct sn65dsi84_data *sn65dsi84)
{
	printf("%s \n", __func__);
	if (dm_gpio_is_valid(&sn65dsi84->sn65dsi84_en_gpio)) {
		dm_gpio_set_value(&sn65dsi84->sn65dsi84_en_gpio, 1);
		mdelay(10);
	}

	sn65dsi84->powered = true;
}

void sn65dsi84_chip_shutdown(struct sn65dsi84_data *sn65dsi84)
{
	printf("%s \n", __func__);
	if (dm_gpio_is_valid(&sn65dsi84->sn65dsi84_en_gpio)) {
		dm_gpio_set_value(&sn65dsi84->sn65dsi84_en_gpio, 0);
		mdelay(10);
	}

	sn65dsi84->powered = false;
}

static unsigned int sn65dsi84_clk_src(struct sn65dsi84_data *sn65dsi84)
{
	#define LVDS_CDLK (sn65dsi84->lvds_clk_rate)
	unsigned int val = 0;

	if ((25000000 <= LVDS_CDLK) && (LVDS_CDLK < 37500000))
		val = 0;
	else if ((37500000 <= LVDS_CDLK) && (LVDS_CDLK < 62500000))
		val = 1;
	else if ((62500000 <= LVDS_CDLK) && (LVDS_CDLK < 87500000))
		val = 2;
	else if ((87500000 <= LVDS_CDLK) && (LVDS_CDLK < 112500000))
		val = 3;
	else if ((112500000 <= LVDS_CDLK) && (LVDS_CDLK < 137500000))
		val = 4;
	else if ((137500000 <= LVDS_CDLK) && (LVDS_CDLK < 154000000))
		val = 5;

	val = (val << LVDS_CLK_RANGE_OFFSET);
	if (!sn65dsi84->clk_from_refclk)
		val |= (1 << HS_CLK_SRC_OFFSET);
	printf("%s val=%x\n", __func__, val);

	return val;
}

static unsigned int sn65dsi84_clk_div(struct sn65dsi84_data *sn65dsi84)
{
	unsigned long long rate = sn65dsi84->lvds_clk_rate;
	unsigned long long dsi_ch_clk;
	unsigned int val;

	if (sn65dsi84->dual_link)
		dsi_ch_clk = (rate * sn65dsi84->lvds_bpp * 2) / (2 * sn65dsi84->dsi_lanes);
	else
		dsi_ch_clk = (rate * sn65dsi84->lvds_bpp) / (2 * sn65dsi84->dsi_lanes);

	val = dsi_ch_clk/rate;
	val -= 1;
	val = (val << DSI_CLK_DIVIDER);

	printf("%s val=%x\n", __func__, val);

	return val;
}

static unsigned int sn65dsi84_refclk_multiplier(struct sn65dsi84_data *sn65dsi84)
{
	unsigned int val = 0;

	if (sn65dsi84->refclk_multiplier == MULTIPLY_BY_1)
		val = 0;
	else if (sn65dsi84->refclk_multiplier == MULTIPLY_BY_2)
		val = BIT(0);
	else if (sn65dsi84->refclk_multiplier == MULTIPLY_BY_3)
		val = BIT(1);
	else if (sn65dsi84->refclk_multiplier == MULTIPLY_BY_4)
		val = BIT(0) | BIT(1);
	else
		val = BIT(1);

	printf("%s val=%x\n", __func__, val);

	return val;
}


static unsigned int sn65dsi84_dsi_clk(struct sn65dsi84_data *sn65dsi84)
{
	unsigned long long rate = sn65dsi84->lvds_clk_rate;
	unsigned long long dsi_ch_clk;
	unsigned int val;

	if (sn65dsi84->dual_link)
		dsi_ch_clk = (rate * sn65dsi84->lvds_bpp * 2) / (2 * sn65dsi84->dsi_lanes);
	else
		dsi_ch_clk = (rate * sn65dsi84->lvds_bpp) / (2 * sn65dsi84->dsi_lanes);

	if (dsi_ch_clk >= 40000000)
		val = dsi_ch_clk/5000000;
	else
		val = 0;

	printf("%s val=%x\n", __func__, val);

	return val;

}

static unsigned int sn65dsi84_format(struct sn65dsi84_data *sn65dsi84)
{
	unsigned int val = 0;

	if (sn65dsi84->vm.flags & DISPLAY_FLAGS_DE_LOW)
		val |= (1 << DE_NEG_POLARITY_OFFSET);

	if (sn65dsi84->vm.flags & DISPLAY_FLAGS_HSYNC_LOW)
		val |= (1 << HS_NEG_POLARITY_OFFSET);

	if (sn65dsi84->vm.flags & DISPLAY_FLAGS_VSYNC_LOW)
		val |= (1 << VS_NEG_POLARITY_OFFSET);

	if (!sn65dsi84->dual_link)
		val |= (1 << LVDS_LINK_CFG_OFFSET);

	if (sn65dsi84->lvds_bpp == 24)
		val |= (1 << CHA_24BPP_MODE_OFFSET);

	if (sn65dsi84->lvds_format == 1)
		val |= (1 << CHA_24BPP_FMT1_OFFSET);

	if (sn65dsi84->dual_link) {
		if (sn65dsi84->lvds_bpp == 24)
			val |= (1 << CHB_24BPP_MODE_OFFSET);

		if (sn65dsi84->lvds_format == 1)
			val |= (1 << CHB_24BPP_FMT1_OFFSET);
	}

	printf("%s val=%x\n", __func__, val);

	return val;
}

static void sn65dsi84_init_csr_registers(struct sn65dsi84_data *sn65dsi84)
{
	bool test_pattern = sn65dsi84->test_pattern_en;
	int dual_link_div = sn65dsi84->dual_link ? 2 : 1;
	uint8_t val;
	#define HIGH_BYTE(X) ((X & 0xFF00 ) >> 8)
	#define LOW_BYTE(X)  (X & 0x00FF)

	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_SOFT_RESET, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_PLL_EN, 0x00);
	mdelay(10);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_CLK_SRC, sn65dsi84_clk_src(sn65dsi84));
	if (sn65dsi84->clk_from_refclk)
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_CLK_DIV, sn65dsi84_refclk_multiplier(sn65dsi84));
	else
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_CLK_DIV, sn65dsi84_clk_div(sn65dsi84));

	/* Configure DSI_LANES  */
	val = sn65dsi84_i2c_reg_read(sn65dsi84->sn65dsi84_i2c_dev, SN_DSI_LANES);
	val &= ~(3 << CHA_DSI_LANES);
	val |= ((4 - sn65dsi84->dsi_lanes) << CHA_DSI_LANES);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_DSI_LANES, val);

	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_DSI_EQ, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_DSI_CLK, sn65dsi84_dsi_clk(sn65dsi84));
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x13, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_FORMAT, sn65dsi84_format(sn65dsi84));
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_LVDS_VOLTAGE, sn65dsi84->lvds_voltage);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_LVDS_TERM, 0x03);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_LVDS_CM_VOLTAGE, 0x00);

	if (!test_pattern) {
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_HACTIVE_LOW, LOW_BYTE(sn65dsi84->vm.hactive));
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_HACTIVE_HIGH, HIGH_BYTE(sn65dsi84->vm.hactive));
	} else {
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_HACTIVE_LOW, LOW_BYTE(sn65dsi84->vm.hactive/dual_link_div));
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_HACTIVE_HIGH, HIGH_BYTE(sn65dsi84->vm.hactive/dual_link_div));
	}

	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x22, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x23, 0x00);

	if (!test_pattern) {
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_VACTIVE_LOW, 0x00);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_VACTIVE_HIGH, 0x00);
	} else {
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_VACTIVE_LOW, LOW_BYTE(sn65dsi84->vm.vactive));
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_VACTIVE_HIGH, HIGH_BYTE(sn65dsi84->vm.vactive));
	}

	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x26, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x27, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_SYNC_DELAY_LOW, LOW_BYTE(sn65dsi84->sync_delay));
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_SYNC_DELAY_HIGH, HIGH_BYTE(sn65dsi84->sync_delay));
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x2A, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x2B, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_HSYNC_LOW, LOW_BYTE(sn65dsi84->vm.hsync_len/dual_link_div));
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_HSYNC_HIGH, HIGH_BYTE(sn65dsi84->vm.hsync_len/dual_link_div));
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x2E, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x2F, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_VSYNC_LOW, LOW_BYTE(sn65dsi84->vm.vsync_len));
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_VSYNC_HIGH, HIGH_BYTE(sn65dsi84->vm.vsync_len));
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x32, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x33, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_HBP, sn65dsi84->vm.hback_porch/dual_link_div);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x35, 0x00);

	if (!test_pattern) {
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_VBP, 0x00);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x37, 0x00);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_HFP, 0x00);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x39, 0x00);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_VFP,  0x00);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x3B, 0x00);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_TEST_PATTERN, 0x00);
	} else {
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_VBP, sn65dsi84->vm.vback_porch);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x37, 0x00);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_HFP, sn65dsi84->vm.hfront_porch/dual_link_div);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x39, 0x00);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_VFP,  sn65dsi84->vm.vfront_porch);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x3B, 0x00);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_TEST_PATTERN, 0x10);
	}

	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x3D, 0x00);
	sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, 0x3E, 0x00);

	printf("%s -\n", __func__);
}

void sn65dsi84_init_sequence(struct sn65dsi84_data *sn65dsi84)
{
	uint8_t val = 0xFF;
	int retry = 0;

	printf("%s +\n", __func__);

	do {
		sn65dsi84_chip_shutdown(sn65dsi84);
		sn65dsi84_chip_enable(sn65dsi84);
		sn65dsi84_init_csr_registers(sn65dsi84);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_PLL_EN, 0x1);
		mdelay(10);
		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_SOFT_RESET, 1);
		mdelay(10);

		sn65dsi84_i2c_reg_write(sn65dsi84->sn65dsi84_i2c_dev, SN_IRQ_STAT, 0xFF);
		mdelay(2);
		val = sn65dsi84_i2c_reg_read(sn65dsi84->sn65dsi84_i2c_dev, SN_IRQ_STAT);
		val &= 0xFE;
		if (val !=0x00)
			printf("sn65dsi84_init_sequence SN_IRQ_STAT= %x\n", val);
		printf("sn65dsi84_init_sequence SN_IRQ_STAT= %x\n", val);
	} while((val !=0x00) && (++retry < 3));

	printf("%s - \n", __func__);
}

void sn65dsi84_bridge_enable(void)
{
	if (!g_sn65dsi84) {
		printf("%s g_sn65dsi84 is null!\n", __func__);
		return;
	}

	printf("%s sn65dsi84->enabled=%d\n", __func__, g_sn65dsi84->enabled);
	if (g_sn65dsi84->enabled)
		return;

	lvds_power_on(g_sn65dsi84);
	mdelay(g_sn65dsi84->t2);
	sn65dsi84_init_sequence(g_sn65dsi84);
	mdelay(g_sn65dsi84->t3);

	g_sn65dsi84->enabled = true;

	return;
}

void sn65dsi84_bridge_disable(void)
{
	if (!g_sn65dsi84) {
		printf("%s g_sn65dsi84 is null!\n", __func__);
		return;
	}

	printf("%s sn65dsi84->enabled=%d\n", __func__, g_sn65dsi84->enabled);
	if (!g_sn65dsi84->enabled)
		return;

	sn65dsi84_i2c_reg_write(g_sn65dsi84->sn65dsi84_i2c_dev, SN_PLL_EN, 0);
	sn65dsi84_chip_shutdown(g_sn65dsi84);
	mdelay(g_sn65dsi84->t5);
	lvds_power_off(g_sn65dsi84);
	mdelay(g_sn65dsi84->t7);

	g_sn65dsi84->enabled = false;

	return;
}

void sn65dsi84_setup_desc( struct drm_display_mode *dmode)
{
	g_sn65dsi84->vm.hactive = dmode->hdisplay;
	g_sn65dsi84->vm.hfront_porch = dmode->hsync_start - dmode->hdisplay;
	g_sn65dsi84->vm.hsync_len = dmode->hsync_end - dmode->hsync_start;
	g_sn65dsi84->vm.hback_porch = dmode->htotal - dmode->hsync_end;

	g_sn65dsi84->vm.vactive = dmode->vdisplay;
	g_sn65dsi84->vm.vfront_porch = dmode->vsync_start - dmode->vdisplay;
	g_sn65dsi84->vm.vsync_len = dmode->vsync_end - dmode->vsync_start;
	g_sn65dsi84->vm.vback_porch = dmode->vtotal - dmode->vsync_end;

	g_sn65dsi84->vm.pixelclock = dmode->clock * 1000;

	g_sn65dsi84->vm.flags = 0;
	if (dmode->flags & DRM_MODE_FLAG_PHSYNC)
		g_sn65dsi84->vm.flags |= DISPLAY_FLAGS_HSYNC_HIGH;
	else if (dmode->flags & DRM_MODE_FLAG_NHSYNC)
		g_sn65dsi84->vm.flags |= DISPLAY_FLAGS_HSYNC_LOW;
	if (dmode->flags & DRM_MODE_FLAG_PVSYNC)
		g_sn65dsi84->vm.flags |= DISPLAY_FLAGS_VSYNC_HIGH;
	else if (dmode->flags & DRM_MODE_FLAG_NVSYNC)
		g_sn65dsi84->vm.flags |= DISPLAY_FLAGS_VSYNC_LOW;
	if (dmode->flags & DRM_MODE_FLAG_INTERLACE)
		g_sn65dsi84->vm.flags |= DISPLAY_FLAGS_INTERLACED;
	if (dmode->flags & DRM_MODE_FLAG_DBLSCAN)
		g_sn65dsi84->vm.flags |= DISPLAY_FLAGS_DOUBLESCAN;
	if (dmode->flags & DRM_MODE_FLAG_DBLCLK)
		g_sn65dsi84->vm.flags |= DISPLAY_FLAGS_DOUBLECLK;
}

bool sn65dsi84_detect(struct sn65dsi84_data  *sn65dsi84)
{
	#define ID_REGISTERS_SZIE (9)
	uint8_t id[ID_REGISTERS_SZIE] = {0x35, 0x38, 0x49, 0x53, 0x44, 0x20, 0x20, 0x20, 0x01};
	uint8_t return_id[ID_REGISTERS_SZIE] = {0};
	bool status = sn65dsi84->status;
	int i;

	printf(KERN_INFO "%s \n", __func__);

	if (status == true)
		return status;

	for (i = 0; i < sizeof(id) /sizeof(uint8_t); i++) {
		return_id[i] = sn65dsi84_i2c_reg_read(sn65dsi84->sn65dsi84_i2c_dev, i);
	}

	if (!memcmp(id, return_id, sizeof(id) /sizeof(uint8_t))) {
		printf( "sn65dsi84_detect successful\n");
		status = true;
	} else {
		printf("sn65dsi84_detect fail, 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
			return_id[0], return_id[1], return_id[2], return_id[3], return_id[4], return_id[5], return_id[6], return_id[7], return_id[8]);
	}

	sn65dsi84->status = status;

	printf("%s sn65dsi84->status=%d\n", __func__, sn65dsi84->status);

	return status;
}

static int sn65dsi84_probe(struct udevice *dev)
{
	struct sn65dsi84_data  *sn65dsi84 = dev_get_priv(dev);
	int ret;

	printf("%s\n", __func__);
	sn65dsi84->dev = dev;

	sn65dsi84->i2c_id = of_alias_get_id(ofnode_to_np(dev->node)->parent, "i2c");
	if (sn65dsi84->i2c_id < 0) {
		printf("sn65dsi86_parse_dt: error data->i2c_id = %d, use default number 8. \n", sn65dsi84->i2c_id);
		sn65dsi84->i2c_id = 8;
	}

	sn65dsi84->addr = dev_read_u32_default(dev, "reg", 0x2C);

	ret = gpio_request_by_name(dev, "lvds_vdd_en-gpios", 0, &sn65dsi84->lvds_vdd_en_gpio , GPIOD_IS_OUT);
	if (ret) {
		printf("Cannot get lvds_vdd_en: %d\n", ret);
	}

	ret = gpio_request_by_name(dev, "pwr_source-gpios", 0, &sn65dsi84->pwr_source_gpio, GPIOD_IS_OUT);
	if (ret) {
		printf("Cannot get pwr_source-gpios: %d\n", ret);
	}

	ret = gpio_request_by_name(dev, "EN-gpios", 0, &sn65dsi84->sn65dsi84_en_gpio, GPIOD_IS_OUT);
	if (ret) {
		printf("Cannot get EN-gpios: %d\n", ret);
	}
	if (dev->node.np) {
		if (dev->node.np->name)
			printf("sn65dsi84_probe  name=%s full_name=%s \n", dev->node.np->name, dev->node.np->full_name);
	}

	sn65dsi84->lvds_clk_rate = dev_read_u32_default(dev, "lvds-clk-rate", 255);
	sn65dsi84->lvds_format = dev_read_u32_default(dev, "lvds-format", 4);
	sn65dsi84->lvds_bpp = dev_read_u32_default(dev, "lvds-bpp", 24);
	sn65dsi84->sync_delay = dev_read_u32_default(dev, "sync_delay", 33);
	sn65dsi84->dual_link = dev_read_bool(dev, "dual-link");
	sn65dsi84->test_pattern_en = dev_read_bool(dev, "test-pattern");
	printf("sn65dsi84_probe  lvds_clk_rate=%u lvds_format =%u lvds_bpp = %u sync_delay = %u dual_link = %u test_pattern_en = %u\n",
		sn65dsi84->lvds_clk_rate, sn65dsi84->lvds_format, sn65dsi84->lvds_bpp,sn65dsi84->sync_delay,sn65dsi84->dual_link, sn65dsi84->test_pattern_en);

	sn65dsi84->dsi_lanes = dev_read_u32_default(dev, "dsi-lanes", 4);
	sn65dsi84->clk_from_refclk = dev_read_bool(dev, "clk_from_refclk");
	sn65dsi84->refclk_multiplier = dev_read_u32_default(dev, "refclk_multiplier", 0);
	sn65dsi84->lvds_voltage = dev_read_u32_default(dev, "lvds_voltage",0x0F);
	printf("sn65dsi84_probe  dsi_lanes = %u clk_from_refclk = %u refclk_multiplier = %u lvds_voltage  = %u\n",
		sn65dsi84->dsi_lanes, sn65dsi84->clk_from_refclk, sn65dsi84->refclk_multiplier, sn65dsi84->lvds_voltage);

	sn65dsi84->t1 = dev_read_u32_default(dev, "t1", 0);
	sn65dsi84->t2 = dev_read_u32_default(dev, "t2", 0);
	sn65dsi84->t3 = dev_read_u32_default(dev, "t3", 0);
	sn65dsi84->t4 = dev_read_u32_default(dev, "t4", 0);
	sn65dsi84->t5 = dev_read_u32_default(dev, "t5", 0);
	sn65dsi84->t6 = dev_read_u32_default(dev, "t6", 0);
	sn65dsi84->t7 = dev_read_u32_default(dev, "t7", 0);
	printf("sn65dsi84_probe  t1 = %u t2 = %u t3 = %u t4 = %u t5 = %u t6 = %u t7 = %u\n",
		sn65dsi84->t1, sn65dsi84->t2, sn65dsi84->t3, sn65dsi84->t4, sn65dsi84->t5, sn65dsi84->t6, sn65dsi84->t7);

	i2c_get_chip_for_busnum(sn65dsi84->i2c_id, sn65dsi84->addr, 1, &sn65dsi84->sn65dsi84_i2c_dev);

	ConvertBoard_power_on(sn65dsi84);
	lvds_power_off(sn65dsi84);
	sn65dsi84_chip_shutdown(sn65dsi84);
	sn65dsi84_chip_enable(sn65dsi84);
	if (sn65dsi84_detect(sn65dsi84)) {
		printf("sn65dsi84_probe  sn65dsi84 is detected\n");
		switch_to_lvds = true;
	} else {
		if (dm_gpio_is_valid(&sn65dsi84->lvds_vdd_en_gpio))
			gpio_free(gpio_get_number(&sn65dsi84->lvds_vdd_en_gpio));
		if (dm_gpio_is_valid(&sn65dsi84->pwr_source_gpio))
			gpio_free(gpio_get_number(&sn65dsi84->pwr_source_gpio));
		if (dm_gpio_is_valid(&sn65dsi84->sn65dsi84_en_gpio))
			gpio_free(gpio_get_number(&sn65dsi84->sn65dsi84_en_gpio));

		switch_to_lvds = false;
		ret = -ENODEV;
		return ret;
	}

	g_sn65dsi84 = sn65dsi84;

	return 0;
}

static const struct udevice_id sn65dsi84_of_match[] = {
	{ .compatible = "sn65dsi84" },
	{}
};

U_BOOT_DRIVER(sn65dsi84) = {
	.name = "sn65dsi84",
	.id = UCLASS_I2C_GENERIC,
	.of_match = sn65dsi84_of_match,
	.probe = sn65dsi84_probe,
	.bind = dm_scan_fdt_dev,
	.priv_auto_alloc_size = sizeof(struct sn65dsi84_data),
};
