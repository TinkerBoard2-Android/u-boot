/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2008-2018 Fuzhou Rockchip Electronics Co., Ltd
 */

#ifndef __SN65DSI84_H__
#define __SN65DSI84_H__

//Rk618.h (u-boot\drivers\video\drm)	2984	2021/2/2
#include <common.h>
#include <dm.h>
#include <fdtdec.h>
#include <fdt_support.h>
#include <linux/libfdt.h>
#include <dm/of_access.h>
#include <dm/of_addr.h>
#include <dm/ofnode.h>
#include <linux/err.h>
#include <linux/ioport.h>

//Rk618.h (u-boot\drivers\video\drm)
#include <asm/gpio.h>

//Rockchip_display.h (u-boot\drivers\video\drm)
#include <drm_modes.h>

#define SN_SOFT_RESET			0x09
#define SN_CLK_SRC				0x0a
#define SN_CLK_DIV				0x0b
#define SN_PLL_EN				0x0d
#define SN_DSI_LANES			0x10
#define SN_DSI_EQ				0x11
#define SN_DSI_CLK				0x12
#define SN_FORMAT				0x18
#define SN_LVDS_VOLTAGE		0x19
#define SN_LVDS_TERM			0x1a
#define SN_LVDS_CM_VOLTAGE	0x1b
#define SN_HACTIVE_LOW			0x20
#define SN_HACTIVE_HIGH		0x21
#define SN_VACTIVE_LOW			0x24
#define SN_VACTIVE_HIGH		0x25
#define SN_SYNC_DELAY_LOW		0x28
#define SN_SYNC_DELAY_HIGH	0x29
#define SN_HSYNC_LOW			0x2c
#define SN_HSYNC_HIGH			0x2d
#define SN_VSYNC_LOW			0x30
#define SN_VSYNC_HIGH			0x31
#define SN_HBP					0x34
#define SN_VBP					0x36
#define SN_HFP					0x38
#define SN_VFP					0x3a
#define SN_TEST_PATTERN		0x3c
#define SN_IRQ_EN				0xe0
#define SN_IRQ_MASK				0xe1
#define SN_IRQ_STAT				0xe5

#define CHA_DSI_LANES (3)

/*SN_CLK_DIV*/
#define DSI_CLK_DIVIDER 			(3)

/*SN_CLK_SRC*/
#define LVDS_CLK_RANGE_OFFSET		(1)
#define HS_CLK_SRC_OFFSET 			(0)

/*SN_FORMAT	*/
#define DE_NEG_POLARITY_OFFSET 	(7)
#define HS_NEG_POLARITY_OFFSET 	(6)
#define VS_NEG_POLARITY_OFFSET 	(5)
#define LVDS_LINK_CFG_OFFSET 		(4)
#define CHA_24BPP_MODE_OFFSET 	(3)
#define CHB_24BPP_MODE_OFFSET 	(2)
#define CHA_24BPP_FMT1_OFFSET 	(1)
#define CHB_24BPP_FMT1_OFFSET 	(0)

#define MULTIPLY_BY_1	(1)
#define MULTIPLY_BY_2	(2)
#define MULTIPLY_BY_3	(3)
#define MULTIPLY_BY_4	(4)

struct videomode {
	unsigned long pixelclock;	/* pixelclock in Hz */

	u32 hactive;
	u32 hfront_porch;
	u32 hback_porch;
	u32 hsync_len;

	u32 vactive;
	u32 vfront_porch;
	u32 vback_porch;
	u32 vsync_len;

	enum display_flags flags; /* display flags */
};

struct sn65dsi84_data {
	//struct backlight_device *backlight;
	//struct i2c_client *client;
	struct udevice *dev;
	struct udevice *sn65dsi84_i2c_dev;
	unsigned int addr;
	int i2c_id;

	unsigned int lvds_clk_rate;
	unsigned int dsi_lanes;
	unsigned int lvds_format;;
	unsigned int lvds_bpp;
	unsigned int width_mm;
	unsigned int height_mm;
	unsigned int sync_delay;
	unsigned int refclk_multiplier;
	unsigned int lvds_voltage;
	bool test_pattern_en;
	bool dual_link;
	bool clk_from_refclk;
	bool enabled;
	bool debug;
	bool status;
	bool powered;

	struct gpio_desc lvds_vdd_en_gpio;
	struct gpio_desc sn65dsi84_en_gpio;
	struct gpio_desc pwr_source_gpio;

	struct videomode  vm;

	unsigned int bus_format;
	unsigned int bpc;
	unsigned int format;
	unsigned int mode_flags;
	unsigned int t1;
	unsigned int t2;
	unsigned int t3;
	unsigned int t4;
	unsigned int t5;
	unsigned int t6;
	unsigned int t7;
};

#endif

