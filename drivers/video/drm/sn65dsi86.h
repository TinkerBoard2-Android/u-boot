/*
 * Analog Devices ADV7511 HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef __SN65DSI86_H__
#define __SN65DSI86_H__

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
#include <linux/drm_dp_helper.h>
#include <drm/drm_mipi_dsi.h>


struct drm_dp_aux_msg {
	unsigned int address;
	u8 request;
	u8 reply;
	void *buffer;
	size_t size;
};

struct sn65dsi86_data {
	//struct backlight_device *backlight;
	//struct i2c_client *client;
	///struct device *dev;
	struct udevice *dev;
	struct udevice *sn65dsi86_i2c_dev;
	unsigned int addr;
	int i2c_id;

	unsigned int dsi_lanes;
	unsigned int width_mm;
	unsigned int height_mm;
	bool test_pattern_en;
	bool status;
	bool enabled;

	bool powered;

	struct gpio_desc edp_vdd_en_gpio;
	struct gpio_desc sn65dsi86_en_gpio;
	struct gpio_desc pwr_source_gpio;

	struct drm_display_mode mode;

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
	unsigned int t8;
	unsigned int t12;
	unsigned int t14;
	unsigned int t15;
	unsigned int t16;
	unsigned int t17;

	int				dp_lanes;
	u8				ln_assign;
	u8				ln_polrs;
};

#define SN_DEVICE_REV_REG			0x08
#define SN_DPPLL_SRC_REG			0x0A
#define  DPPLL_CLK_SRC_DSICLK			BIT(0)
#define  REFCLK_FREQ_MASK			GENMASK(3, 1)
#define  REFCLK_FREQ_27M			0x3
#define  REFCLK_FREQ(x)				((x) << 1)
#define  DPPLL_SRC_DP_PLL_LOCK			BIT(7)
#define SN_PLL_ENABLE_REG			0x0D
#define SN_DSI_LANES_REG			0x10
#define  CHA_DSI_LANES_MASK			GENMASK(4, 3)
#define  CHA_DSI_LANES(x)			((x) << 3)
#define SN_DSIA_CLK_FREQ_REG			0x12
#define SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG	0x20
#define SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG	0x24
#define SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG	0x2C
#define SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG	0x2D
#define  CHA_HSYNC_POLARITY			BIT(7)
#define SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG	0x30
#define SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG	0x31
#define  CHA_VSYNC_POLARITY			BIT(7)
#define SN_CHA_HORIZONTAL_BACK_PORCH_REG	0x34
#define SN_CHA_VERTICAL_BACK_PORCH_REG		0x36
#define SN_CHA_HORIZONTAL_FRONT_PORCH_REG	0x38
#define SN_CHA_VERTICAL_FRONT_PORCH_REG		0x3A
#define TEST_PATTERN		0x3C
#define COLOR_BAR_EN		BIT(4)
#define SN_LN_ASSIGN_REG			0x59
#define  LN_ASSIGN_WIDTH			2
#define SN_ENH_FRAME_REG			0x5A
#define  AUTHEN_METHOD_MASK			GENMASK(0, 1)
#define  SCRAMBLER_SEED_RESET			BIT(0)
#define  ENH_FRAME_ENABLE				BIT(2)
#define  VSTREAM_ENABLE				BIT(3)
#define  LN_POLRS_OFFSET			4
#define  LN_POLRS_MASK				0xf0
#define SN_DATA_FORMAT_REG			0x5B
#define  BPP_18_RGB				BIT(0)
#define SN_HPD_DISABLE_REG			0x5C
#define  HPD_DISABLE				BIT(0)
#define SN_GPIO_IO_REG				0x5E
#define  SN_GPIO_INPUT_SHIFT			4
#define  SN_GPIO_OUTPUT_SHIFT			0
#define SN_GPIO_CTRL_REG			0x5F
#define  SN_GPIO_MUX_INPUT			0
#define  SN_GPIO_MUX_OUTPUT			1
#define  SN_GPIO_MUX_SPECIAL			2
#define  SN_GPIO_MUX_MASK			0x3
#define SN_I2C_ADDR_CLAIM1			0x60
#define I2C_CLAM1_EN_MASK			0x1
#define I2C_CLAM1_EN				BIT(0)
#define SN_AUX_WDATA_REG(x)			(0x64 + (x))
#define SN_AUX_ADDR_19_16_REG			0x74
#define SN_AUX_ADDR_15_8_REG			0x75
#define SN_AUX_ADDR_7_0_REG			0x76
#define SN_AUX_LENGTH_REG			0x77
#define SN_AUX_CMD_REG				0x78
#define  AUX_CMD_SEND				BIT(0)
#define  AUX_CMD_REQ(x)				((x) << 4)
#define SN_AUX_RDATA_REG(x)			(0x79 + (x))
#define SN_SSC_CONFIG_REG			0x93
#define  DP_NUM_LANES_MASK			GENMASK(5, 4)
#define  DP_NUM_LANES(x)			((x) << 4)
#define SN_DATARATE_CONFIG_REG			0x94
#define  DP_DATARATE_MASK			GENMASK(7, 5)
#define  DP_DATARATE(x)				((x) << 5)
#define SN_ML_TX_MODE_REG			0x96
#define  ML_TX_MAIN_LINK_OFF			0
#define  ML_TX_NORMAL_MODE			BIT(0)
#define SN_AUX_CMD_STATUS_REG			0xF4
#define  AUX_IRQ_STATUS_AUX_RPLY_TOUT		BIT(3)
#define  AUX_IRQ_STATUS_AUX_SHORT		BIT(5)
#define  AUX_IRQ_STATUS_NAT_I2C_FAIL		BIT(6)

#define MIN_DSI_CLK_FREQ_MHZ   40
#define DSI_CLK_FREQ_INCREMENT 5

/* fudge factor required to account for 8b/10b encoding */
#define DP_CLK_FUDGE_NUM	10
#define DP_CLK_FUDGE_DEN	8

/* Matches DP_AUX_MAX_PAYLOAD_BYTES (for now) */
#define SN_AUX_MAX_PAYLOAD_BYTES	16

#define SN_REGULATOR_SUPPLY_NUM		4

#define SN_MAX_DP_LANES			4
#define SN_NUM_GPIOS			4
#define SN_GPIO_PHYSICAL_OFFSET		1


#define SN_IRQ_EN 0xE0
#define SN_IRQ_EN_1 0xE1
#define SN_IRQ_EN_2 0xE2
#define SN_IRQ_EN_3 0xE3
#define SN_IRQ_EN_4 0xE4
#define SN_IRQ_EN_5 0xE5
#define SN_IRQ_EN_6 0xE6
#define SN_IRQ_EN_7 0xE7
#define SN_IRQ_EN_8 0xE8
#define SN_IRQ_EN_9 0xE9
#define SN_IRQ_STATUS0 0xF0
#define SN_IRQ_STATUS1 0xF1
#define SN_IRQ_STATUS2 0xF2
#define SN_IRQ_STATUS3 0xF3
#define SN_IRQ_STATUS4 0xF4
#define SN_IRQ_STATUS5 0xF5
#define SN_IRQ_STATUS6 0xF6
#define SN_IRQ_STATUS7 0xF7
#define SN_IRQ_STATUS8 0xF8
#define  SN_PWM_PRE_DIV			0xA0
#define  SN_BACKLIGHT_SCAL_LOW 	0xA1
#define  SN_BACKLIGHT_SCAL_HIGH 	0xA2
#define  SN_BACKLIGHT_LOW 			0xA3
#define  SN_BACKLIGHT_HIGH			0xA4
#define  SN_PWM_EN 					0xA5
#define  PWM_EN_OFFSET		1
#define  PWM_EN_MASK		0x1
#define  SN_GPIO4_CTL_OFFSET		6
#define  SN_GPIO4_MUX_PWM			0x2


#define MAX_BRIGHENESS 		(255)
#define DDC_ADDR		0x50
#define EDID_SIZE		128

#endif /* __DRM_I2C_ADV7511_H__ */
