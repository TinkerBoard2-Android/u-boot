/*
 * Analog Devices sn65dsi86 HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <common.h>
#include <i2c.h>
#include <errno.h>
#include <dm.h>
#include <dm/uclass.h>
#include <dm/uclass-id.h>

#include "sn65dsi86.h"
#include <asm/gpio.h>

#define DP_ALTERNATE_SCRAMBLER_RESET_ENABLE (1 << 0)

#define DP_EDP_DPCD_REV			    0x700    /* eDP 1.2 */
#define DP_EDP_11			    0x00
#define DP_EDP_12			    0x01
#define DP_EDP_13			    0x02
#define DP_EDP_14			    0x03

#define DP_SUPPORTED_LINK_RATES		    0x010 /* eDP 1.4 */
#define DP_MAX_SUPPORTED_RATES		    8

static bool sn65dsi86_exist = false;
struct sn65dsi86_data *g_sn65dsi86 = NULL;

static ssize_t sn65dsi86_read_edid(struct sn65dsi86_data *pdata);
static void sn65dsi86_dump_status_register(struct sn65dsi86_data *sn65dsi86);
ssize_t sn65dsi86_aux_transfer(struct sn65dsi86_data *pdata, struct drm_dp_aux_msg *msg);

bool sn65dsi86_is_connected(void)
{
	return sn65dsi86_exist;
}
EXPORT_SYMBOL_GPL(sn65dsi86_is_connected);

void edp_convertBoard_power_on(struct sn65dsi86_data *sn65dsi86)
{
	if (dm_gpio_is_valid(&sn65dsi86->pwr_source_gpio)) {
		dm_gpio_set_value(&sn65dsi86->pwr_source_gpio, 1);
		mdelay(20);
	}
}

void edp_convertBoard_power_off(struct sn65dsi86_data *sn65dsi86)
{
	if (dm_gpio_is_valid(&sn65dsi86->pwr_source_gpio)) {
		dm_gpio_set_value(&sn65dsi86->pwr_source_gpio, 0);
	}
}

void edp_power_on(struct sn65dsi86_data *sn65dsi86)
{
	if (dm_gpio_is_valid(&sn65dsi86->edp_vdd_en_gpio)) {
		dm_gpio_set_value(&sn65dsi86->edp_vdd_en_gpio, 1);
		//msleep(20);//T2: 0.01ms ~50ms
	}
}

void edp_power_off(struct sn65dsi86_data *sn65dsi86)
{
	if (dm_gpio_is_valid(&sn65dsi86->edp_vdd_en_gpio)) {
		//msleep(10);//T5: 0.01ms ~50ms
		dm_gpio_set_value(&sn65dsi86->edp_vdd_en_gpio, 0);
		//msleep(1000);//T7: 1000ms
	}
}

void sn65dsi86_chip_enable(struct sn65dsi86_data *sn65dsi86)
{
	if (dm_gpio_is_valid(&sn65dsi86->sn65dsi86_en_gpio)) {
		dm_gpio_set_value(&sn65dsi86->sn65dsi86_en_gpio, 1);
		mdelay(10);
	}

	sn65dsi86->powered = true;
}

void sn65dsi86_chip_shutdown(struct sn65dsi86_data *sn65dsi86)
{
	if (dm_gpio_is_valid(&sn65dsi86->sn65dsi86_en_gpio)) {
		dm_gpio_set_value(&sn65dsi86->sn65dsi86_en_gpio, 0);
		mdelay(10);
	}

	sn65dsi86->powered = false;
}

int sn65dsi86_read(struct udevice *dev, int reg, uint8_t *val)
{
	#define SN65DSI86_I2C_READ_RETRY_COUNT (3)
	int ret = 0;
	int i = 0;

	do {
		ret = dm_i2c_reg_read(dev, reg);
		if (ret < 0) {
			printf("sn65dsi86_i2c_reg_read fail, i = %d  reg = %x ret = %d\n", i, reg, ret);
			mdelay(20);
		} else
			break;
	} while ((++i <= SN65DSI86_I2C_READ_RETRY_COUNT) && (ret < 0));

	if (ret >= 0) {
		*val = ret;
		return 0;
	}

	return ret;
}

int  sn65dsi86_write(struct udevice *dev, u8 reg, u8 value)
{
	#define SN65DSI86_I2C_WRITE_RETRY_COUNT (6)
	int ret = 0;
	int i = 0;

	do {
		ret = dm_i2c_reg_write(dev, reg, value);
		if (ret < 0) {
			printf("sn65dsi86_write reg = %x value = %x  i = %d ret = %d\n", reg, value, i, ret);
			mdelay(20);
		}
	} while ((++i <= SN65DSI86_I2C_WRITE_RETRY_COUNT) && (ret < 0));

	return ret;
}

int sn65dsi86_update_bit(struct udevice *dev, u8 reg, u8 mask, u8 data)
{
	u8 buf, tmp;
	int ret;

	ret = sn65dsi86_read(dev, reg, &buf);
	if (ret < 0) {
		printf("failed to read 0x%.2x\n", reg);
		return ret;
	}

	tmp = buf;
	tmp &= ~mask;
	tmp |= data & mask;

	return sn65dsi86_write(dev, reg, tmp);
}

#define sn65dsi86_read_poll_timeout(dev, addr, val, cond, sleep_ms, timeout_ms) \
({ \
	unsigned int  timeout_cont = 0; \
	int ret; \
	for (;;) { \
		ret = sn65dsi86_read((dev), (addr), &(val)); \
		if (ret) \
			break; \
		if (cond) \
			break; \
		if (timeout_ms && (timeout_cont > timeout_ms)) { \
			ret = sn65dsi86_read((dev), (addr), &(val)); \
			break; \
		} \
		if (sleep_ms) \
			mdelay(sleep_ms); \
		timeout_cont += sleep_ms;\
	} \
	ret ?: ((cond) ? 0 : -110); \
})

bool sn65dsi86_detect(struct sn65dsi86_data *sn65dsi86)
{
	#define ID_REGISTERS_SZIE (8)
	uint8_t id[ID_REGISTERS_SZIE] = {0x36, 0x38, 0x49, 0x53, 0x44, 0x20, 0x20, 0x20};
	uint8_t return_id[ID_REGISTERS_SZIE] = {0};
	int i;

	if (sn65dsi86->status)
		return sn65dsi86->status;

	for (i = 0; i < sizeof(id) /sizeof(uint8_t); i++) {
		sn65dsi86_read(sn65dsi86->sn65dsi86_i2c_dev, i, &return_id[i]);
	}

	if (!memcmp(id, return_id, sizeof(id) /sizeof(uint8_t))) {
		printf("sn65dsi86_detect successful\n");
		sn65dsi86->status = true;
	} else {
		printf("sn65dsi86_detect fail, 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
			return_id[0], return_id[1], return_id[2], return_id[3], return_id[4], return_id[5], return_id[6], return_id[7]);
		sn65dsi86->status = false;
	}

	return sn65dsi86->status;
}


void sn65dsi86_bridge_disable(void)
{
	struct sn65dsi86_data *pdata = g_sn65dsi86;

	printf("%s pdata->enabled =%d +\n", __func__, pdata->enabled);
	if (!pdata->enabled)
		return;

	/* disable video stream */
	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_ENH_FRAME_REG, VSTREAM_ENABLE, 0);
	/* semi auto link training mode OFF */
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_ML_TX_MODE_REG, 0);
	/* disable DP PLL */
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_PLL_ENABLE_REG, 0);

	sn65dsi86_chip_shutdown(pdata);

	//drm_panel_unprepare(pdata->panel);
	edp_power_off(pdata);
	mdelay(pdata->t12);
	pdata->enabled = false;

	printf("%s -\n", __func__);
}

u32 sn65dsi86_get_dsi_freq(struct sn65dsi86_data *pdata)
{
	u32 bit_rate_khz, clk_freq_khz;
	int clock = pdata->mode.clock;

	bit_rate_khz = clock *
			mipi_dsi_pixel_format_to_bpp(pdata->format);
	clk_freq_khz = bit_rate_khz / (pdata->dsi_lanes * 2);

	printf("sn65dsi86_get_dsi_freq clock=%d bit_rate_khz=%u  clk_freq_khz=%u \n", clock, bit_rate_khz, clk_freq_khz);
	return clk_freq_khz;
}

/* clk frequencies supported by bridge in Hz in case derived from DACP/N pin */
static const u32 sn65dsi86_dsiclk_lut[] = {
	468000000,
	384000000,
	416000000,
	486000000,
	460800000,
};

void sn65dsi86_set_refclk_freq(struct sn65dsi86_data *pdata)
{
	int i;
	u32 refclk_rate;
	const u32 *refclk_lut;
	size_t refclk_lut_size;

	refclk_rate = sn65dsi86_get_dsi_freq(pdata) * 1000;
	refclk_lut = sn65dsi86_dsiclk_lut;
	refclk_lut_size = ARRAY_SIZE(sn65dsi86_dsiclk_lut);

	/* for i equals to refclk_lut_size means default frequency */
	for (i = 0; i < refclk_lut_size; i++)
		if (refclk_lut[i] == refclk_rate)
			break;
	printf("sn65dsi86_set_refclk_freq REFCLK_FREQ(REFCLK_FREQ_27M)=%x \n", REFCLK_FREQ(REFCLK_FREQ_27M));
	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_DPPLL_SRC_REG, REFCLK_FREQ_MASK,
			   REFCLK_FREQ(REFCLK_FREQ_27M)/*REFCLK_FREQ(i)*/);
}

void sn65dsi86_set_dsi_rate(struct sn65dsi86_data *pdata)
{
	unsigned int bit_rate_mhz, clk_freq_mhz;
	unsigned int val;
	int clock = pdata->mode.clock;

	/* set DSIA clk frequency */
	bit_rate_mhz = (clock / 1000) *
			mipi_dsi_pixel_format_to_bpp(pdata->format);
	clk_freq_mhz = bit_rate_mhz / (pdata->dsi_lanes * 2);

	/* for each increment in val, frequency increases by 5MHz */
	if (clk_freq_mhz  >= MIN_DSI_CLK_FREQ_MHZ)
		val = clk_freq_mhz / DSI_CLK_FREQ_INCREMENT;
	else
		val = 0;

	printf("sn65dsi86_set_dsi_rate clock=%d bit_rate_mhz=%u  clk_freq_mhz=%u val=%x \n", clock, bit_rate_mhz, clk_freq_mhz, val);

	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_DSIA_CLK_FREQ_REG, val);
}


unsigned int sn65dsi86_get_bpp(struct sn65dsi86_data *pdata)
{
	printf("sn65dsi86_get_bpp = %u \n", pdata->bpc);

	if (pdata->bpc <= 6)
		return 18;
	else
		return 24;
}

/*
 * LUT index corresponds to register value and
 * LUT values corresponds to dp data rate supported
 * by the bridge in Mbps unit.
 */
static const unsigned int sn65dsi86_dp_rate_lut[] = {
	0, 1620, 2160, 2430, 2700, 3240, 4320, 5400
};

int sn65dsi86_calc_min_dp_rate_idx(struct sn65dsi86_data *pdata)
{
	unsigned int bit_rate_khz, dp_rate_mhz;
	unsigned int i;
	int clock = pdata->mode.clock;

	/* Calculate minimum bit rate based on our pixel clock. */
	bit_rate_khz = clock * sn65dsi86_get_bpp(pdata);

	/* Calculate minimum DP data rate, taking 80% as per DP spec */
	dp_rate_mhz = DIV_ROUND_UP(bit_rate_khz * DP_CLK_FUDGE_NUM,
				   1000 * pdata->dp_lanes * DP_CLK_FUDGE_DEN);

	for (i = 1; i < ARRAY_SIZE(sn65dsi86_dp_rate_lut) - 1; i++)
		if (sn65dsi86_dp_rate_lut[i] >= dp_rate_mhz)
			break;
	printf("sn65dsi86_calc_min_dp_rate_idx bit_rate_khz=%u dp_rate_mhz=%u  pdata->dp_lanes =%d i=%u\n", bit_rate_khz, dp_rate_mhz,pdata->dp_lanes , i);
	return i;
}

void sn65dsi86_read_valid_rates(struct sn65dsi86_data *pdata,
					  bool rate_valid[])
{
	struct drm_dp_aux_msg msg;
	unsigned int rate_per_200khz;
	unsigned int rate_mhz;
	u8 dpcd_val;
	int ret;
	int i, j;

	msg.address = DP_EDP_DPCD_REV;
	msg.request = DP_AUX_NATIVE_READ;
	msg.buffer = &dpcd_val;
	msg.size = sizeof(dpcd_val);
	//ret = drm_dp_dpcd_readb(&pdata->aux, DP_EDP_DPCD_REV, &dpcd_val);
	ret = sn65dsi86_aux_transfer(pdata, &msg);
	if (ret != 1) {
		printf("sn65dsi86_read_valid_rates:Can't read eDP rev (%d), assuming 1.1\n", ret);
		dpcd_val = DP_EDP_11;
	}
	printf("sn65dsi86_read_valid_rates: eDP rev (%u)\n",dpcd_val);

	if (dpcd_val >= DP_EDP_14) {
		/* eDP 1.4 devices must provide a custom table */
		__le16 sink_rates[DP_MAX_SUPPORTED_RATES];

		msg.address = DP_SUPPORTED_LINK_RATES;
		msg.request = DP_AUX_NATIVE_READ;
		msg.buffer = &sink_rates;
		msg.size = sizeof(sink_rates);
		ret = sn65dsi86_aux_transfer(pdata, &msg);
		//ret = drm_dp_dpcd_read(&pdata->aux, DP_SUPPORTED_LINK_RATES,
			//	       sink_rates, sizeof(sink_rates));

		if (ret != sizeof(sink_rates)) {
			printf("sn65dsi86_read_valid_rates: Can't read supported rate table (%d)\n", ret);

			/* By zeroing we'll fall back to DP_MAX_LINK_RATE. */
			memset(sink_rates, 0, sizeof(sink_rates));
		}

		for (i = 0; i < ARRAY_SIZE(sink_rates); i++) {
			rate_per_200khz = le16_to_cpu(sink_rates[i]);

			if (!rate_per_200khz)
				break;

			rate_mhz = rate_per_200khz * 200 / 1000;
			for (j = 0;
			     j < ARRAY_SIZE(sn65dsi86_dp_rate_lut);
			     j++) {
				if (sn65dsi86_dp_rate_lut[j] == rate_mhz)
					rate_valid[j] = true;
			}
		}

		for (i = 0; i < ARRAY_SIZE(sn65dsi86_dp_rate_lut); i++) {
			if (rate_valid[i])
				return;
		}
		printf("sn65dsi86_read_valid_rates: No matching eDP rates in table; falling back\n");
	}

	/* On older versions best we can do is use DP_MAX_LINK_RATE */
	msg.address = DP_MAX_LINK_RATE;
	msg.request = DP_AUX_NATIVE_READ;
	msg.buffer = &dpcd_val;
	msg.size = sizeof(dpcd_val);
	ret = sn65dsi86_aux_transfer(pdata, &msg);
	//ret = drm_dp_dpcd_readb(&pdata->aux, DP_MAX_LINK_RATE, &dpcd_val);
	if (ret != 1) {
		printf("sn65dsi86_read_valid_rates: Can't read max rate (%d); assuming 5.4 GHz\n",
			      ret);
		dpcd_val = DP_LINK_BW_5_4;
	}

	printf("sn65dsi86_read_valid_rates: dpcd_val=0x%x\n", dpcd_val);

	switch (dpcd_val) {
	default:
		printf("sn65dsi86_read_valid_rates: Unexpected max rate (%#x); assuming 5.4 GHz\n",
			      (int)dpcd_val);
		//fallthrough;
	case DP_LINK_BW_5_4:
		rate_valid[7] = 1;
		//fallthrough;
	case DP_LINK_BW_2_7:
		rate_valid[4] = 1;
		//fallthrough;
	case DP_LINK_BW_1_62:
		rate_valid[1] = 1;
		break;
	}
}

void sn65dsi86_set_video_timings(struct sn65dsi86_data *pdata)
{
	struct drm_display_mode *mode = &pdata->mode;
	u8 hsync_polarity = 0, vsync_polarity = 0;

	printf("sn65dsi86_set_video_timings mode->flags=%x\n", mode->flags);
	printf("sn65dsi86_set_video_timings hdisplay=%d hsync_start=%d hsync_end=%d htotal=%d \n", mode->hdisplay, mode->hsync_start, mode->hsync_end, mode->htotal);
	printf("sn65dsi86_set_video_timings vdisplay=%d vsync_start=%d vsync_end=%d vtotal=%d \n", mode->vdisplay, mode->vsync_start, mode->vsync_end, mode->vtotal);
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		hsync_polarity = CHA_HSYNC_POLARITY;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		vsync_polarity = CHA_VSYNC_POLARITY;

	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG,
			       mode->hdisplay & 0xFF);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG + 1,
			       mode->hdisplay >> 8);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG,
			       mode->vdisplay & 0xFF);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG + 1,
			       mode->vdisplay >> 8);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG,
		     (mode->hsync_end - mode->hsync_start) & 0xFF);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG,
		     (((mode->hsync_end - mode->hsync_start) >> 8) & 0x7F) |
		     hsync_polarity);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG,
		     (mode->vsync_end - mode->vsync_start) & 0xFF);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG,
		     (((mode->vsync_end - mode->vsync_start) >> 8) & 0x7F) |
		     vsync_polarity);

	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_HORIZONTAL_BACK_PORCH_REG,
		     (mode->htotal - mode->hsync_end) & 0xFF);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_VERTICAL_BACK_PORCH_REG,
		     (mode->vtotal - mode->vsync_end) & 0xFF);

	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_HORIZONTAL_FRONT_PORCH_REG,
		     (mode->hsync_start - mode->hdisplay) & 0xFF);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_CHA_VERTICAL_FRONT_PORCH_REG,
		     (mode->vsync_start - mode->vdisplay) & 0xFF);

	mdelay(10); /* 10ms delay recommended by spec */
}

static unsigned int sn65dsi86_get_max_lanes(struct sn65dsi86_data *pdata)
{
	struct drm_dp_aux_msg msg;
	ssize_t  ret;
	u8 data;

	msg.address = DP_MAX_LANE_COUNT;
	msg.request = DP_AUX_NATIVE_READ;
	msg.buffer = &data;
	msg.size = sizeof(data);

	//ret = drm_dp_dpcd_readb(&pdata->aux, DP_MAX_LANE_COUNT, &data);
	ret = sn65dsi86_aux_transfer(pdata, &msg);
	if (ret != 1) {
		printf("error: sn65dsi86_get_max_lanes: Can't read lane count (%ld); assuming 4\n", ret);
		return 4;
	}
	printf("sn65dsi86_get_max_lanes: ret=%ld lane=%x\n", ret, data & DP_LANE_COUNT_MASK);
	return data & DP_LANE_COUNT_MASK;
}
int sn65dsi86_link_training(struct sn65dsi86_data *pdata, int dp_rate_idx,
			       const char **last_err_str)
{
	uint8_t PLL_result = 0;
	uint8_t val;
	int ret;

	*last_err_str = "Link training successful";

	/* set dp clk frequency value */
	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_DATARATE_CONFIG_REG, DP_DATARATE_MASK, DP_DATARATE(dp_rate_idx));

	/* enable DP PLL */
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_PLL_ENABLE_REG, 1);
	mdelay(10);
	sn65dsi86_read(pdata->sn65dsi86_i2c_dev, 0x0A, &PLL_result);
	printf("sn65dsi86_link_training: PLL_result=0x%x (%s)\n", PLL_result, PLL_result & BIT(7) ? "DP PLL locked" : "DP PLL not locked");

	ret = sn65dsi86_read_poll_timeout(pdata->sn65dsi86_i2c_dev, SN_DPPLL_SRC_REG, val,
				       val & DPPLL_SRC_DP_PLL_LOCK, 1,
				       50);
	if (ret) {
		*last_err_str = "DP_PLL_LOCK polling failed";
		goto exit;
	}

	/* Semi auto link training mode */
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_ML_TX_MODE_REG, 0x0A);
	ret = sn65dsi86_read_poll_timeout(pdata->sn65dsi86_i2c_dev, SN_ML_TX_MODE_REG, val,
				       val == ML_TX_MAIN_LINK_OFF ||
				       val == ML_TX_NORMAL_MODE, 1,
				       500);
	if (ret) {
		*last_err_str = "Training complete polling failed";
	} else if (val == ML_TX_MAIN_LINK_OFF) {
		*last_err_str = "Link training failed, link is off";
		ret = -EIO;
	}

exit:
	/* Disable the PLL if we failed */
	if (ret)
		sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_PLL_ENABLE_REG, 0);

	return ret;
}

void showDPCDInfo(struct sn65dsi86_data *pdata)
{
	#define ASSR_SUPPORT				(1<<0)
	#define ENHANCE_FRAMING			(1<<1)
	#define DPCD_DISPLAY_CONTORL_CAP   (1<<3)
	struct drm_dp_aux_msg msg;
	uint8_t buf[16];

	msg.address = DP_DPCD_REV ;
	msg.request = DP_AUX_NATIVE_READ;
	msg.buffer = buf;
	msg.size = sizeof(buf);
	//drm_dp_dpcd_read(&pdata->aux, 0, buf, sizeof(buf));
	sn65dsi86_aux_transfer(pdata, &msg);

	printf("DPCD: REV:%d.%d, MAX_LINK_RATE:", (buf[0] >> 4), (buf[0]&0xF));
	if (buf[1] == 0x06) {
		printf("1.62Gbps");
	} else if (buf[1] == 0x0A) {
		printf("2.7Gbps");
	}
	printf(" MAX_LINK_LANE:%d\n", buf[2] & DP_LANE_COUNT_MASK);
	if (buf[0x0D] & ASSR_SUPPORT) {
		printf(" support ASSR");
	} else {
		printf(" not support ASSR");
	}
	if (buf[0x0D] & ENHANCE_FRAMING) {
		printf(" support Enhance framing");
	} else {
		printf(" not support Enhance framing");
	}
	printf("\n");
}

void sn65dsi86_bridge_enable(void )
{
	struct sn65dsi86_data *pdata = g_sn65dsi86;
	bool rate_valid[ARRAY_SIZE(sn65dsi86_dp_rate_lut)] = { };
	const char *last_err_str = "No supported DP rate";
	struct drm_dp_aux_msg msg;
	int dp_rate_idx;
	unsigned int val;
	int ret = -EINVAL;
	int max_dp_lanes;
	uint8_t training_result = 0, PLL_result = 0;
	u8 dpcd_val;

	printf("%s  pdata->enabled = %d+\n", __func__, pdata->enabled);

	if (pdata->enabled)
		return;

if (0) {
	edp_power_on(pdata);
	mdelay(pdata->t3);
	sn65dsi86_chip_enable(pdata);

	//======REFCLK Frequency  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x0A,0x6);

	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_HPD_DISABLE_REG, HPD_DISABLE,
			   HPD_DISABLE);
	showDPCDInfo(pdata);
	//======DSI Mode  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x10, 0x26);

	//======DSIA Clock  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x12, 0x55);

	//======DSIB Clock  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x13, 0x55);

	//======DP Datarate  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x94, 0x80);

	//======Enable PLL  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x0D, 0x01);
	mdelay(10);
	sn65dsi86_read(pdata->sn65dsi86_i2c_dev, 0x0A, &PLL_result);
	printf("sn65dsi86_bridge_enable: PLL_result=0x%x (%s)\n", PLL_result, PLL_result & BIT(7) ? "DP PLL locked" : "DP PLL not locked");

	//======Enable ASSR in Panel  ======
	dpcd_val =  DP_ALTERNATE_SCRAMBLER_RESET_ENABLE;
	msg.address = DP_EDP_CONFIGURATION_SET;
	msg.request = DP_AUX_NATIVE_WRITE;
	msg.buffer = &dpcd_val;
	msg.size = sizeof(dpcd_val);
	ret = sn65dsi86_aux_transfer(pdata, &msg);
	//drm_dp_dpcd_writeb(&pdata->aux, DP_EDP_CONFIGURATION_SET,
	//		   DP_ALTERNATE_SCRAMBLER_RESET_ENABLE);
	mdelay(10);

	//======Enable enhanced frame and ASSR in DSI86  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x5A, 0x05);

	//======Number of DP lanes  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x93, 0x24);

      //======Start Semi-Auto Link Training  =====
	mdelay(pdata->t4);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x96, 0x0A);
	mdelay(20);
	sn65dsi86_read(pdata->sn65dsi86_i2c_dev, 0x96, &training_result);
	printf("sn65dsi86_bridge_enable: sn65dsi86_link_training: training_result=%d(%s)\n",
		training_result, training_result == 0x1 ? "normal mode" : (training_result == 0x0 ? "main link off" : "unknow error"));

	 sn65dsi86_read(pdata->sn65dsi86_i2c_dev, 0xF8, &training_result);
	 printf("sn65dsi86_bridge_enable: sn65dsi86_link_training: training status =0x%x(%s)\n", training_result, training_result == 0x1 ? "LT_PASS" : "LT_FAIL");

	//======CHA Active Line Length  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x20, 0x80);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x21, 0x07);

	//======CHB Active Line Length  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x22, 0x0);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x23, 0x0);

	//======Vertical Active Size   ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x24, 0x38);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x25, 0x04);

	//======Horizontal Pulse Width   ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x2C, 48);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x2D, 0x00);//polarity

	//======Vertical Pulse Width   ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x30, 10);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x31, 0x00);//polarity

	//======HBP   ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x34, 24);

	//======VBP   ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x36, 26);

	//===== HFP  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x38, 108);

	//===== VFP  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x3A, 10);

	//===== DP-18BPP Disable  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x5B, 0x0);

	//===== Color Bar Enable  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x3C, 0/*0x12*/);

	//===== Enhanced Frame, ASSR, and Vstream Enable  ======
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, 0x5A, 0x0d);

	sn65dsi86_dump_status_register(pdata);
	pdata->enabled = true;
} else {
	edp_power_on(pdata);
	mdelay(pdata->t3);
	sn65dsi86_chip_enable(pdata);

	/* configure bridge ref_clk */
	sn65dsi86_set_refclk_freq(pdata);

	/*
	 * HPD on this bridge chip is a bit useless.  This is an eDP bridge
	 * so the HPD is an internal signal that's only there to signal that
	 * the panel is done powering up.  ...but the bridge chip debounces
	 * this signal by between 100 ms and 400 ms (depending on process,
	 * voltage, and temperate--I measured it at about 200 ms).  One
	 * particular panel asserted HPD 84 ms after it was powered on meaning
	 * that we saw HPD 284 ms after power on.  ...but the same panel said
	 * that instead of looking at HPD you could just hardcode a delay of
	 * 200 ms.  We'll assume that the panel driver will have the hardcoded
	 * delay in its prepare and always disable HPD.
	 *
	 * If HPD somehow makes sense on some future panel we'll have to
	 * change this to be conditional on someone specifying that HPD should
	 * be used.
	 */
	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_HPD_DISABLE_REG, HPD_DISABLE,
			   HPD_DISABLE);
	showDPCDInfo(pdata);

	max_dp_lanes = sn65dsi86_get_max_lanes(pdata);
	pdata->dp_lanes = min(pdata->dp_lanes, max_dp_lanes);
	printf("%s pdata->dp_lanes=%d\n", __func__, pdata->dp_lanes);
	/* DSI_A lane config */
	val = CHA_DSI_LANES(SN_MAX_DP_LANES - pdata->dsi_lanes);
	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_DSI_LANES_REG,
			   CHA_DSI_LANES_MASK, val);

	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_LN_ASSIGN_REG, pdata->ln_assign);
	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_ENH_FRAME_REG, LN_POLRS_MASK,
			   pdata->ln_polrs << LN_POLRS_OFFSET);

	/* set dsi clk frequency value */
	sn65dsi86_set_dsi_rate(pdata);

	/**
	 * The SN65DSI86 only supports ASSR Display Authentication method and
	 * this method is enabled by default. An eDP panel must support this
	 * authentication method. We need to enable this method in the eDP panel
	 * at DisplayPort address 0x0010A prior to link training.
	 */
	dpcd_val =  DP_ALTERNATE_SCRAMBLER_RESET_ENABLE;
	msg.address = DP_EDP_CONFIGURATION_SET;
	msg.request = DP_AUX_NATIVE_WRITE;
	msg.buffer = &dpcd_val;
	msg.size = sizeof(dpcd_val);
	ret = sn65dsi86_aux_transfer(pdata, &msg);
	//drm_dp_dpcd_writeb(&pdata->aux, DP_EDP_CONFIGURATION_SET,
	//		   DP_ALTERNATE_SCRAMBLER_RESET_ENABLE);
	mdelay(10);

	//======Enable enhanced frame and ASSR in DSI86  ======
	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_ENH_FRAME_REG, AUTHEN_METHOD_MASK,
			   SCRAMBLER_SEED_RESET);
	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_ENH_FRAME_REG, ENH_FRAME_ENABLE,
			   ENH_FRAME_ENABLE);


	/* Set the DP output format (18 bpp or 24 bpp) */
	val = (sn65dsi86_get_bpp(pdata) == 18) ? BPP_18_RGB : 0;
	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_DATA_FORMAT_REG, BPP_18_RGB, val);

	/* DP lane config */
	val = DP_NUM_LANES(min(pdata->dp_lanes, 3));
	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_SSC_CONFIG_REG, DP_NUM_LANES_MASK,
			   val);

	sn65dsi86_read_valid_rates(pdata, rate_valid);

	/* Train until we run out of rates */
	mdelay(pdata->t4);
	for (dp_rate_idx = sn65dsi86_calc_min_dp_rate_idx(pdata);
	     dp_rate_idx < ARRAY_SIZE(sn65dsi86_dp_rate_lut);
	     dp_rate_idx++) {
		if (!rate_valid[dp_rate_idx])
			continue;

		ret = sn65dsi86_link_training(pdata, dp_rate_idx, &last_err_str);
		if (!ret)
			break;
	}
	if (ret) {
		printf("sn65dsi86_bridge_enable: %s (%d)\n", last_err_str, ret);
		//return;
	} else {
		printf("sn65dsi86_bridge_enable dp_rate_idx=%d\n", dp_rate_idx);
		mdelay(20);
		sn65dsi86_read(pdata->sn65dsi86_i2c_dev, SN_ML_TX_MODE_REG, &training_result);
		printf("sn65dsi86_bridge_enable: sn65dsi86_link_training: training_result=%d(%s)\n",
			training_result, training_result == 0x1 ? "normal mode" : (training_result == 0x0 ? "main link off" : "unknow error"));

		sn65dsi86_read(pdata->sn65dsi86_i2c_dev, SN_IRQ_STATUS8, &training_result);
		printf("sn65dsi86_bridge_enable: sn65dsi86_link_training: training status =0x%x(%s)\n", training_result, training_result == 0x1 ? "LT_PASS" : "LT_FAIL");
	}

	/* config video parameters */
	sn65dsi86_set_video_timings(pdata);

	/* enable video stream */
	sn65dsi86_update_bit(pdata->sn65dsi86_i2c_dev, SN_ENH_FRAME_REG, VSTREAM_ENABLE,
			   VSTREAM_ENABLE);
	if (pdata->test_pattern_en)
		sn65dsi86_write(pdata->sn65dsi86_i2c_dev, TEST_PATTERN, COLOR_BAR_EN);

	pdata->enabled = true;

	mdelay(pdata->t8);

	sn65dsi86_dump_status_register(pdata);
}

	printf("%s -\n", __func__);
}

ssize_t sn65dsi86_aux_transfer(struct sn65dsi86_data *pdata,
				  struct drm_dp_aux_msg *msg)
{
	u32 request = msg->request & ~DP_AUX_I2C_MOT;
	u32 request_val = AUX_CMD_REQ(msg->request);
	u8 *buf = (u8 *)msg->buffer;
	int retry = 0, ret, i;
	uint8_t val;

	if (msg->size > SN_AUX_MAX_PAYLOAD_BYTES)
		return -EINVAL;

	switch (request) {
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_REG, request_val);
		break;
	default:
		return -EINVAL;
	}

	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_ADDR_19_16_REG,
		     (msg->address >> 16) & 0xF);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_ADDR_15_8_REG,
		     (msg->address >> 8) & 0xFF);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_ADDR_7_0_REG, msg->address & 0xFF);

	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_LENGTH_REG, msg->size);

	if (request == DP_AUX_NATIVE_WRITE || request == DP_AUX_I2C_WRITE) {
		for (i = 0; i < msg->size; i++)
			sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_WDATA_REG(i),
				     buf[i]);
	}

	/* Clear old status bits before start so we don't get confused */
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_STATUS_REG,
			ML_TX_NORMAL_MODE |
			AUX_IRQ_STATUS_NAT_I2C_FAIL |
			AUX_IRQ_STATUS_AUX_RPLY_TOUT |
			AUX_IRQ_STATUS_AUX_SHORT);

	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_REG, request_val | AUX_CMD_SEND);

	ret = sn65dsi86_read_poll_timeout(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_REG, val,
				       !(val & AUX_CMD_SEND), 1,
				       50);
	if (ret)
		return ret;

	do {
		ret = sn65dsi86_read(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_STATUS_REG, &val);
		if (ret)
			return ret;
		else if ((val & AUX_IRQ_STATUS_NAT_I2C_FAIL)
			|| (val & AUX_IRQ_STATUS_AUX_RPLY_TOUT)
			|| (val & AUX_IRQ_STATUS_AUX_SHORT)) {
			printf("sn65dsi86_aux_transfer: return -ENXIO val=%x\n", val);
			return -ENXIO;
		}
		mdelay(1);
	}while(!(val & ML_TX_NORMAL_MODE) && retry++ < 20);

	if (request == DP_AUX_NATIVE_WRITE || request == DP_AUX_I2C_WRITE)
		return msg->size;

	for (i = 0; i < msg->size; i++) {
		uint8_t val;
		ret = sn65dsi86_read(pdata->sn65dsi86_i2c_dev, SN_AUX_RDATA_REG(i), &val);
		if (ret)
			return ret;

		buf[i] = (u8)(val & 0xFF);
	}

	return msg->size;
}

ssize_t sn65dsi86_send_aux_cmd(struct sn65dsi86_data *pdata, u32 request_val)
{
	int retry = 0, ret;
	uint8_t val;

	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_STATUS_REG,
			ML_TX_NORMAL_MODE |
			AUX_IRQ_STATUS_NAT_I2C_FAIL |
			AUX_IRQ_STATUS_AUX_RPLY_TOUT |
			AUX_IRQ_STATUS_AUX_SHORT);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_REG, request_val | AUX_CMD_SEND);
	ret = sn65dsi86_read_poll_timeout(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_REG, val,
				       !(val & AUX_CMD_SEND), 1,
				       50);
	if (ret) {
		printf("sn65dsi86_send_aux_cmd: send cmd fail, val=%x\n", val);
	}

	do {
		ret = sn65dsi86_read(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_STATUS_REG, &val);
		if (ret)
			return ret;
		else if ((val & AUX_IRQ_STATUS_NAT_I2C_FAIL)
			|| (val & AUX_IRQ_STATUS_AUX_RPLY_TOUT)
			|| (val & AUX_IRQ_STATUS_AUX_SHORT)) {
			printf("sn65dsi86_send_aux_cmd: return -ENXIO val=%x\n", val);
			return -ENXIO;
		}
		mdelay(1);
	}while(!(val & ML_TX_NORMAL_MODE) && retry++ < 20);

	return 0;
}

ssize_t sn65dsi86_read_edid(struct sn65dsi86_data *pdata)
{
	#define AUX_CMD(x)  (x<<4)
	u8 buf[EDID_SIZE] = {0}, csum = 0;
	u32 request_val;
	int  i = 0, j = 0, index = 0;

	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_ADDR_19_16_REG, 0);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_ADDR_15_8_REG, 0);

	//step 1
	request_val = AUX_CMD(0x04);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_REG, request_val);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_ADDR_7_0_REG, 0x50);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_LENGTH_REG, 0);
	sn65dsi86_send_aux_cmd(pdata, request_val);


	//step 5
	request_val = AUX_CMD(0x04);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_REG, request_val);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_ADDR_7_0_REG, 0x50);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_LENGTH_REG, 1);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_WDATA_REG(0), 0);
	sn65dsi86_send_aux_cmd(pdata, request_val);

	//step 9
	request_val = AUX_CMD(0x05);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_REG, request_val);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_ADDR_7_0_REG, 0x50);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_LENGTH_REG, 0);
	sn65dsi86_send_aux_cmd(pdata, request_val);


	//step 13
	do {
		request_val = AUX_CMD(0x05);
		sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_REG, request_val);
		sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_ADDR_7_0_REG, 0x50);
		sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_LENGTH_REG, 0x10);
		sn65dsi86_send_aux_cmd(pdata, request_val);
		for (i = 0; i < 16; i++) {
			uint8_t val;
			sn65dsi86_read(pdata->sn65dsi86_i2c_dev, SN_AUX_RDATA_REG(i), &val);
			//if (ret)
			//	return ret;

			buf[index++] = (u8)(val & 0xFF);
		}
	}while(index < EDID_SIZE);

	//step 18
	request_val = AUX_CMD(0x01);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_CMD_REG, request_val);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_ADDR_7_0_REG, 0x50);
	sn65dsi86_write(pdata->sn65dsi86_i2c_dev, SN_AUX_LENGTH_REG, 0);
	sn65dsi86_send_aux_cmd(pdata, request_val);

	i = 0;
	for (j = 0; j < (EDID_SIZE/16); j++) {
		//printf("sn65dsi86_read_edid 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \n", buf[i+0], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7], buf[i+8], buf[i+9], buf[i+10], buf[i+11], buf[i+12], buf[i+13], buf[i+14], buf[i+15]);
		i += 16;
	}

	for (i = 0;  i < EDID_SIZE;  i++)
		csum += buf[i];

	printf("sn65dsi86_read_edid checksum =%x \n", csum);

	return index;
}

void sn65dsi86_parse_lanes(struct udevice *dev,
			   struct sn65dsi86_data *data)
{
	u32 lane_assignments[SN_MAX_DP_LANES] = { 0, 1, 2, 3 };
	u32 lane_polarities[SN_MAX_DP_LANES] = { };
	u8 ln_assign = 0;
	u8 ln_polrs = 0;
	int dp_lanes;
	int i;

	/*
	 * Read config from the device tree about lane remapping and lane
	 * polarities.  These are optional and we assume identity map and
	 * normal polarity if nothing is specified.  It's OK to specify just
	 * data-lanes but not lane-polarities but not vice versa.
	 *
	 * Error checking is light (we just make sure we don't crash or
	 * buffer overrun) and we assume dts is well formed and specifying
	 * mappings that the hardware supports.
	 */
	dp_lanes = dev_read_u32_default(dev, "data-lanes-size", 4);
	if (dp_lanes > 0 && dp_lanes <= SN_MAX_DP_LANES) {
		dev_read_u32_array(dev, "data-lanes",
				lane_assignments, dp_lanes);
		dev_read_u32_array(dev, "lane-polarities",
				lane_polarities, dp_lanes);
	} else {
		dp_lanes = SN_MAX_DP_LANES;
	}

	/*
	 * Convert into register format.  Loop over all lanes even if
	 * data-lanes had fewer elements so that we nicely initialize
	 * the LN_ASSIGN register.
	 */
	for (i = SN_MAX_DP_LANES - 1; i >= 0; i--) {
		ln_assign = ln_assign << LN_ASSIGN_WIDTH | lane_assignments[i];
		ln_polrs = ln_polrs << 1 | lane_polarities[i];
	}

	/* Stash in our struct for when we power on */
	data->dp_lanes = dp_lanes;
	data->ln_assign = ln_assign;
	data->ln_polrs = ln_polrs;

	printf("sn65dsi86_parse_lanes dp_lanes=%d ln_assign=0x%x ln_polrs=0x%x\n", dp_lanes, ln_assign, ln_polrs);
}

int sn65dsi86_parse_dt(struct udevice *dev,
			   struct sn65dsi86_data *data)
{
	int ret;

	data->i2c_id = of_alias_get_id(ofnode_to_np(dev->node)->parent, "i2c");
	if (data->i2c_id < 0) {
		printf("sn65dsi86_parse_dt: error data->i2c_id = %d, use default number 8. \n", data->i2c_id);
		data->i2c_id = 8;
	}

	data->addr = dev_read_u32_default(dev, "reg", 0x2D);
	data->dsi_lanes = dev_read_u32_default(dev, "dsi-lanes", 4);
	if (data->dsi_lanes < 1 || data->dsi_lanes > 4) {
		printf("Invalid dsi-lanes: %d\n", data->dsi_lanes);
		return -EINVAL;
	}

	data->test_pattern_en = dev_read_bool(dev,"test-pattern");
	data->bpc = dev_read_u32_default(dev, "bpc", 8);
	data->t1 = dev_read_u32_default(dev, "t1", 10);
	data->t2 = dev_read_u32_default(dev, "t2", 10);
	data->t3 = dev_read_u32_default(dev, "t3", 200);
	data->t4 = dev_read_u32_default(dev, "t4", 20);
	data->t5 = dev_read_u32_default(dev, "t5", 10);
	data->t6 = dev_read_u32_default(dev, "t6", 10);
	data->t7 = dev_read_u32_default(dev, "t7", 10);
	data->t8 = dev_read_u32_default(dev, "t8", 100);
	data->t12 = dev_read_u32_default(dev, "t12", 500);
	data->t14 = dev_read_u32_default(dev, "t14", 15);
	data->t15 = dev_read_u32_default(dev, "t15", 15);
	data->t16 = dev_read_u32_default(dev, "t16", 10);
	data->t17 = dev_read_u32_default(dev, "t17", 10);

	printf("sn65dsi86_parse_dt i2c_id =%d addr=%d t1=%u t2=%u t3=%u t4=%u t5=%u t6=%u t7=%u t8=%u t12=%u\n",
		data->i2c_id, data->addr, data->t1, data->t2, data->t3, data->t4, data->t5, data->t6, data->t7, data->t8, data->t12);

	ret = gpio_request_by_name(dev, "EN-gpios", 0, &data->sn65dsi86_en_gpio , GPIOD_IS_OUT);
	if (ret) {
		printf("Cannot get EN-gpios: %d\n", ret);
	}

	ret = gpio_request_by_name(dev, "edp_vdd_en-gpios", 0, &data->edp_vdd_en_gpio , GPIOD_IS_OUT);
	if (ret) {
		printf("Cannot get edp_vdd_en-gpios: %d\n", ret);
	}

	ret = gpio_request_by_name(dev, "pwr_source-gpios", 0, &data->pwr_source_gpio , GPIOD_IS_OUT);
	if (ret) {
		printf("Cannot get pwr_source-gpios: %d\n", ret);
	}

	printf("sn65dsi86_parse_dt  dsi_lanes=%u test_pattern_en=%u\n", data->dsi_lanes, data->test_pattern_en);

	return ret;
}

void sn65dsi86_dump_status_register(struct sn65dsi86_data *sn65dsi86)
{
	uint8_t val = 0xFF;

	sn65dsi86_read(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS0, &val);
	printf("sn65dsi86_dump_status_register: SN_IRQ_STATUS0 = %x\n", val);
	sn65dsi86_read(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS1, &val);
	printf("sn65dsi86_dump_status_register: SN_IRQ_STATUS1 = %x\n", val);
	sn65dsi86_read(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS2, &val);
	printf("sn65dsi86_dump_status_register: SN_IRQ_STATUS2 = %x\n", val);
	sn65dsi86_read(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS3, &val);
	printf("sn65dsi86_dump_status_register: SN_IRQ_STATUS3 = %x\n", val);
	sn65dsi86_read(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS4, &val);
	printf("sn65dsi86_dump_status_register: SN_IRQ_STATUS4 = %x\n", val);
	sn65dsi86_read(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS5, &val);
	printf("sn65dsi86_dump_status_register: SN_IRQ_STATUS5 = %x\n", val);
	sn65dsi86_read(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS6, &val);
	printf("sn65dsi86_dump_status_register: SN_IRQ_STATUS6 = %x\n", val);
	sn65dsi86_read(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS7, &val);
	printf("sn65dsi86_dump_status_register: SN_IRQ_STATUS7 = %x\n", val);
	sn65dsi86_read(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS8, &val);
	printf("sn65dsi86_dump_status_register: SN_IRQ_STATUS8 = %x\n", val);

	sn65dsi86_write(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS0, 0xFF);
	sn65dsi86_write(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS1, 0xFF);
	sn65dsi86_write(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS2, 0xFF);
	sn65dsi86_write(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS3, 0xFF);
	sn65dsi86_write(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS4, 0xFF);
	sn65dsi86_write(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS5, 0xFF);
	sn65dsi86_write(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS6, 0xFF);
	sn65dsi86_write(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS7, 0xFF);
	sn65dsi86_write(sn65dsi86->sn65dsi86_i2c_dev, SN_IRQ_STATUS8, 0xFF);
}

void sn65dsi86_proch_from_edid(struct sn65dsi86_data *sn65dsi86)
{
	edp_power_on(sn65dsi86);
	mdelay(sn65dsi86->t3);
	sn65dsi86_chip_enable(sn65dsi86);

	//======REFCLK Frequency  ======
	sn65dsi86_write(sn65dsi86->sn65dsi86_i2c_dev, 0x0A,0x6);
	sn65dsi86_update_bit(sn65dsi86->sn65dsi86_i2c_dev, SN_HPD_DISABLE_REG, HPD_DISABLE,
			   HPD_DISABLE);
	sn65dsi86_read_edid(sn65dsi86);
	edp_power_off(sn65dsi86);
	sn65dsi86_chip_shutdown(sn65dsi86);
}

void sn65dsi86_setup_desc( struct drm_display_mode *dmode)
{
	memcpy(&g_sn65dsi86->mode, dmode, sizeof(struct drm_display_mode));
}

static int sn65dsi86_probe(struct udevice *dev)
{
	struct sn65dsi86_data  *sn65dsi86 = dev_get_priv(dev);
	int ret;

	printf("%s +\n", __func__);

	sn65dsi86->dev = dev;
	sn65dsi86->enabled = false;
	sn65dsi86->powered = false;
	sn65dsi86->status = false;
	g_sn65dsi86 = sn65dsi86;

	if (dev->node.np) {
		if (dev->node.np->name)
			printf("sn65dsi86_probe  name=%s full_name=%s \n", dev->node.np->name, dev->node.np->full_name);
	}

	ret = sn65dsi86_parse_dt(dev, sn65dsi86);
	if (ret)
		return ret;


	i2c_get_chip_for_busnum(sn65dsi86->i2c_id, sn65dsi86->addr, 1, &sn65dsi86->sn65dsi86_i2c_dev);

	edp_convertBoard_power_on(sn65dsi86);
	sn65dsi86_chip_shutdown(sn65dsi86);
	sn65dsi86_chip_enable(sn65dsi86);
	sn65dsi86_detect(sn65dsi86);
	if (sn65dsi86->status) {
		printf("%s : sn65dsi86 is %s !\n", __func__, sn65dsi86->status ? "connected" : "disconnected");
		sn65dsi86_exist = true;
	} else {
		printf("%s : sn65dsi86 is disconnected!\n", __func__);
		if (dm_gpio_is_valid(&sn65dsi86->edp_vdd_en_gpio))
			gpio_free(gpio_get_number(&sn65dsi86->edp_vdd_en_gpio));
		if (dm_gpio_is_valid(&sn65dsi86->pwr_source_gpio))
			gpio_free(gpio_get_number(&sn65dsi86->pwr_source_gpio));
		if (dm_gpio_is_valid(&sn65dsi86->sn65dsi86_en_gpio))
			gpio_free(gpio_get_number(&sn65dsi86->sn65dsi86_en_gpio));

		sn65dsi86_exist = false;
		ret = -ENODEV;
		return ret;
	}

	sn65dsi86_parse_lanes(dev, sn65dsi86);
	sn65dsi86_proch_from_edid(sn65dsi86);
	sn65dsi86_chip_shutdown(sn65dsi86);

	printf("%s -\n", __func__);
	return 0;
}

static const struct udevice_id sn65dsi86_of_match[] = {
	{ .compatible = "sn65dsi86" },
	{}
};

U_BOOT_DRIVER(sn65dsi86) = {
	.name = "sn65dsi846",
	.id = UCLASS_I2C_GENERIC,
	.of_match = sn65dsi86_of_match,
	.probe = sn65dsi86_probe,
	.bind = dm_scan_fdt_dev,
	.priv_auto_alloc_size = sizeof(struct sn65dsi86_data),
};
