/**
 * Copyright (c) 2014, Marek Koza (qyx@krtko.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdio.h>
#include <stdlib.h>

#include "mcp3424.h"


static uint8_t mcp3424_create_config_byte(struct mcp3424 *d)
{
	if (d == NULL) {
		return 0;
	}

	/* Construct the config byte, bit 7 is always 0 (RDY). */
	uint8_t cfg = 0;
	cfg |= (uint8_t)((d->channel & 0x03) << 5);
	if (d->mode == MCP3424_MODE_CONTINUOUS) {
		cfg |= MCP3424_CONFIG_OC;
	}
	cfg |= (uint8_t)((d->rate & 0x03) << 2);
	cfg |= (uint8_t)(d->gain & 0x03);

	return cfg;
}


#define MCP3424_UPDATE_CONFIG_OK 0
#define MCP3424_UPDATE_CONFIG_FAILED -1
static int32_t mcp3424_update_config(struct mcp3424 *d)
{
	if (d == NULL) {
		return MCP3424_UPDATE_CONFIG_FAILED;
	}

	uint8_t cfg = mcp3424_create_config_byte(d);

	int32_t status = mcp3424_port_i2c_master_transmit(d, &cfg, 1, NULL, 0);

	if (status != 0) {
		return MCP3424_UPDATE_CONFIG_FAILED;
	}

	// Check written setting
	uint8_t rxbuf[4];
	status = mcp3424_port_i2c_master_transmit(d, NULL, 0, rxbuf, 4);
	if(rxbuf[3] != cfg) {
		return MCP3424_UPDATE_CONFIG_FAILED;
	}

	if (status != 0) {
		return MCP3424_UPDATE_CONFIG_FAILED;
	}

	return MCP3424_UPDATE_CONFIG_OK;
}


#define MCP3424_PROBE_FOUND 0
#define MCP3424_PROBE_NOTFOUND -1
#define MCP3424_PROBE_FAILED -2
static int32_t mcp3424_probe(struct mcp3424 *d)
{
	if (d->dev_i2c == NULL) {
		return MCP3424_PROBE_FAILED;
	}

	uint8_t rxbuf[2];

	/* Try to read anything from the device and see if it is
	 * successful */
	int32_t status = mcp3424_port_i2c_master_transmit(d, NULL, 0, rxbuf, 2);

	if (status != 0) {
		return MCP3424_PROBE_NOTFOUND;
	}

	return MCP3424_PROBE_FOUND;
}


int32_t mcp3424_init(struct mcp3424 *d, void *drv, uint8_t addr)
{
	if ((d == NULL) ||
	    (drv == NULL)) {
		return MCP3424_INIT_FAILED;
	}

	/* Initialize driver structure context. */
	d->dev_i2c = drv;
	d->addr = addr;
	d->mode = MCP3424_MODE_SINGLE_SHOT;
	d->gain = MCP3424_GAIN_1X;
	d->rate = MCP3424_RATE_3_75;
	d->channel = 1;

	/* Probe the device first. */
	if (mcp3424_probe(d) != MCP3424_PROBE_FOUND) {
		return MCP3424_INIT_FAILED;
	}

	/* And set the default config. */
	if (mcp3424_update_config(d) != MCP3424_UPDATE_CONFIG_OK) {
		return MCP3424_INIT_FAILED;
	}

	return MCP3424_INIT_OK;
}


int32_t mcp3424_free(struct mcp3424 *d)
{
	if (d == NULL) {
		return MCP3424_FREE_FAILED;
	}

	if(mcp3424_port_i2c_master_deinit(d)) {
		return MCP3424_FREE_FAILED;
	}

	/* Nothing to do here. */
	return MCP3424_FREE_OK;
}


int32_t mcp3424_start(struct mcp3424 *d)
{
	if (d == NULL) {
		return MCP3424_START_FAILED;
	}

	/* TODO */
	return MCP3424_START_FAILED;
}


int32_t mcp3424_stop(struct mcp3424 *d)
{
	if (d == NULL) {
		return MCP3424_STOP_FAILED;
	}

	/* TODO */
	return MCP3424_STOP_FAILED;
}


int32_t mcp3424_set_mode(struct mcp3424 *d, enum mcp3424_mode mode)
{
	if (d == NULL) {
		return MCP3424_SET_MODE_FAILED;
	}

	d->mode = mode;

	if (mcp3424_update_config(d) != MCP3424_UPDATE_CONFIG_OK) {
		return MCP3424_SET_MODE_FAILED;
	}

	return MCP3424_SET_MODE_OK;
}


int32_t mcp3424_convert(struct mcp3424 *d)
{
	if (d == NULL) {
		return MCP3424_CONVERT_FAILED;
	}

	/* Conversion can be initiated manually only in single shot mode. */
	if (d->mode != MCP3424_MODE_SINGLE_SHOT) {
		return MCP3424_CONVERT_FAILED;
	}

	uint8_t cfg = mcp3424_create_config_byte(d);
	/* Set RDY bit to 1 to initiate single shot conversion. */
	cfg |= MCP3424_CONFIG_RDY;

	int32_t status = mcp3424_port_i2c_master_transmit(d, &cfg, 1, NULL, 0);

	if (status != 0) {
		return MCP3424_CONVERT_FAILED;
	}

	return MCP3424_CONVERT_OK;
}


int32_t mcp3424_check_result(struct mcp3424 *d)
{
	// Check written setting
	uint8_t rxbuf[4];
	int32_t status = mcp3424_port_i2c_master_transmit(d, NULL, 0, rxbuf, 4);

	if (status != 0) {
		return MCP3424_CHECK_RESULT_FAILED;
	}

	if(rxbuf[3] & 0x80) {
		return MCP3424_CHECK_RESULT_OLD;
	}

	// TODO Fake
	return MCP3424_CHECK_RESULT_UPDATED;
}


int32_t mcp3424_set_gain(struct mcp3424 *d, enum mcp3424_gain gain)
{
	if (d == NULL) {
		return MCP3424_SET_GAIN_FAILED;
	}

	d->gain = gain;

	if (mcp3424_update_config(d) != MCP3424_UPDATE_CONFIG_OK) {
		return MCP3424_SET_GAIN_FAILED;
	}

	return MCP3424_SET_GAIN_OK;
}


int32_t mcp3424_set_channel(struct mcp3424 *d, uint8_t channel)
{
	if (d == NULL) {
		return MCP3424_SET_CHANNEL_FAILED;
	}

	d->channel = channel;

	if (mcp3424_update_config(d) != MCP3424_UPDATE_CONFIG_OK) {
		return MCP3424_SET_CHANNEL_FAILED;
	}

	return MCP3424_SET_CHANNEL_OK;
}


int32_t mcp3424_set_sample_rate(struct mcp3424 *d, enum mcp3424_sample_rate rate)
{
	if (d == NULL) {
		return MCP3424_SET_SAMPLE_RATE_FAILED;
	}

	d->rate = rate;

	if (mcp3424_update_config(d) != MCP3424_UPDATE_CONFIG_OK) {
		return MCP3424_SET_SAMPLE_RATE_FAILED;
	}

	return MCP3424_SET_SAMPLE_RATE_OK;
}


int32_t mcp3424_read_result(struct mcp3424 *d, int32_t *val)
{
	if (d == NULL || val == NULL) {
		return MCP3424_READ_RESULT_FAILED;
	}

	/* We can safely read 3 bytes even if the result is only in the first
	 * 2 bytes (the third one will contain the config byte). */
	uint8_t rxbuf[3];
	int32_t status = mcp3424_port_i2c_master_transmit(d, NULL, 0, rxbuf, 3);

	if (status != 0) {
		return MCP3424_READ_RESULT_FAILED;
	}

	/* If the device is set to 18bit mode (3.75msps), result is stored in
	 * 3 bytes, otherwise it is stored in 2 bytes.
	 * Result is always sign extended to full 24 or 16 bits regardless of the
	 * configured sample rate. See MCP3424 datasheet, Table 5-3. */
	if (d->rate == MCP3424_RATE_3_75) {
		*val = (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
		/* Sign extend the result. */
		*val = ((*val << 8) >> 8);
	} else {
		*val = (rxbuf[0] << 8) | rxbuf[1];
		*val = ((*val << 16) >> 16);
	}

	return MCP3424_READ_RESULT_OK;
}

