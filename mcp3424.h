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


#include <stdint.h>

#include "ch.h"
#include "hal.h"

#ifndef _MCP3424_H_
#define _MCP3424_H_

enum mcp3424_mode {
	MCP3424_MODE_SINGLE_SHOT,
	MCP3424_MODE_CONTINUOUS
};

enum mcp3424_gain {
	MCP3424_GAIN_1X,
	MCP3424_GAIN_2X,
	MCP3424_GAIN_4X,
	MCP3424_GAIN_8X
};

enum mcp3424_sample_rate {
	MCP3424_RATE_240,
	MCP3424_RATE_60,
	MCP3424_RATE_15,
	MCP3424_RATE_3_75
};

struct mcp3424 {
	uint8_t addr;
	enum mcp3424_mode mode;
	enum mcp3424_gain gain;
	enum mcp3424_sample_rate rate;

	I2CDriver *drv;
};


/**
 * @brief Check if there is a MCP3424 device on the selected address.
 *
 * @param addr I2C address of the MCP3424 slave.
 * @param i2cd TODO: ChibiOS i2c driver.
 */
int32_t mcp3424_probe(I2CDriver *drv, uint8_t addr);
#define MCP3424_PROBE_FOUND 0
#define MCP3424_PROBE_NOTFOUND -1
#define MCP3424_PROBE_FAILED -2

/**
 * @brief Init a MCP3424 driver.
 *
 * Must be called prior to calling any other function accessing the driver
 * context structure. It sets all required initial values and probes the
 * device to check if it is working properly.
 *
 * @param d A driver context structure.
 * @param addr I2C address of the MCP3424 slave.
 * @param i2cd TODO: ChibiOS i2c driver.
 *
 * @return MCP3424_INIT_OK if the device is working and was properly
 *                         initialized or
 *         MCP3424_INIT_FAILED if an error eccured.
 */
int32_t mcp3424_init(struct mcp3424 *d, I2CDriver *drv, uint8_t addr);
#define MCP3424_INIT_OK 0
#define MCP3424_INIT_FAILED -1

/**
 * @brief Uninitialize driver and free all used resources.
 */
int32_t mcp3424_free(struct mcp3424 *d);
#define MCP3424_FREE_OK 0
#define MCP3424_FREE_FAILED -1

/**
 * @brief Start the driver if set to continuous mode.
 *
 * Initializes the driver continuous conversion mode. If the driver is
 * set to single shot mode, this function does nothing.
 */
int32_t mcp3424_start(struct mcp3424 *d);
#define MCP3424_START_OK 0
#define MCP3424_START_FAILED -1

/**
 * @brief Stop the driver if it was started previously.
 */
int32_t mcp3424_stop(struct mcp3424 *d);
#define MCP3424_STOP_OK 0
#define MCP3424_STOP_FAILED -1

/**
 * @brief Set driver conversion mode.
 */
int32_t mcp3424_set_mode(struct mcp3424 *d, enum mcp3424_mode mode);
#define MCP3424_SET_MODE_OK 0
#define MCP3424_SET_MODE_FAILED -1

/**
 * @brief Do a single conversion. Must be used when the driver is stopped.
 */
int32_t mcp3424_convert(struct mcp3424 *d);
#define MCP3424_CONVERT_OK 0
#define MCP3424_CONVERT_FAILED -1

/**
 * @brief Check if there is new conversion result available.
 *
 * Function checks for new result of current conversion. New result flag
 * is cleared so the function never returns true two times for the same
 * conversion.
 */
int32_t mcp3424_check_result(struct mcp3424 *d);
#define MCP3424_CHECK_RESULT_UPDATED 0
#define MCP3424_CHECK_RESULT_OLD -1
#define MCP3424_CHECK_RESULT_FAILED -2

/**
 * @brief Set PGA gain.
 */
int32_t mcp3424_set_gain(struct mcp3424 *d, enum mcp3424_gain gain);
#define MCP3424_SET_GAIN_OK 0
#define MCP3424_SET_GAIN_FAILED -1

/**
 * @brief Set input channel.
 */
int32_t mcp3424_set_channel(struct mcp3424 *d, uint8_t channel);
#define MCP3424_SET_CHANNEL_OK 0
#define MCP3424_SET_CHANNEL_FAILED -1

/**
 * @brief Set conversion sample rate.
 */
int32_t mcp3424_set_sample_rate(struct mcp3424 *d, enum mcp3424_sample_rate rate);
#define MCP3424_SET_SAMPLE_RATE_OK 0
#define MCP3424_SET_SAMPLE_RATE_FAILED -1



#endif
