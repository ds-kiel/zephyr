/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 * @author Patrick Rathje git@patrickrathje.de
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_H_
#define ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_H_

#include <zephyr/device.h>
#include <zephyr/sys/timeutil.h>


/**
 * Set the upper 32 bits of the dwt timestamp
 * This method should be called just before the invocation of the tx method and from within the same
 * thread
 */
void dwt_set_delayed_tx_short_ts(const struct device *dev, uint32_t short_ts);

/**
 * Sets the delayed tx and returns the estimated tx ts
 * @param dev The dw1000 device
 * @param uus_delay the delay in uwb microseconds
 * @return estimated tx ts (corrected by antenna delay)
 */
uint64_t dwt_plan_delayed_tx(const struct device *dev, uint64_t uus_delay);
uint64_t dwt_rx_ts(const struct device *dev);
uint64_t dwt_system_ts(const struct device *dev);
uint32_t dwt_system_short_ts(const struct device *dev);
uint64_t dwt_ts_to_fs(uint64_t ts);
uint64_t dwt_fs_to_ts(uint64_t fs);
uint64_t dwt_short_ts_to_fs(uint32_t ts);
uint32_t dwt_fs_to_short_ts(uint64_t fs);
uint64_t dwt_calculate_actual_tx_ts(uint32_t planned_short_ts, uint16_t tx_antenna_delay);
void     dwt_set_frame_filter(const struct device *dev, bool ff_enable, uint8_t ff_type);
uint8_t *dwt_get_mac(const struct device *dev);



/** Ranging Utility Functions **/
struct mtm_ranging_timing {
	uint64_t min_slot_length_us,
		phy_activate_rx_delay,
		phase_setup_delay,
		preamble_chunk_duration;
	uint16_t frame_timeout_period, preamble_timeout;
};


typedef int (*cir_memory_callback_t)(int slot, int phase, const uint8_t *cir_memory, size_t size);

struct mtm_ranging_config {
	uint8_t ranging_id;
	uint8_t slots_per_phase, phases;
	uint8_t tx_slot_offset;
	uint32_t slot_duration_us, guard_period_us;
	uint64_t micro_slot_offset_ns;

	// options
	uint8_t use_initiation_frame, cca, reject_frames, cfo;

	cir_memory_callback_t cir_handler;

	uint16_t fp_index_threshold; // if reject frames is set, remove frames below this threshold
	uint16_t timeout_us;
	uint16_t cca_duration;
};

typedef uint8_t dwt_packed_ts_t[5];
typedef uint64_t dwt_ts_t;

dwt_ts_t from_packed_dwt_ts(const dwt_packed_ts_t ts);
void to_packed_dwt_ts(dwt_packed_ts_t ts, dwt_ts_t value);

struct __attribute__((__packed__)) dwt_tagged_timestamp {
	dwt_packed_ts_t ts;
	uint8_t ranging_id;
};

struct __attribute__((__packed__)) dwt_ranging_frame_buffer {
	uint8_t  prot_id;     // identifier that this is a MTM ranging protocol execution
	uint8_t  msg_id;      // identifier of which message type during the protocol run we are sending
	uint8_t  ranging_id;  // unique identifier of this node for ranging
	uint8_t  rx_ts_count; // amount of received timestamps
	dwt_packed_ts_t tx_ts;
	dwt_packed_ts_t phase_start_ts; // this will eventually replace tx_ts.
	struct dwt_tagged_timestamp rx_ts[15];   // for now allocate space for 10 timestamps
};

enum dwt_ranging_frame_status {
	DWT_MTM_FRAME_OKAY,
	DWT_MTM_FRAME_REJECTED
};

struct dwt_ranging_frame_info {
	struct dwt_ranging_frame_buffer *frame;
	enum dwt_ranging_frame_status status;

	enum dwt_ranging_frame_info_type {
		DWT_RANGING_TRANSMITTED_FRAME = 0,
		DWT_RANGING_RECEIVED_FRAME = 1,
	} type;

	uint32_t rx_pacc;
	uint32_t cir_pwr;
	uint16_t fp_index, fp_ampl1, fp_ampl2, fp_ampl3, std_noise;
	uint8_t slot; // might be unecessary since we currently index the return array by the respective slot number
	float cfo_ppm;
	int8_t rx_level;

	dwt_ts_t timestamp;
};

struct dwt_glossy_tx_result {
	struct timeutil_sync_instant clock_sync_instant;
	uint8_t dist_to_root; // aka hop counter
};

enum dwt_mtm_ranging_slot {
	DWT_TX_AUTO = 0xFE, // TODO implement, this indicates that a transmission is wanted, but no transmission slot is specified
	DWT_NO_TX_SLOT = 0xFF, // indicates that a node should not transmit during this round
};

int      dwt_mtm_ranging(const struct device *dev, const struct mtm_ranging_config *conf, struct dwt_ranging_frame_info **buffers);
int      dwt_mtm_ranging_estimate_duration(const struct device *dev, const struct mtm_ranging_config *conf);
int      dwt_glossy_tx_timesync(const struct  device *dev, uint8_t initiator, uint8_t node_id, uint16_t timeout_us, struct dwt_glossy_tx_result *result);

void     dwt_set_antenna_delay_rx(const struct device *dev, uint16_t rx_delay_ts);
void     dwt_set_antenna_delay_tx(const struct device *dev, uint16_t tx_delay_ts);
uint16_t dwt_antenna_delay_rx(const struct device *dev);
uint16_t dwt_antenna_delay_tx(const struct device *dev);
uint32_t dwt_otp_antenna_delay(const struct device *dev);

uint8_t  dwt_rx_ttcko_rc_phase(const struct device *dev);
int      dwt_readcarrierintegrator(const struct device *dev);
float    dwt_rx_clock_ratio_offset(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_H_ */
