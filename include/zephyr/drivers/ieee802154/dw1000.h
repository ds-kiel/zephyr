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

#warning "Don't hard code the maximum payload of a ranging frame"
#define DWT_MTM_MAX_PAYLOAD 250 // requires non-compliant frame size mode in sys configuration
#define DWT_RANGING_FRAME_PAYLOAD_OFFSET(FRAME) (FRAME->payload + sizeof(struct deca_tagged_timestamp) * FRAME->rx_ts_count)

typedef uint16_t deca_short_addr_t;
typedef uint8_t dwt_packed_ts_t[5];
typedef uint64_t dwt_ts_t;
typedef int (*cir_memory_callback_t)(int slot, const uint8_t *cir_memory, size_t size);

uint64_t correct_overflow(dwt_ts_t end_ts, dwt_ts_t start_ts);

#define DECA_NO_ADDRESS UINT16_MAX

struct mtm_ranging_timing {
	uint64_t min_slot_length_us,
		phy_activate_rx_delay,
		phase_setup_delay, round_setup_delay,
		preamble_chunk_duration;
	uint16_t frame_timeout_period, preamble_timeout;
};

struct deca_slot {
	enum slot_type {
		DENSE_LOAD_TX_BUFFER,
		DENSE_RX_SLOT,
		DENSE_TX_SLOT,
		DENSE_IDLE_SLOT,
	} type;

	/* use dwt_calculate_slot_duration to calculate this duration */
	uint16_t duration_us;

	union {
		// Meta information for LOAD_TX_BUFFER
		struct {
			uint8_t *payload;
			size_t payload_size;

			bool load_stored_timestamps;

			/* use this this together with duration_us. If you include a payload in this
			   transmission slot, you should not max out the slot duration to just fit
			   the expected amount of timestamps collected, rather you should also leave
			   some headroom for the payload by decreasing the amount of timestamps to
			   include in the frame. */
			int max_load_timestamps;
		};

		// Meta information for rx
		struct {
			/* for most boards running this code it will not feasible to buffer CIRs for all slots, thus we
			   allow here to active the globally configured CIR handler to process the data directly */
			bool with_cir_handler;
			uint16_t from_index, to_index;
		};
	} meta;
};

struct deca_schedule {
	uint16_t slot_count;
	struct deca_slot *slots;
};


struct deca_glossy_time_pair {
	uint64_t local, ref;
};

struct deca_ranging_configuration {
	deca_short_addr_t addr;

	struct deca_schedule *schedule;
	struct deca_glossy_time_pair *deca_clock_synchronization_instance;
	uint64_t round_start_offset_us; // only relevant if time sync instant is used
	uint64_t deca_round_start_ts; // only relevant if time sync instant is used

	uint32_t slot_duration_us, guard_period_us;
	uint64_t micro_slot_offset_ns;

	// options
	uint8_t cca, reject_frames, cfo, correct_timestamp_bias;

	cir_memory_callback_t cir_handler;

	uint16_t fp_index_threshold; // if reject frames is set, remove frames below this threshold
	uint16_t timeout_us;
	uint16_t cca_duration;
};

struct deca_glossy_configuration {
	deca_short_addr_t node_addr;
	bool isRoot;
	uint16_t guard_period_us;
	uint16_t max_depth;
	uint8_t *payload;
	size_t payload_size;
	uint16_t transmission_delay_us;
};

struct __attribute__((__packed__)) deca_tagged_timestamp {
	dwt_packed_ts_t ts;
	deca_short_addr_t addr;
	uint16_t slot;
};

struct __attribute__((__packed__)) deca_ranging_frame  {
	uint8_t  msg_id;      // identifier of which message type during the protocol run we are sending
	deca_short_addr_t addr;  // unique identifier of this node for ranging
	dwt_packed_ts_t tx_ts;
	uint8_t  rx_ts_count; // amount of received timestamps
	uint8_t  payload_size;
	uint8_t payload[DWT_MTM_MAX_PAYLOAD]; // payload will be located AFTER reception timestamps
};

#define DECA_RANGING_FRAME_MAX_FRAME_SIZE sizeof(struct deca_ranging_frame)

struct deca_ranging_frame_container {
	struct deca_ranging_frame *frame;

	enum deca_frame_type {
		DECA_TRANSMITTED = 0,
		DECA_RECEIVED = 1,
	} type;

	enum deca_ranging_frame_status {
		DECA_FRAME_OKAY,
		DECA_FRAME_REJECTED
	} status;

	uint32_t rx_pacc;
	uint32_t cir_pwr;
	uint16_t fp_index, fp_ampl1, fp_ampl2, fp_ampl3, std_noise;
	uint8_t slot; // might be unecessary since we currently index the return array by the respective slot number
	float cfo_ppm;
	int8_t rx_level;

	dwt_ts_t timestamp;
};

struct deca_ranging_digest {
	struct deca_ranging_frame_container *frames;
	size_t length;
};

struct deca_glossy_result {
	struct deca_glossy_time_pair rtc_clock_pair;
	struct deca_glossy_time_pair deca_clock_pair;
	uint8_t dist_to_root; // aka hop counter
	size_t payload_size;
	uint8_t *payload;
};

int deca_ranging_frame_get_tagged_timestamps(const struct deca_ranging_frame *frame, struct deca_tagged_timestamp **timestamps);

void dwt_set_delayed_tx_short_ts(const struct device *dev, uint32_t short_ts);
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
int dwt_calculate_slot_duration(const struct device *dev, int timestamps_to_load, int payload_size, int guard_us);
int dwt_set_channel(const struct device *dev, uint16_t channel);
dwt_ts_t from_packed_dwt_ts(const dwt_packed_ts_t ts);
void to_packed_dwt_ts(dwt_packed_ts_t ts, dwt_ts_t value);

int      deca_ranging(const struct device *dev, const struct deca_ranging_configuration *conf, struct deca_ranging_digest *digest);
int      deca_glossy_time_synchronization(const struct  device *dev, struct deca_glossy_configuration *conf, struct deca_glossy_result *result);

int      dwt_mtm_ranging_estimate_duration(const struct device *dev, const struct deca_ranging_configuration *conf);
uint32_t dwt_get_pkt_duration_ns(const struct device *dev, uint16_t psdu_len);
void     dwt_set_antenna_delay_rx(const struct device *dev, uint16_t rx_delay_ts);
void     dwt_set_antenna_delay_tx(const struct device *dev, uint16_t tx_delay_ts);
uint16_t dwt_antenna_delay_rx(const struct device *dev);
uint16_t dwt_antenna_delay_tx(const struct device *dev);
uint32_t dwt_otp_antenna_delay(const struct device *dev);
uint8_t  dwt_rx_ttcko_rc_phase(const struct device *dev);
int      dwt_readcarrierintegrator(const struct device *dev);
float    dwt_rx_clock_ratio_offset(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_H_ */
