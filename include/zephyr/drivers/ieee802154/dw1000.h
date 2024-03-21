/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 * @author Patrick Rathje git@patrickrathje.de
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_H_
#define ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_H_

#include <zephyr/device.h>

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
struct dwt_timestamp {
	uint64_t dwt_ts;
	uint8_t ranging_id;
};

/* struct dwt_ranging_result { */
/* 	struct dwt_timestamp *timestamps; */
/* 	uint8_t  len; */
/* }; */

struct __attribute__((__packed__)) dwt_ranging_frame_buffer {
	uint8_t  prot_id;     // some identifier that this is a MTM ranging protocol execution
	uint8_t  msg_id;      // some identifier of which message type during the protocol run we are sending
	uint8_t  ranging_id;      // some ranging id, in case of time slotted access this is equivalent to the transmission slot in the schedule
	uint8_t  rx_ts_count; // amount of received timestamps
	struct dwt_timestamp tx_ts;
	struct dwt_timestamp rx_ts[5];   // for now allocate space for 12 timestamps
};


/* Comment: this is similar to how the kernel GNSS interface is structured, however, note that the most
   recent IEEE802.15.4z standard already has a pretty good abstraction for how ranging data shall be passed
   between MAC and higher levels (As is true for the execution of the ranging itself. In the future
   we should adapt our code to the standard.

   After the conclusion of the callback, the memory of the dwt_ranging_result struct will be freed.
   You should therefore copy the data, in case that you need it for a longer time.
 */
typedef void (*ranging_data_callback_t)(const struct device *dev, struct dwt_ranging_frame_buffer **buffers, size_t round_length, size_t repetitions);

struct ranging_data_callback {
	const struct device *dev;
	/** Callback called when ranging data from round is published */
	ranging_data_callback_t callback;
};

#define RANGING_DATA_CALLBACK_DEFINE(_dev, _callback)                                              \
	static const STRUCT_SECTION_ITERABLE(ranging_data_callback,                                \
					     _ranging_data_callback__##_callback) = {              \
		.dev = _dev,                                                                    \
		.callback = _callback,                                                          \
	}

int dwt_mtm_ranging(const struct device *dev, uint8_t round_length, uint8_t slot_offset, uint8_t ranging_id);

void     dwt_set_antenna_delay_rx(const struct device *dev, uint16_t rx_delay_ts);
void     dwt_set_antenna_delay_tx(const struct device *dev, uint16_t tx_delay_ts);
uint16_t dwt_antenna_delay_rx(const struct device *dev);
uint16_t dwt_antenna_delay_tx(const struct device *dev);
uint32_t dwt_otp_antenna_delay(const struct device *dev);

uint8_t  dwt_rx_ttcko_rc_phase(const struct device *dev);
int      dwt_readcarrierintegrator(const struct device *dev);
float    dwt_rx_clock_ratio_offset(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_IEEE802154_DW1000_H_ */
