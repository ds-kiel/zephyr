/*
 * Copyright (c) 2020 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "hal/nrf_spi.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dw1000, LOG_LEVEL_INF);

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/debug/stack.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#include <zephyr/sys/slist.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/random/random.h>
#include <zephyr/debug/stack.h>
#include <math.h>
#include <stdlib.h>
#include <zephyr/timing/timing.h>


#include <zephyr/drivers/gpio.h>

#define ZEPHYR_SPI 0
#define SPIM       0
#define EXTREMELY_LEAN_DRIVER 0

#if ZEPHYR_SPI
#include <zephyr/drivers/spi.h>
#else
/* #include <nrfx_spi.h> */
#if SPIM
#include <nrfx_spim.h>
#else
#include <nrfx_spi.h>
#endif
#include <drivers/src/prs/nrfx_prs.h>
#include <zephyr/drivers/pinctrl.h>
#endif

#if !ZEPHYR_SPI
#if SPIM
#define SPIM_NODE DT_NODELABEL(spi2)
PINCTRL_DT_DEFINE(SPIM_NODE);
#else
#define SPI_NODE DT_NODELABEL(spi2)
PINCTRL_DT_DEFINE(SPI_NODE);
#endif


static K_SEM_DEFINE(spi_transfer_finished, 0, 1);

#if SPIM
static nrfx_spim_t spi = NRFX_SPIM_INSTANCE(2);
#else
static nrfx_spi_t spi = NRFX_SPI_INSTANCE(2);
#endif
static bool spi_initialized;
#endif


#include <zephyr/net/ieee802154_radio.h>
#include "ieee802154_dw1000_regs.h"

#include <zephyr/drivers/ieee802154/dw1000.h>

#define ANALYZE_DWT_TIMING 0
#define USE_GPIO_DEBUG 1

#if USE_GPIO_DEBUG

#define DEB0_NODE DT_ALIAS(deb0)
#define DEB1_NODE DT_ALIAS(deb1)


static const struct gpio_dt_spec deb0 = GPIO_DT_SPEC_GET(DEB0_NODE, gpios);;
static const struct gpio_dt_spec deb1 = GPIO_DT_SPEC_GET(DEB1_NODE, gpios);;

static void setup_debug_gpios() {
	gpio_pin_configure_dt(&deb0, GPIO_OUTPUT | GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&deb1, GPIO_OUTPUT | GPIO_OUTPUT_ACTIVE);
}

#define TOGGLE_DEBUG_GPIO(GPIO) do { \
    gpio_pin_toggle_dt(&deb##GPIO); \
} while(0)

#define SET_GPIO_HIGH(GPIO) do { \
		gpio_pin_set_dt(&deb##GPIO, 1);	\
} while(0)

#define SET_GPIO_LOW(GPIO) do { \
		gpio_pin_set_dt(&deb##GPIO, 0);	\
} while(0)
#else
#define TOGGLE_DEBUG_GPIO(GPIO)
#define TOGGLE_DEBUG_GPIOS()
#endif

#define DT_DRV_COMPAT decawave_dw1000

#define DWT_FCS_LENGTH			2U

#if ZEPHYR_SPI
#define DWT_SPI_CSWAKEUP_FREQ 500000U
#define DWT_SPI_SLOW_FREQ 2000000U
#else
#define DWT_SPIM_CSWAKEUP_FREQ NRFX_KHZ_TO_HZ(500)
#define DWT_SPIM_SLOW_FREQ     NRFX_MHZ_TO_HZ(2)
#define DWT_SPIM_FAST_FREQ     NRFX_MHZ_TO_HZ(8)

#define DWT_SPI_CSWAKEUP_FREQ NRF_SPI_FREQ_500K
#define DWT_SPI_SLOW_FREQ     NRF_SPI_FREQ_2M
#define DWT_SPI_FAST_FREQ     NRF_SPI_FREQ_8M
#endif

#define DWT_SPI_TRANS_MAX_HDR_LEN	3
#define DWT_SPI_TRANS_REG_MAX_RANGE	0x3F
#define DWT_SPI_TRANS_SHORT_MAX_OFFSET	0x7F
#define DWT_SPI_TRANS_WRITE_OP		BIT(7)
#define DWT_SPI_TRANS_SUB_ADDR		BIT(6)
#define DWT_SPI_TRANS_EXTEND_ADDR	BIT(7)

#define DWT_TS_TIME_UNITS_FS		15650U /* DWT_TIME_UNITS in fs */

#define DW1000_TX_ANT_DLY		16450
#define DW1000_RX_ANT_DLY		16450

/* SHR Symbol Duration in ns */
#define UWB_PHY_TPSYM_PRF64		IEEE802154_PHY_HRP_UWB_PRF64_TPSYM_SYMBOL_PERIOD_NS
#define UWB_PHY_TPSYM_PRF16		IEEE802154_PHY_HRP_UWB_PRF16_TPSYM_SYMBOL_PERIOD_NS

#define UWB_PHY_NUMOF_SYM_SHR_SFD	8

/* PHR Symbol Duration Tdsym in ns */
#define UWB_PHY_TDSYM_PHR_110K		8205.13
#define UWB_PHY_TDSYM_PHR_850K		1025.64
#define UWB_PHY_TDSYM_PHR_6M8		1025.64

#define UWB_PHY_NUMOF_SYM_PHR		18

/* Data Symbol Duration Tdsym in ns */
#define UWB_PHY_TDSYM_DATA_110K		8205.13
#define UWB_PHY_TDSYM_DATA_850K		1025.64
#define UWB_PHY_TDSYM_DATA_6M8		128.21

#define DWT_WORK_QUEUE_STACK_SIZE	512

#define DWT_OTP_DELAY_ADDR 0x1C


// added by ente
#define DWT_DOUBLE_BUFFERING 1
#define DWT_AUTO_REENABLE_RX_AFTER_TX 0

static struct k_work_q dwt_work_queue;
static K_KERNEL_STACK_DEFINE(dwt_work_queue_stack,
			     DWT_WORK_QUEUE_STACK_SIZE);

struct dwt_phy_config {
	uint8_t channel;	/* Channel 1, 2, 3, 4, 5, 7 */
	uint8_t dr;	/* Data rate DWT_BR_110K, DWT_BR_850K, DWT_BR_6M8 */
	uint8_t prf;	/* PRF DWT_PRF_16M or DWT_PRF_64M */

	uint8_t rx_pac_l;		/* DWT_PAC8..DWT_PAC64 */
	uint8_t rx_shr_code;	/* RX SHR preamble code */
	uint8_t rx_ns_sfd;		/* non-standard SFD */
	uint16_t rx_sfd_to;	/* SFD timeout value (in symbols)
				 * (tx_shr_nsync + 1 + SFD_length - rx_pac_l)
				 */

	uint8_t tx_shr_code;	/* TX SHR preamble code */
	uint32_t tx_shr_nsync;	/* PLEN index, e.g. DWT_PLEN_64 */

	float t_shr;
	float t_phr;
	float t_dsym;
};

struct dwt_hi_cfg {
#if ZEPHYR_SPI
	struct spi_dt_spec bus;
#endif
	struct gpio_dt_spec irq_gpio;
	struct gpio_dt_spec rst_gpio;
};

#define DWT_STATE_TX		0
#define DWT_STATE_CCA		1
#define DWT_STATE_RX_DEF_ON	2
#define DWT_STATE_IRQ_POLLING_EMU	3

// these are used when using the irq handler in a pseudo polling manner in the dwt ranging methods
#define DWT_IRQ_NONE 0
#define DWT_IRQ_TX 1
#define DWT_IRQ_RX 2
#define DWT_IRQ_FRAME_WAIT_TIMEOUT 4
#define DWT_IRQ_PREAMBLE_DETECT_TIMEOUT 5
#define DWT_IRQ_ERR 6
#define DWT_IRQ_HALF_DELAY_WARNING 7

struct dwt_context {
	const struct device *dev;
	struct net_if *iface;


#if ZEPHYR_SPI
	const struct spi_config *spi_cfg;
	struct spi_config spi_cfg_slow;
#else
#if SPIM
	nrfx_spim_config_t spi_cfg;
#else
	nrfx_spi_config_t spi_cfg;
#endif

	const struct pinctrl_dev_config *pcfg;


#endif

	struct gpio_callback gpio_cb;
	struct k_sem dev_lock;
	struct k_sem phy_sem;
	struct k_work irq_cb_work;
	struct k_thread thread;
	struct dwt_phy_config rf_cfg;
	atomic_t state;
	bool cca_busy;
	uint8_t phy_irq_event;
	uint32_t phy_irq_sys_stat;
	uint16_t sleep_mode;
	uint8_t mac_addr[8];

	uint16_t rx_ant_dly, tx_ant_dly;
	bool delayed_tx_short_ts_set;
	uint32_t delayed_tx_short_ts;

	uint64_t rx_ts;
	uint8_t rx_ttcko_rc_phase;
};

static const struct dwt_hi_cfg dw1000_0_config = {
#if ZEPHYR_SPI
	.bus = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8), 0),
#endif
	.irq_gpio = GPIO_DT_SPEC_INST_GET(0, int_gpios),
	.rst_gpio = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
};

static struct dwt_context dwt_0_context = {
	.dev_lock = Z_SEM_INITIALIZER(dwt_0_context.dev_lock, 1, 1),
	.phy_sem = Z_SEM_INITIALIZER(dwt_0_context.phy_sem, 0, 1),
#if !ZEPHYR_SPI
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(SPI_NODE),
#endif
	.rf_cfg = {
		.channel = 5,
		.dr = DWT_BR_6M8,
		.prf = DWT_PRF_64M,

		.rx_pac_l = DWT_PAC8,
		.rx_shr_code = 10,
		.rx_ns_sfd = 0,
		/* .rx_sfd_to = (129 + 8 - 8), */
		.rx_sfd_to = (129 + 8 - 8)*2, // use double timeout in case the receiver jumps between transmitters in case of concurrent transmitters

		.tx_shr_code = 10,
		.tx_shr_nsync = DWT_PLEN_128,
	},
};

/* This struct is used to read all additional RX frame info at one push */
struct dwt_rx_info_regs {
	uint8_t rx_fqual[DWT_RX_FQUAL_LEN];
	uint8_t rx_ttcki[DWT_RX_TTCKI_LEN];
	uint8_t rx_ttcko[DWT_RX_TTCKO_LEN];
	/* RX_TIME without RX_RAWST */
	uint8_t rx_time[DWT_RX_TIME_FP_RAWST_OFFSET];
} _packed;

#define RANGE_CORR_MAX_RSSI (-61)
#define RANGE_CORR_MIN_RSSI (-93)

int8_t range_bias_by_rssi[RANGE_CORR_MAX_RSSI-RANGE_CORR_MIN_RSSI+1] = {
    -23, // -61dBm (-11 cm)
    -23, // -62dBm (-10.75 cm)
    -22, // -63dBm (-10.5 cm)
    -22, // -64dBm (-10.25 cm)
    -21, // -65dBm (-10.0 cm)
    -21, // -66dBm (-9.65 cm)
    -20, // -67dBm (-9.3 cm)
    -19, // -68dBm (-8.75 cm)
    -17, // -69dBm (-8.2 cm)
    -16, // -70dBm (-7.55 cm)
    -15, // -71dBm (-6.9 cm)
    -13, // -72dBm (-6.0 cm)
    -11, // -73dBm (-5.1 cm)
    -8, // -74dBm (-3.9 cm)
    -6, // -75dBm (-2.7 cm)
    -3, // -76dBm (-1.35 cm)
    0, // -77dBm (0.0 cm)
    2, // -78dBm (1.05 cm)
    4, // -79dBm (2.1 cm)
    6, // -80dBm (2.8 cm)
    7, // -81dBm (3.5 cm)
    8, // -82dBm (3.85 cm)
    9, // -83dBm (4.2 cm)
    10, // -84dBm (4.55 cm)
    10, // -85dBm (4.9 cm)
    12, // -86dBm (5.55 cm)
    13, // -87dBm (6.2 cm)
    14, // -88dBm (6.65 cm)
    15, // -89dBm (7.1 cm)
    16, // -90dBm (7.35 cm)
    16, // -91dBm (7.6 cm)
    17, // -92dBm (7.85 cm)
    17, // -93dBm (8.1 cm)
};

static int8_t get_range_bias_by_rssi(int8_t rssi) {
    rssi = MAX(MIN(RANGE_CORR_MAX_RSSI, rssi), RANGE_CORR_MIN_RSSI);
    return range_bias_by_rssi[-(rssi-RANGE_CORR_MAX_RSSI)];
}


static int dwt_configure_rf_phy(const struct device *dev);

#if ZEPHYR_SPI
static int dwt_spi_read(const struct device *dev,
			uint16_t hdr_len, const uint8_t *hdr_buf,
			uint32_t data_len, uint8_t *data)
{
	struct dwt_context *ctx = dev->data;
	const struct dwt_hi_cfg *hi_cfg = dev->config;
	const struct spi_buf tx_buf = {
		.buf = (uint8_t *)hdr_buf,
		.len = hdr_len
	};
	const struct spi_buf_set tx = {
		.frames = &tx_buf,
		.count = 1
	};
	struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = hdr_len,
		},
		{
			.buf = (uint8_t *)data,
			.len = data_len,
		},
	};
	const struct spi_buf_set rx = {
		.frames = rx_buf,
		.count = 2
	};

	LOG_DBG("spi read, header length %u, data length %u",
		(uint16_t)hdr_len, (uint32_t)data_len);
	LOG_HEXDUMP_DBG(hdr_buf, (uint16_t)hdr_len, "rd: header");

	if (spi_transceive(hi_cfg->bus.bus, ctx->spi_cfg, &tx, &rx)) {
		LOG_ERR("SPI transfer failed");
		return -EIO;
	}

	LOG_HEXDUMP_DBG(data, (uint32_t)data_len, "rd: data");

	return 0;
}


static int dwt_spi_write(const struct device *dev,
			 uint16_t hdr_len, const uint8_t *hdr_buf,
			 uint32_t data_len, const uint8_t *data)
{
	struct dwt_context *ctx = dev->data;
	const struct dwt_hi_cfg *hi_cfg = dev->config;
	struct spi_buf buf[2] = {
		{.buf = (uint8_t *)hdr_buf, .len = hdr_len},
		{.buf = (uint8_t *)data, .len = data_len}
	};
	struct spi_buf_set buf_set = {.frames = buf, .count = 2};

	LOG_DBG("spi write, header length %u, data length %u",
		(uint16_t)hdr_len, (uint32_t)data_len);
	LOG_HEXDUMP_DBG(hdr_buf, (uint16_t)hdr_len, "wr: header");
	LOG_HEXDUMP_DBG(data, (uint32_t)data_len, "wr: data");

	if (spi_write(hi_cfg->bus.bus, ctx->spi_cfg, &buf_set)) {
		LOG_ERR("SPI read failed");
		return -EIO;
	}

	return 0;
}
#else

static bool spi_transfer(const uint8_t *tx_data, size_t tx_data_len,
			  uint8_t *rx_buf, size_t rx_buf_size)
{
	nrfx_err_t err;
#if SPIM
	nrfx_spim_xfer_desc_t
#else
	nrfx_spi_xfer_desc_t
#endif
		xfer_desc = {
		.p_tx_buffer = tx_data,
		.tx_length = tx_data_len,
		.p_rx_buffer = rx_buf,
		.rx_length = rx_buf_size,
	};

#if SPIM
	err = nrfx_spim_xfer(&spi, &xfer_desc, 0);
#else
	err = nrfx_spi_xfer(&spi, &xfer_desc, 0);
#endif

	if (err != NRFX_SUCCESS) {
		printk("nrfx_spi_xfer() failed: 0x%08x\n", err);
		return false;
	}

#if WITH_SPI_IRQ
	if (k_sem_take(&spi_transfer_finished, K_MSEC(100)) != 0) {
		printk("SPI transfer timeout\n");
		return false;
	}
#endif
	return true;
}


// TODO: this is sadly really unoptimal. Sadly if we use the nrfx spi API directly, we are not able
// to chain together multiple transfers. Usually this is not a problem and by giving up that feature
// we save a lot of time when doing small transfers, however when doing larger transfers we will run
// into problems with the amount of stack memory since we have to copy the data to the stack to
// merge with the header before dispatching the transfer.
static int dwt_spi_read(const struct device *dev, uint16_t hdr_len, const uint8_t *hdr_buf,
			uint32_t data_len, uint8_t *data)
{
	// merge together frames, before calling spi_transfer
	uint8_t transfer_buf[hdr_len + data_len];

	// Copy header to transfer buffer
	memcpy(transfer_buf, hdr_buf, hdr_len);

	// Copy data to transfer buffer, starting right after the header
	memcpy(transfer_buf + hdr_len, data, data_len);

	// Now call spi_transfer with the combined buffer
	uint8_t rx_buf[hdr_len + data_len]; // Assuming you want to read the same amount as written
	bool success = spi_transfer(transfer_buf, hdr_len + data_len, rx_buf, hdr_len + data_len);

	if (!success) {
		printk("SPI transfer failed\n");
		return -1; // Or any other error handling code
	}

	// Copy back to data
	memcpy(data, rx_buf + hdr_len, data_len);

	return 0; // Success
}

// isn't it same same?
static int dwt_spi_write(const struct device *dev, uint16_t hdr_len, const uint8_t *hdr_buf,
	uint32_t data_len, uint8_t *data)
{
	dwt_spi_read(dev, hdr_len, hdr_buf, data_len, data);

	return 0;
}

#endif



/* See 2.2.1.2 Transaction formats of the SPI interface */
static int dwt_spi_transfer(const struct device *dev,
			    uint8_t reg, uint16_t offset,
			    size_t buf_len, uint8_t *buf, bool write)
{
	uint8_t hdr[DWT_SPI_TRANS_MAX_HDR_LEN] = {0};
	size_t hdr_len = 0;

	hdr[0] = reg & DWT_SPI_TRANS_REG_MAX_RANGE;
	hdr_len += 1;

	if (offset != 0) {
		hdr[0] |= DWT_SPI_TRANS_SUB_ADDR;
		hdr[1] = (uint8_t)offset & DWT_SPI_TRANS_SHORT_MAX_OFFSET;
		hdr_len += 1;

		if (offset > DWT_SPI_TRANS_SHORT_MAX_OFFSET) {
			hdr[1] |= DWT_SPI_TRANS_EXTEND_ADDR;
			hdr[2] =  (uint8_t)(offset >> 7);
			hdr_len += 1;
		}

	}

	if (write) {
		hdr[0] |= DWT_SPI_TRANS_WRITE_OP;
		return dwt_spi_write(dev, hdr_len, hdr, buf_len, buf);
	} else {
		return dwt_spi_read(dev, hdr_len, hdr, buf_len, buf);
	}
}

static int dwt_register_read(const struct device *dev,
			     uint8_t reg, uint16_t offset, size_t buf_len, uint8_t *buf)
{
	return dwt_spi_transfer(dev, reg, offset, buf_len, buf, false);
}

static int dwt_register_write(const struct device *dev,
			      uint8_t reg, uint16_t offset, size_t buf_len, uint8_t *buf)
{
	return dwt_spi_transfer(dev, reg, offset, buf_len, buf, true);
}

static inline uint32_t dwt_reg_read_u32(const struct device *dev,
				     uint8_t reg, uint16_t offset)
{
	uint8_t buf[sizeof(uint32_t)];

	dwt_spi_transfer(dev, reg, offset, sizeof(buf), buf, false);

	return sys_get_le32(buf);
}

static inline uint16_t dwt_reg_read_u16(const struct device *dev,
				     uint8_t reg, uint16_t offset)
{
	uint8_t buf[sizeof(uint16_t)];

	dwt_spi_transfer(dev, reg, offset, sizeof(buf), buf, false);

	return sys_get_le16(buf);
}

static inline uint8_t dwt_reg_read_u8(const struct device *dev,
				   uint8_t reg, uint16_t offset)
{
	uint8_t buf;

	dwt_spi_transfer(dev, reg, offset, sizeof(buf), &buf, false);

	return buf;
}

static inline void dwt_reg_write_u32(const struct device *dev,
				     uint8_t reg, uint16_t offset, uint32_t val)
{
	uint8_t buf[sizeof(uint32_t)];

	sys_put_le32(val, buf);
	dwt_spi_transfer(dev, reg, offset, sizeof(buf), buf, true);
}

static inline void dwt_reg_write_u16(const struct device *dev,
				     uint8_t reg, uint16_t offset, uint16_t val)
{
	uint8_t buf[sizeof(uint16_t)];

	sys_put_le16(val, buf);
	dwt_spi_transfer(dev, reg, offset, sizeof(buf), buf, true);
}

static inline void dwt_reg_write_u8(const struct device *dev,
				    uint8_t reg, uint16_t offset, uint8_t val)
{
	dwt_spi_transfer(dev, reg, offset, sizeof(uint8_t), &val, true);
}

static ALWAYS_INLINE void dwt_setup_int(const struct device *dev,
					bool enable)
{
	const struct dwt_hi_cfg *hi_cfg = dev->config;

	unsigned int flags = enable
		? GPIO_INT_EDGE_TO_ACTIVE
		: GPIO_INT_DISABLE;

	gpio_pin_interrupt_configure_dt(&hi_cfg->irq_gpio, flags);
}

static void dwt_reset_rfrx(const struct device *dev)
{
	/*
	 * Apply a receiver-only soft reset,
	 * see SOFTRESET field description in DW1000 User Manual.
	 */
	dwt_reg_write_u8(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_SOFTRESET_OFFSET,
			 DWT_PMSC_CTRL0_RESET_RX);
	dwt_reg_write_u8(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_SOFTRESET_OFFSET,
			 DWT_PMSC_CTRL0_RESET_CLEAR);
}

static void dwt_disable_txrx(const struct device *dev)
{
	dwt_setup_int(dev, false);

	dwt_reg_write_u8(dev, DWT_SYS_CTRL_ID, DWT_SYS_CTRL_OFFSET,
			 DWT_SYS_CTRL_TRXOFF);

	dwt_reg_write_u32(dev, DWT_SYS_STATUS_ID, DWT_SYS_STATUS_OFFSET,
			  (DWT_SYS_STATUS_ALL_RX_GOOD |
			   DWT_SYS_STATUS_ALL_RX_TO |
			   DWT_SYS_STATUS_ALL_RX_ERR |
			   DWT_SYS_STATUS_ALL_TX));

	dwt_setup_int(dev, true);
}

static void dwt_setup_rx_timeout(const struct device *dev, uint16_t timeout) {
	uint32_t sys_cfg;

	sys_cfg = dwt_reg_read_u32(dev, DWT_SYS_CFG_ID, 0);

	if (timeout != 0) {
		dwt_reg_write_u16(dev, DWT_RX_FWTO_ID, DWT_RX_FWTO_OFFSET,
				  timeout);
		sys_cfg |= DWT_SYS_CFG_RXWTOE;
	} else {
		sys_cfg &= ~DWT_SYS_CFG_RXWTOE;
	}

	dwt_reg_write_u32(dev, DWT_SYS_CFG_ID, 0, sys_cfg);
}


/* 7.2.40.9:  Supposing our preamble length is 1024 symbols and the PAC size is set to 32 (in line with Table 6) */
/* and, we send a message and know that the response (if present) will come after exactly 30 ms */
/* (because the responder is using delayed send to begin the response exactly 30 ms after receiving our */
/* message). We can command a 30 ms delayed receive (timed from our message transmission time) and have */
/* DRX_PRETOC programmed to a value of 32, which is the preamble length (1024) divided by the PAC
 * size */
/* (32). */
static void dwt_setup_preamble_detection_timeout(const struct device *dev, uint16_t pac_symbols) {
	dwt_reg_write_u16(dev, DWT_DRX_CONF_ID, DWT_DRX_PRETOC_OFFSET, pac_symbols);
}

static inline void dwt_switch_buffers(const struct device *dev) {
	uint32_t sys_ctrl = DWT_SYS_CTRL_HSRBTOGGLE;
	dwt_reg_write_u32(dev, DWT_SYS_CTRL_ID, DWT_SYS_CTRL_OFFSET, sys_ctrl);
}

static inline void dwt_fast_enable_rx(const struct device *dev, uint64_t dwt_rx_ts)
{
	struct dwt_context *ctx = dev->data;
	uint16_t sys_ctrl = DWT_SYS_CTRL_RXENAB;

	// program delayed receive
	if(dwt_rx_ts != 0) {
		dwt_reg_write_u32(dev, DWT_DX_TIME_ID, 1, (uint32_t) (dwt_rx_ts >> 8));
		sys_ctrl |= DWT_SYS_CTRL_RXDLYE;
	} else {
		sys_ctrl &= ~DWT_SYS_CTRL_RXDLYE;
	}

	if (atomic_test_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU)) {
		k_sem_reset(&ctx->phy_sem);
	}

	dwt_reg_write_u16(dev, DWT_SYS_CTRL_ID, DWT_SYS_CTRL_OFFSET, sys_ctrl);
}

static inline void dwt_read_rx_info(const struct device *dev, struct dwt_rx_info_regs *rx_inf_reg) {
	dwt_register_read(dev, DWT_RX_FQUAL_ID, 0, sizeof(struct dwt_rx_info_regs),
			  (uint8_t *)rx_inf_reg);
}

static inline uint64_t dwt_rx_timestamp_from_rx_info(const struct dwt_rx_info_regs *rx_inf_reg) {
	uint8_t ts_buf[sizeof(uint64_t)] = {0};
	memcpy(ts_buf, rx_inf_reg->rx_time, DWT_RX_TIME_RX_STAMP_LEN);
	return sys_get_le64(ts_buf);
}

static inline uint16_t dwt_fp_index_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg) {
	return sys_get_le16((uint8_t*)rx_inf_reg->rx_time + DWT_RX_TIME_FP_INDEX_OFFSET);
}

static inline uint16_t dwt_fp_ampl1_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg) {
	return sys_get_le16((uint8_t*)rx_inf_reg->rx_time + DWT_RX_TIME_FP_AMPL1_OFFSET);
}

static inline uint16_t dwt_std_noise_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg) {
	return sys_get_le16((uint8_t*)rx_inf_reg->rx_fqual + DWT_RX_FQUAL_STD_NOISE_OFFSET);
}

static inline uint16_t dwt_fp_ampl2_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg) {
	return sys_get_le16((uint8_t*)rx_inf_reg->rx_fqual + DWT_RX_FQUAL_FP_AMPL2_OFFSET);
}

static inline uint16_t dwt_fp_ampl3_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg) {
	return sys_get_le16((uint8_t*)rx_inf_reg->rx_fqual + DWT_RX_FQUAL_FP_AMPL3_OFFSET);
}

static inline uint16_t dwt_cir_pwr_from_info_reg(const struct dwt_rx_info_regs *rx_inf_reg) {
	return sys_get_le16(&rx_inf_reg->rx_fqual[DWT_RX_FQUAL_FP_CIR_PWR_OFFSET]);
}


static inline void dwt_double_buffering_align(const struct device *dev) {
	uint32_t sys_stat;
	sys_stat = dwt_reg_read_u32(dev, DWT_SYS_STATUS_ID, 0);
	if(((sys_stat & DWT_SYS_STATUS_ICRBP) > 0) !=  ((sys_stat & DWT_SYS_STATUS_HSRBP) > 0)) {
		dwt_reg_write_u32(dev, DWT_SYS_CTRL_ID, DWT_SYS_CTRL_OFFSET,
			DWT_SYS_CTRL_HSRBTOGGLE);
	}
}

static inline void dwt_double_buffering(const struct device *dev, uint8_t enable) {
	uint32_t sys_cfg;
	sys_cfg = dwt_reg_read_u32(dev, DWT_SYS_CFG_ID, 0);

	if(enable) {
		sys_cfg &= ~DWT_SYS_CFG_DIS_DRXB;
		sys_cfg &= ~DWT_SYS_CFG_RXAUTR;
	} else {
		sys_cfg |= DWT_SYS_CFG_DIS_DRXB;
	}

	dwt_reg_write_u32(dev, DWT_SYS_CFG_ID, 0, sys_cfg);

	dwt_double_buffering_align(dev);
}

/* timeout time in units of 1.026 microseconds */
static int dwt_enable_rx(const struct device *dev, uint16_t timeout, uint64_t dwt_rx_ts)
{
	struct dwt_context *ctx = dev->data;
	uint32_t sys_cfg;
	uint16_t sys_ctrl = DWT_SYS_CTRL_RXENAB;

	sys_cfg = dwt_reg_read_u32(dev, DWT_SYS_CFG_ID, 0);


#if DWT_DOUBLE_BUFFERING
	dwt_double_buffering_align(dev);
#endif

	if (timeout != 0) {
		dwt_reg_write_u16(dev, DWT_RX_FWTO_ID, DWT_RX_FWTO_OFFSET,
				  timeout);
		sys_cfg |= DWT_SYS_CFG_RXWTOE;
	} else {
		sys_cfg &= ~DWT_SYS_CFG_RXWTOE;
	}

	// program delayed receive
	if(dwt_rx_ts != 0) {
		dwt_reg_write_u32(dev, DWT_DX_TIME_ID, 1, (uint32_t) (dwt_rx_ts >> 8));
		sys_ctrl |= DWT_SYS_CTRL_RXDLYE;
	} else {
		sys_ctrl &= ~DWT_SYS_CTRL_RXDLYE;
	}

	if (atomic_test_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU)) {
		k_sem_reset(&ctx->phy_sem);
	}

	dwt_reg_write_u32(dev, DWT_SYS_CFG_ID, 0, sys_cfg);
	dwt_reg_write_u16(dev, DWT_SYS_CTRL_ID, DWT_SYS_CTRL_OFFSET, sys_ctrl);

	return 0;
}

static inline void dwt_irq_handle_rx_cca(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;

	ctx->cca_busy = true;

	/* Clear all RX event bits */
	dwt_reg_write_u32(dev, DWT_SYS_STATUS_ID, 0,
			  DWT_SYS_STATUS_ALL_RX_GOOD);
}

/* static inline void dwt_read_received_frame(const struct device *dev) { */
/*     uint32_t rx_finfo; */
/*     uint32_t rx_pacc; */
/*     uint16_t pkt_len; */

/*     rx_finfo = dwt_reg_read_u32(dev, DWT_RX_FINFO_ID, DWT_RX_FINFO_OFFSET); */
/*     pkt_len = rx_finfo & DWT_RX_FINFO_RXFLEN_MASK; */
/*     rx_pacc = (rx_finfo & DWT_RX_FINFO_RXPACC_MASK) >> DWT_RX_FINFO_RXPACC_SHIFT; */

/*     dwt_register_read(dev, DWT_RX_BUFFER_ID, 0, pkt_len, pkt->buffer->data); */
/* } */

static inline void dwt_irq_handle_rx(const struct device *dev, uint32_t sys_stat)
{
	struct dwt_context *ctx = dev->data;
	struct net_pkt *pkt = NULL;
	struct dwt_rx_info_regs rx_inf_reg;
	float a_const;
	uint32_t rx_finfo;
	uint32_t ttcki;
	uint32_t rx_pacc;
	uint32_t cir_pwr;
	uint32_t flags_to_clear;
	int32_t ttcko;
	uint16_t pkt_len;
	uint8_t *fctrl;
	int8_t rx_level = INT8_MIN;

	LOG_DBG("RX OK event, SYS_STATUS 0x%08x", sys_stat);
	flags_to_clear = sys_stat & DWT_SYS_STATUS_ALL_RX_GOOD;

	rx_finfo = dwt_reg_read_u32(dev, DWT_RX_FINFO_ID, DWT_RX_FINFO_OFFSET);

	pkt_len = rx_finfo & DWT_RX_FINFO_RXFLEN_MASK;
	rx_pacc = (rx_finfo & DWT_RX_FINFO_RXPACC_MASK) >>
		   DWT_RX_FINFO_RXPACC_SHIFT;

	if (!(IS_ENABLED(CONFIG_IEEE802154_RAW_MODE))) {
		pkt_len -= DWT_FCS_LENGTH;
	}

	pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, pkt_len,
					   AF_UNSPEC, 0, K_NO_WAIT);
	if (!pkt) {
		LOG_ERR("No buf available");
		goto rx_out_enable_rx;
	}

	dwt_register_read(dev, DWT_RX_BUFFER_ID, 0, pkt_len, pkt->buffer->data);

	dwt_read_rx_info(dev, &rx_inf_reg);

	net_buf_add(pkt->buffer, pkt_len);
	fctrl = pkt->buffer->data;

	/*
	 * Get Ranging tracking offset and tracking interval
	 * for Crystal characterization
	 */
	ttcki = sys_get_le32(rx_inf_reg.rx_ttcki);
	ttcko = sys_get_le32(rx_inf_reg.rx_ttcko) & DWT_RX_TTCKO_RXTOFS_MASK;
	/* Tracking offset value is a 19-bit signed integer */
	if (ttcko & BIT(18)) {
		ttcko |= ~DWT_RX_TTCKO_RXTOFS_MASK;
	}

	/* TODO add:
	 * net_pkt_set_ieee802154_tcki(pkt, ttcki);
	 * net_pkt_set_ieee802154_tcko(pkt, ttcko);
	 */
	LOG_DBG("ttcko %d ttcki: 0x%08x", ttcko, ttcki);

	if (IS_ENABLED(CONFIG_NET_PKT_TIMESTAMP)) {
		uint8_t ts_buf[sizeof(uint64_t)] = {0};
		uint64_t ts_nsec;

		memcpy(ts_buf, rx_inf_reg.rx_time, DWT_RX_TIME_RX_STAMP_LEN);

		uint64_t rx_ts = sys_get_le64(ts_buf);
		ctx->rx_ts = rx_ts;

		ts_nsec = rx_ts * DWT_TS_TIME_UNITS_FS;
		net_pkt_set_timestamp_ns(pkt, ts_nsec);

		// set phase offset, 7 unsigned 7 bits
		ctx->rx_ttcko_rc_phase = (rx_inf_reg.rx_ttcko[4] & 0x7FUL);
	}

	/* See 4.7.2 Estimating the receive signal power */
	cir_pwr = sys_get_le16(&rx_inf_reg.rx_fqual[6]);
	if (ctx->rf_cfg.prf == DWT_PRF_16M) {
		a_const = DWT_RX_SIG_PWR_A_CONST_PRF16;
	} else {
		a_const = DWT_RX_SIG_PWR_A_CONST_PRF64;
	}

	if (rx_pacc != 0) {

		// ctx->rx_cir_pwr = cir_pwr;
		// ctx->rx_pacc = rx_pacc;
		// ctx->rx_a_const = a_const

#if defined(CONFIG_NEWLIB_LIBC)
		/* From 4.7.2 Estimating the receive signal power */
		rx_level = 10.0 * log10f(cir_pwr * BIT(17) /
					 (rx_pacc * rx_pacc)) - a_const;
#endif
	}

	net_pkt_set_ieee802154_rssi_dbm(pkt, rx_level);

	/*
	 * Workaround for AAT status bit issue,
	 * From 5.3.5 Host Notification in DW1000 User Manual:
	 * "Note: there is a situation that can result in the AAT bit being set
	 * for the current frame as a result of a previous frame that was
	 * received and rejected due to frame filtering."
	 */
	if ((sys_stat & DWT_SYS_STATUS_AAT) && ((fctrl[0] & 0x20) == 0)) {
		flags_to_clear |= DWT_SYS_STATUS_AAT;
	}

	if (ieee802154_handle_ack(ctx->iface, pkt) == NET_OK) {
		LOG_INF("ACK packet handled");
		goto rx_out_unref_pkt;
	}

	/* LQI not implemented */
	LOG_DBG("Caught a packet (%u) (RSSI: %d)", pkt_len, rx_level);
	LOG_HEXDUMP_DBG(pkt->buffer->data, pkt_len, "RX buffer:");

	if (net_recv_data(ctx->iface, pkt) == NET_OK) {
		goto rx_out_enable_rx;
	} else {
		LOG_DBG("Packet dropped by NET stack");
	}

rx_out_unref_pkt:
	if (pkt) {
		net_pkt_unref(pkt);
	}

rx_out_enable_rx:
#if DWT_DOUBLE_BUFFERING
	dwt_switch_buffers(dev);
#endif

	dwt_reg_write_u32(dev, DWT_SYS_STATUS_ID, 0, flags_to_clear);
	LOG_DBG("Cleared SYS_STATUS flags 0x%08x", flags_to_clear);
	if (atomic_test_bit(&ctx->state, DWT_STATE_RX_DEF_ON)) {
		/*
		 * Re-enable reception but in contrast to dwt_enable_rx()
		 * without to read SYS_STATUS and set delayed option.
		 */
		dwt_reg_write_u16(dev, DWT_SYS_CTRL_ID, DWT_SYS_CTRL_OFFSET,
				  DWT_SYS_CTRL_RXENAB);
	}
}

static inline void dwt_irq_minimal_rx_handler(const struct device *dev, uint32_t sys_stat)
{
	struct dwt_context *ctx = dev->data;

	dwt_reg_write_u32(dev, DWT_SYS_STATUS_ID, 0, sys_stat & DWT_SYS_STATUS_ALL_RX_GOOD);

	ctx->phy_irq_event = DWT_IRQ_RX;
}

static void dwt_irq_handle_tx(const struct device *dev, uint32_t sys_stat)
{
	struct dwt_context *ctx = dev->data;

	/* Clear TX event bits */
	dwt_reg_write_u32(dev, DWT_SYS_STATUS_ID, 0,
			  DWT_SYS_STATUS_ALL_TX);

	if (atomic_test_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU)) {
		ctx->phy_irq_event = DWT_IRQ_TX;
	}
}

static void dwt_irq_handle_rxto(const struct device *dev, uint32_t sys_stat)
{
	struct dwt_context *ctx = dev->data;

	/* /\* Clear RX timeout event bits *\/ */
	/* dwt_reg_write_u32(dev, DWT_SYS_STATUS_ID, 0, */
	/* 		  DWT_SYS_STATUS_RXRFTO | DWT_SYS_STATUS_RXPTO); */
	if(sys_stat & DWT_SYS_STATUS_RXRFTO) {
		/* Receiver reset necessary, see 4.1.6 RX Message timestamp */
		dwt_disable_txrx(dev);
		dwt_reset_rfrx(dev);
		ctx->phy_irq_event = DWT_IRQ_FRAME_WAIT_TIMEOUT;
	} else {
		dwt_reg_write_u32(dev, DWT_SYS_STATUS_ID, 0, DWT_SYS_STATUS_ALL_RX_TO);
		ctx->phy_irq_event = DWT_IRQ_PREAMBLE_DETECT_TIMEOUT;
	}

	LOG_DBG("RX timeout event");

	if (atomic_test_bit(&ctx->state, DWT_STATE_CCA)) {
		ctx->cca_busy = false;
	}
}


// Whole handler takes roughly 90us
static void dwt_irq_handle_error(const struct device *dev, uint32_t sys_stat)
{
	struct dwt_context *ctx = dev->data;
	/* Clear RX error event bits */
	dwt_reg_write_u32(dev, DWT_SYS_STATUS_ID, 0, DWT_SYS_STATUS_ALL_RX_ERR);

	dwt_disable_txrx(dev);
	/* Receiver reset necessary, see 4.1.6 RX Message timestamp */
	dwt_reset_rfrx(dev);

	if (sys_stat & DWT_SYS_STATUS_RXPHE) {
		LOG_DBG("RX error DWT_SYS_STATUS_RXPHE");
	}

	if (sys_stat & DWT_SYS_STATUS_RXFCE) {
		LOG_DBG("RX error DWT_SYS_STATUS_RXFCE");
	}

	if (sys_stat & DWT_SYS_STATUS_RXRFSL) {
		LOG_DBG("RX error DWT_SYS_STATUS_RXRFSL");
	}

	if (sys_stat & DWT_SYS_STATUS_RXOVRR) {
		LOG_DBG("RX error DWT_SYS_STATUS_RXOVRR");
	}

	if (sys_stat & DWT_SYS_STATUS_RXSFDTO) {
		LOG_DBG("RX error DWT_SYS_STATUS_RXSFDTO");
	}

	if (sys_stat & DWT_SYS_STATUS_AFFREJ) {
		LOG_DBG("RX error DWT_SYS_STATUS_AFFREJ");
	}

	if (atomic_test_bit(&ctx->state, DWT_STATE_CCA)) {
		ctx->cca_busy = true;
	}

	if (atomic_test_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU)) {
		ctx->phy_irq_event = DWT_IRQ_ERR;
	}

	if (atomic_test_bit(&ctx->state, DWT_STATE_RX_DEF_ON)) {
		dwt_enable_rx(dev, 0, 0);
	}
}

static void dwt_irq_handle_half_delay(const struct device *dev, uint32_t sys_stat)
{
	struct dwt_context *ctx = dev->data;

	/* Clear RX error event bits */
	dwt_reg_write_u32(dev, DWT_SYS_STATUS_ID, 0, DWT_SYS_STATUS_HPDWARN);

	dwt_disable_txrx(dev);

	if (atomic_test_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU)) {
		ctx->phy_irq_event = DWT_IRQ_HALF_DELAY_WARNING;
	}
}


static void dwt_irq_work_handler(struct k_work *item)
{
	struct dwt_context *ctx = CONTAINER_OF(item, struct dwt_context,
					       irq_cb_work);
	const struct device *dev = ctx->dev;
	uint32_t sys_stat;
	uint8_t free_phybet = 0; // there might be other interrupt events upon which we don't want to free the phy semaphore

	k_sem_take(&ctx->dev_lock, K_FOREVER);

	sys_stat = dwt_reg_read_u32(dev, DWT_SYS_STATUS_ID, 0);

	if (sys_stat & DWT_SYS_STATUS_RXFCG) {
		if (atomic_test_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU)) {
			dwt_irq_minimal_rx_handler(dev, sys_stat);
		} else if (atomic_test_bit(&ctx->state, DWT_STATE_CCA)) {
			dwt_irq_handle_rx_cca(dev);
		} else {
			dwt_irq_handle_rx(dev, sys_stat);
		}
		free_phybet = 1;
	}

	if (sys_stat & DWT_SYS_STATUS_HPDWARN) {
		dwt_irq_handle_half_delay(dev, sys_stat);
		free_phybet = 1;
	}

	if (sys_stat & DWT_SYS_STATUS_TXFRS) {
		dwt_irq_handle_tx(dev, sys_stat);
		free_phybet = 1;
	}

	if (sys_stat & DWT_SYS_STATUS_ALL_RX_TO) {
		dwt_irq_handle_rxto(dev, sys_stat);
		free_phybet = 1;
	}

	if (sys_stat & DWT_SYS_STATUS_ALL_RX_ERR) {
		dwt_irq_handle_error(dev, sys_stat);
		free_phybet = 1;
	}

	ctx->phy_irq_sys_stat = sys_stat;

	k_sem_give(&ctx->dev_lock);

	if (atomic_test_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU) && free_phybet) {
		k_sem_give(&ctx->phy_sem);
	}
}

static void dwt_gpio_callback(const struct device *dev,
			      struct gpio_callback *cb, uint32_t pins)
{
	struct dwt_context *ctx = CONTAINER_OF(cb, struct dwt_context, gpio_cb);

	LOG_DBG("IRQ callback triggered %p", ctx);
	/* k_work_submit(&ctx->irq_cb_work); */

	k_work_submit_to_queue(&dwt_work_queue, &ctx->irq_cb_work);
}

static enum ieee802154_hw_caps dwt_get_capabilities(const struct device *dev)
{
	/* TODO: Implement HW-supported AUTOACK + frame pending bit handling. */
	return IEEE802154_HW_FCS | IEEE802154_HW_FILTER |
	       IEEE802154_HW_TXTIME;
}

static uint32_t dwt_get_pkt_duration_ns(struct dwt_context *ctx, uint8_t psdu_len)
{
	struct dwt_phy_config *rf_cfg = &ctx->rf_cfg;
	float t_psdu = rf_cfg->t_dsym * psdu_len * 8;

	return (rf_cfg->t_shr + rf_cfg->t_phr + t_psdu);
}

static int dwt_cca(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;
	uint32_t cca_dur = (dwt_get_pkt_duration_ns(ctx, 127) +
			 dwt_get_pkt_duration_ns(ctx, 5)) /
			 UWB_PHY_TDSYM_PHR_6M8;

	if (atomic_test_and_set_bit(&ctx->state, DWT_STATE_CCA)) {
		LOG_ERR("Transceiver busy");
		return -EBUSY;
	}

	/* Perform CCA Mode 5 */
	k_sem_take(&ctx->dev_lock, K_FOREVER);
	dwt_disable_txrx(dev);
	LOG_DBG("CCA duration %u us", cca_dur);

	dwt_enable_rx(dev, cca_dur, 0);
	k_sem_give(&ctx->dev_lock);

	k_sem_take(&ctx->phy_sem, K_FOREVER);
	LOG_DBG("CCA finished %p", ctx);

	atomic_clear_bit(&ctx->state, DWT_STATE_CCA);
	if (atomic_test_bit(&ctx->state, DWT_STATE_RX_DEF_ON)) {
		k_sem_take(&ctx->dev_lock, K_FOREVER);
		dwt_enable_rx(dev, 0, 0);
		k_sem_give(&ctx->dev_lock);
	}

	return ctx->cca_busy ? -EBUSY : 0;
}

static int dwt_ed(const struct device *dev, uint16_t duration,
		  energy_scan_done_cb_t done_cb)
{
	/* TODO: see description Sub-Register 0x23:02 – AGC_CTRL1 */

	return -ENOTSUP;
}

static int dwt_set_channel(const struct device *dev, uint16_t channel)
{
	struct dwt_context *ctx = dev->data;
	struct dwt_phy_config *rf_cfg = &ctx->rf_cfg;

	if (channel > 15) {
		return -EINVAL;
	}

	if (channel == 0 || channel == 6 || channel > 7) {
		return -ENOTSUP;
	}

	rf_cfg->channel = channel;
	LOG_INF("Set channel %u", channel);

	k_sem_take(&ctx->dev_lock, K_FOREVER);

	dwt_disable_txrx(dev);
	dwt_configure_rf_phy(dev);

	if (atomic_test_bit(&ctx->state, DWT_STATE_RX_DEF_ON)) {
		dwt_enable_rx(dev, 0, 0);
	}

	k_sem_give(&ctx->dev_lock);

	return 0;
}

static int dwt_set_pan_id(const struct device *dev, uint16_t pan_id)
{
	struct dwt_context *ctx = dev->data;

	k_sem_take(&ctx->dev_lock, K_FOREVER);
	dwt_reg_write_u16(dev, DWT_PANADR_ID, DWT_PANADR_PAN_ID_OFFSET, pan_id);
	k_sem_give(&ctx->dev_lock);

	LOG_INF("Set PAN ID 0x%04x %p", pan_id, ctx);

	return 0;
}

static int dwt_set_short_addr(const struct device *dev, uint16_t short_addr)
{
	struct dwt_context *ctx = dev->data;

	k_sem_take(&ctx->dev_lock, K_FOREVER);
	dwt_reg_write_u16(dev, DWT_PANADR_ID, DWT_PANADR_SHORT_ADDR_OFFSET,
			  short_addr);
	k_sem_give(&ctx->dev_lock);

	LOG_INF("Set short 0x%x %p", short_addr, ctx);

	return 0;
}

static int dwt_set_ieee_addr(const struct device *dev,
			     const uint8_t *ieee_addr)
{
	struct dwt_context *ctx = dev->data;

	LOG_INF("IEEE address %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
		ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
		ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);

	k_sem_take(&ctx->dev_lock, K_FOREVER);
	dwt_register_write(dev, DWT_EUI_64_ID, DWT_EUI_64_OFFSET,
			   DWT_EUI_64_LEN, (uint8_t *)ieee_addr);
	k_sem_give(&ctx->dev_lock);

	return 0;
}

static int dwt_filter(const struct device *dev,
		      bool set,
		      enum ieee802154_filter_type type,
		      const struct ieee802154_filter *filter)
{
	if (!set) {
		return -ENOTSUP;
	}

	if (type == IEEE802154_FILTER_TYPE_IEEE_ADDR) {
		return dwt_set_ieee_addr(dev, filter->ieee_addr);
	} else if (type == IEEE802154_FILTER_TYPE_SHORT_ADDR) {
		return dwt_set_short_addr(dev, filter->short_addr);
	} else if (type == IEEE802154_FILTER_TYPE_PAN_ID) {
		return dwt_set_pan_id(dev, filter->pan_id);
	}

	return -ENOTSUP;
}

static int dwt_set_power(const struct device *dev, int16_t dbm)
{
	struct dwt_context *ctx = dev->data;

	LOG_INF("set_txpower not supported %p", ctx);

	return 0;
}

#define DWT_TS_TO_US(X) (((X)*15650)/1000000000)
#define DWT_TS_MASK (0xFFFFFFFFFF)
#define UUS_TO_DWT_TS(X) (((uint64_t)X)*(uint64_t)65536)
#define NS_TO_DWT_TS(ns) ((((uint64_t)ns*1000*1000)/15650))

static inline int setup_tx_frame(const struct device *dev, const uint8_t *data, uint8_t len) {
	uint32_t tx_fctrl;
	uint8_t data_cpy[len];
	LOG_DBG("Setting up TX Frame");

	// copy data otherwise it will be overwritten by dwt_register_write
	memcpy(data_cpy, data, len);


	/*
	 * See "3 Message Transmission" in DW1000 User Manual for
	 * more details about transmission configuration.
	 */
	if (dwt_register_write(dev, DWT_TX_BUFFER_ID, 0, len, data_cpy)) {
		LOG_ERR("Failed to write TX data");
		return -EIO;
	}

	tx_fctrl = dwt_reg_read_u32(dev, DWT_TX_FCTRL_ID, 0);
	/* Clear TX buffer index offset, frame length, and length extension */
	tx_fctrl &= ~(DWT_TX_FCTRL_TFLEN_MASK | DWT_TX_FCTRL_TFLE_MASK |
		DWT_TX_FCTRL_TXBOFFS_MASK);
	/* Set frame length and ranging flag */
	tx_fctrl |= (len + DWT_FCS_LENGTH) & DWT_TX_FCTRL_TFLEN_MASK;
	tx_fctrl |= DWT_TX_FCTRL_TR;

	dwt_reg_write_u32(dev, DWT_TX_FCTRL_ID, 0, tx_fctrl);
	/* k_sem_give(&ctx->dev_lock); */

	return 0;
}

static inline int dwt_fast_enable_tx(const struct device *dev, uint64_t dwt_tx_ts) {
	struct dwt_context *ctx = dev->data;

	/* k_sem_take(&ctx->dev_lock, K_FOREVER); */
	uint8_t sys_ctrl = DWT_SYS_CTRL_TXSTRT;

	if(dwt_tx_ts > 0) {
		sys_ctrl |= DWT_SYS_CTRL_TXDLYS;
		dwt_reg_write_u32(dev, DWT_DX_TIME_ID, 1, (uint32_t) (dwt_tx_ts >> 8));
	}

	// we reset the phy semaphore back to 0. The tx interrupt routine will clear it again later.
	k_sem_reset(&ctx->phy_sem);
	dwt_reg_write_u16(dev, DWT_SYS_CTRL_ID, DWT_SYS_CTRL_OFFSET, sys_ctrl);

	return 0;
}

static inline int wait_for_phy(const struct device *dev) {
	struct dwt_context *ctx = dev->data;

	if (k_sem_take(&ctx->phy_sem, K_FOREVER));

	uint8_t irq_state = ctx->phy_irq_event;
	ctx->phy_irq_event = DWT_IRQ_NONE;

	return irq_state;
}



static int dwt_tx(const struct device *dev, enum ieee802154_tx_mode tx_mode,
	struct net_pkt *pkt, struct net_buf *frag)
{
	struct dwt_context *ctx = dev->data;
	size_t len = frag->len;
	uint32_t tx_time = 0;
	uint64_t tmp_fs;

	if (atomic_test_and_set_bit(&ctx->state, DWT_STATE_TX)) {
		LOG_ERR("Transceiver busy");
		return -EBUSY;
	}

	k_sem_take(&ctx->dev_lock, K_FOREVER);

	LOG_HEXDUMP_DBG(frag->data, len, "TX buffer:");

	switch (tx_mode) {
        case IEEE802154_TX_MODE_DIRECT:
		break;
        case IEEE802154_TX_MODE_TXTIME:
		/*
		 * tx_time is the high 32-bit of the 40-bit system
		 * time value at which to send the message.
		 */
		if (ctx->delayed_tx_short_ts_set) {
			tx_time = ctx->delayed_tx_short_ts;
			ctx->delayed_tx_short_ts_set = 0; // disable the flag again
		} else {
			tmp_fs = net_pkt_timestamp_ns(pkt);
			tmp_fs *= 1000U * 1000U;
			tx_time = (tmp_fs / DWT_TS_TIME_UNITS_FS) >> 8;
		}

		LOG_DBG("ntx hi32 %x", tx_time);
		LOG_DBG("sys hi32 %x",
			dwt_reg_read_u32(dev, DWT_SYS_TIME_ID, 1));

		break;
        default:
		LOG_ERR("TX mode %d not supported", tx_mode);
		goto error;
	}

	// setting up tx buffer, this operations takes up much of this function
	setup_tx_frame(dev, frag->data, len);

	dwt_disable_txrx(dev);

	dwt_fast_enable_tx(dev, tx_time << 8);

	k_sem_give(&ctx->dev_lock);

	wait_for_phy(dev);

	if (IS_ENABLED(CONFIG_NET_PKT_TIMESTAMP)) {
		uint8_t ts_buf[sizeof(uint64_t)] = {0};

		k_sem_take(&ctx->dev_lock, K_FOREVER);
		dwt_register_read(dev, DWT_TX_TIME_ID,
				  DWT_TX_TIME_TX_STAMP_OFFSET,
				  DWT_TX_TIME_TX_STAMP_LEN,
				  ts_buf);
		LOG_DBG("ts  hi32 %x", (uint32_t)(sys_get_le64(ts_buf) >> 8));
		LOG_DBG("sys hi32 %x",
			dwt_reg_read_u32(dev, DWT_SYS_TIME_ID, 1));
		k_sem_give(&ctx->dev_lock);

		tmp_fs = sys_get_le64(ts_buf) * DWT_TS_TIME_UNITS_FS;
		net_pkt_set_timestamp_ns(pkt, tmp_fs / 1000000U);
	}

	atomic_clear_bit(&ctx->state, DWT_STATE_TX);

	if (atomic_test_bit(&ctx->state, DWT_STATE_RX_DEF_ON)) {
		k_sem_take(&ctx->dev_lock, K_FOREVER);
		dwt_enable_rx(dev, 0, 0);
		k_sem_give(&ctx->dev_lock);
	}

	return 0;

error:
	atomic_clear_bit(&ctx->state, DWT_STATE_TX);
	k_sem_give(&ctx->dev_lock);

	return -EIO;
}

void dwt_set_frame_filter(const struct device *dev,
				 bool ff_enable, uint8_t ff_type)
{
	uint32_t sys_cfg_ff = ff_enable ? DWT_SYS_CFG_FFE : 0;

	sys_cfg_ff |= ff_type & DWT_SYS_CFG_FF_ALL_EN;

	dwt_reg_write_u8(dev, DWT_SYS_CFG_ID, 0, (uint8_t)sys_cfg_ff);
}

static int dwt_configure(const struct device *dev,
			 enum ieee802154_config_type type,
			 const struct ieee802154_config *config)
{
	struct dwt_context *ctx = dev->data;

	LOG_DBG("API configure %p", ctx);

	switch (type) {
	case IEEE802154_CONFIG_AUTO_ACK_FPB:
		LOG_DBG("IEEE802154_CONFIG_AUTO_ACK_FPB");
		break;

	case IEEE802154_CONFIG_ACK_FPB:
		LOG_DBG("IEEE802154_CONFIG_ACK_FPB");
		break;

	case IEEE802154_CONFIG_PAN_COORDINATOR:
		LOG_DBG("IEEE802154_CONFIG_PAN_COORDINATOR");
		break;

	case IEEE802154_CONFIG_PROMISCUOUS:
		LOG_DBG("IEEE802154_CONFIG_PROMISCUOUS");
		break;

	case IEEE802154_CONFIG_EVENT_HANDLER:
		LOG_DBG("IEEE802154_CONFIG_EVENT_HANDLER");
		break;

	default:
		return -EINVAL;
	}

	return -ENOTSUP;
}

/* driver-allocated attribute memory - constant across all driver instances */
static const struct {
	const struct ieee802154_phy_channel_range phy_channel_range[2];
	const struct ieee802154_phy_supported_channels phy_supported_channels;
} drv_attr = {
	.phy_channel_range = {
		{ .from_channel = 1, .to_channel = 5 },
		{ .from_channel = 7, .to_channel = 7 },
	},
	.phy_supported_channels = {
		.ranges = drv_attr.phy_channel_range,
		.num_ranges = 2U,
	},
};

static int dwt_attr_get(const struct device *dev, enum ieee802154_attr attr,
			struct ieee802154_attr_value *value)
{
	if (ieee802154_attr_get_channel_page_and_range(
		    attr, IEEE802154_ATTR_PHY_CHANNEL_PAGE_FOUR_HRP_UWB,
		    &drv_attr.phy_supported_channels, value) == 0) {
		return 0;
	}

	switch (attr) {
	case IEEE802154_ATTR_PHY_HRP_UWB_SUPPORTED_PRFS: {
		struct dwt_context *ctx = dev->data;
		struct dwt_phy_config *rf_cfg = &ctx->rf_cfg;

		value->phy_hrp_uwb_supported_nominal_prfs =
			rf_cfg->prf == DWT_PRF_64M ? IEEE802154_PHY_HRP_UWB_NOMINAL_64_M
						   : IEEE802154_PHY_HRP_UWB_NOMINAL_16_M;
		return 0;
	}

	default:
		return -ENOENT;
	}
}

/*
 * Note, the DW_RESET pin should not be driven high externally.
 */
static int dwt_hw_reset(const struct device *dev)
{
	const struct dwt_hi_cfg *hi_cfg = dev->config;

	if (gpio_pin_configure_dt(&hi_cfg->rst_gpio, GPIO_OUTPUT_ACTIVE)) {
		LOG_ERR("Failed to configure GPIO pin %u", hi_cfg->rst_gpio.pin);
		return -EINVAL;
	}

	k_sleep(K_MSEC(1));
	gpio_pin_set_dt(&hi_cfg->rst_gpio, 0);
	k_sleep(K_MSEC(5));

	if (gpio_pin_configure_dt(&hi_cfg->rst_gpio, GPIO_INPUT)) {
		LOG_ERR("Failed to configure GPIO pin %u", hi_cfg->rst_gpio.pin);
		return -EINVAL;
	}

	return 0;
}

/*
 * SPI speed in INIT state or for wake-up sequence,
 * see 2.3.2 Overview of main operational states
 */
static void dwt_set_spi_slow(const struct device *dev, const uint32_t freq)
{
	struct dwt_context *ctx = dev->data;

#if ZEPHYR_SPI
	ctx->spi_cfg_slow.frequency = freq;
	ctx->spi_cfg = &ctx->spi_cfg_slow;
#else
	ctx->spi_cfg.frequency = freq;
#if SPIM
	nrfx_spim_reconfigure(&spi, &ctx->spi_cfg);
#else
	nrfx_spi_reconfigure(&spi, &ctx->spi_cfg);
#endif
	/* pinctrl_apply_state(ctx->pcfg, PINCTRL_STATE_DEFAULT); */
#endif
}

/* SPI speed in IDLE, RX, and TX state */
static void dwt_set_spi_fast(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;

#if ZEPHYR_SPI
	const struct dwt_hi_cfg *hi_cfg = dev->config;
	ctx->spi_cfg = &hi_cfg->bus.config;
#else
	ctx->spi_cfg.frequency = DWT_SPI_FAST_FREQ;
#if SPIM
	nrfx_spim_reconfigure(&spi, &ctx->spi_cfg);
#else
	nrfx_spi_reconfigure(&spi, &ctx->spi_cfg);
#endif
	/* pinctrl_apply_state(ctx->pcfg, PINCTRL_STATE_DEFAULT); */
#endif
}

static void dwt_set_rx_mode(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;
	struct dwt_phy_config *rf_cfg = &ctx->rf_cfg;
	uint32_t pmsc_ctrl0;
	uint32_t t_on_us;
	uint8_t rx_sniff[2];

	/* SNIFF Mode ON time in units of PAC */
	rx_sniff[0] = CONFIG_IEEE802154_DW1000_SNIFF_ONT &
		      DWT_RX_SNIFF_SNIFF_ONT_MASK;
	/* SNIFF Mode OFF time in microseconds */
	rx_sniff[1] = CONFIG_IEEE802154_DW1000_SNIFF_OFFT;

	t_on_us = (rx_sniff[0] + 1) * (BIT(3) << rf_cfg->rx_pac_l);
	LOG_INF("RX duty cycle %u%%", t_on_us * 100 / (t_on_us + rx_sniff[1]));

	dwt_register_write(dev, DWT_RX_SNIFF_ID, DWT_RX_SNIFF_OFFSET,
			   sizeof(rx_sniff), rx_sniff);

	pmsc_ctrl0 = dwt_reg_read_u32(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_OFFSET);
	/* Enable PLL2 on/off sequencing for SNIFF mode */
	pmsc_ctrl0 |= DWT_PMSC_CTRL0_PLL2_SEQ_EN;
	dwt_reg_write_u32(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_OFFSET, pmsc_ctrl0);
}

static int dwt_start(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;
	uint8_t cswakeup_buf[32] = {0};

	k_sem_take(&ctx->dev_lock, K_FOREVER);

	/* Set SPI clock to lowest frequency */
	dwt_set_spi_slow(dev, DWT_SPI_CSWAKEUP_FREQ);

	if (dwt_reg_read_u32(dev, DWT_DEV_ID_ID, 0) != DWT_DEVICE_ID) {
		/* Keep SPI CS line low for 500 microseconds */
		dwt_register_read(dev, 0, 0, sizeof(cswakeup_buf),
				  cswakeup_buf);
		/* Give device time to initialize */
		k_sleep(K_MSEC(5));

		if (dwt_reg_read_u32(dev, DWT_DEV_ID_ID, 0) != DWT_DEVICE_ID) {
			LOG_ERR("Failed to wake-up %p", dev);
			k_sem_give(&ctx->dev_lock);
			return -1;
		}
	} else {
		LOG_WRN("Device not in a sleep mode");
	}

	/* Restore SPI clock settings */
	dwt_set_spi_slow(dev, DWT_SPI_SLOW_FREQ);
	dwt_set_spi_fast(dev);

	dwt_setup_int(dev, true);
	dwt_disable_txrx(dev);
	dwt_reset_rfrx(dev);

	if (CONFIG_IEEE802154_DW1000_SNIFF_ONT != 0) {
		dwt_set_rx_mode(dev);
	}

	/* Re-enable RX after packet reception */
#if DWT_AUTO_REENABLE_RX_AFTER_TX
	atomic_set_bit(&ctx->state, DWT_STATE_RX_DEF_ON);
	dwt_enable_rx(dev, 0, 0);
#endif

	k_sem_give(&ctx->dev_lock);

	LOG_INF("Started %p", dev);

	return 0;
}

static int dwt_stop(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;

	k_sem_take(&ctx->dev_lock, K_FOREVER);
	dwt_disable_txrx(dev);
	dwt_reset_rfrx(dev);
	dwt_setup_int(dev, false);

	/* Copy the user configuration and enter sleep mode */
	dwt_reg_write_u8(dev, DWT_AON_ID, DWT_AON_CTRL_OFFSET,
			 DWT_AON_CTRL_SAVE);
	k_sem_give(&ctx->dev_lock);

	LOG_INF("Stopped %p", dev);

	return 0;
}

static inline void dwt_set_sysclks_xti(const struct device *dev, bool ldeload)
{
	uint16_t clks = BIT(9) | DWT_PMSC_CTRL0_SYSCLKS_19M;

	/*
	 * See Table 4: Register accesses required to load LDE microcode,
	 * set PMSC_CTRL0 0x0301, load LDE, set PMSC_CTRL0 0x0200.
	 */
	if (ldeload) {
		clks |= BIT(8);
	}

	/* Force system clock to be the 19.2 MHz XTI clock */
	dwt_reg_write_u16(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_OFFSET, clks);
}

static inline void dwt_set_sysclks_auto(const struct device *dev)
{
	uint8_t sclks = DWT_PMSC_CTRL0_SYSCLKS_AUTO |
		     DWT_PMSC_CTRL0_RXCLKS_AUTO |
		     DWT_PMSC_CTRL0_TXCLKS_AUTO;

	dwt_reg_write_u8(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_OFFSET, sclks);
}


static inline void dwt_enable_accumulator_memory_access(const struct device *dev) {
	uint8_t sclks = dwt_reg_read_u8(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_OFFSET);

	// clear previous value of RXCLK
	sclks = (sclks & ~DWT_PMSC_CTRL0_RXCLKS_MASK);
	sclks |= DWT_PMSC_CTRL0_FACE | DWT_PMSC_CTRL0_AMCE | DWT_PMSC_CTRL0_RXCLKS_125M;

	dwt_reg_write_u8(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_OFFSET, sclks);
}

static inline void dwt_disable_accumulator_memory_access(const struct device *dev) {
	uint8_t sclks = dwt_reg_read_u8(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_OFFSET);

	// clear previous value of RXCLK
	sclks = (sclks & ~DWT_PMSC_CTRL0_RXCLKS_MASK) | DWT_PMSC_CTRL0_RXCLKS_AUTO;
	sclks &= ~(DWT_PMSC_CTRL0_FACE | DWT_PMSC_CTRL0_AMCE);

	dwt_reg_write_u8(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_OFFSET, sclks);
}

static uint32_t dwt_otpmem_read(const struct device *dev, uint16_t otp_addr)
{
	dwt_reg_write_u16(dev, DWT_OTP_IF_ID, DWT_OTP_ADDR, otp_addr);

	dwt_reg_write_u8(dev, DWT_OTP_IF_ID, DWT_OTP_CTRL,
				DWT_OTP_CTRL_OTPREAD | DWT_OTP_CTRL_OTPRDEN);
	/* OTPREAD is self clearing but OTPRDEN is not */
	dwt_reg_write_u8(dev, DWT_OTP_IF_ID, DWT_OTP_CTRL, 0x00);

	/* Read read data, available 40ns after rising edge of OTP_READ */
	return dwt_reg_read_u32(dev, DWT_OTP_IF_ID, DWT_OTP_RDAT);
}

void dwt_set_antenna_delay_rx(const struct device *dev, uint16_t rx_delay_ts)
{
	struct dwt_context *ctx = dev->data;
	ctx->rx_ant_dly = rx_delay_ts;
	dwt_reg_write_u16(dev, DWT_LDE_IF_ID, DWT_LDE_RXANTD_OFFSET, rx_delay_ts);
}

void dwt_set_antenna_delay_tx(const struct device *dev, uint16_t tx_delay_ts)
{
	struct dwt_context *ctx = dev->data;
	ctx->tx_ant_dly = tx_delay_ts;
	dwt_reg_write_u16(dev, DWT_TX_ANTD_ID, DWT_TX_ANTD_OFFSET, tx_delay_ts);
}
uint16_t dwt_antenna_delay_rx(const struct device *dev)
{
	return dwt_reg_read_u16(dev, DWT_LDE_IF_ID, DWT_LDE_RXANTD_OFFSET);
}

inline uint16_t dwt_antenna_delay_tx(const struct device *dev)
{
	return dwt_reg_read_u16(dev, DWT_TX_ANTD_ID, DWT_TX_ANTD_OFFSET);
}

uint32_t dwt_otp_antenna_delay(const struct device *dev)
{
	return dwt_otpmem_read(dev, DWT_OTP_DELAY_ADDR);
}

inline void dwt_set_delayed_tx_short_ts(const struct device *dev, uint32_t short_ts)
{
	struct dwt_context *ctx = dev->data;
	ctx->delayed_tx_short_ts = short_ts;
	ctx->delayed_tx_short_ts_set = 1;
}

inline uint64_t dwt_system_ts(const struct device *dev)
{
	uint8_t ts_buf[sizeof(uint64_t)] = {0};

	// TODO: Should we lock?!
	/* k_sem_take(&ctx->dev_lock, K_FOREVER); */
	dwt_register_read(dev, DWT_SYS_TIME_ID, DWT_SYS_TIME_OFFSET, DWT_SYS_TIME_LEN, ts_buf);
	/* k_sem_give(&ctx->dev_lock); */
	return sys_get_le64(ts_buf);
}

inline uint32_t dwt_system_short_ts(const struct device *dev)
{
	uint64_t ts = dwt_system_ts(dev);
	return (uint32_t)(ts >> 8);
}

uint64_t dwt_ts_to_fs(uint64_t ts)
{
	return ts * DWT_TS_TIME_UNITS_FS;
}

uint64_t dwt_fs_to_ts(uint64_t fs)
{
	return fs / DWT_TS_TIME_UNITS_FS;
}

uint64_t dwt_short_ts_to_fs(uint32_t ts)
{
	uint64_t tmp_ts = (uint64_t)ts;
	return (tmp_ts << 8) * DWT_TS_TIME_UNITS_FS;
}

uint32_t dwt_fs_to_short_ts(uint64_t fs)
{
	return (fs / DWT_TS_TIME_UNITS_FS) >> 8;
}

inline uint64_t dwt_estimate_actual_tx_ts(uint32_t planned_short_ts, uint16_t tx_antenna_delay)
{
	return (((uint64_t)(planned_short_ts & 0xFFFFFFFEUL)) << 8) + tx_antenna_delay;
}

uint64_t dwt_plan_delayed_tx(const struct device *dev, uint64_t uus_delay)
{
	// query the antenna delay before
	uint16_t antenna_delay = dwt_antenna_delay_tx(dev);
	uint64_t cur_ts = dwt_system_ts(dev);
	/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
	 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
	uint32_t planned_tx_short_ts = (cur_ts + (uus_delay * 65536)) >> 8;
	// LOG_WRN("current %u planned %u", (uint32_t)(cur_ts >> 8), planned_tx_short_ts);
	dwt_set_delayed_tx_short_ts(dev, planned_tx_short_ts);

	uint64_t estimated_ts = dwt_estimate_actual_tx_ts(planned_tx_short_ts, antenna_delay);
	// LOG_WRN("Current: %llu, Plan: %llu, Est: %llu", cur_ts, (uint64_t)(planned_tx_short_ts)
	// << 8, estimated_ts); LOG_WRN("Current: %llu, Plan: %llu, Est: %llu", dwt_ts_to_fs(cur_ts)
	// /  1000000000U, dwt_ts_to_fs((uint64_t)(planned_tx_short_ts) << 8) /  1000000000U,
	// dwt_ts_to_fs(estimated_ts) /  1000000000U);

	return estimated_ts;
}

uint64_t dwt_rx_ts(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;
	return ctx->rx_ts;
}

uint8_t dwt_rx_ttcko_rc_phase(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;
	return ctx->rx_ttcko_rc_phase;
}

#define B20_SIGN_EXTEND_TEST (0x00100000UL)
#define B20_SIGN_EXTEND_MASK (0xFFF00000UL)

int dwt_readcarrierintegrator(const struct device *dev)
{
	uint32_t regval = 0;
	uint8_t buf[DWT_DRX_CARRIER_INT_LEN];
	/* Read 3 bytes into buffer (21-bit quantity) */
	dwt_spi_transfer(dev, DWT_DRX_CONF_ID, DWT_DRX_CARRIER_INT_OFFSET, sizeof(buf), buf, false);

	for (int j = 2; j >= 0; j--) // arrange the three bytes into an unsigned integer value
	{
		regval = (regval << 8) + buf[j];
	}

	if (regval & B20_SIGN_EXTEND_TEST) {
		regval |= B20_SIGN_EXTEND_MASK; // sign extend bit #20 to whole word
	} else {
		regval &= DWT_DRX_CARRIER_INT_MASK; // make sure upper bits are clear if not sign
						    // extending
	}

	return (int)regval; // cast unsigned value to signed quantity.
}

// Multiplication factors to convert carrier integrator value to a frequency offset in Hertz

#define DWT_FREQ_OFFSET_MULTIPLIER	 (998.4e6 / 2.0 / 1024.0 / 131072.0)
#define DWT_FREQ_OFFSET_MULTIPLIER_110KB (998.4e6 / 2.0 / 8192.0 / 131072.0)

// Multiplication factors to convert frequency offset in Hertz to PPM crystal offset
// NB: also changes sign so a positive value means the local RX clock is running slower than the
// remote TX device.

#define DWT_HERTZ_TO_PPM_MULTIPLIER_CHAN_1 (-1.0e6 / 3494.4e6)
#define DWT_HERTZ_TO_PPM_MULTIPLIER_CHAN_2 (-1.0e6 / 3993.6e6)
#define DWT_HERTZ_TO_PPM_MULTIPLIER_CHAN_3 (-1.0e6 / 4492.8e6)
#define DWT_HERTZ_TO_PPM_MULTIPLIER_CHAN_5 (-1.0e6 / 6489.6e6)

float dwt_rx_clock_ratio_offset(const struct device *dev)
{
	// TODO: Use correct channel!
	return dwt_readcarrierintegrator(dev) *
	       (DWT_FREQ_OFFSET_MULTIPLIER * DWT_HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);
}

static int dwt_initialise_dev(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;
	uint32_t otp_val = 0;
	uint32_t otp_antenna_delay = 0;
	uint8_t xtal_trim;

	ctx->delayed_tx_short_ts_set = 0;
	ctx->delayed_tx_short_ts = 0;

	dwt_set_sysclks_xti(dev, false);
	ctx->sleep_mode = 0;

	/* Disable PMSC control of analog RF subsystem */
	dwt_reg_write_u16(dev, DWT_PMSC_ID, DWT_PMSC_CTRL1_OFFSET,
			  DWT_PMSC_CTRL1_PKTSEQ_DISABLE);

	/* Clear all status flags */
	dwt_reg_write_u32(dev, DWT_SYS_STATUS_ID, 0, DWT_SYS_STATUS_MASK_32);

	/*
	 * Apply soft reset,
	 * see SOFTRESET field description in DW1000 User Manual.
	 */
	dwt_reg_write_u8(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_SOFTRESET_OFFSET,
			 DWT_PMSC_CTRL0_RESET_ALL);
	k_sleep(K_MSEC(1));
	dwt_reg_write_u8(dev, DWT_PMSC_ID, DWT_PMSC_CTRL0_SOFTRESET_OFFSET,
			 DWT_PMSC_CTRL0_RESET_CLEAR);

	dwt_set_sysclks_xti(dev, false);

	/*
	 * This bit (a.k.a PLLLDT) should be set to ensure reliable
	 * operation of the CPLOCK bit.
	 */
	dwt_reg_write_u8(dev, DWT_EXT_SYNC_ID, DWT_EC_CTRL_OFFSET,
			 DWT_EC_CTRL_PLLLCK);

	/* Kick LDO if there is a value programmed. */
	otp_val = dwt_otpmem_read(dev, DWT_OTP_LDOTUNE_ADDR);
	if ((otp_val & 0xFF) != 0) {
		dwt_reg_write_u8(dev, DWT_OTP_IF_ID, DWT_OTP_SF,
				 DWT_OTP_SF_LDO_KICK);
		ctx->sleep_mode |= DWT_AON_WCFG_ONW_LLDO;
		LOG_INF("Load LDOTUNE_CAL parameter");
	}

	otp_val = dwt_otpmem_read(dev, DWT_OTP_XTRIM_ADDR);
	xtal_trim = otp_val & DWT_FS_XTALT_MASK;
	LOG_INF("OTP Revision 0x%02x, XTAL Trim 0x%02x",
		(uint8_t)(otp_val >> 8), xtal_trim);

	LOG_DBG("CHIP ID 0x%08x", dwt_otpmem_read(dev, DWT_OTP_PARTID_ADDR));
	LOG_DBG("LOT ID 0x%08x", dwt_otpmem_read(dev, DWT_OTP_LOTID_ADDR));
	LOG_DBG("Vbat 0x%02x", dwt_otpmem_read(dev, DWT_OTP_VBAT_ADDR));
	LOG_DBG("Vtemp 0x%02x", dwt_otpmem_read(dev, DWT_OTP_VTEMP_ADDR));

	otp_antenna_delay = dwt_otpmem_read(dev, DWT_OTP_DELAY_ADDR);
	LOG_INF("OTP AntDelay 0x%02x", otp_antenna_delay); // TODO: change this to a debug log

	if (xtal_trim == 0) {
		/* Set to default */
		xtal_trim = DWT_FS_XTALT_MIDRANGE;
	}

	/* For FS_XTALT bits 7:5 must always be set to binary “011” */
	xtal_trim |= BIT(6) | BIT(5);
	dwt_reg_write_u8(dev, DWT_FS_CTRL_ID, DWT_FS_XTALT_OFFSET, xtal_trim);

	/* Load LDE microcode into RAM, see 2.5.5.10 LDELOAD */
	dwt_set_sysclks_xti(dev, true);
	dwt_reg_write_u16(dev, DWT_OTP_IF_ID, DWT_OTP_CTRL,
			  DWT_OTP_CTRL_LDELOAD);
	k_sleep(K_MSEC(1));
	dwt_set_sysclks_xti(dev, false);
	ctx->sleep_mode |= DWT_AON_WCFG_ONW_LLDE;

	dwt_set_sysclks_auto(dev);

	if (!(dwt_reg_read_u8(dev, DWT_SYS_STATUS_ID, 0) &
	     DWT_SYS_STATUS_CPLOCK)) {
		LOG_WRN("PLL has not locked");
		return -EIO;
	}

	dwt_set_spi_fast(dev);

	/* Setup default antenna delay values */
#if CONFIG_IEEE802154_DW1000_OTP_ANTENNA_DELAY
	dwt_set_antenna_delay_rx(dev, otp_antenna_delay & 0xFFFF);
	dwt_set_antenna_delay_tx(dev, otp_antenna_delay & 0xFFFF);
	LOG_WRN("Loading antenna delay (%u) from OTP", otp_antenna_delay & 0xFFFF);
#else
	dwt_set_antenna_delay_rx(dev, DW1000_RX_ANT_DLY);
	dwt_set_antenna_delay_tx(dev, DW1000_TX_ANT_DLY);
#endif

 	/* Clear AON_CFG1 register */
	dwt_reg_write_u8(dev, DWT_AON_ID, DWT_AON_CFG1_OFFSET, 0);
	/*
	 * Configure sleep mode:
	 *  - On wake-up load configurations from the AON memory
	 *  - preserve sleep mode configuration
	 *  - On Wake-up load the LDE microcode
	 *  - When available, on wake-up load the LDO tune value
	 */
	ctx->sleep_mode |= DWT_AON_WCFG_ONW_LDC |
			   DWT_AON_WCFG_PRES_SLEEP;
	dwt_reg_write_u16(dev, DWT_AON_ID, DWT_AON_WCFG_OFFSET,
			  ctx->sleep_mode);
	LOG_DBG("sleep mode 0x%04x", ctx->sleep_mode);
	/* Enable sleep and wake using SPI CSn */
	dwt_reg_write_u8(dev, DWT_AON_ID, DWT_AON_CFG0_OFFSET,
			 DWT_AON_CFG0_WAKE_SPI | DWT_AON_CFG0_SLEEP_EN);

#if DWT_DOUBLE_BUFFERING
	dwt_double_buffering(dev, 1);
#endif

	return 0;
}

/*
 * RF PHY configuration. Must be carried out as part of initialization and
 * for every channel change. See also 2.5 Default Configuration on Power Up.
 */
static int dwt_configure_rf_phy(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;
	struct dwt_phy_config *rf_cfg = &ctx->rf_cfg;
	uint8_t chan = rf_cfg->channel;
	uint8_t prf_idx = rf_cfg->prf;
	uint32_t chan_ctrl = 0;
	uint8_t rxctrlh;
	uint8_t pll_tune;
	uint8_t tune4h;
	uint8_t pgdelay;
	uint16_t lde_repc;
	uint16_t agc_tune1;
	uint16_t sfdto;
	uint16_t tune1a;
	uint16_t tune0b;
	uint16_t tune1b;
	uint32_t txctrl;
	uint32_t pll_cfg;
	uint32_t tune2;
	uint32_t sys_cfg;
	uint32_t tx_fctrl;
	uint32_t power;

	if ((chan < 1) || (chan > 7) || (chan == 6)) {
		LOG_ERR("Channel not supported %u", chan);
		return -ENOTSUP;
	}

	if (rf_cfg->rx_shr_code >= ARRAY_SIZE(dwt_lde_repc_defs)) {
		LOG_ERR("Preamble code not supported %u",
			rf_cfg->rx_shr_code);
		return -ENOTSUP;
	}

	if (prf_idx >= DWT_NUMOF_PRFS) {
		LOG_ERR("PRF not supported %u", prf_idx);
		return -ENOTSUP;
	}

	if (rf_cfg->rx_pac_l >= DWT_NUMOF_PACS) {
		LOG_ERR("RX PAC not supported %u", rf_cfg->rx_pac_l);
		return -ENOTSUP;
	}

	if (rf_cfg->rx_ns_sfd > 1) {
		LOG_ERR("Wrong NS SFD configuration");
		return -ENOTSUP;
	}

	if (rf_cfg->tx_shr_nsync >= DWT_NUM_OF_PLEN) {
		LOG_ERR("Wrong SHR configuration");
		return -ENOTSUP;
	}

	lde_repc = dwt_lde_repc_defs[rf_cfg->rx_shr_code];
	agc_tune1 = dwt_agc_tune1_defs[prf_idx];
	sfdto = rf_cfg->rx_sfd_to;
	rxctrlh = dwt_rxctrlh_defs[dwt_ch_to_cfg[chan]];
	txctrl = dwt_txctrl_defs[dwt_ch_to_cfg[chan]];
	pll_tune = dwt_plltune_defs[dwt_ch_to_cfg[chan]];
	pll_cfg = dwt_pllcfg_defs[dwt_ch_to_cfg[chan]];
	tune2 = dwt_tune2_defs[prf_idx][rf_cfg->rx_pac_l];
	tune1a = dwt_tune1a_defs[prf_idx];
	tune0b = dwt_tune0b_defs[rf_cfg->dr][rf_cfg->rx_ns_sfd];
	pgdelay = dwt_pgdelay_defs[dwt_ch_to_cfg[chan]];

	sys_cfg = dwt_reg_read_u32(dev, DWT_SYS_CFG_ID, 0);
	tx_fctrl = dwt_reg_read_u32(dev, DWT_TX_FCTRL_ID, 0);

	/* Don't allow 0 - SFD timeout will always be enabled */
	if (sfdto == 0) {
		sfdto = DWT_SFDTOC_DEF;
	}

	/* Set IEEE 802.15.4 compliant mode */
	sys_cfg &= ~DWT_SYS_CFG_PHR_MODE_11;

	if (rf_cfg->dr == DWT_BR_110K) {
		/* Set Receiver Mode 110 kbps data rate */
		sys_cfg |= DWT_SYS_CFG_RXM110K;
		lde_repc = lde_repc >> 3;
		tune1b = DWT_DRX_TUNE1b_110K;
		tune4h = DWT_DRX_TUNE4H_PRE64;
	} else {
		sys_cfg &= ~DWT_SYS_CFG_RXM110K;
		if (rf_cfg->tx_shr_nsync == DWT_PLEN_64) {
			tune1b = DWT_DRX_TUNE1b_6M8_PRE64;
			tune4h = DWT_DRX_TUNE4H_PRE64;
		} else {
			tune1b = DWT_DRX_TUNE1b_850K_6M8;
			tune4h = DWT_DRX_TUNE4H_PRE128PLUS;
		}
	}

	if (sys_cfg & DWT_SYS_CFG_DIS_STXP) {
		if (rf_cfg->prf == DWT_PRF_64M) {
			power = dwt_txpwr_stxp1_64[dwt_ch_to_cfg[chan]];
		} else {
			power = dwt_txpwr_stxp1_16[dwt_ch_to_cfg[chan]];
		}
	} else {
		if (rf_cfg->prf == DWT_PRF_64M) {
			power = dwt_txpwr_stxp0_64[dwt_ch_to_cfg[chan]];
		} else {
			power = dwt_txpwr_stxp0_16[dwt_ch_to_cfg[chan]];
		}
	}

	dwt_reg_write_u32(dev, DWT_SYS_CFG_ID, 0, sys_cfg);
	LOG_DBG("SYS_CFG: 0x%08x", sys_cfg);

	dwt_reg_write_u16(dev, DWT_LDE_IF_ID, DWT_LDE_REPC_OFFSET, lde_repc);
	LOG_DBG("LDE_REPC: 0x%04x", lde_repc);

	dwt_reg_write_u8(dev, DWT_LDE_IF_ID, DWT_LDE_CFG1_OFFSET,
			 DWT_DEFAULT_LDE_CFG1);

	if (rf_cfg->prf == DWT_PRF_64M) {
		dwt_reg_write_u16(dev, DWT_LDE_IF_ID, DWT_LDE_CFG2_OFFSET,
				  DWT_DEFAULT_LDE_CFG2_PRF64);
		LOG_DBG("LDE_CFG2: 0x%04x", DWT_DEFAULT_LDE_CFG2_PRF64);
	} else {
		dwt_reg_write_u16(dev, DWT_LDE_IF_ID, DWT_LDE_CFG2_OFFSET,
				  DWT_DEFAULT_LDE_CFG2_PRF16);
		LOG_DBG("LDE_CFG2: 0x%04x", DWT_DEFAULT_LDE_CFG2_PRF16);
	}

	/* Configure PLL2/RF PLL block CFG/TUNE (for a given channel) */
	dwt_reg_write_u32(dev, DWT_FS_CTRL_ID, DWT_FS_PLLCFG_OFFSET, pll_cfg);
	LOG_DBG("PLLCFG: 0x%08x", pll_cfg);
	dwt_reg_write_u8(dev, DWT_FS_CTRL_ID, DWT_FS_PLLTUNE_OFFSET, pll_tune);
	LOG_DBG("PLLTUNE: 0x%02x", pll_tune);
	/* Configure RF RX blocks (for specified channel/bandwidth) */
	dwt_reg_write_u8(dev, DWT_RF_CONF_ID, DWT_RF_RXCTRLH_OFFSET, rxctrlh);
	LOG_DBG("RXCTRLH: 0x%02x", rxctrlh);
	/* Configure RF/TX blocks for specified channel and PRF */
	dwt_reg_write_u32(dev, DWT_RF_CONF_ID, DWT_RF_TXCTRL_OFFSET, txctrl);
	LOG_DBG("TXCTRL: 0x%08x", txctrl);

	/* Digital receiver configuration, DRX_CONF */
	dwt_reg_write_u16(dev, DWT_DRX_CONF_ID, DWT_DRX_TUNE0b_OFFSET, tune0b);
	LOG_DBG("DRX_TUNE0b: 0x%04x", tune0b);
	dwt_reg_write_u16(dev, DWT_DRX_CONF_ID, DWT_DRX_TUNE1a_OFFSET, tune1a);
	LOG_DBG("DRX_TUNE1a: 0x%04x", tune1a);
	dwt_reg_write_u16(dev, DWT_DRX_CONF_ID, DWT_DRX_TUNE1b_OFFSET, tune1b);
	LOG_DBG("DRX_TUNE1b: 0x%04x", tune1b);
	dwt_reg_write_u32(dev, DWT_DRX_CONF_ID, DWT_DRX_TUNE2_OFFSET, tune2);
	LOG_DBG("DRX_TUNE2: 0x%08x", tune2);
	dwt_reg_write_u8(dev, DWT_DRX_CONF_ID, DWT_DRX_TUNE4H_OFFSET, tune4h);
	LOG_DBG("DRX_TUNE4H: 0x%02x", tune4h);
	dwt_reg_write_u16(dev, DWT_DRX_CONF_ID, DWT_DRX_SFDTOC_OFFSET, sfdto);
	LOG_DBG("DRX_SFDTOC: 0x%04x", sfdto);

	/* Automatic Gain Control configuration and control, AGC_CTRL */
	dwt_reg_write_u16(dev, DWT_AGC_CTRL_ID, DWT_AGC_TUNE1_OFFSET,
			  agc_tune1);
	LOG_DBG("AGC_TUNE1: 0x%04x", agc_tune1);
	dwt_reg_write_u32(dev, DWT_AGC_CTRL_ID, DWT_AGC_TUNE2_OFFSET,
			  DWT_AGC_TUNE2_VAL);

	if (rf_cfg->rx_ns_sfd) {
		/*
		 * SFD_LENGTH, length of the SFD sequence used when
		 * the data rate is 850 kbps or 6.8 Mbps,
		 * must be set to either 8 or 16.
		 */
		dwt_reg_write_u8(dev, DWT_USR_SFD_ID, 0x00,
				 dwt_ns_sfdlen[rf_cfg->dr]);
		LOG_DBG("USR_SFDLEN: 0x%02x", dwt_ns_sfdlen[rf_cfg->dr]);
		chan_ctrl |= DWT_CHAN_CTRL_DWSFD;
	}

	/* Set RX_CHAN and TX CHAN */
	chan_ctrl |= (chan & DWT_CHAN_CTRL_TX_CHAN_MASK) |
		     ((chan << DWT_CHAN_CTRL_RX_CHAN_SHIFT) &
		      DWT_CHAN_CTRL_RX_CHAN_MASK);

	/* Set RXPRF */
	chan_ctrl |= (BIT(rf_cfg->prf) << DWT_CHAN_CTRL_RXFPRF_SHIFT) &
		     DWT_CHAN_CTRL_RXFPRF_MASK;

	/* Set TX_PCOD */
	chan_ctrl |= (rf_cfg->tx_shr_code << DWT_CHAN_CTRL_TX_PCOD_SHIFT) &
		     DWT_CHAN_CTRL_TX_PCOD_MASK;

	/* Set RX_PCOD */
	chan_ctrl |= (rf_cfg->rx_shr_code << DWT_CHAN_CTRL_RX_PCOD_SHIFT) &
		     DWT_CHAN_CTRL_RX_PCOD_MASK;

	/* Set Channel Control */
	dwt_reg_write_u32(dev, DWT_CHAN_CTRL_ID, 0, chan_ctrl);
	LOG_DBG("CHAN_CTRL 0x%08x", chan_ctrl);

	/* Set up TX Preamble Size, PRF and Data Rate */
	tx_fctrl = dwt_plen_cfg[rf_cfg->tx_shr_nsync] |
		   (BIT(rf_cfg->prf) << DWT_TX_FCTRL_TXPRF_SHFT) |
		   (rf_cfg->dr << DWT_TX_FCTRL_TXBR_SHFT);

	dwt_reg_write_u32(dev, DWT_TX_FCTRL_ID, 0, tx_fctrl);
	LOG_DBG("TX_FCTRL 0x%08x", tx_fctrl);

	/* Set the Pulse Generator Delay */
	dwt_reg_write_u8(dev, DWT_TX_CAL_ID, DWT_TC_PGDELAY_OFFSET, pgdelay);
	LOG_DBG("PGDELAY 0x%02x", pgdelay);
	/* Set Transmit Power Control */
	dwt_reg_write_u32(dev, DWT_TX_POWER_ID, 0, power);
	LOG_DBG("TX_POWER 0x%08x", power);

	/*
	 * From 5.3.1.2 SFD Initialisation,
	 * SFD sequence initialisation for Auto ACK frame.
	 */
	dwt_reg_write_u8(dev, DWT_SYS_CTRL_ID, DWT_SYS_CTRL_OFFSET,
			 DWT_SYS_CTRL_TXSTRT | DWT_SYS_CTRL_TRXOFF);

	/*
	 * Calculate PHY timing parameters
	 *
	 * From (9.4) Std 802.15.4-2011
	 * Tshr = Tpsym * (NSYNC + NSFD )
	 * Tphr = NPHR * Tdsym1m
	 * Tpsdu = Tdsym * NPSDU * NSYMPEROCTET / Rfec
	 *
	 * PRF: pulse phase frequency
	 * PSR: preamble symbol phases
	 * SFD: start of frame delimiter
	 * SHR: synchronisation header (SYNC + SFD)
	 * PHR: PHY header
	 */
	uint16_t nsync = BIT(rf_cfg->tx_shr_nsync + 6);

	if (rf_cfg->prf == DWT_PRF_64M) {
		rf_cfg->t_shr = UWB_PHY_TPSYM_PRF64 *
				(nsync + UWB_PHY_NUMOF_SYM_SHR_SFD);
	} else {
		rf_cfg->t_shr = UWB_PHY_TPSYM_PRF16 *
				(nsync + UWB_PHY_NUMOF_SYM_SHR_SFD);
	}

	if (rf_cfg->dr == DWT_BR_6M8) {
		rf_cfg->t_phr = UWB_PHY_NUMOF_SYM_PHR * UWB_PHY_TDSYM_PHR_6M8;
		rf_cfg->t_dsym = UWB_PHY_TDSYM_DATA_6M8 / 0.44;
	} else if (rf_cfg->dr == DWT_BR_850K) {
		rf_cfg->t_phr = UWB_PHY_NUMOF_SYM_PHR * UWB_PHY_TDSYM_PHR_850K;
		rf_cfg->t_dsym = UWB_PHY_TDSYM_DATA_850K / 0.44;
	} else {
		rf_cfg->t_phr = UWB_PHY_NUMOF_SYM_PHR * UWB_PHY_TDSYM_PHR_110K;
		rf_cfg->t_dsym = UWB_PHY_TDSYM_DATA_110K / 0.44;
	}

	return 0;
}


#if !ZEPHYR_SPI
#if SPIM
static void spi_handler(const nrfx_spim_evt_t *p_event, void *p_context)
{
	if (p_event->type == NRFX_SPIM_EVENT_DONE) {
		k_sem_give(&spi_transfer_finished);
	}
}
#else
static void spi_handler(const nrfx_spi_evt_t *p_event, void *p_context)
{
	if (p_event->type == NRFX_SPI_EVENT_DONE) {
		k_sem_give(&spi_transfer_finished);
	}
}
#endif

static bool dwt_spi_init(const struct device *dev)
{
	int ret;
	nrfx_err_t err;
	struct dwt_context *ctx = dev->data;

	/* IRQ_CONNECT(DT_IRQN(SPI_NODE), DT_IRQ(SPI_NODE, priority), nrfx_isr, */
	/* 	    nrfx_prs_box_2_irq_phandler, 0); */

#if SPIM
	IRQ_CONNECT(DT_IRQN(SPI_NODE), DT_IRQ(SPI_NODE, priority), nrfx_isr,
		    nrfx_spim_2_irq_handler, 0);
#else
#if WITH_SPI_IRQ
	IRQ_DIRECT_CONNECT(DT_IRQN(SPI_NODE), DT_IRQ(SPI_NODE, priority), nrfx_spi_2_irq_handler,
			   0);
#endif
#endif


	if (spi_initialized) {
		return true;
	}

	// ACHTUNG! use NRF_SPI_PIN_NOT_CONNECTED for NRFX_SPIM_DEFAULT_CONFIG

#if SPIM
	nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG(
		NRF_SPI_PIN_NOT_CONNECTED, NRF_SPI_PIN_NOT_CONNECTED, NRF_SPI_PIN_NOT_CONNECTED,
		NRF_DT_GPIOS_TO_PSEL(SPI_NODE, cs_gpios))
		;
#else

#define SPI_PINCTRL_NODE DT_CHILD(DT_PINCTRL_0(SPI_NODE, 0), group1)
#define SCK_PIN (DT_PROP_BY_IDX(SPI_PINCTRL_NODE, psels, 0) & 0x3F)
#define MOSI_PIN (DT_PROP_BY_IDX(SPI_PINCTRL_NODE, psels, 1) & 0x3F)
#define MISO_PIN         (DT_PROP_BY_IDX(SPI_PINCTRL_NODE, psels, 2) & 0x3F)

	nrfx_spi_config_t spi_config = NRFX_SPI_DEFAULT_CONFIG(
		SCK_PIN, MOSI_PIN, MISO_PIN,
		NRF_DT_GPIOS_TO_PSEL(SPI_NODE, cs_gpios));
#endif
	spi_config.frequency = DWT_SPI_SLOW_FREQ;
	spi_config.skip_gpio_cfg = false;
	spi_config.skip_psel_cfg = false; // only when using pinctrl
	spi_config.mode = NRF_SPI_MODE_0;

	ctx->spi_cfg = spi_config;

	/* ret = pinctrl_apply_state(ctx->pcfg, PINCTRL_STATE_DEFAULT); */

	if (ret < 0) {
		return ret;
	}

#if SPIM
	err = nrfx_spim_init(&spi, &ctx->spi_cfg, spi_handler, NULL);
#else
#if WITH_SPI_IRQ
	err = nrfx_spi_init(&spi, &ctx->spi_cfg, spi_handler, NULL);
#else
	err = nrfx_spi_init(&spi, &ctx->spi_cfg, NULL, NULL);
#endif
#endif
	if (err != NRFX_SUCCESS) {
		printk("nrfx_spi_init() failed: 0x%08x\n", err);
		return false;
	}

	spi_initialized = true;
	printk("Switched to SPI\n");
	return true;
}
#endif



static int dw1000_init(const struct device *dev)
{
	struct dwt_context *ctx = dev->data;
	const struct dwt_hi_cfg *hi_cfg = dev->config;

	LOG_INF("Initialize DW1000 Transceiver");
	k_sem_init(&ctx->phy_sem, 0, 1);

#if USE_GPIO_DEBUG
	setup_debug_gpios();
#endif



	/* slow SPI config */
#if ZEPHYR_SPI
	memcpy(&ctx->spi_cfg_slow, &hi_cfg->bus.config, sizeof(ctx->spi_cfg_slow));
	ctx->spi_cfg_slow.frequency = DWT_SPI_SLOW_FREQ;

	if (!spi_is_ready_dt(&hi_cfg->bus)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}
#else
	dwt_spi_init(dev);
#endif

	dwt_set_spi_slow(dev, DWT_SPI_SLOW_FREQ);

	/* Initialize IRQ GPIO */
	if (!gpio_is_ready_dt(&hi_cfg->irq_gpio)) {
		LOG_ERR("IRQ GPIO device not ready");
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&hi_cfg->irq_gpio, GPIO_INPUT)) {
		LOG_ERR("Unable to configure GPIO pin %u", hi_cfg->irq_gpio.pin);
		return -EINVAL;
	}

	gpio_init_callback(&(ctx->gpio_cb), dwt_gpio_callback,
			   BIT(hi_cfg->irq_gpio.pin));

	if (gpio_add_callback(hi_cfg->irq_gpio.port, &(ctx->gpio_cb))) {
		LOG_ERR("Failed to add IRQ callback");
		return -EINVAL;
	}

	/* Initialize RESET GPIO */
	if (!gpio_is_ready_dt(&hi_cfg->rst_gpio)) {
		LOG_ERR("Reset GPIO device not ready");
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&hi_cfg->rst_gpio, GPIO_INPUT)) {
		LOG_ERR("Unable to configure GPIO pin %u", hi_cfg->rst_gpio.pin);
		return -EINVAL;
	}

	LOG_INF("GPIO and SPI configured");

	dwt_hw_reset(dev);

	if (dwt_reg_read_u32(dev, DWT_DEV_ID_ID, 0) != DWT_DEVICE_ID) {
		LOG_ERR("Failed to read device ID %p", dev);
		return -ENODEV;
	}

	if (dwt_initialise_dev(dev)) {
		LOG_ERR("Failed to initialize DW1000");
		return -EIO;
	}

	if (dwt_configure_rf_phy(dev)) {
		LOG_ERR("Failed to configure RF PHY");
		return -EIO;
	}

	/* Allow Beacon, Data, Acknowledgement, MAC command */
	dwt_set_frame_filter(dev, true, DWT_SYS_CFG_FFAB | DWT_SYS_CFG_FFAD |
			     DWT_SYS_CFG_FFAA | DWT_SYS_CFG_FFAM);

	/*
	 * Enable system events:
	 *  - transmit frame sent,
	 *  - receiver FCS good,
	 *  - receiver PHY header error,
	 *  - receiver FCS error,
	 *  - receiver Reed Solomon Frame Sync Loss,
	 *  - receive Frame Wait Timeout,
	 *  - preamble detection timeout,
	 *  - receive SFD timeout
	 */
	dwt_reg_write_u32(dev, DWT_SYS_MASK_ID, 0,
			  DWT_SYS_MASK_MTXFRS |
			  DWT_SYS_MASK_MRXFCG |
			  DWT_SYS_MASK_MRXPHE |
		          DWT_SYS_MASK_MHPDWARN |
			  DWT_SYS_MASK_MRXFCE |
			  DWT_SYS_MASK_MRXRFSL |
			  DWT_SYS_MASK_MRXRFTO |
			  DWT_SYS_MASK_MRXPTO |
			  DWT_SYS_MASK_MRXSFDTO);

	/* Initialize IRQ event work queue */
	ctx->dev = dev;

	k_work_queue_start(&dwt_work_queue, dwt_work_queue_stack,
			   K_KERNEL_STACK_SIZEOF(dwt_work_queue_stack),
			   CONFIG_SYSTEM_WORKQUEUE_PRIORITY, NULL);

	k_work_init(&ctx->irq_cb_work, dwt_irq_work_handler);

	dwt_setup_int(dev, true);

	LOG_INF("DW1000 device initialized and configured");

	return 0;
}

static inline uint8_t *get_mac(const struct device *dev)
{
	struct dwt_context *dw1000 = dev->data;
	uint32_t *ptr = (uint32_t *)(dw1000->mac_addr);

	UNALIGNED_PUT(sys_rand32_get(), ptr);
	ptr = (uint32_t *)(dw1000->mac_addr + 4);
	UNALIGNED_PUT(sys_rand32_get(), ptr);

	dw1000->mac_addr[0] = (dw1000->mac_addr[0] & ~0x01) | 0x02;

	return dw1000->mac_addr;
}

uint8_t *dwt_get_mac(const struct device *dev)
{
	return get_mac(dev);
}

static void dwt_iface_api_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct dwt_context *dw1000 = dev->data;
	uint8_t *mac = get_mac(dev);

	net_if_set_link_addr(iface, mac, 8, NET_LINK_IEEE802154);

	dw1000->iface = iface;

	ieee802154_init(iface);

#if ANALYZE_DWT_TIMING
	timing_init();
#endif

	LOG_INF("Iface initialized");
}


// TODO other information from info reg

static inline uint64_t dwt_read_tx_timestamp(const struct device *dev) {
	uint8_t ts_buf[sizeof(uint64_t)] = {0};

	dwt_register_read(dev, DWT_TX_TIME_ID,
				  DWT_TX_TIME_TX_STAMP_OFFSET,
				  DWT_TX_TIME_TX_STAMP_LEN,
				  ts_buf);

	return sys_get_le64(ts_buf);
}

struct mtm_ranging_timing mtm_ranging_conf = {
	.phy_activate_rx_delay = UUS_TO_DWT_TS(128 + 16),
	/* .phy_activate_rx_delay = UUS_TO_DWT_TS(128),	 */
	.phase_setup_delay = UUS_TO_DWT_TS(200),
	.round_setup_delay = UUS_TO_DWT_TS(200),
	.min_slot_length_us = 150,
	.preamble_timeout = 128/8,
	.preamble_chunk_duration = UUS_TO_DWT_TS(8), // 8 symbols per cross-correlated chunk
};

#warning "we don't receive the full 128 pacc symbols, is our timing completely correct here? Maybe check phy_activate_rx_delay again"

struct __attribute__((__packed__)) dwt_glossy_frame_buffer {
	uint8_t  prot_id;     // some identifier that this is a MTM ranging protocol execution
	uint8_t  msg_id;      // some identifier of which message type during the protocol run we are sending
	uint8_t  flood_initiator_id;      // some ranging id, in case of time slotted access this is equivalent to the transmission slot in the schedule
	uint8_t hop_count;
	uint64_t rtc_initiation_timestamp;
};

struct mtm_glossy_setup_struct {
	uint64_t transmission_delay_us;
} mtm_glossy_conf = {
	.transmission_delay_us = 500, // measured externally, includes also frame setup duration, thus a little harder to estimate
};


#define DWT_MTM_PROTOCOL_ID 0xCA
#define DWT_MTM_START_FRAME_ID 0x01
#define DWT_MTM_RANGIN_FRAME_ID 0x02
#define DWT_MTM_GLOSSY_TX_ID 0x03
#define DWT_MTM_MAX_ROUND_LENGTH 15
#define DWT_MTM_MAX_REPETITIONS 3

int dwt_glossy_tx_timesync(const struct  device *dev, uint8_t initiator, uint8_t node_id, uint16_t timeout_us, struct dwt_glossy_tx_result *result) {
	int ret = 0;
	struct dwt_context *ctx = dev->data;
	struct timeutil_sync_instant *rtc_inst = &result->clock_sync_instant;

	int irq_state;
	uint64_t initiator_rtc_ts, local_rtc_ts;
	uint64_t transmission_ts;
	atomic_t old_state;

	/* uint64_t psdu_duration_sans_preamble = dwt_get_pkt_duration_ns(ctx, sizeof(dwt_glossy_frame_buffer)) - */

	// --- Prevent execution of multiple ranging tasks
	if (atomic_test_and_set_bit(&ctx->state, DWT_STATE_TX)) {
		LOG_ERR("Transceiver busy");
		return -EBUSY;
	}

	/* LOG_WRN("%d us", (dwt_get_pkt_duration_ns(ctx, sizeof(struct dwt_glossy_frame_buffer)) / 1000) - 128); */ //currently 56us

	old_state = ctx->state;

	// reset to idle state and disable auto rx enabling
	atomic_set_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU);
	atomic_clear_bit(&ctx->state, DWT_STATE_RX_DEF_ON);

	k_sem_take(&ctx->dev_lock, K_FOREVER);
	dwt_disable_txrx(dev);
	dwt_set_frame_filter(dev, 0, 0);
	dwt_double_buffering_align(dev); // for the following execution we require that host and receiver side are aligned
	k_sem_give(&ctx->dev_lock);

	// --- Round Initiation ---
	if (initiator) {
		k_sem_take(&ctx->dev_lock, K_FOREVER);

		initiator_rtc_ts = k_uptime_ticks();// + ((CONFIG_SYS_CLOCK_TICKS_PER_SEC * mtm_ranging_conf.initial_tx_delay_us) / 1000000);
		transmission_ts = dwt_system_ts(dev) + UUS_TO_DWT_TS(mtm_glossy_conf.transmission_delay_us);

		/* uint8_t buf[] = {DWT_MTM_PROTOCOL_ID, DWT_MTM_GLOSSY_TX_ID, node_id, hop}; */
		struct dwt_glossy_frame_buffer initial_glossy_frame = {
			.prot_id = DWT_MTM_PROTOCOL_ID,
			.msg_id = DWT_MTM_GLOSSY_TX_ID,
			.flood_initiator_id = node_id,
			.hop_count = 0,
			.rtc_initiation_timestamp = initiator_rtc_ts
		};

		setup_tx_frame(dev, (uint8_t *)&initial_glossy_frame, sizeof(struct dwt_glossy_frame_buffer));

		dwt_fast_enable_tx(dev, transmission_ts & DWT_TS_MASK);

		rtc_inst->ref = initiator_rtc_ts;
		rtc_inst->local = initiator_rtc_ts;
		result->dist_to_root = 0; // i am groot

		k_sem_give(&ctx->dev_lock);

		irq_state = wait_for_phy(dev);
	} else {
		k_sem_take(&ctx->dev_lock, K_FOREVER);
		dwt_enable_rx(dev, timeout_us, 0);
		k_sem_give(&ctx->dev_lock);

		irq_state = wait_for_phy(dev);

		if(irq_state == DWT_IRQ_RX) {
			/* local_rtc_ts = k_uptime_ticks(); */
			local_rtc_ts = k_uptime_ticks();
			transmission_ts = dwt_system_ts(dev) + UUS_TO_DWT_TS(mtm_glossy_conf.transmission_delay_us);

			// read received packet
			struct dwt_rx_info_regs rx_info;
			struct dwt_glossy_frame_buffer glossy_frame;
			uint32_t rx_finfo;
			uint16_t pkt_len;

			k_sem_take(&ctx->dev_lock, K_FOREVER);
			dwt_read_rx_info(dev, &rx_info);
			rx_finfo = dwt_reg_read_u32(dev, DWT_RX_FINFO_ID, DWT_RX_FINFO_OFFSET);
			pkt_len = rx_finfo & DWT_RX_FINFO_RXFLEN_MASK;
			uint8_t buf[pkt_len];

			// --- read received frame and check for validity
			dwt_register_read(dev, DWT_RX_BUFFER_ID, 0, pkt_len, buf);

			if(buf[0] != DWT_MTM_PROTOCOL_ID || buf[1] != DWT_MTM_GLOSSY_TX_ID) {
				dwt_switch_buffers(dev);
				k_sem_give(&ctx->dev_lock);
				ret = -EIO;
				goto cleanup;
			} else {
				memcpy(&glossy_frame, buf, sizeof(struct dwt_glossy_frame_buffer));
			}

			glossy_frame.hop_count++;
			setup_tx_frame(dev, (uint8_t *)&glossy_frame, sizeof(struct dwt_glossy_frame_buffer));

			dwt_fast_enable_tx(dev, transmission_ts & DWT_TS_MASK);
			// now we have some time for doing further work on the mcu without affecting the timing above
			initiator_rtc_ts = glossy_frame.rtc_initiation_timestamp;

			rtc_inst->ref = initiator_rtc_ts;
			rtc_inst->local = local_rtc_ts - (glossy_frame.hop_count * (uint64_t) (mtm_glossy_conf.transmission_delay_us + 152) * CONFIG_SYS_CLOCK_TICKS_PER_SEC) / 1000000 ;
			result->dist_to_root = glossy_frame.hop_count;

			dwt_switch_buffers(dev);

			k_sem_give(&ctx->dev_lock);
			irq_state = wait_for_phy(dev);
		} else {
			ret = -EIO;
			goto cleanup;
		}
	}

	if(irq_state == DWT_IRQ_ERR) {
		ret = -EIO;
		goto cleanup;
	}

  cleanup:
	// --- clear bits ----
	/* atomic_clear_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU); */
	/* atomic_set_bit(&ctx->state, DWT_STATE_RX_DEF_ON); */
	ctx->state = old_state;
	atomic_clear_bit(&ctx->state, DWT_STATE_TX);

	if(atomic_test_bit(&ctx->state, DWT_STATE_RX_DEF_ON)) {
		dwt_enable_rx(dev, 0, 0);
	}

	return ret;
}

// create a memory pool again for frame frames
// doesnt currently work, after a certain amount of nodes, not sure if its my fault
// but it will fail after having to free 10 nodes -> Update: fixed this i am stupid and c should have optional access bound checking on arrays...
/* K_HEAP_DEFINE(dwt_ranging_result_buf, sizeof(struct dwt_ranging_frame_buffer)*100); */

static struct dwt_ranging_frame_buffer ranging_frames[DWT_MTM_MAX_ROUND_LENGTH * DWT_MTM_MAX_REPETITIONS];
static struct dwt_ranging_frame_info frame_infos[DWT_MTM_MAX_ROUND_LENGTH * DWT_MTM_MAX_REPETITIONS];

#if CONFIG_DWT_MTM_OUTPUT_CIR
// we will directly insert the header into the buffer, because of the limitiations of using nrfx spi
// directly .Write into this buffer starting from offset 0, but read into it simultanously from
// offset 1
static uint8_t cir_acc_mem[4066];
#endif

dwt_ts_t from_packed_dwt_ts(const dwt_packed_ts_t ts) {
	return (dwt_ts_t) ts[0] | ((dwt_ts_t) ts[1] << 8) | ((dwt_ts_t) ts[2] << 16) | ((dwt_ts_t) ts[3] << 24) | ((dwt_ts_t) ts[4] << 32);
}

void to_packed_dwt_ts(dwt_packed_ts_t ts, dwt_ts_t value) {
	ts[0] = value & 0xFF;
	ts[1] = (value >> 8) & 0xFF;
	ts[2] = (value >> 16) & 0xFF;
	ts[3] = (value >> 24) & 0xFF;
	ts[4] = (value >> 32) & 0xFF;
}


struct timing_log {
	char *label;
	uint64_t timestamp_ns;
};

struct {
	uint8_t count;
	struct timing_log logs[100];
} timing_logs = {
	.count = 0,
};


#if ANALYZE_DWT_TIMING
#define SW_DEFINE(NAME) static timing_t dbts_start_##NAME = 0; static timing_t dbts_end_##NAME = 0;
#define SW_START(NAME)  dbts_start_##NAME = timing_counter_get(); //k_cycle_get_32();
#define SW_END(NAME)    do { dbts_end_##NAME = timing_counter_get(); timing_logs.logs[timing_logs.count].label = #NAME; timing_logs.logs[timing_logs.count].timestamp_ns = timing_cycles_to_ns(timing_cycles_get(&dbts_start_##NAME, &dbts_end_##NAME)); timing_logs.count++; } while(0)

void reset_timing_logs() {
	timing_logs.count = 0;
}

void output_timing_logs() {
	for(int i = 0; i < timing_logs.count; i++) {
		LOG_WRN("%s: %llu [us]", timing_logs.logs[i].label, timing_logs.logs[i].timestamp_ns / 1000);
	}
}

#else
#define SW_DEFINE(NAME)
#define SW_START(NAME)
#define SW_END(NAME)
#endif

SW_DEFINE(ROUND_INIT);
SW_DEFINE(INITIATION_FRAME);
SW_DEFINE(INIT_ROUND_SETUP);
SW_DEFINE(PREPARE_TX);
SW_DEFINE(PROG_RX_TX);
SW_DEFINE(FRAME_HANDLING);
SW_DEFINE(IRQ_WAIT_DELAY);
SW_DEFINE(IRQ_RX);


struct measured_timing {
	uint16_t reception_spi_read, packed_transmission_duration, transmission_enable;
	uint16_t err_irq_handler;
} measured_timing  = {
	.packed_transmission_duration = 400,
	.transmission_enable = 40,
	.reception_spi_read = 330,
	.err_irq_handler = 90,
};



// cca duration is again in units of pac size, i.e., generally for our setting it will be in the range of 1..16
int dwt_mtm_ranging(const struct device *dev, const struct mtm_ranging_config *conf, struct dwt_ranging_frame_info **frames, int *frame_count) {
	int ret = 0;
	struct dwt_context *ctx = dev->data;
	struct mtm_ranging_timing *ranging_conf = &mtm_ranging_conf;

	struct mtm_ranging_dense_slot_schedule *schedule = conf->schedule;

	int irq_state;
	dwt_ts_t round_start_dw_ts, slot_start_ts, slot_duration;
	uint16_t antenna_delay = ctx->tx_ant_dly;
	atomic_t old_state;

	int frame_counter = 0, frame_info_counter = 0;

#if ANALYZE_DWT_TIMING
	reset_timing_logs();
	timing_start();
#endif

	SW_START(ROUND_INIT);


	slot_duration = UUS_TO_DWT_TS(conf->slot_duration_us);

	// check input arguments
	if(conf->slot_duration_us < ranging_conf->min_slot_length_us) {
		LOG_ERR("slot duration too short");
		return -EINVAL;
	}

	// --- Prevent execution of multiple ranging tasks
	if (atomic_test_and_set_bit(&ctx->state, DWT_STATE_TX)) {
		LOG_ERR("Transceiver busy");
		return -EBUSY;
	}

	// store old state
	old_state = ctx->state;

	// reset to idle state and disable auto rx enabling
	atomic_set_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU);
	atomic_clear_bit(&ctx->state, DWT_STATE_RX_DEF_ON);

	for(int i = 0; i < DWT_MTM_MAX_ROUND_LENGTH * DWT_MTM_MAX_REPETITIONS; i++) {
		frame_infos[i].frame = NULL;
	}

	k_sem_take(&ctx->dev_lock, K_FOREVER);
	dwt_disable_txrx(dev);
	dwt_set_frame_filter(dev, 0, 0);
	dwt_double_buffering_align(dev); // for the following execution we require that host and receiver side are aligned
	k_sem_give(&ctx->dev_lock);

	/* LOG_WRN("estimated transmission duration %d us", dwt_get_pkt_duration_ns(ctx, sizeof(struct dwt_ranging_frame_buffer))/1000); */

	SW_END(ROUND_INIT);
	SW_START(INITIATION_FRAME);

	// --- Optional: Round Initiation ---
	if (conf->use_initiation_frame) {
		if (conf->node_is_initiator) {
			k_sem_take(&ctx->dev_lock, K_FOREVER);

			static uint8_t buf[2] = {DWT_MTM_PROTOCOL_ID, DWT_MTM_START_FRAME_ID};

			setup_tx_frame(dev, buf, sizeof(buf));
			dwt_fast_enable_tx(dev, 0);

			k_sem_give(&ctx->dev_lock);

			irq_state = wait_for_phy(dev);

			if(irq_state != DWT_IRQ_TX) {
				LOG_ERR("failed to send initiation frame");
				ret = -EIO;
				goto cleanup;
			}

			k_sem_take(&ctx->dev_lock, K_FOREVER);
			round_start_dw_ts = dwt_read_tx_timestamp(dev);
			k_sem_give(&ctx->dev_lock);
		} else {
			k_sem_take(&ctx->dev_lock, K_FOREVER);
			dwt_enable_rx(dev, conf->timeout_us, 0);
			k_sem_give(&ctx->dev_lock);

			irq_state = wait_for_phy(dev);

			if(irq_state == DWT_IRQ_RX) {
				struct dwt_rx_info_regs rx_info;
				k_sem_take(&ctx->dev_lock, K_FOREVER);

				dwt_read_rx_info(dev, &rx_info);
				round_start_dw_ts = dwt_rx_timestamp_from_rx_info(&rx_info);

				// read received packet
				uint32_t rx_finfo;
				uint16_t pkt_len;
				rx_finfo = dwt_reg_read_u32(dev, DWT_RX_FINFO_ID, DWT_RX_FINFO_OFFSET);
				pkt_len = rx_finfo & DWT_RX_FINFO_RXFLEN_MASK;

				uint8_t buf[2];
				if((pkt_len-2) != sizeof(buf)) {
					LOG_ERR("invalid start of round frame, wrong length (%u bytes)", pkt_len);
					/* dwt_switch_buffers(dev); */ // should not be needed because we align the buffers anyway
					k_sem_give(&ctx->dev_lock);
					ret = -EIO;
					goto cleanup;
				}

				dwt_register_read(dev, DWT_RX_BUFFER_ID, 0, pkt_len, buf);

				dwt_switch_buffers(dev);

				if(buf[0] != DWT_MTM_PROTOCOL_ID || buf[1] != DWT_MTM_START_FRAME_ID) {
					LOG_ERR("invalid start of round frame, wrong protocol id");
					k_sem_give(&ctx->dev_lock);
					ret = -EIO;
					goto cleanup;
				}

				k_sem_give(&ctx->dev_lock);
			} else {
				LOG_ERR("failed to receive initiation frame (%u)", irq_state);
				ret = -EIO;
				goto cleanup;
			}
		}
	} else {
		// use current transmission timestamp and configuration with guard periods
		round_start_dw_ts = dwt_system_ts(dev);
	}
	SW_END(INITIATION_FRAME);

	SW_START(INIT_ROUND_SETUP);
	// --- PHY setup for ranging round ---
	k_sem_take(&ctx->dev_lock, K_FOREVER);
	dwt_setup_rx_timeout(dev, conf->slot_duration_us - 100);
	// !!frame timeouts are expensive as they require a full receiver soft reset, thus we setup a preamble timeout as well!!

	dwt_setup_preamble_detection_timeout(dev, ranging_conf->preamble_timeout + conf->guard_period_us/8);
	k_sem_give(&ctx->dev_lock);

	// --- lock execute ranging round ---
	slot_start_ts = (round_start_dw_ts + slot_duration) & DWT_TS_MASK;

	// create compiler warning to remember that we have to optimize this here
#warning "we can probably start right away with a small delay, since we are not really doing anything in the refactored version"

	uint8_t cca_got_slot = 0;

	struct dwt_ranging_frame_buffer *outgoing_frame = NULL;

	// -- prepare an initial outgoing frame, do the absolute minimum here --
	outgoing_frame      = &ranging_frames[frame_counter];
	outgoing_frame->rx_ts_count = 0;
	frame_counter++;
	SW_END(INIT_ROUND_SETUP);

	uint8_t have_frame = 0;
	uint16_t pkt_len;
	int cfo;
	// -- do one more iteration because of double buffered operation --
	for(size_t s = 0; s < schedule->slot_count + 1; s++) {
		enum slot_type type = schedule->slots[s].type;

		// ---- I) kicking of next PHY action ----
		if ((type == DENSE_RX_SLOT || type == DENSE_TX_SLOT) && s < schedule->slot_count) {
			// --- Decide the PHY action to execute ---
			k_sem_take(&ctx->dev_lock, K_FOREVER);
			// --- schedule next PHY action ---
			SW_START(PROG_RX_TX);
			if(type == DENSE_TX_SLOT) {
				dwt_fast_enable_tx(dev, (slot_start_ts
						+ NS_TO_DWT_TS(conf->micro_slot_offset_ns)
						+ ranging_conf->phy_activate_rx_delay
						+ UUS_TO_DWT_TS(conf->guard_period_us)/2) & DWT_TS_MASK);
			} else if(type == DENSE_RX_SLOT) {
				dwt_fast_enable_rx(dev, slot_start_ts & DWT_TS_MASK);
			}
			SW_END(PROG_RX_TX);
		}

		// ---- II) double buffered operation -----
		if(have_frame) {
			SW_START(FRAME_HANDLING);
			// Note: in slot N we process the frame of slot N-1
			// -- grab frame and frame info struct for storing the received data --

			struct dwt_ranging_frame_buffer *incoming_frame = &ranging_frames[frame_counter];
			frame_counter++;

			struct dwt_rx_info_regs rx_info;
			uint32_t rx_finfo;
			int8_t rx_level = INT8_MIN, bias_correction;
			uint32_t rx_pacc, cir_pwr;
			uint16_t fp_index;
			int fp_deviation;
			float a_const;

			dwt_read_rx_info(dev, &rx_info);

			rx_finfo = dwt_reg_read_u32(dev, DWT_RX_FINFO_ID, DWT_RX_FINFO_OFFSET);
			pkt_len = rx_finfo & DWT_RX_FINFO_RXFLEN_MASK;
			uint8_t rx_buf[pkt_len];

			// --- calculate relative signal strength for bias correction
			cir_pwr = dwt_cir_pwr_from_info_reg(&rx_info);
			rx_pacc = (rx_finfo & DWT_RX_FINFO_RXPACC_MASK) >> DWT_RX_FINFO_RXPACC_SHIFT;
			fp_index = dwt_fp_index_from_info_reg(&rx_info);

			if (ctx->rf_cfg.prf == DWT_PRF_16M) {
				a_const = DWT_RX_SIG_PWR_A_CONST_PRF16;
			} else {
				a_const = DWT_RX_SIG_PWR_A_CONST_PRF64;
			}

			rx_level = 10.0 * log10f(cir_pwr * BIT(17) /
				(rx_pacc * rx_pacc)) - a_const;

			bias_correction = get_range_bias_by_rssi(rx_level);

			// --- read incoming frame and check for validity
			dwt_register_read(dev, DWT_RX_BUFFER_ID, 0, pkt_len, rx_buf);

			// --- retrieve incoming frame ---
			memcpy(incoming_frame, rx_buf, pkt_len-2);

			if(incoming_frame->msg_id != DWT_MTM_RANGIN_FRAME_ID) {
				LOG_ERR("invalid ranging frame");
			} else {
				dwt_ts_t reception_ts = dwt_rx_timestamp_from_rx_info(&rx_info);
				struct dwt_ranging_frame_info   *incoming_frame_info = &frame_infos[frame_info_counter];
				frame_info_counter++;

				// store frame into frame_infos
				incoming_frame_info->frame = incoming_frame;
				incoming_frame_info->status = DWT_MTM_FRAME_OKAY;
				incoming_frame_info->timestamp = reception_ts - bias_correction;
				incoming_frame_info->type = DWT_RANGING_RECEIVED_FRAME;
				incoming_frame_info->slot = s - 1;
				incoming_frame_info->cir_pwr = cir_pwr;
				incoming_frame_info->rx_pacc = rx_pacc;
				incoming_frame_info->fp_index = fp_index;
				incoming_frame_info->fp_ampl1 = dwt_fp_ampl1_from_info_reg(&rx_info);
				incoming_frame_info->fp_ampl2 = dwt_fp_ampl2_from_info_reg(&rx_info);
				incoming_frame_info->fp_ampl3 = dwt_fp_ampl3_from_info_reg(&rx_info);
				incoming_frame_info->std_noise = dwt_std_noise_from_info_reg(&rx_info);
				incoming_frame_info->rx_level = rx_level;

				if(conf->cfo) {
					// warning this has to be adjusted for other data rates. -1e6 * (((((float) cfo)) / (((float) (2 * 1024) / 998.4e6) * 2^17)) / 6489.6e6)
					incoming_frame_info->cfo_ppm = (float) cfo * -0.000573121584378756;
				}

				// --- update outgoing frame ---
				if(conf->reject_frames && ((int) fp_index >> 6) <= conf->fp_index_threshold) {
					// this informs the upper layer that the frame was rejected and not included in the outgoing frame
					incoming_frame_info->status = DWT_MTM_FRAME_REJECTED;
				} else if(outgoing_frame && outgoing_frame->rx_ts_count < sizeof(outgoing_frame->rx_ts)/sizeof(struct dwt_tagged_timestamp)) {
					to_packed_dwt_ts(outgoing_frame->rx_ts[outgoing_frame->rx_ts_count].ts, reception_ts - bias_correction);
					outgoing_frame->rx_ts[outgoing_frame->rx_ts_count].ranging_id = incoming_frame->ranging_id;
					outgoing_frame->rx_ts[outgoing_frame->rx_ts_count].slot = s - 1;
					outgoing_frame->rx_ts_count++;
				}
			}

			dwt_switch_buffers(dev);
			have_frame = 0;
			SW_END(FRAME_HANDLING);
		}

		// -- has to happen after frame handling, since the frame handler during double buffered operation may still add to the outgoing frame --
		if(type == DENSE_LOAD_TX_BUFFER  && s < schedule->slot_count) {
			SW_START(PREPARE_TX);

			dwt_ts_t current_frame_transmission_ts = slot_start_ts;
			uint16_t future_tx_slot = UINT16_MAX;

			// seek next transmission slot
			for(size_t i = s+1; i < schedule->slot_count; i++) {
				// later in case we want to have differently sized slots, we can further distinguish here
				if(schedule->slots[i].type == DENSE_TX_SLOT || schedule->slots[i].type == DENSE_RX_SLOT || schedule->slots[i].type == DENSE_LOAD_TX_BUFFER) {
					current_frame_transmission_ts += slot_duration;
				}

				if(schedule->slots[i].type == DENSE_TX_SLOT) {
					future_tx_slot = i;
					break;
				}
			}

			// --- in the following phase we will send data that we collected throughout the round
			if(future_tx_slot != UINT16_MAX) {
				current_frame_transmission_ts =
					((current_frame_transmission_ts + ranging_conf->phy_activate_rx_delay
						+ UUS_TO_DWT_TS(conf->guard_period_us)/2
						+ NS_TO_DWT_TS(conf->micro_slot_offset_ns)
						) & ( (uint64_t) 0xFFFFFFFE00ULL )) + antenna_delay;

				outgoing_frame->msg_id  = DWT_MTM_RANGIN_FRAME_ID;
				outgoing_frame->ranging_id = conf->ranging_id;

				to_packed_dwt_ts(outgoing_frame->tx_ts, current_frame_transmission_ts);

				// -- store buffer into frame info struct --
				struct dwt_ranging_frame_info   *outgoing_frame_info = &frame_infos[frame_info_counter];
				frame_info_counter++;

				outgoing_frame_info->frame = outgoing_frame;
				outgoing_frame_info->timestamp = current_frame_transmission_ts;
				outgoing_frame_info->slot = future_tx_slot;
				outgoing_frame_info->type = DWT_RANGING_TRANSMITTED_FRAME;

				// --- finally load frame into transmission buffer ---
				k_sem_take(&ctx->dev_lock, K_FOREVER);
				setup_tx_frame(dev, (uint8_t*) outgoing_frame,
					offsetof(struct dwt_ranging_frame_buffer, rx_ts)
					+ (outgoing_frame->rx_ts_count * sizeof(struct dwt_tagged_timestamp)));

				k_sem_give(&ctx->dev_lock);

				// -- already grab a frame for the next upcoming transmission --
				outgoing_frame      = &ranging_frames[frame_counter];
				outgoing_frame->rx_ts_count = 0;
				frame_counter++;

				/* if(conf->cca) { */
				/* 	k_sem_take(&ctx->dev_lock, K_FOREVER); */
				/* 	if(!current_phase) { */
				/* 		dwt_setup_preamble_detection_timeout(dev, conf->cca_duration + 150/8); */
				/* 	} else { */
				/* 		dwt_setup_preamble_detection_timeout(dev, ranging_conf->preamble_timeout); */
				/* 	} */
				/* 	k_sem_give(&ctx->dev_lock); */
				/* } */
			}

			SW_END(PREPARE_TX);
		}

		k_sem_give(&ctx->dev_lock);

		// ---- II) Joining with PHY (only relevant if our current slot is a rx or tx operation) -----
		if ((type == DENSE_RX_SLOT || type == DENSE_TX_SLOT) && s < schedule->slot_count) {
			SW_START(IRQ_WAIT_DELAY);
			irq_state = wait_for_phy(dev);
			SW_END(IRQ_WAIT_DELAY);

			if(irq_state == DWT_IRQ_FRAME_WAIT_TIMEOUT) {
				ret = -ETIMEDOUT;
				goto cleanup;
			} else if( irq_state == DWT_IRQ_PREAMBLE_DETECT_TIMEOUT ) {
				// in cca case this is our signal that the channel is most
				// likely free i.e., we grab that for the subsequent rounds
				/* if(conf->cca && !current_phase && !cca_got_slot) { */
				/* 	cca_got_slot = 1; */
				/* 	transmission_slot = current_slot; */

				/* 	k_sem_take(&ctx->dev_lock, K_FOREVER); */
				/* 	// takes about 40 microseconds to setup next transmission */
				/* 	dwt_fast_enable_tx(dev, (slot_start_ts + ranging_conf->phy_activate_rx_delay + UUS_TO_DWT_TS(conf->guard_period_us)/2 + (conf->cca_duration + 10) * ranging_conf->preamble_chunk_duration) & DWT_TS_MASK); */
				/* 	k_sem_give(&ctx->dev_lock); */

				/* 	wait_for_phy(dev); */
				/* } else { */
				/* 	// nothing to do so far */
				/* } */
			} else if(irq_state == DWT_IRQ_RX) {
				SW_START(IRQ_RX);
				have_frame = 1;

				// ----- NON DOUBLE BUFFERED REGS ------
				// Read registers not in double buffere swinging register set
				if(conf->cfo) {
					cfo = dwt_readcarrierintegrator(dev);
				}
#if CONFIG_DWT_MTM_OUTPUT_CIR
				if(conf->cir_handler != NULL) {
					// first we do a bulk extract of the impulse memory
					/* dwt_register_read(dev, DWT_ACC_MEM_ID, 0, sizeof(cir_acc_mem), cir_acc_mem); */
					dwt_enable_accumulator_memory_access(dev);
					char tx_buf[1] = {DWT_ACC_MEM_ID};
					spi_transfer(tx_buf, 1, cir_acc_mem, 4066);

					// We are not able to buffer the CIR for every
					// reception in the round. Therefore, we directly
					// call a upper-layer handler for the cir This might
					// also be useful in the future since the upper
					// layer might want to decide whether to throw away
					// the transmission based on the cir.

					dwt_disable_accumulator_memory_access(dev);

					conf->cir_handler(s, cir_acc_mem+2, 4064);
				}
#endif
				SW_END(IRQ_RX);
			} else if(irq_state == DWT_IRQ_TX) {
			} else if(irq_state == DWT_IRQ_ERR) {
				/* handling rx errors takes a longer time than the other
				   events, since a receiver reset has to be performed.  thus
				   if we don't cancel out of this round after a error event,
				   we should add about 90 microseconds to the slot duration,
				   in order to not miss subsequent frames. */
			} else if(irq_state == DWT_IRQ_HALF_DELAY_WARNING) {
				ret = -EOVERFLOW;
				LOG_ERR("HALF_DELAY_WARNING");
				goto cleanup;
			}
		}

		// -- Currently always iterate by slot_duration --
#warning "optimize so we dont waste time with other slot times, for instance when current slot was a load tx operation, we could use another duration here"
		slot_start_ts = (slot_start_ts + slot_duration) & DWT_TS_MASK;
	}

	*frames = frame_infos;
	*frame_count = frame_info_counter;

  cleanup:
	// --- clear bits ----
	// restore old state
	ctx->state = old_state;
	/* atomic_clear_bit(&ctx->state, DWT_STATE_IRQ_POLLING_EMU); */
	/* atomic_set_bit(&ctx->state, DWT_STATE_RX_DEF_ON); */
	atomic_clear_bit(&ctx->state, DWT_STATE_TX);

	k_sem_take(&ctx->dev_lock, K_FOREVER);
	dwt_setup_preamble_detection_timeout(dev, 0);
	k_sem_give(&ctx->dev_lock);

	if(atomic_test_bit(&ctx->state, DWT_STATE_RX_DEF_ON)) {
		dwt_enable_rx(dev, 0, 0);
	}

#if ANALYZE_DWT_TIMING
	timing_stop();
	output_timing_logs();
#endif

	return ret;
}

static const struct ieee802154_radio_api dwt_radio_api = {
	.iface_api.init		= dwt_iface_api_init,

	.get_capabilities	= dwt_get_capabilities,
	.cca			= dwt_cca,
	.set_channel		= dwt_set_channel,
	.filter			= dwt_filter,
	.set_txpower		= dwt_set_power,
	.start			= dwt_start,
	.stop			= dwt_stop,
	.configure		= dwt_configure,
	.ed_scan		= dwt_ed,
	.tx			= dwt_tx,
	.attr_get		= dwt_attr_get,
};

#define DWT_PSDU_LENGTH		(127 - DWT_FCS_LENGTH)

#if defined(CONFIG_IEEE802154_RAW_MODE)
DEVICE_DT_INST_DEFINE(0, dw1000_init, NULL,
		    &dwt_0_context, &dw1000_0_config,
		    POST_KERNEL, CONFIG_IEEE802154_DW1000_INIT_PRIO,
		    &dwt_radio_api);
#else
NET_DEVICE_DT_INST_DEFINE(0,
		dw1000_init,
		NULL,
		&dwt_0_context,
		&dw1000_0_config,
		CONFIG_IEEE802154_DW1000_INIT_PRIO,
		&dwt_radio_api,
		IEEE802154_L2,
		NET_L2_GET_CTX_TYPE(IEEE802154_L2),
		DWT_PSDU_LENGTH);
#endif
