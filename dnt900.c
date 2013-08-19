/*
    Linux line discipline for RFM DNT900 & DNT2400 radio transceiver
    modules.
    
    Copyright (C) 2012, 2013 Matthew Hollingworth.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/printk.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/tty_ldisc.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/poll.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/list.h>
#include <linux/completion.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/kfifo.h>
#include <linux/delay.h>
#include <linux/bitops.h>

#define LDISC_NAME "dnt900"
#define CLASS_NAME "dnt900"
#define TTY_DRIVER_NAME "ttyDNT"
#define TTY_DEV_NAME "ttyDNT"
#define LOCAL_SYMLINK_NAME "local"

#define MARK() pr_info("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__)

#define MAX_PACKET_SIZE (2+255)

// buffer sizes must be powers of 2
#define  RX_BUFFER_SIZE (512)  // no less
#define  TX_BUFFER_SIZE (2048)
#define TTY_BUFFER_SIZE (512)

#define REGISTER_TIMEOUT_MS (300000)
#define STARTUP_DELAY_MS (500)
#define REMOTE_REGISTER_RETRIES (2)

#define START_OF_PACKET (0xFB)

#define COMMAND_ENTER_PROTOCOL_MODE (0x00)
#define COMMAND_EXIT_PROTOCOL_MODE  (0x01)
#define COMMAND_SOFTWARE_RESET      (0x02)
#define COMMAND_GET_REGISTER        (0x03)
#define COMMAND_SET_REGISTER        (0x04)
#define COMMAND_TX_DATA             (0x05)
#define COMMAND_DISCOVER            (0x06)
#define COMMAND_GET_REMOTE_REGISTER (0x0A)
#define COMMAND_SET_REMOTE_REGISTER (0x0B)
#define COMMAND_JOIN_REPLY          (0x0C)
#define COMMAND_REMOTE_LEAVE        (0x0D)

#define EVENT_RX_DATA      (0x26)
#define EVENT_ANNOUNCE     (0x27)
#define EVENT_RX_EVENT     (0x28)
#define EVENT_JOIN_REQUEST (0x2C)

#define MASK_COMMAND (0x0F)
#define TYPE_REPLY   (0x10)
#define TYPE_EVENT   (0x20)

#define STATUS_ACKNOWLEDGED     (0x00)
#define STATUS_NOT_ACKNOWLEDGED (0x01)
#define STATUS_NOT_LINKED       (0x02)
#define STATUS_HOLDING_FOR_FLOW (0x03)

#define ANNOUNCEMENT_STARTED           (0xA0)
#define ANNOUNCEMENT_REMOTE_JOINED     (0xA2)
#define ANNOUNCEMENT_JOINED            (0xA3)
#define ANNOUNCEMENT_EXITED            (0xA4)
#define ANNOUNCEMENT_BASE_REBOOTED     (0xA5)
#define ANNOUNCEMENT_REMOTE_EXITED     (0xA7)
#define ANNOUNCEMENT_HEARTBEAT         (0xA8)
#define ANNOUNCEMENT_HEARTBEAT_TIMEOUT (0xA9)

#define ERROR_PROTOCOL_TYPE     (0xE0)
#define ERROR_PROTOCOL_ARGUMENT (0xE1)
#define ERROR_PROTOCOL_GENERAL  (0xE2)
#define ERROR_PROTOCOL_TIMEOUT  (0xE3)
#define ERROR_PROTOCOL_READONLY (0xE4)
#define ERROR_UART_OVERFLOW     (0xE8)
#define ERROR_UART_OVERRUN      (0xE9)
#define ERROR_UART_FRAMING      (0xEA)
#define ERROR_HARDWARE          (0xEE)

#define DEVICE_MODE_REMOTE (0x00)
#define DEVICE_MODE_BASE   (0x01)
#define DEVICE_MODE_ROUTER (0x03)

#define PROTOCOL_OPTIONS_ENABLE_TX_REPLY (0x04)
#define PROTOCOL_OPTIONS_ENABLE_ANNOUNCE (0x01)

#define ANNOUNCE_OPTIONS_LINKS (0x02)
#define ANNOUNCE_OPTIONS_INIT  (0x01)

#define AUTH_MODE_ANY     (0x00)
#define AUTH_MODE_TABLE   (0x01)
#define AUTH_MODE_HOST    (0x02)
#define AUTH_MODE_CURRENT (0x03)

#define ACCESS_MODE_POLLING      (0x00)
#define ACCESS_MODE_CSMA         (0x01)
#define ACCESS_MODE_TDMA_DYNAMIC (0x02)
#define ACCESS_MODE_TDMA_FIXED   (0x03)
#define ACCESS_MODE_TDMA_PTT     (0x04)

#define PERMIT_STATUS_DENIED    (0x00)
#define PERMIT_STATUS_PERMITTED (0x01)

#define LINK_STATUS_READY (0x04)

#define MAX_NETWORK_ID (0x3F)

#define ATTR_R  (S_IRUGO)
#define ATTR_W  (S_IWUSR)
#define ATTR_RW (S_IRUGO | S_IWUSR)

#define EQUAL_ADDRESSES(address1, address2) ( \
	(address1)[0] == (address2)[0] && \
	(address1)[1] == (address2)[1] && \
	(address1)[2] == (address2)[2])

#define DEV_TO_LOCAL(_dev) container_of(_dev, struct dnt900_local, dev)
#define DEV_TO_RADIO(_dev) container_of(_dev, struct dnt900_radio, dev)
#define RADIO_TO_LOCAL(radio) DEV_TO_LOCAL((radio)->dev.parent)
#define TTY_TO_LOCAL(tty) ((struct dnt900_local *)(tty)->disc_data)
#define TTY_TO_RADIO(tty) DEV_TO_RADIO((tty)->dev->parent)
#define PORT_TO_RADIO(_port) container_of(_port, struct dnt900_radio, port)

#define PRINT_RANGE(buffer, value) do { sprintf(buffer, "%d\n", 46671 * (unsigned char)(value) / 100); } while (0)
#define PRINT_SUCCESS(buffer, attempts_x4) do { sprintf(buffer, "%d\n", (attempts_x4) ? 400 / (unsigned char)(attempts_x4) : 100); } while (0)
#define PRINT_RSSI(buffer, value) do { sprintf(buffer, "%+d\n", (signed char)(value)); } while (0)

#define TRY(expression) do { \
	int err = (expression); \
	if (err) \
		return err; \
} while (0)

#define UNWIND(err, expression, exit) do { \
	err = (expression); \
	if (err) \
		goto exit; \
} while (0)

#define COPY3(dest, src) do { \
	(dest)[0] = (src)[0]; \
	(dest)[1] = (src)[1]; \
	(dest)[2] = (src)[2]; \
} while (0)

#define ARG_COUNT(...) (sizeof((char[]){__VA_ARGS__})/sizeof(char))
#define PACKET(name, ...) unsigned char name[] = { START_OF_PACKET, ARG_COUNT(__VA_ARGS__), __VA_ARGS__ }

/**
 * kfifo_out_prepare - setup a pointer for direct output buffer access
 * @fifo: address of the fifo to be used
 * @pbuf: address of buffer pointer to be initialized
 *
 * This macro sets the provided buffer pointer to the address of the
 * current fifo output location.
 * It returns the maximum number of contiguous elements which can be read
 * out from this location. A zero means there is no data in the fifo.
 *
 * Note that with only one concurrent reader and one concurrent
 * writer, you don't need extra locking to use these macros.
 */
#define kfifo_out_prepare(fifo, pbuf) \
__kfifo_uint_must_check_helper( \
({ \
	typeof((fifo) + 1) __tmp = (fifo);  \
	typeof((*pbuf) + 1) *__pbuf = (pbuf); \
	struct __kfifo *__kfifo = &__tmp->kfifo; \
	unsigned int size = __kfifo->mask + 1; \
	unsigned int off = __kfifo->out & __kfifo->mask; \
	*__pbuf = (__is_kfifo_ptr(__tmp) ? \
		((typeof(__tmp->type))__kfifo->data) : \
		(__tmp->buf)) + off; \
	min(size - off, __kfifo->in - __kfifo->out); \
}) \
)

/**
 * kfifo_out_finish - finish a direct output buffer access operation
 * @fifo: address of the fifo to be used
 * @len: number of elements which were read out
 *
 * This macro finishes a direct output buffer access operation. The out
 * counter will be updated by the len parameter. No error checking will
 * be done.
 *
 * Note that with only one concurrent reader and one concurrent
 * writer, you don't need extra locking to use these macros.
 */
#define kfifo_out_finish(fifo, len) \
(void)({ \
	typeof((fifo) + 1) __tmp = (fifo);  \
	struct __kfifo *__kfifo = &__tmp->kfifo; \
	unsigned int __len = (len); \
	__kfifo->out += __len; \
})

struct dnt900_transaction {
	struct list_head list;
	struct completion completed;
	unsigned char *result;
	int err;
	const unsigned char *packet;
};

struct dnt900_local_params {
	unsigned char device_mode;
	unsigned char tree_routing_en;
	unsigned char slot_size;
	unsigned char link_status;
	unsigned char base_mac_address[3];
	unsigned char base_mode_net_id;
};

struct dnt900_local;

struct dnt900_radio_params {
	unsigned char sys_address[3];
	unsigned char device_mode;
	unsigned char curr_nwk_id;
	unsigned char base_mode_net_id;
};

struct dnt900_radio;

struct dnt900_bufdata {
	const unsigned char *buf;
	unsigned int len;
};

struct dnt900_work {
	struct work_struct ws;
	unsigned char address[3];
	struct dnt900_local *local;
};

struct dnt900_register {
	unsigned char bank;
	unsigned char offset;
	unsigned char span;
};

struct dnt900_reg_attribute {
	struct device_attribute attr;
	struct dnt900_register reg;
	int (*print)(const unsigned char *value, char *buf);
	int (*parse)(const char *buf, size_t count, unsigned char *value);
};

struct dnt900_matcher {
	void *data;
	int (*match)(struct device *, void *);
	int (*action)(struct dnt900_radio *);
};

static int dnt900_print_bytes(int bytes, const unsigned char *value, char *buf, size_t size);
static int dnt900_print_1_bytes(const unsigned char *value, char *buf);
static int dnt900_print_2_bytes(const unsigned char *value, char *buf);
static int dnt900_print_3_bytes(const unsigned char *value, char *buf);
static int dnt900_print_4_bytes(const unsigned char *value, char *buf);
static int dnt900_print_hex(int bytes, const unsigned char *value, char *buf, size_t size);
static int dnt900_print_32_hex(const unsigned char *value, char *buf);
static int dnt900_print_8_ascii(const unsigned char *value, char *buf);
static int dnt900_print_16_ascii(const unsigned char *value, char *buf);
static int dnt900_print_5_macs(const unsigned char *value, char *buf);

static int dnt900_parse_bytes(int bytes, const char *buf, size_t count, unsigned char *value);
static int dnt900_parse_1_bytes(const char *buf, size_t count, unsigned char *value);
static int dnt900_parse_2_bytes(const char *buf, size_t count, unsigned char *value);
static int dnt900_parse_3_bytes(const char *buf, size_t count, unsigned char *value);
static int dnt900_parse_4_bytes(const char *buf, size_t count, unsigned char *value);
static int dnt900_parse_hex(int bytes, const char *buf, size_t count, unsigned char *value);
static int dnt900_parse_16_hex(const char *buf, size_t count, unsigned char *value);
static int dnt900_parse_32_hex(const char *buf, size_t count, unsigned char *value);
static int dnt900_parse_16_ascii(const char *buf, size_t count, unsigned char *value);

static int dnt900_radio_add_attributes(struct dnt900_radio *radio);
static int dnt900_local_add_attributes(struct dnt900_local *local);

static int dnt900_enter_protocol_mode(struct dnt900_local *local);
static int dnt900_get_register(struct dnt900_local *local, const struct dnt900_register *reg, unsigned char *value);
static int dnt900_get_remote_register(struct dnt900_local *local, const unsigned char *sys_address, const struct dnt900_register *reg, unsigned char *value);
static int dnt900_set_register(struct dnt900_local *local, const struct dnt900_register *reg, const unsigned char *value);
static int dnt900_set_remote_register(struct dnt900_local *local, const unsigned char *sys_address, const struct dnt900_register *reg, const unsigned char *value);
static int dnt900_discover(struct dnt900_local *local, const unsigned char *mac_address, unsigned char *sys_address);

static void dnt900_radio_read_params(struct dnt900_radio *radio, struct dnt900_radio_params *params);
static void dnt900_local_read_params(struct dnt900_local *local, struct dnt900_local_params *params);

static int dnt900_radio_get_register(struct dnt900_radio *radio, const struct dnt900_register *reg, unsigned char *value);
static int dnt900_radio_set_register(struct dnt900_radio *radio, const struct dnt900_register *reg, const unsigned char *value);

static ssize_t dnt900_show_reg(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t dnt900_store_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t dnt900_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t dnt900_store_discover(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t dnt900_show_local_attr(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t dnt900_show_radio_attr(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t dnt900_store_join_permit(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t dnt900_store_join_deny(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t dnt900_store_leave(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static int dnt900_local_get_params(struct dnt900_local *local);
static int dnt900_radio_get_params(struct dnt900_radio *radio);
static int dnt900_radio_map_remotes(struct dnt900_radio *radio);

static struct dnt900_radio *dnt900_create_radio(struct dnt900_local *local, const unsigned char *mac_address);
static void dnt900_release_radio(struct device *dev);
static int dnt900_add_radio(struct dnt900_local *local, const unsigned char *mac_address);
static int dnt900_unregister_radio(struct device *dev, void *unused);

static int dnt900_radio_matches_sys_address(struct device *dev, void *data);
static int dnt900_radio_matches_mac_address(struct device *dev, void *data);
static int dnt900_radio_is_local(struct device *dev, void *data);
static int dnt900_radio_matches_net_id(struct device *dev, void *data);
static int dnt900_radio_routes_net_id(struct device *dev, void *data);
static int dnt900_radio_in_subnet(struct dnt900_radio *radio, void *data);

static bool dnt900_radio_exists(struct dnt900_local *local, const unsigned char *mac_address);
static int dnt900_dispatch_to_radio(struct dnt900_local *local, void *finder_data, int (*finder)(struct device *, void *), void *action_data, int (*action)(struct dnt900_radio *, void *));
static int dnt900_dispatch_to_radio_no_data(struct dnt900_local *local, void *finder_data, int (*finder)(struct device *, void *), int (*action)(struct dnt900_radio *));
static int dnt900_apply_to_radio(struct device *dev, void *data);
static void dnt900_for_each_radio(struct dnt900_local *local, int (*action)(struct dnt900_radio *));
static int dnt900_apply_to_matching(struct device *dev, void *data);
static void dnt900_for_matching_radios(struct dnt900_radio *radio, void *data, int (*match)(struct device *, void *), int (*action)(struct dnt900_radio *));

static int dnt900_send_packet(struct dnt900_local *local, const unsigned char *packet);
static int dnt900_send_packet_get_result(struct dnt900_local *local, const unsigned char *packet, unsigned char *result);
static int dnt900_clear_packets(struct dnt900_local *local);

static int dnt900_radio_wake_tty(struct dnt900_radio *radio);
static int dnt900_radio_hangup_tty(struct dnt900_radio *radio);
static int dnt900_radio_hangup_ttys(struct dnt900_radio *radio);

static int dnt900_radio_drain_fifo(struct dnt900_radio *radio);
static void dnt900_local_drain_fifo(struct dnt900_local *local);

static int dnt900_tty_port_activate(struct tty_port *port, struct tty_struct *tty);
static void dnt900_tty_port_shutdown(struct tty_port *port);

static int dnt900_radio_write(struct dnt900_radio *radio, void *data);

static int dnt900_tty_open(struct tty_struct *tty, struct file *filp);
static void dnt900_tty_close(struct tty_struct *tty, struct file *filp);
static void dnt900_tty_hangup(struct tty_struct *tty);
static int dnt900_tty_write(struct tty_struct *tty, const unsigned char *buf, int len);
static int dnt900_tty_put_char(struct tty_struct *tty, unsigned char ch);
static int dnt900_tty_write_room(struct tty_struct *tty);
static int dnt900_tty_chars_in_buffer(struct tty_struct *tty);
static void dnt900_tty_flush_chars(struct tty_struct *tty);
static void dnt900_tty_wait_until_sent(struct tty_struct *tty, int timeout);
static void dnt900_tty_flush_buffer(struct tty_struct *tty);

static ssize_t dnt900_ldisc_write(struct tty_struct *tty, struct file *filp, const unsigned char *buf, size_t len);
static void dnt900_ldisc_receive_buf(struct tty_struct *tty, const unsigned char *cp, char *fp, int count);
static void dnt900_ldisc_write_wakeup(struct tty_struct *tty);
static int dnt900_ldisc_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg);
static void dnt900_ldisc_flush_buffer(struct tty_struct *tty);

static int dnt900_process_reply(struct dnt900_local *local, unsigned char *response);
static int dnt900_process_event(struct dnt900_local *local, unsigned char *response);
static int dnt900_process_rx_event(struct dnt900_radio *radio, void *data);
static int dnt900_process_announcement(struct dnt900_local *local, unsigned char *annc);
static int dnt900_radio_process_announcement(struct dnt900_radio *radio, void *data);
static int dnt900_process_argument_error(struct dnt900_local *local);
static int dnt900_process_join_request(struct dnt900_local *local, unsigned char *request);
static int dnt900_radio_report_rssi(struct dnt900_radio *radio, void *data);
static int dnt900_radio_check_heartbeat(struct dnt900_radio *radio, void *data);
static int dnt900_radio_out(struct dnt900_radio *radio, void *data);

static void dnt900_schedule_work(struct dnt900_local *local, const unsigned char *address, void (*work_function)(struct work_struct *));
static void dnt900_add_new_mac_address(struct work_struct *ws);
static void dnt900_add_new_sys_address(struct work_struct *ws);
static void dnt900_refresh_radio(struct work_struct *ws);
static void dnt900_refresh_local(struct work_struct *ws);
static void dnt900_init_local(struct work_struct *ws);
static void dnt900_map_remotes(struct work_struct *ws);

static irqreturn_t dnt900_cts_handler(int irq, void *dev_id);

static struct dnt900_local *dnt900_create_local(struct tty_struct *tty);
static void dnt900_unregister_local(struct dnt900_local *local);
static void dnt900_release_local(struct device *dev);

static int dnt900_ldisc_open(struct tty_struct *tty);
static void dnt900_ldisc_close(struct tty_struct *tty);
static int dnt900_ldisc_hangup(struct tty_struct *tty);

#ifndef N_DNT900
#define N_DNT900 29
#endif

static int n_dnt900 = N_DNT900;
static int radios = 255;
static int gpio_cts = -1;

module_param(radios, int, S_IRUGO);
MODULE_PARM_DESC(radios, "maximum number of radios");
module_param(n_dnt900, int, S_IRUGO);
MODULE_PARM_DESC(n_dnt900, "line discipline number");
module_param(gpio_cts, int, S_IRUGO);
MODULE_PARM_DESC(gpio_cts, "GPIO number for /HOST_CTS signal");

static const struct device_attribute dnt900_local_command_attributes[] = {
	__ATTR(reset,       ATTR_W, NULL, dnt900_store_reset),
	__ATTR(discover,    ATTR_W, NULL, dnt900_store_discover),
	__ATTR(join_permit, ATTR_W, NULL, dnt900_store_join_permit),
	__ATTR(join_deny,   ATTR_W, NULL, dnt900_store_join_deny),
};

static const struct device_attribute dnt900_radio_command_attributes[] = {
	__ATTR(leave, ATTR_W, NULL, dnt900_store_leave),
};

enum {
	announce = 0,
	error,
	parent,
	join_request,
};

static const struct device_attribute dnt900_local_attributes[] = {
	__ATTR(announce,     ATTR_R, dnt900_show_local_attr, NULL),
	__ATTR(error,        ATTR_R, dnt900_show_local_attr, NULL),
	__ATTR(parent,       ATTR_R, dnt900_show_local_attr, NULL),
	__ATTR(join_request, ATTR_R, dnt900_show_local_attr, NULL),
};

enum {
	report_GPIO0 = 0,
	report_GPIO1,
	report_GPIO2,
	report_GPIO3,
	report_GPIO4,
	report_GPIO5,
	report_ADC0,
	report_ADC1,
	report_ADC2,
	report_EventFlags,
	range,
	success_rate,
	beacon_rssi,
	parent_rssi,
	rssi,
};

static const struct device_attribute dnt900_radio_attributes[] = {
	__ATTR(report_GPIO0,      ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(report_GPIO1,      ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(report_GPIO2,      ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(report_GPIO3,      ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(report_GPIO4,      ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(report_GPIO5,      ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(report_ADC0,       ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(report_ADC1,       ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(report_ADC2,       ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(report_EventFlags, ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(range,             ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(success_rate,      ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(beacon_rssi,       ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(parent_rssi,       ATTR_R, dnt900_show_radio_attr, NULL),
	__ATTR(rssi,              ATTR_R, dnt900_show_radio_attr, NULL),
};

enum {
	DeviceMode = 0,
	RF_DataRate,
	HopDuration,
	InitialParentNwkID,
	SecurityKey,
	SleepMode,
	WakeResponseTime,
	WakeLinkTimeout,
	TxPower,
	ExtSyncEnable,
	DiversityMode,
	UserTag,
	RegDenialDelay,
	RmtTransDestAddr,
	TreeRoutingEn,
	BaseModeNetID,
	StaticNetAddr,
	HeartbeatIntrvl,
	TreeRoutingSysID,
	EnableRtAcks,
	FrequencyBand,
	AccessMode,
	BaseSlotSize,
	LeasePeriod,
	ARQ_Mode,
	ARQ_AttemptLimit,
	MaxSlots,
	CSMA_Predelay,
	CSMA_Backoff,
	MaxPropDelay,
	LinkDropThreshold,
	CSMA_RemtSlotSize,
	CSMA_BusyThreshold,
	RangingInterval,
	AuthMode,
	P2PReplyTimeout,
	MacAddress,
	CurrNwkAddr,
	CurrNwkID,
	CurrRF_DataRate,
	CurrFreqBand,
	LinkStatus,
	RemoteSlotSize,
	TDMA_NumSlots,
	TDMA_CurrSlot,
	HardwareVersion,
	FirmwareVersion,
	FirmwareBuildNum,
	SuperframeCount,
	RSSI_Idle,
	RSSI_Last,
	CurrTxPower,
	CurrAttemptLimit,
	CurrRangeDelay,
	FirmwareBuildDate,
	FirmwareBuildTime,
	ModelNumber,
	CurrBaseModeNetID,
	AveRXPwrOvHopSeq,
	// ParentACKQual,
	SerialRate,
	SerialParams,
	SerialControls,
	SPI_Mode,
	SPI_Divisor,
	SPI_Options,
	SPI_MasterCmdLen,
	SPI_MasterCmdStr,
	ProtocolMode,
	ProtocolOptions,
	TxTimeout,
	MinPacketLength,
	AnnounceOptions,
	TransLinkAnnEn,
	ProtocolSequenceEn,
	TransPtToPtMode,
	MaxPktsPerHop,
	GPIO0,
	GPIO1,
	GPIO2,
	GPIO3,
	GPIO4,
	GPIO5,
	ADC0,
	ADC1,
	ADC2,
	EventFlags,
	PWM0,
	PWM1,
	GPIO_Dir,
	GPIO_Init,
	GPIO_Alt,
	GPIO_Edge_Trigger,
	GPIO_SleepMode,
	GPIO_SleepDir,
	GPIO_SleepState,
	PWM0_Init,
	PWM1_Init,
	ADC_SampleIntvl,
	ADC0_ThresholdLo,
	ADC0_ThresholdHi,
	ADC1_ThresholdLo,
	ADC1_ThresholdHi,
	ADC2_ThresholdLo,
	ADC2_ThresholdHi,
	IO_ReportTrigger,
	IO_ReportInterval,
	IO_ReportPreDel,
	IO_ReportRepeat,
	ApprovedAddr00,
	ApprovedAddr01,
	ApprovedAddr02,
	ApprovedAddr03,
	ApprovedAddr04,
	ApprovedAddr05,
	ApprovedAddr06,
	ApprovedAddr07,
	ApprovedAddr08,
	ApprovedAddr09,
	ApprovedAddr10,
	ApprovedAddr11,
	ApprovedAddr12,
	ApprovedAddr13,
	ApprovedAddr14,
	ApprovedAddr15,
	BaseNetworkID,
	ParentNetworkID01,
	ParentNetworkID02,
	ParentNetworkID03,
	ParentNetworkID04,
	ParentNetworkID05,
	ParentNetworkID06,
	ParentNetworkID07,
	ParentNetworkID08,
	ParentNetworkID09,
	ParentNetworkID10,
	ParentNetworkID11,
	ParentNetworkID12,
	ParentNetworkID13,
	ParentNetworkID14,
	ParentNetworkID15,
	ParentNetworkID16,
	ParentNetworkID17,
	ParentNetworkID18,
	ParentNetworkID19,
	ParentNetworkID20,
	ParentNetworkID21,
	ParentNetworkID22,
	ParentNetworkID23,
	ParentNetworkID24,
	ParentNetworkID25,
	ParentNetworkID26,
	ParentNetworkID27,
	ParentNetworkID28,
	ParentNetworkID29,
	ParentNetworkID30,
	ParentNetworkID31,
	ParentNetworkID32,
	ParentNetworkID33,
	ParentNetworkID34,
	ParentNetworkID35,
	ParentNetworkID36,
	ParentNetworkID37,
	ParentNetworkID38,
	ParentNetworkID39,
	ParentNetworkID40,
	ParentNetworkID41,
	ParentNetworkID42,
	ParentNetworkID43,
	ParentNetworkID44,
	ParentNetworkID45,
	ParentNetworkID46,
	ParentNetworkID47,
	ParentNetworkID48,
	ParentNetworkID49,
	ParentNetworkID50,
	ParentNetworkID51,
	ParentNetworkID52,
	ParentNetworkID53,
	ParentNetworkID54,
	ParentNetworkID55,
	ParentNetworkID56,
	ParentNetworkID57,
	ParentNetworkID58,
	ParentNetworkID59,
	ParentNetworkID60,
	ParentNetworkID61,
	ParentNetworkID62,
	ParentNetworkID63,
	RegMACAddr00,
	RegMACAddr01,
	RegMACAddr02,
	RegMACAddr03,
	RegMACAddr04,
	RegMACAddr05,
	RegMACAddr06,
	RegMACAddr07,
	RegMACAddr08,
	RegMACAddr09,
	RegMACAddr10,
	RegMACAddr11,
	RegMACAddr12,
	RegMACAddr13,
	RegMACAddr14,
	RegMACAddr15,
	RegMACAddr16,
	RegMACAddr17,
	RegMACAddr18,
	RegMACAddr19,
	RegMACAddr20,
	RegMACAddr21,
	RegMACAddr22,
	RegMACAddr23,
	RegMACAddr24,
	RegMACAddr25,
	UcReset,
	SleepModeOverride,
	RoutingTableUpd,
	DiagSerialRate,
	MemorySave,
};

#define DNT900_REG_ATTR(_name, _mode, _bank, _offset, _span, _print, _parse) { \
	.attr = __ATTR(_name, _mode, dnt900_show_reg, dnt900_store_reg), \
	.reg = { \
		.bank = _bank, \
		.offset = _offset, \
		.span = _span \
	}, \
	.print = _print, \
	.parse = _parse, \
}

static const struct dnt900_reg_attribute dnt900_reg_attributes[] = {
	DNT900_REG_ATTR(DeviceMode,         ATTR_RW, 0x00, 0x00, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(RF_DataRate,        ATTR_RW, 0x00, 0x01, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(HopDuration,        ATTR_RW, 0x00, 0x02, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(InitialParentNwkID, ATTR_RW, 0x00, 0x04, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(SecurityKey,        ATTR_W,  0x00, 0x05, 0x10, NULL, dnt900_parse_16_hex),
	DNT900_REG_ATTR(SleepMode,          ATTR_RW, 0x00, 0x15, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(WakeResponseTime,   ATTR_RW, 0x00, 0x16, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(WakeLinkTimeout,    ATTR_RW, 0x00, 0x17, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(TxPower,            ATTR_RW, 0x00, 0x18, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(ExtSyncEnable,      ATTR_RW, 0x00, 0x19, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(DiversityMode,      ATTR_RW, 0x00, 0x1A, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(UserTag,            ATTR_RW, 0x00, 0x1C, 0x10, dnt900_print_16_ascii, dnt900_parse_16_ascii),
	DNT900_REG_ATTR(RegDenialDelay,     ATTR_RW, 0x00, 0x2C, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(RmtTransDestAddr,   ATTR_RW, 0x00, 0x2E, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(TreeRoutingEn,      ATTR_RW, 0x00, 0x34, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(BaseModeNetID,      ATTR_RW, 0x00, 0x35, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(StaticNetAddr,      ATTR_RW, 0x00, 0x36, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(HeartbeatIntrvl,    ATTR_RW, 0x00, 0x37, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(TreeRoutingSysID,   ATTR_RW, 0x00, 0x39, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(EnableRtAcks,       ATTR_RW, 0x00, 0x3A, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(FrequencyBand,      ATTR_RW, 0x01, 0x00, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(AccessMode,         ATTR_RW, 0x01, 0x01, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(BaseSlotSize,       ATTR_RW, 0x01, 0x02, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(LeasePeriod,        ATTR_RW, 0x01, 0x03, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(ARQ_Mode,           ATTR_RW, 0x01, 0x04, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(ARQ_AttemptLimit,   ATTR_RW, 0x01, 0x05, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(MaxSlots,           ATTR_RW, 0x01, 0x06, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(CSMA_Predelay,      ATTR_RW, 0x01, 0x07, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(CSMA_Backoff,       ATTR_RW, 0x01, 0x08, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(MaxPropDelay,       ATTR_RW, 0x01, 0x09, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(LinkDropThreshold,  ATTR_RW, 0x01, 0x0A, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(CSMA_RemtSlotSize,  ATTR_RW, 0x01, 0x0B, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(CSMA_BusyThreshold, ATTR_RW, 0x01, 0x0C, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(RangingInterval,    ATTR_RW, 0x01, 0x0D, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(AuthMode,           ATTR_RW, 0x01, 0x0E, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(P2PReplyTimeout,    ATTR_RW, 0x01, 0x0F, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(MacAddress,         ATTR_R,  0x02, 0x00, 0x03, dnt900_print_3_bytes, NULL),
	DNT900_REG_ATTR(CurrNwkAddr,        ATTR_R,  0x02, 0x03, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(CurrNwkID,          ATTR_R,  0x02, 0x04, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(CurrRF_DataRate,    ATTR_R,  0x02, 0x05, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(CurrFreqBand,       ATTR_R,  0x02, 0x06, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(LinkStatus,         ATTR_R,  0x02, 0x07, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(RemoteSlotSize,     ATTR_R,  0x02, 0x08, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(TDMA_NumSlots,      ATTR_R,  0x02, 0x09, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(TDMA_CurrSlot,      ATTR_R,  0x02, 0x0B, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(HardwareVersion,    ATTR_R,  0x02, 0x0C, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(FirmwareVersion,    ATTR_R,  0x02, 0x0D, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(FirmwareBuildNum,   ATTR_R,  0x02, 0x0E, 0x02, dnt900_print_2_bytes, NULL),
	DNT900_REG_ATTR(SuperframeCount,    ATTR_R,  0x02, 0x11, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(RSSI_Idle,          ATTR_R,  0x02, 0x12, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(RSSI_Last,          ATTR_R,  0x02, 0x13, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(CurrTxPower,        ATTR_R,  0x02, 0x14, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(CurrAttemptLimit,   ATTR_R,  0x02, 0x15, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(CurrRangeDelay,     ATTR_R,  0x02, 0x16, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(FirmwareBuildDate,  ATTR_R,  0x02, 0x17, 0x08, dnt900_print_8_ascii, NULL),
	DNT900_REG_ATTR(FirmwareBuildTime,  ATTR_R,  0x02, 0x1F, 0x08, dnt900_print_8_ascii, NULL),
	DNT900_REG_ATTR(ModelNumber,        ATTR_R,  0x02, 0x27, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(CurrBaseModeNetID,  ATTR_R,  0x02, 0x28, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(AveRXPwrOvHopSeq,   ATTR_R,  0x02, 0x29, 0x01, dnt900_print_1_bytes, NULL),
	// DNT900_REG_ATTR(ParentACKQual,      ATTR_R,  0x02, 0x2A, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(SerialRate,         ATTR_RW, 0x03, 0x00, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(SerialParams,       ATTR_RW, 0x03, 0x02, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(SerialControls,     ATTR_RW, 0x03, 0x03, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(SPI_Mode,           ATTR_RW, 0x03, 0x04, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(SPI_Divisor,        ATTR_RW, 0x03, 0x05, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(SPI_Options,        ATTR_RW, 0x03, 0x06, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(SPI_MasterCmdLen,   ATTR_RW, 0x03, 0x07, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(SPI_MasterCmdStr,   ATTR_RW, 0x03, 0x08, 0x20, dnt900_print_32_hex, dnt900_parse_32_hex),
	DNT900_REG_ATTR(ProtocolMode,       ATTR_RW, 0x04, 0x00, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(ProtocolOptions,    ATTR_RW, 0x04, 0x01, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(TxTimeout,          ATTR_RW, 0x04, 0x02, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(MinPacketLength,    ATTR_RW, 0x04, 0x03, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(AnnounceOptions,    ATTR_RW, 0x04, 0x04, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(TransLinkAnnEn,     ATTR_RW, 0x04, 0x05, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(ProtocolSequenceEn, ATTR_RW, 0x04, 0x06, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(TransPtToPtMode,    ATTR_RW, 0x04, 0x07, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(MaxPktsPerHop,      ATTR_RW, 0x04, 0x08, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO0,              ATTR_RW, 0x05, 0x00, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO1,              ATTR_RW, 0x05, 0x01, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO2,              ATTR_RW, 0x05, 0x02, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO3,              ATTR_RW, 0x05, 0x03, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO4,              ATTR_RW, 0x05, 0x04, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO5,              ATTR_RW, 0x05, 0x05, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(ADC0,               ATTR_R,  0x05, 0x06, 0x02, dnt900_print_2_bytes, NULL),
	DNT900_REG_ATTR(ADC1,               ATTR_R,  0x05, 0x08, 0x02, dnt900_print_2_bytes, NULL),
	DNT900_REG_ATTR(ADC2,               ATTR_R,  0x05, 0x0A, 0x02, dnt900_print_2_bytes, NULL),
	DNT900_REG_ATTR(EventFlags,         ATTR_R,  0x05, 0x0C, 0x02, dnt900_print_2_bytes, NULL),
	DNT900_REG_ATTR(PWM0,               ATTR_RW, 0x05, 0x0E, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(PWM1,               ATTR_RW, 0x05, 0x10, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(GPIO_Dir,           ATTR_RW, 0x06, 0x00, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO_Init,          ATTR_RW, 0x06, 0x01, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO_Alt,           ATTR_RW, 0x06, 0x02, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO_Edge_Trigger,  ATTR_RW, 0x06, 0x03, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO_SleepMode,     ATTR_RW, 0x06, 0x04, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO_SleepDir,      ATTR_RW, 0x06, 0x05, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(GPIO_SleepState,    ATTR_RW, 0x06, 0x06, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(PWM0_Init,          ATTR_RW, 0x06, 0x07, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(PWM1_Init,          ATTR_RW, 0x06, 0x09, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(ADC_SampleIntvl,    ATTR_RW, 0x06, 0x0B, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(ADC0_ThresholdLo,   ATTR_RW, 0x06, 0x0D, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(ADC0_ThresholdHi,   ATTR_RW, 0x06, 0x0F, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(ADC1_ThresholdLo,   ATTR_RW, 0x06, 0x11, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(ADC1_ThresholdHi,   ATTR_RW, 0x06, 0x13, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(ADC2_ThresholdLo,   ATTR_RW, 0x06, 0x15, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(ADC2_ThresholdHi,   ATTR_RW, 0x06, 0x17, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(IO_ReportTrigger,   ATTR_RW, 0x06, 0x19, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(IO_ReportInterval,  ATTR_RW, 0x06, 0x1A, 0x04, dnt900_print_4_bytes, dnt900_parse_4_bytes),
	DNT900_REG_ATTR(IO_ReportPreDel,    ATTR_RW, 0x06, 0x1E, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(IO_ReportRepeat,    ATTR_RW, 0x06, 0x1F, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(ApprovedAddr00,     ATTR_RW, 0x07, 0x00, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr01,     ATTR_RW, 0x07, 0x03, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr02,     ATTR_RW, 0x07, 0x06, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr03,     ATTR_RW, 0x07, 0x09, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr04,     ATTR_RW, 0x07, 0x0C, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr05,     ATTR_RW, 0x07, 0x0F, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr06,     ATTR_RW, 0x07, 0x12, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr07,     ATTR_RW, 0x07, 0x15, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr08,     ATTR_RW, 0x07, 0x18, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr09,     ATTR_RW, 0x07, 0x1B, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr10,     ATTR_RW, 0x07, 0x1E, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr11,     ATTR_RW, 0x07, 0x21, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr12,     ATTR_RW, 0x07, 0x24, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr13,     ATTR_RW, 0x07, 0x27, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr14,     ATTR_RW, 0x07, 0x2A, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(ApprovedAddr15,     ATTR_RW, 0x07, 0x2D, 0x03, dnt900_print_3_bytes, dnt900_parse_3_bytes),
	DNT900_REG_ATTR(BaseNetworkID,      ATTR_R,  0x08, 0x00, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID01,  ATTR_R,  0x08, 0x01, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID02,  ATTR_R,  0x08, 0x02, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID03,  ATTR_R,  0x08, 0x03, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID04,  ATTR_R,  0x08, 0x04, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID05,  ATTR_R,  0x08, 0x05, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID06,  ATTR_R,  0x08, 0x06, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID07,  ATTR_R,  0x08, 0x07, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID08,  ATTR_R,  0x08, 0x08, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID09,  ATTR_R,  0x08, 0x09, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID10,  ATTR_R,  0x08, 0x0A, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID11,  ATTR_R,  0x08, 0x0B, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID12,  ATTR_R,  0x08, 0x0C, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID13,  ATTR_R,  0x08, 0x0D, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID14,  ATTR_R,  0x08, 0x0E, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID15,  ATTR_R,  0x08, 0x0F, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID16,  ATTR_R,  0x08, 0x10, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID17,  ATTR_R,  0x08, 0x11, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID18,  ATTR_R,  0x08, 0x12, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID19,  ATTR_R,  0x08, 0x13, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID20,  ATTR_R,  0x08, 0x14, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID21,  ATTR_R,  0x08, 0x15, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID22,  ATTR_R,  0x08, 0x16, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID23,  ATTR_R,  0x08, 0x17, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID24,  ATTR_R,  0x08, 0x18, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID25,  ATTR_R,  0x08, 0x19, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID26,  ATTR_R,  0x08, 0x1A, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID27,  ATTR_R,  0x08, 0x1B, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID28,  ATTR_R,  0x08, 0x1C, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID29,  ATTR_R,  0x08, 0x1D, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID30,  ATTR_R,  0x08, 0x1E, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID31,  ATTR_R,  0x08, 0x1F, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID32,  ATTR_R,  0x08, 0x20, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID33,  ATTR_R,  0x08, 0x21, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID34,  ATTR_R,  0x08, 0x22, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID35,  ATTR_R,  0x08, 0x23, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID36,  ATTR_R,  0x08, 0x24, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID37,  ATTR_R,  0x08, 0x25, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID38,  ATTR_R,  0x08, 0x26, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID39,  ATTR_R,  0x08, 0x27, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID40,  ATTR_R,  0x08, 0x28, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID41,  ATTR_R,  0x08, 0x29, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID42,  ATTR_R,  0x08, 0x2A, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID43,  ATTR_R,  0x08, 0x2B, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID44,  ATTR_R,  0x08, 0x2C, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID45,  ATTR_R,  0x08, 0x2D, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID46,  ATTR_R,  0x08, 0x2E, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID47,  ATTR_R,  0x08, 0x2F, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID48,  ATTR_R,  0x08, 0x30, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID49,  ATTR_R,  0x08, 0x31, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID50,  ATTR_R,  0x08, 0x32, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID51,  ATTR_R,  0x08, 0x33, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID52,  ATTR_R,  0x08, 0x34, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID53,  ATTR_R,  0x08, 0x35, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID54,  ATTR_R,  0x08, 0x36, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID55,  ATTR_R,  0x08, 0x37, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID56,  ATTR_R,  0x08, 0x38, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID57,  ATTR_R,  0x08, 0x39, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID58,  ATTR_R,  0x08, 0x3A, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID59,  ATTR_R,  0x08, 0x3B, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID60,  ATTR_R,  0x08, 0x3C, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID61,  ATTR_R,  0x08, 0x3D, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID62,  ATTR_R,  0x08, 0x3E, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(ParentNetworkID63,  ATTR_R,  0x08, 0x3F, 0x01, dnt900_print_1_bytes, NULL),
	DNT900_REG_ATTR(RegMACAddr00,       ATTR_R,  0x09, 0x00, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr01,       ATTR_R,  0x09, 0x01, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr02,       ATTR_R,  0x09, 0x02, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr03,       ATTR_R,  0x09, 0x03, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr04,       ATTR_R,  0x09, 0x04, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr05,       ATTR_R,  0x09, 0x05, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr06,       ATTR_R,  0x09, 0x06, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr07,       ATTR_R,  0x09, 0x07, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr08,       ATTR_R,  0x09, 0x08, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr09,       ATTR_R,  0x09, 0x09, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr10,       ATTR_R,  0x09, 0x0A, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr11,       ATTR_R,  0x09, 0x0B, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr12,       ATTR_R,  0x09, 0x0C, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr13,       ATTR_R,  0x09, 0x0D, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr14,       ATTR_R,  0x09, 0x0E, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr15,       ATTR_R,  0x09, 0x0F, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr16,       ATTR_R,  0x09, 0x10, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr17,       ATTR_R,  0x09, 0x11, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr18,       ATTR_R,  0x09, 0x12, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr19,       ATTR_R,  0x09, 0x13, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr20,       ATTR_R,  0x09, 0x14, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr21,       ATTR_R,  0x09, 0x15, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr22,       ATTR_R,  0x09, 0x16, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr23,       ATTR_R,  0x09, 0x17, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr24,       ATTR_R,  0x09, 0x18, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(RegMACAddr25,       ATTR_R,  0x09, 0x19, 0x0F, dnt900_print_5_macs, NULL),
	DNT900_REG_ATTR(UcReset,            ATTR_W,  0xFF, 0x00, 0x01, NULL, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(SleepModeOverride,  ATTR_RW, 0xFF, 0x0C, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(RoutingTableUpd,    ATTR_RW, 0xFF, 0x1C, 0x01, dnt900_print_1_bytes, dnt900_parse_1_bytes),
	DNT900_REG_ATTR(DiagSerialRate,     ATTR_RW, 0xFF, 0x20, 0x02, dnt900_print_2_bytes, dnt900_parse_2_bytes),
	DNT900_REG_ATTR(MemorySave,         ATTR_W,  0xFF, 0xFF, 0x01, NULL, dnt900_parse_1_bytes),
};

#define REG(name) (&dnt900_reg_attributes[(name)].reg)

struct dnt900_local {
	struct device dev;
	struct tty_struct *tty;
	int gpio_cts;
	struct list_head transactions;
	struct mutex transactions_lock;
	struct mutex radios_lock;
	struct rw_semaphore closed_lock;
	spinlock_t param_lock;
	spinlock_t tx_fifo_lock;
	spinlock_t rx_fifo_lock;
	spinlock_t attributes_lock;
	struct workqueue_struct *workqueue;
	struct dnt900_local_params params;
	unsigned char mac_address[3];
	DECLARE_KFIFO(rx_fifo, unsigned char, RX_BUFFER_SIZE);
	DECLARE_KFIFO(tx_fifo, unsigned char, TX_BUFFER_SIZE);
	wait_queue_head_t tx_queue;
	unsigned char attributes[ARRAY_SIZE(dnt900_local_attributes)][32];
};

struct dnt900_radio {
	struct device dev;
	bool is_local;
	spinlock_t param_lock;
	spinlock_t attributes_lock;
	unsigned char mac_address[3];
	struct dnt900_radio_params params;
	char name[80];
	unsigned int tty_index;
	struct tty_port port;
	DECLARE_KFIFO(fifo, unsigned char, TTY_BUFFER_SIZE);
	unsigned char attributes[ARRAY_SIZE(dnt900_radio_attributes)][8];
};

static struct class *dnt900_class;
static struct tty_driver *dnt900_tty_driver;
static bool *dnt900_tty_indices;

static struct tty_ldisc_ops dnt900_ldisc_ops = {
	.magic        = TTY_LDISC_MAGIC,
	.name         = LDISC_NAME,
	.open         = dnt900_ldisc_open,
	.close        = dnt900_ldisc_close,
	.hangup       = dnt900_ldisc_hangup,
	.write        = dnt900_ldisc_write,
	.receive_buf  = dnt900_ldisc_receive_buf,
	.write_wakeup = dnt900_ldisc_write_wakeup,
	.ioctl        = dnt900_ldisc_ioctl,
	.flush_buffer = dnt900_ldisc_flush_buffer,
	.owner        = THIS_MODULE,
};

static struct tty_port_operations dnt900_tty_port_ops = {
	.activate = dnt900_tty_port_activate,
	.shutdown = dnt900_tty_port_shutdown,
};

static struct tty_operations dnt900_tty_ops = {
	.open            = dnt900_tty_open,
	.close           = dnt900_tty_close,
	.hangup          = dnt900_tty_hangup,
	.write           = dnt900_tty_write,
	.put_char        = dnt900_tty_put_char,
	.write_room      = dnt900_tty_write_room,
	.chars_in_buffer = dnt900_tty_chars_in_buffer,
	.flush_chars     = dnt900_tty_flush_chars,
	.wait_until_sent = dnt900_tty_wait_until_sent,
	.flush_buffer    = dnt900_tty_flush_buffer,
};

static struct ktermios dnt900_init_termios = {
	.c_iflag = 0,
	.c_oflag = 0,
	.c_cflag = B38400 | CS8 | CREAD | CLOCAL,
	.c_lflag = 0,
	.c_ispeed = 38400,
	.c_ospeed = 38400,
	.c_cc = INIT_C_CC,
};

static int dnt900_print_bytes(int bytes, const unsigned char *value, char *buf, size_t size)
{
	unsigned int count = scnprintf(buf, size, "0x");

	for (; bytes > 0; --bytes)
		count += scnprintf(buf + count, size - count, "%02X", value[bytes-1]);
	count += scnprintf(buf + count, size - count, "\n");
	return count;
}

static int dnt900_print_1_bytes(const unsigned char *value, char *buf)
{
	return dnt900_print_bytes(1, value, buf, PAGE_SIZE);
}

static int dnt900_print_2_bytes(const unsigned char *value, char *buf)
{
	return dnt900_print_bytes(2, value, buf, PAGE_SIZE);
}

static int dnt900_print_3_bytes(const unsigned char *value, char *buf)
{
	return dnt900_print_bytes(3, value, buf, PAGE_SIZE);
}

static int dnt900_print_4_bytes(const unsigned char *value, char *buf)
{
	return dnt900_print_bytes(4, value, buf, PAGE_SIZE);
}

static int dnt900_print_hex(int bytes, const unsigned char *value, char *buf, size_t size)
{
	unsigned int count = scnprintf(buf, size, "0x");

	for (; bytes > 0; ++value, --bytes)
		count += scnprintf(buf + count, size - count, "%02X", *value);
	count += scnprintf(buf + count, size - count, "\n");
	return count;
}

static int dnt900_print_32_hex(const unsigned char *value, char *buf)
{
	return dnt900_print_hex(32, value, buf, PAGE_SIZE);
}

static int dnt900_print_8_ascii(const unsigned char *value, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%.8s\n", value);
}

static int dnt900_print_16_ascii(const unsigned char *value, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%.16s\n", value);
}

static int dnt900_print_5_macs(const unsigned char *value, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
		"0x%02X%02X%02X 0x%02X%02X%02X 0x%02X%02X%02X 0x%02X%02X%02X 0x%02X%02X%02X\n",
		value[ 2], value[ 1], value[ 0],
		value[ 5], value[ 4], value[ 3],
		value[ 8], value[ 7], value[ 6],
		value[11], value[10], value[ 9],
		value[14], value[13], value[12]);
}

static int dnt900_parse_bytes(int bytes, const char *buf, size_t count, unsigned char *value)
{
	unsigned long result;

	TRY(kstrtoul(buf, 0, &result));
	if (result >> (8 * bytes))
		return -ERANGE;
	for (; bytes > 0; --bytes, ++value, result >>= 8)
		*value = result & 0xFF;
	return 0;
}

static int dnt900_parse_1_bytes(const char *buf, size_t count, unsigned char *value)
{
	return dnt900_parse_bytes(1, buf, count, value);
}

static int dnt900_parse_2_bytes(const char *buf, size_t count, unsigned char *value)
{
	return dnt900_parse_bytes(2, buf, count, value);
}

static int dnt900_parse_3_bytes(const char *buf, size_t count, unsigned char *value)
{
	return dnt900_parse_bytes(3, buf, count, value);
}

static int dnt900_parse_4_bytes(const char *buf, size_t count, unsigned char *value)
{
	return dnt900_parse_bytes(4, buf, count, value);
}

static int dnt900_parse_hex(int bytes, const char *buf, size_t count, unsigned char *value)
{
	int n;

	if (bytes * 2 + 2 != count)
		return -EINVAL;
	if (*buf++ != '0')
		return -EINVAL;
	if (tolower(*buf++) != 'x')
		return -EINVAL;
	for (; bytes > 0; --bytes, ++value) {
		*value = 0;
		for (n = 0; n < 2; ++n, ++buf) {
			if (!isxdigit(*buf))
				return -EINVAL;
			*value <<= 4;
			*value += isdigit(*buf) ? (*buf - '0') : (tolower(*buf) - 'a' + 10);
		}
	}
	return 0;
}

static int dnt900_parse_16_hex(const char *buf, size_t count, unsigned char *value)
{
	return dnt900_parse_hex(16, buf, count, value);
}

static int dnt900_parse_32_hex(const char *buf, size_t count, unsigned char *value)
{
	return dnt900_parse_hex(32, buf, count, value);
}

static int dnt900_parse_16_ascii(const char *buf, size_t count, unsigned char *value)
{
	int n;

	for (n = 0; n < 16; ++n)
		value[n] = n < count ? buf[n] : 0;
	return 0;
}

static int dnt900_radio_add_attributes(struct dnt900_radio *radio)
{
	int n;

	for (n = 0; n < ARRAY_SIZE(dnt900_reg_attributes); ++n)
		TRY(device_create_file(&radio->dev, &dnt900_reg_attributes[n].attr));
	if (radio->is_local)
		return 0;
	for (n = 0; n < ARRAY_SIZE(dnt900_radio_command_attributes); ++n)
		TRY(device_create_file(&radio->dev, dnt900_radio_command_attributes + n));
	for (n = 0; n < ARRAY_SIZE(dnt900_radio_attributes); ++n)
		TRY(device_create_file(&radio->dev, dnt900_radio_attributes + n));
	return 0;
}

static int dnt900_local_add_attributes(struct dnt900_local *local)
{
	int n;

	for (n = 0; n < ARRAY_SIZE(dnt900_local_command_attributes); ++n)
		TRY(device_create_file(&local->dev, dnt900_local_command_attributes + n));
	for (n = 0; n < ARRAY_SIZE(dnt900_local_attributes); ++n)
		TRY(device_create_file(&local->dev, dnt900_local_attributes + n));
	return 0;
}

static int dnt900_enter_protocol_mode(struct dnt900_local *local)
{
	PACKET(packet, COMMAND_ENTER_PROTOCOL_MODE, 'D', 'N', 'T', 'C', 'F', 'G');

	TRY(msleep_interruptible(STARTUP_DELAY_MS) ? -EINTR : 0);
	TRY(dnt900_send_packet_get_result(local, packet, NULL));
	TRY(dnt900_clear_packets(local));
	return 0;
}

static int dnt900_get_register(struct dnt900_local *local, const struct dnt900_register *reg, unsigned char *value)
{
	PACKET(packet, COMMAND_GET_REGISTER, reg->offset, reg->bank, reg->span);

	return dnt900_send_packet_get_result(local, packet, value);
}

static int dnt900_get_remote_register(struct dnt900_local *local, const unsigned char *sys_address, const struct dnt900_register *reg, unsigned char *value)
{
	PACKET(packet, COMMAND_GET_REMOTE_REGISTER, sys_address[0], sys_address[1], sys_address[2], reg->offset, reg->bank, reg->span);
	int tries, err = -EAGAIN;

	for (tries = REMOTE_REGISTER_RETRIES + 1; err == -EAGAIN && tries; --tries)
		err = dnt900_send_packet_get_result(local, packet, value);
	return err;
}

static int dnt900_set_register(struct dnt900_local *local, const struct dnt900_register *reg, const unsigned char *value)
{
	bool expect_reply = reg->bank != 0xFF || (reg->offset != 0x00 && (reg->offset != 0xFF || *value != 0x02));
	unsigned char packet[32 + 6] = { START_OF_PACKET, reg->span + 4, COMMAND_SET_REGISTER, reg->offset, reg->bank, reg->span };

	memcpy(packet + 6, value, reg->span);
	return expect_reply ? dnt900_send_packet_get_result(local, packet, NULL) : dnt900_send_packet(local, packet);
}

static int dnt900_set_remote_register(struct dnt900_local *local, const unsigned char *sys_address, const struct dnt900_register *reg, const unsigned char *value)
{
	bool expect_reply = reg->bank != 0xFF || (reg->offset != 0x00 && (reg->offset != 0xFF || *value != 0x02));
	unsigned char packet[32 + 9] = { START_OF_PACKET, reg->span + 7, COMMAND_SET_REMOTE_REGISTER, sys_address[0], sys_address[1], sys_address[2], reg->offset, reg->bank, reg->span };

	memcpy(packet + 9, value, reg->span);
	return expect_reply ? dnt900_send_packet_get_result(local, packet, NULL) : dnt900_send_packet(local, packet);
}

static int dnt900_discover(struct dnt900_local *local, const unsigned char *mac_address, unsigned char *sys_address)
{
	int err;
	PACKET(packet, COMMAND_DISCOVER, mac_address[0], mac_address[1], mac_address[2]);

	err = dnt900_send_packet_get_result(local, packet, sys_address);
	return err == -EAGAIN ? -ENODEV : err;
}

static void dnt900_radio_read_params(struct dnt900_radio *radio, struct dnt900_radio_params *params)
{
	unsigned long flags;

	spin_lock_irqsave(&radio->param_lock, flags);
	*params = radio->params;
	spin_unlock_irqrestore(&radio->param_lock, flags);
}

static void dnt900_local_read_params(struct dnt900_local *local, struct dnt900_local_params *params)
{
	unsigned long flags;

	spin_lock_irqsave(&local->param_lock, flags);
	*params = local->params;
	spin_unlock_irqrestore(&local->param_lock, flags);
}

static int dnt900_radio_get_register(struct dnt900_radio *radio, const struct dnt900_register *reg, unsigned char *value)
{
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	struct dnt900_radio_params params;

	if (radio->is_local)
		return dnt900_get_register(local, reg, value);
	dnt900_radio_read_params(radio, &params);
	return dnt900_get_remote_register(local, params.sys_address, reg, value);
}

static int dnt900_radio_set_register(struct dnt900_radio *radio, const struct dnt900_register *reg, const unsigned char *value)
{
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	struct dnt900_radio_params params;

	if (radio->is_local)
		return dnt900_set_register(local, reg, value);
	dnt900_radio_read_params(radio, &params);
	return dnt900_set_remote_register(local, params.sys_address, reg, value);
}

static ssize_t dnt900_show_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);
	struct dnt900_reg_attribute *attribute = container_of(attr, struct dnt900_reg_attribute, attr);
	unsigned char value[32];
	int err;

	if (!attribute->print)
		return -EPERM;
	err = dnt900_radio_get_register(radio, &attribute->reg, value);
	return err ? err : attribute->print(value, buf);
}

static ssize_t dnt900_store_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	struct dnt900_reg_attribute *attribute = container_of(attr, struct dnt900_reg_attribute, attr);
	unsigned char value[32];
	unsigned long flags;

	if (!attribute->parse)
		return -EPERM;
	TRY(attribute->parse(buf, count, value));
	TRY(dnt900_radio_set_register(radio, &attribute->reg, value));
	if (radio->is_local) {
		if (attr == &dnt900_reg_attributes[UcReset].attr)
			TRY(dnt900_enter_protocol_mode(local));
		if (attr == &dnt900_reg_attributes[BaseSlotSize].attr) {
			spin_lock_irqsave(&local->param_lock, flags);
			if (local->params.device_mode == DEVICE_MODE_BASE)
				local->params.slot_size = *value;
			spin_unlock_irqrestore(&local->param_lock, flags);
		}
	}
	return count;
}

static ssize_t dnt900_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dnt900_local *local = DEV_TO_LOCAL(dev);
	PACKET(packet, COMMAND_SOFTWARE_RESET, 0);

	TRY(dnt900_send_packet_get_result(local, packet, NULL));
	TRY(dnt900_enter_protocol_mode(local));
	return count;
}

static ssize_t dnt900_store_discover(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int mac_address_int;
	unsigned char mac_address[3];
	struct dnt900_local *local = DEV_TO_LOCAL(dev);
	int n;

	TRY(kstrtouint(buf, 16, &mac_address_int));
	for (n = 0; n < 3; ++n, mac_address_int >>= 8)
		mac_address[n] = mac_address_int & 0xFF;
	if (mac_address_int)
		return -EINVAL;
	TRY(dnt900_add_radio(local, mac_address));
	return count;
}

static ssize_t dnt900_show_local_attr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dnt900_local *local = DEV_TO_LOCAL(dev);
	int index = attr - dnt900_local_attributes;
	unsigned long flags;

	spin_lock_irqsave(&local->attributes_lock, flags);
	strcpy(buf, local->attributes[index]);
	spin_unlock_irqrestore(&local->attributes_lock, flags);
	return strlen(buf);
}

static ssize_t dnt900_show_radio_attr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);
	int index = attr - dnt900_radio_attributes;
	unsigned long flags;

	spin_lock_irqsave(&radio->attributes_lock, flags);
	strcpy(buf, radio->attributes[index]);
	spin_unlock_irqrestore(&radio->attributes_lock, flags);
	return strlen(buf);
}

static ssize_t dnt900_store_join_permit(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dnt900_local *local = DEV_TO_LOCAL(dev);
	PACKET(packet, COMMAND_JOIN_REPLY, 0, 0, 0, PERMIT_STATUS_PERMITTED);

	TRY(dnt900_parse_bytes(3, buf, count, packet + 3));
	TRY(dnt900_send_packet(local, packet));
	return count;
}

static ssize_t dnt900_store_join_deny(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dnt900_local *local = DEV_TO_LOCAL(dev);
	PACKET(packet, COMMAND_JOIN_REPLY, 0, 0, 0, PERMIT_STATUS_DENIED);

	TRY(dnt900_parse_bytes(3, buf, count, packet + 3));
	TRY(dnt900_send_packet(local, packet));
	return count;
}

static ssize_t dnt900_store_leave(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	struct dnt900_radio_params radio_params;
	struct dnt900_local_params local_params;
	PACKET(packet, COMMAND_REMOTE_LEAVE, 0, 0, 0, 0, 0);

	dnt900_local_read_params(local, &local_params);
	switch(local_params.device_mode) {
	case DEVICE_MODE_BASE:
		if (!radio->is_local)
			break;
		return -EPERM;
	case DEVICE_MODE_ROUTER:
		if (dnt900_radio_in_subnet(radio, &local_params.base_mode_net_id))
			break;
	default:
		return -EPERM;
	}
	TRY(dnt900_parse_bytes(2, buf, count, packet + 6));
	dnt900_radio_read_params(radio, &radio_params);
	COPY3(packet + 3, radio_params.sys_address);
	TRY(dnt900_send_packet(local, packet));
	dnt900_radio_hangup_ttys(radio);
	return count;
}

static int dnt900_local_get_params(struct dnt900_local *local)
{
	struct dnt900_local_params local_params;
	unsigned char announce_options, protocol_options, access_mode;
	unsigned long flags;

	TRY(dnt900_get_register(local, REG(LinkStatus), &local_params.link_status));
	TRY(dnt900_get_register(local, REG(AnnounceOptions), &announce_options));
	TRY(dnt900_get_register(local, REG(ProtocolOptions), &protocol_options));
	TRY(dnt900_get_register(local, REG(TreeRoutingEn), &local_params.tree_routing_en));
	TRY(dnt900_get_register(local, REG(DeviceMode), &local_params.device_mode));
	TRY(dnt900_get_register(local, REG(local_params.device_mode == DEVICE_MODE_BASE ? BaseSlotSize : RemoteSlotSize), &local_params.slot_size));
	TRY(dnt900_get_register(local, REG(BaseModeNetID), &local_params.base_mode_net_id));
	TRY(dnt900_get_register(local, REG(AccessMode), &access_mode));
	if (local_params.device_mode == DEVICE_MODE_BASE)
		COPY3(local_params.base_mac_address, local->mac_address);
	else if (local_params.link_status == LINK_STATUS_READY) {
		unsigned char base_sys_address[3] = { 0x00, 0x00, local_params.tree_routing_en ? 0xFF : 0x00 };
		TRY(dnt900_get_remote_register(local, base_sys_address, REG(MacAddress), local_params.base_mac_address));
	} else
		memset(local_params.base_mac_address, 0x00, 3);
	if (!(announce_options & ANNOUNCE_OPTIONS_LINKS) || !(announce_options & ANNOUNCE_OPTIONS_INIT))
		pr_err(LDISC_NAME ": set radio AnnounceOptions register to 0x07 for correct driver operation\n");
	if (!(protocol_options & PROTOCOL_OPTIONS_ENABLE_ANNOUNCE))
		pr_err(LDISC_NAME ": set radio ProtocolOptions register to 0x01 or 0x05 for correct driver operation\n");
	if (access_mode == ACCESS_MODE_TDMA_DYNAMIC && local_params.device_mode != DEVICE_MODE_BASE)
		pr_warn(LDISC_NAME ": driver may not operate correctly on a remote when using dynamic TDMA access mode\n");
	if (local_params.link_status == LINK_STATUS_READY && local_params.slot_size <= 10)
		pr_warn(LDISC_NAME ": slot size of %i likely to be insufficient\n", local_params.slot_size);
	spin_lock_irqsave(&local->param_lock, flags);
	local->params = local_params;
	spin_unlock_irqrestore(&local->param_lock, flags);
	return 0;
}

static int dnt900_radio_get_params(struct dnt900_radio *radio)
{
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	struct dnt900_local_params local_params;
	struct dnt900_radio_params radio_params;
	unsigned long flags;

	dnt900_local_read_params(local, &local_params);
	if (radio->is_local)
		memset(radio_params.sys_address, 0x00, 3);
	else if (local_params.tree_routing_en)
		TRY(dnt900_discover(local, radio->mac_address, radio_params.sys_address));
	else if (local_params.device_mode == DEVICE_MODE_BASE)
		COPY3(radio_params.sys_address, radio->mac_address);
	else if (EQUAL_ADDRESSES(local_params.base_mac_address, radio->mac_address))
		memset(radio_params.sys_address, 0x00, 3);
	else
		COPY3(radio_params.sys_address, radio->mac_address);
	if (radio->is_local) {
		TRY(dnt900_get_register(local, REG(CurrNwkID), &radio_params.curr_nwk_id));
		TRY(dnt900_get_register(local, REG(DeviceMode), &radio_params.device_mode));
		TRY(dnt900_get_register(local, REG(BaseModeNetID), &radio_params.base_mode_net_id));
	} else {
		TRY(dnt900_get_remote_register(local, radio_params.sys_address, REG(CurrNwkID), &radio_params.curr_nwk_id));
		TRY(dnt900_get_remote_register(local, radio_params.sys_address, REG(DeviceMode), &radio_params.device_mode));
		TRY(dnt900_get_remote_register(local, radio_params.sys_address, REG(BaseModeNetID), &radio_params.base_mode_net_id));
	}
	spin_lock_irqsave(&radio->param_lock, flags);
	radio->params = radio_params;
	spin_unlock_irqrestore(&radio->param_lock, flags);
	return 0;
}

static int dnt900_radio_map_remotes(struct dnt900_radio *radio)
{
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	unsigned char mac_addresses[15 * 26];
	int n;

	for (n = 0; n < 26; ++n)
		TRY(dnt900_radio_get_register(radio, REG(RegMACAddr00 + n), mac_addresses + 15 * n));
	for (n = 0; n < 26 * 5; ++n) {
		unsigned char *mac_address = mac_addresses + 3 * n;
		if (mac_address[0] || mac_address[1] || mac_address[2])
			dnt900_add_radio(local, mac_address);
	}
	return 0;
}

static struct dnt900_radio *dnt900_create_radio(struct dnt900_local *local, const unsigned char *mac_address)
{
	int err;
	struct dnt900_radio *radio = kzalloc(sizeof(*radio), GFP_KERNEL);

	UNWIND(err, radio ? 0 : -ENOMEM, fail_alloc);
	radio->is_local = EQUAL_ADDRESSES(local->mac_address, mac_address);
	COPY3(radio->mac_address, mac_address);
	spin_lock_init(&radio->param_lock);
	spin_lock_init(&radio->attributes_lock);
	snprintf(radio->name, ARRAY_SIZE(radio->name), "0x%02X%02X%02X", mac_address[2], mac_address[1], mac_address[0]);
	radio->dev.release = dnt900_release_radio;
	radio->dev.parent = &local->dev;
	radio->dev.devt = MKDEV(0, 0);
	radio->dev.class = dnt900_class;
	UNWIND(err, dnt900_radio_get_params(radio), fail_params);
	UNWIND(err, kobject_set_name(&radio->dev.kobj, radio->name), fail_name);
	UNWIND(err, device_register(&radio->dev), fail_register);
	get_device(&local->dev);
	return radio;

fail_register:
fail_name:
fail_params:
	kfree(radio);
fail_alloc:
	return ERR_PTR(err);
}

static void dnt900_release_radio(struct device *dev)
{
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);

	put_device(&local->dev);
	kfree(radio);
}

static int dnt900_add_radio(struct dnt900_local *local, const unsigned char *mac_address)
{
	int err;
	struct dnt900_radio *radio;

	TRY(mutex_lock_interruptible(&local->radios_lock));
	UNWIND(err, dnt900_radio_exists(local, mac_address) ? -EEXIST : 0, fail_exists);
	radio = dnt900_create_radio(local, mac_address);
	UNWIND(err, IS_ERR(radio) ? PTR_ERR(radio) : 0, fail_create);
	tty_port_init(&radio->port);
	radio->port.ops = &dnt900_tty_port_ops;
	UNWIND(err, dnt900_radio_add_attributes(radio), fail_attributes);
	if (radio->is_local)
		UNWIND(err, sysfs_create_link(&local->dev.kobj, &radio->dev.kobj, LOCAL_SYMLINK_NAME), fail_symlink);
	else {
		struct device *tty_dev;
		for (radio->tty_index = 0; radio->tty_index < radios && dnt900_tty_indices[radio->tty_index]; ++radio->tty_index)
			;
		UNWIND(err, radio->tty_index < radios ? 0 : -EMLINK, fail_index);
		dnt900_tty_indices[radio->tty_index] = true;
		tty_dev = tty_register_device(dnt900_tty_driver, radio->tty_index, &radio->dev);
		UNWIND(err, IS_ERR(tty_dev) ? PTR_ERR(tty_dev) : 0, fail_tty);
	}
	INIT_KFIFO(radio->fifo);
	dnt900_schedule_work(local, mac_address, dnt900_map_remotes);
	pr_info(LDISC_NAME ": added new radio %s\n", radio->name);
	goto success;

fail_tty:
fail_index:
fail_symlink:
fail_attributes:
	device_unregister(&radio->dev);
fail_create:
	pr_warn(LDISC_NAME ": unable to add radio with MAC address 0x%02X%02X%02X (error %d)\n", mac_address[2], mac_address[1], mac_address[0], -err);
fail_exists:
success:
	mutex_unlock(&local->radios_lock);
	return err;
}

static int dnt900_unregister_radio(struct device *dev, void *unused)
{
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);

	TRY(mutex_lock_interruptible(&local->radios_lock));
	if (radio->is_local)
		sysfs_remove_link(&radio->dev.kobj, LOCAL_SYMLINK_NAME);
	else {
		tty_unregister_device(dnt900_tty_driver, radio->tty_index);
		dnt900_tty_indices[radio->tty_index] = false;
	}
	device_unregister(dev);
	mutex_unlock(&local->radios_lock);
	return 0;
}

static int dnt900_radio_matches_sys_address(struct device *dev, void *data)
{
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);
	const unsigned char *sys_address = data;
	struct dnt900_radio_params params;

	if (radio->is_local)
		return 0;
	dnt900_radio_read_params(radio, &params);
	return EQUAL_ADDRESSES(params.sys_address, sys_address);
}

static int dnt900_radio_matches_mac_address(struct device *dev, void *data)
{
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);
	const unsigned char *mac_address = data;

	return EQUAL_ADDRESSES(radio->mac_address, mac_address);
}

static int dnt900_radio_is_local(struct device *dev, void *data)
{
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);

	return radio->is_local;
}

static int dnt900_radio_matches_net_id(struct device *dev, void *data)
{
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);
	struct dnt900_radio_params params;
	unsigned char *net_id = data;

	dnt900_radio_read_params(radio, &params);
	return *net_id <= MAX_NETWORK_ID && params.curr_nwk_id == *net_id;
}

static int dnt900_radio_routes_net_id(struct device *dev, void *data)
{
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);
	struct dnt900_radio_params params;
	unsigned char *net_id = data;

	dnt900_radio_read_params(radio, &params);
	return *net_id <= MAX_NETWORK_ID && params.device_mode == DEVICE_MODE_ROUTER && params.base_mode_net_id == *net_id;
}

static int dnt900_radio_in_subnet(struct dnt900_radio *radio, void *data)
{
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	struct dnt900_radio_params radio_params;

	if (dnt900_radio_matches_net_id(&radio->dev, data))
		return true;
	dnt900_radio_read_params(radio, &radio_params);
	if (radio_params.curr_nwk_id == 0x00)
		return false;
	return dnt900_dispatch_to_radio(local, &radio_params.curr_nwk_id, dnt900_radio_routes_net_id, data, dnt900_radio_in_subnet) > 0;
}

static bool dnt900_radio_exists(struct dnt900_local *local, const unsigned char *mac_address)
{
	return device_for_each_child(&local->dev, (void *)mac_address, dnt900_radio_matches_mac_address);
}

static int dnt900_dispatch_to_radio(struct dnt900_local *local, void *finder_data, int (*finder)(struct device *, void *), void *action_data, int (*action)(struct dnt900_radio *, void *))
{
	int err = -ENODEV;
	struct device *dev = device_find_child(&local->dev, finder_data, finder);

	if (dev) {
		struct dnt900_radio *radio = DEV_TO_RADIO(dev);
		err = action(radio, action_data);
		put_device(dev);
	}
	return err;
}

static int dnt900_dispatch_to_radio_no_data(struct dnt900_local *local, void *finder_data, int (*finder)(struct device *, void *), int (*action)(struct dnt900_radio *))
{
	int err = -ENODEV;
	struct device *dev = device_find_child(&local->dev, finder_data, finder);

	if (dev) {
		struct dnt900_radio *radio = DEV_TO_RADIO(dev);
		err = action(radio);
		put_device(dev);
	}
	return err;
}

static int dnt900_apply_to_radio(struct device *dev, void *data)
{
	int (*action)(struct dnt900_radio *) = data;
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);

	action(radio);
	return 0;
}

static void dnt900_for_each_radio(struct dnt900_local *local, int (*action)(struct dnt900_radio *))
{
	device_for_each_child(&local->dev, action, dnt900_apply_to_radio);
}

static int dnt900_apply_to_matching(struct device *dev, void *data)
{
	struct dnt900_matcher *matcher = data;
	struct dnt900_radio *radio = DEV_TO_RADIO(dev);

	if (matcher->match(dev, matcher->data))
		matcher->action(radio);
	return 0;
}

static void dnt900_for_matching_radios(struct dnt900_radio *radio, void *data, int (*match)(struct device *, void *), int (*action)(struct dnt900_radio *))
{
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	struct dnt900_matcher matcher = { .data = data, .match = match, .action = action };

	device_for_each_child(&local->dev, &matcher, dnt900_apply_to_matching);
}

static int dnt900_send_packet(struct dnt900_local *local, const unsigned char *packet)
{
	unsigned int length = 2 + packet[1];
	unsigned long flags;
	bool done;

	for (done = false; !done; ) {
		TRY(wait_event_interruptible(local->tx_queue, kfifo_avail(&local->tx_fifo) >= length));
		spin_lock_irqsave(&local->tx_fifo_lock, flags);
		done = kfifo_avail(&local->tx_fifo) >= length;
		if (done)
			length = kfifo_in(&local->tx_fifo, packet, length);
		spin_unlock_irqrestore(&local->tx_fifo_lock, flags);
	}
	dnt900_local_drain_fifo(local);
	return 0;
}

static int dnt900_send_packet_get_result(struct dnt900_local *local, const unsigned char *packet, unsigned char *result)
{
	struct dnt900_transaction transaction = { .result = result, .err = 0, .packet = packet };
	int err;
	long completed;

	INIT_LIST_HEAD(&transaction.list);
	init_completion(&transaction.completed);

	TRY(mutex_lock_interruptible(&local->transactions_lock));
	list_add_tail(&transaction.list, &local->transactions);
	mutex_unlock(&local->transactions_lock);

	UNWIND(err, dnt900_send_packet(local, packet), exit);
	completed = wait_for_completion_interruptible_timeout(&transaction.completed, msecs_to_jiffies(REGISTER_TIMEOUT_MS));
	err = !completed ? -ETIMEDOUT : completed < 0 ? completed : transaction.err;

exit:
	mutex_lock(&local->transactions_lock);
	list_del(&transaction.list);
	mutex_unlock(&local->transactions_lock);
	return err;
}

static int dnt900_clear_packets(struct dnt900_local *local)
{
	struct dnt900_transaction *transaction;

	TRY(mutex_lock_interruptible(&local->transactions_lock));
	list_for_each_entry(transaction, &local->transactions, list) {
		if (completion_done(&transaction->completed))
			continue;
		transaction->err = -EAGAIN;
		complete(&transaction->completed);
	}
	mutex_unlock(&local->transactions_lock);
	return 0;
}

static int dnt900_radio_wake_tty(struct dnt900_radio *radio)
{
	struct tty_struct *tty = tty_port_tty_get(&radio->port);

	if (tty) {
		tty_wakeup(tty);
		tty_kref_put(tty);
	}
	return 0;
}

static int dnt900_radio_hangup_tty(struct dnt900_radio *radio)
{
	struct tty_struct *tty = tty_port_tty_get(&radio->port);

	if (tty) {
		pr_info(LDISC_NAME ": hanging up radio %s at %s\n", radio->name, tty->name);
		tty_hangup(tty);
		tty_kref_put(tty);
	}
	return 0;
}

static int dnt900_radio_hangup_ttys(struct dnt900_radio *radio)
{
	struct dnt900_radio_params params;

	dnt900_radio_read_params(radio, &params);
	dnt900_radio_hangup_tty(radio);
	if (params.device_mode == DEVICE_MODE_ROUTER && params.base_mode_net_id > 0x00)
		dnt900_for_matching_radios(radio, &params.base_mode_net_id, dnt900_radio_matches_net_id, dnt900_radio_hangup_ttys);
	return 0;
}

static int dnt900_radio_drain_fifo(struct dnt900_radio *radio)
{
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	struct dnt900_local_params local_params;
	struct dnt900_radio_params radio_params;
	unsigned char packet[MAX_PACKET_SIZE] = { START_OF_PACKET, 0, COMMAND_TX_DATA, 0xFF, 0xFF, 0xFF };
	unsigned long flags;
	bool tx_available;

	if (kfifo_is_empty(&radio->fifo))
		return 0;
	dnt900_local_read_params(local, &local_params);
	if (local_params.slot_size <= 6)
		return 0;
	dnt900_radio_read_params(radio, &radio_params);
	if (!radio->is_local)
		COPY3(packet + 3, radio_params.sys_address);
	do {
		unsigned int length = min((unsigned int)local_params.slot_size, kfifo_len(&radio->fifo) + 6);
		spin_lock_irqsave(&local->tx_fifo_lock, flags);
		tx_available = kfifo_avail(&local->tx_fifo) >= length;
		if (tx_available) {
			packet[1] = 4 + kfifo_out(&radio->fifo, packet + 6, length - 6);
			length = kfifo_in(&local->tx_fifo, packet, length);
		}
		spin_unlock_irqrestore(&local->tx_fifo_lock, flags);
	} while (tx_available && !kfifo_is_empty(&radio->fifo));
	return 0;
}

static void dnt900_local_drain_fifo(struct dnt900_local *local)
{
	unsigned long flags;
	unsigned char *buf;

	dnt900_for_each_radio(local, dnt900_radio_drain_fifo);
	if (!spin_trylock_irqsave(&local->tx_fifo_lock, flags))
		return;
	while (true) {
		int room = tty_write_room(local->tty);
		int avail = kfifo_out_prepare(&local->tx_fifo, &buf);
		int len = min(room, avail);

		if (len <= 0)
			break;
		set_bit(TTY_DO_WRITE_WAKEUP, &local->tty->flags);
		local->tty->ops->write(local->tty, buf, len);
		kfifo_out_finish(&local->tx_fifo, len);
	}
	spin_unlock_irqrestore(&local->tx_fifo_lock, flags);

	wake_up_interruptible(&local->tx_queue);
	dnt900_for_each_radio(local, dnt900_radio_wake_tty);
}

static int dnt900_tty_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct dnt900_radio *radio = PORT_TO_RADIO(port);

	get_device(&radio->dev);
	return 0;
}

static void dnt900_tty_port_shutdown(struct tty_port *port)
{
	struct dnt900_radio *radio = PORT_TO_RADIO(port);

	put_device(&radio->dev);
}

static int dnt900_radio_write(struct dnt900_radio *radio, void *data)
{
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	struct dnt900_bufdata *bufdata = data;
	int count = kfifo_in(&radio->fifo, bufdata->buf, bufdata->len);

	dnt900_local_drain_fifo(local);
	return count;
}

static int dnt900_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct dnt900_radio *radio = TTY_TO_RADIO(tty);

	return tty_port_open(&radio->port, tty, filp);
}

static void dnt900_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct dnt900_radio *radio = TTY_TO_RADIO(tty);

	tty_port_close(&radio->port, tty, filp);
}

static void dnt900_tty_hangup(struct tty_struct *tty)
{
	struct dnt900_radio *radio = TTY_TO_RADIO(tty);

	tty_port_hangup(&radio->port);
}

static int dnt900_tty_write(struct tty_struct *tty, const unsigned char *buf, int len)
{
	struct dnt900_radio *radio = TTY_TO_RADIO(tty);
	struct dnt900_bufdata bufdata = { .buf = buf, .len = len };

	return dnt900_radio_write(radio, &bufdata);
}

static int dnt900_tty_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct dnt900_radio *radio = TTY_TO_RADIO(tty);

	return kfifo_put(&radio->fifo, &ch) ? 1 : 0;
}

static int dnt900_tty_write_room(struct tty_struct *tty)
{
	struct dnt900_radio *radio = TTY_TO_RADIO(tty);

	return kfifo_avail(&radio->fifo);
}

static int dnt900_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct dnt900_radio *radio = TTY_TO_RADIO(tty);

	return kfifo_len(&radio->fifo);
}

static void dnt900_tty_flush_chars(struct tty_struct *tty)
{
	struct dnt900_radio *radio = TTY_TO_RADIO(tty);
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);

	dnt900_local_drain_fifo(local);
}

static void dnt900_tty_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct dnt900_radio *radio = TTY_TO_RADIO(tty);
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);

	while (true) {
		if (kfifo_is_empty(&radio->fifo))
			break;
		// TODO: possible bug - should the next line wait for a different condition?
		if (wait_event_interruptible_timeout(local->tx_queue, kfifo_is_empty(&local->tx_fifo), timeout) <= 0)
			break;
		dnt900_local_drain_fifo(local);
	}
}

static void dnt900_tty_flush_buffer(struct tty_struct *tty)
{
	struct dnt900_radio *radio = TTY_TO_RADIO(tty);
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	unsigned long flags;

	spin_lock_irqsave(&local->tx_fifo_lock, flags);
	kfifo_reset_out(&radio->fifo);
	spin_unlock_irqrestore(&local->tx_fifo_lock, flags);
	dnt900_local_drain_fifo(local);
}

static ssize_t dnt900_ldisc_write(struct tty_struct *tty, struct file *filp, const unsigned char *buf, size_t len)
{
	struct dnt900_local *local = TTY_TO_LOCAL(tty);
	struct dnt900_bufdata bufdata = { .buf = buf, .len = len };

	while (true) {
		int sent;
		TRY(wait_event_interruptible(local->tx_queue, true));
		sent = dnt900_dispatch_to_radio(local, NULL, dnt900_radio_is_local, &bufdata, dnt900_radio_write);
		if (sent || !len)
			return sent;
	};
}

static void dnt900_ldisc_write_wakeup(struct tty_struct *tty)
{
	struct dnt900_local *local = TTY_TO_LOCAL(tty);

	dnt900_local_drain_fifo(local);
}

static int dnt900_ldisc_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct dnt900_local *local = TTY_TO_LOCAL(tty);

	switch (cmd) {
	case TIOCOUTQ:
		return put_user(kfifo_len(&local->tx_fifo), (int __user *)arg);
	case TIOCINQ:
		return put_user(kfifo_len(&local->rx_fifo), (int __user *)arg);
	case TCFLSH:
		switch (arg) {
		case TCIOFLUSH:
		case TCIFLUSH:
			dnt900_ldisc_flush_buffer(tty);
		case TCOFLUSH:
			return 0;
		}
		return -EINVAL;
	default:
		return tty_mode_ioctl(tty, file, cmd, arg);
	}
}

static void dnt900_ldisc_flush_buffer(struct tty_struct *tty)
{
	struct dnt900_local *local = TTY_TO_LOCAL(tty);
	unsigned long flags;

	spin_lock_irqsave(&local->rx_fifo_lock, flags);
	kfifo_reset_out(&local->rx_fifo);
	spin_unlock_irqrestore(&local->rx_fifo_lock, flags);
}

static void dnt900_ldisc_receive_buf(struct tty_struct *tty, const unsigned char *cp, char *fp, int count)
{
	struct dnt900_local *local = TTY_TO_LOCAL(tty);
	unsigned char response[MAX_PACKET_SIZE];
	unsigned int length;
	unsigned long flags;

	while (count > 0) {
		for (; count > 0; --count, ++cp, ++fp)
			if (*fp == TTY_NORMAL && !kfifo_put(&local->rx_fifo, cp))
				break;
		spin_lock_irqsave(&local->rx_fifo_lock, flags);
		while (kfifo_out_peek(&local->rx_fifo, response, 2) == 2) {
			if (response[0] != START_OF_PACKET) {
				kfifo_skip(&local->rx_fifo);
				continue;
			}
			length = 2 + response[1];
			if (kfifo_len(&local->rx_fifo) < length)
				break;
			length = kfifo_out(&local->rx_fifo, response, length);
			if (response[1] == 0)
				continue;
			spin_unlock_irqrestore(&local->rx_fifo_lock, flags);
			if (response[2] & TYPE_REPLY)
				dnt900_process_reply(local, response);
			if (response[2] & TYPE_EVENT)
				dnt900_process_event(local, response);
			spin_lock_irqsave(&local->rx_fifo_lock, flags);
		}
		spin_unlock_irqrestore(&local->rx_fifo_lock, flags);
	}
}

static int dnt900_process_reply(struct dnt900_local *local, unsigned char *response)
{
	const unsigned char command = response[2] & MASK_COMMAND;
	struct dnt900_transaction *transaction;

	TRY(mutex_lock_interruptible(&local->transactions_lock));
	list_for_each_entry(transaction, &local->transactions, list) {
		unsigned char tx_status = STATUS_ACKNOWLEDGED;
		int n;

		if (transaction->packet[2] != command)
			continue;
		if (completion_done(&transaction->completed))
			continue;
		switch (command) {
		case COMMAND_GET_REGISTER:
			// match Reg, Bank, span:
			if (transaction->packet[3] != response[3] 
			 || transaction->packet[4] != response[4] 
			 || transaction->packet[5] != response[5])
				continue;
			for (n = 0; n < transaction->packet[5]; ++n)
				transaction->result[n] = response[6 + n];
			break;
		case COMMAND_SET_REGISTER:
			break;
		case COMMAND_DISCOVER:
			// match MacAddr:
			if (transaction->packet[3] != response[4] 
			 || transaction->packet[4] != response[5] 
			 || transaction->packet[5] != response[6])
				continue;
			tx_status = response[3];
			if (tx_status == STATUS_ACKNOWLEDGED)
				for (n = 0; n < 3; ++n)
					transaction->result[n] = response[7 + n];
			break;
		case COMMAND_GET_REMOTE_REGISTER:
			// match Addr:
			if (transaction->packet[3] != response[4] 
			 || transaction->packet[4] != response[5] 
			 || transaction->packet[5] != response[6])
				continue;
			tx_status = response[3];
			if (tx_status == STATUS_ACKNOWLEDGED) {
				// match Reg, Bank, span:
				if (transaction->packet[6] != response[8] 
				 || transaction->packet[7] != response[9] 
				 || transaction->packet[8] != response[10])
					continue;
				for (n = 0; n < transaction->packet[8]; ++n)
					transaction->result[n] = response[11 + n];
			}
			break;
		case COMMAND_SET_REMOTE_REGISTER:
			// match Addr:
			if (transaction->packet[3] != response[4] 
			 || transaction->packet[4] != response[5] 
			 || transaction->packet[5] != response[6])
				continue;
			tx_status = response[3];
			break;
		case COMMAND_SOFTWARE_RESET:
			break;
		case COMMAND_ENTER_PROTOCOL_MODE:
			break;
		}
		switch (tx_status) {
		case STATUS_ACKNOWLEDGED:
			transaction->err = 0;
			break;
		case STATUS_NOT_ACKNOWLEDGED:
		case STATUS_HOLDING_FOR_FLOW:
			transaction->err = -EAGAIN;
			break;
		case STATUS_NOT_LINKED:
		default:
			transaction->err = -ECOMM;
		}
		complete(&transaction->completed);
		break;
	}
	mutex_unlock(&local->transactions_lock);

	switch (command) {
	case COMMAND_GET_REMOTE_REGISTER:
	case COMMAND_SET_REMOTE_REGISTER:
	case COMMAND_TX_DATA:
		if (response[3] == STATUS_ACKNOWLEDGED)
			dnt900_dispatch_to_radio(local, response + 4, dnt900_radio_matches_sys_address, response + 7, dnt900_radio_report_rssi);
	}

	return 0;
}

static int dnt900_process_event(struct dnt900_local *local, unsigned char *response)
{
	int err = 0;
	const unsigned char event = response[2];

	switch (event) {
	case EVENT_RX_DATA:
		err = dnt900_dispatch_to_radio(local, response + 3, dnt900_radio_matches_sys_address, response, dnt900_radio_out);
		if (err == -ENODEV)
			dnt900_schedule_work(local, response + 3, dnt900_add_new_sys_address);
		break;
	case EVENT_ANNOUNCE:
		err = dnt900_process_announcement(local, response + 3);
		break;
	case EVENT_RX_EVENT:
		err = dnt900_dispatch_to_radio(local, response + 3, dnt900_radio_matches_sys_address, response, dnt900_process_rx_event);
		if (err == -ENODEV)
			dnt900_schedule_work(local, response + 3, dnt900_add_new_sys_address);
		break;
	case EVENT_JOIN_REQUEST:
		err = dnt900_process_join_request(local, response + 3);
		break;
	}
	return err;
}

static int dnt900_process_rx_event(struct dnt900_radio *radio, void *data)
{
	unsigned char *response = data;
	unsigned char *report = response + 10;
	int bytes[10] = { 1, 1, 1, 1, 1, 1, 2, 2, 2, 2 };
	unsigned long flags;
	int n;

	spin_lock_irqsave(&radio->attributes_lock, flags);
	for (n = 0; n < 10; report += bytes[n], ++n)
		dnt900_print_bytes(bytes[n], report, radio->attributes[report_GPIO0 + n], ARRAY_SIZE(*radio->attributes));
	PRINT_RSSI(radio->attributes[rssi], response[6]);
	spin_unlock_irqrestore(&radio->attributes_lock, flags);
	for (n = 0; n < 10; ++n)
		sysfs_notify(&radio->dev.kobj, NULL, dnt900_radio_attributes[report_GPIO0 + n].attr.name);
	sysfs_notify(&radio->dev.kobj, NULL, dnt900_radio_attributes[rssi].attr.name);
	return 0;
}

static int dnt900_process_announcement(struct dnt900_local *local, unsigned char *annc)
{
	unsigned long updates = 0;
	unsigned long flags;
	int n, err = 0;

	switch (annc[0]) {
	case ANNOUNCEMENT_STARTED:
		dnt900_clear_packets(local);
		dnt900_for_each_radio(local, dnt900_radio_hangup_tty);
		break;
	case ANNOUNCEMENT_EXITED:
		dnt900_for_each_radio(local, dnt900_radio_hangup_tty);
	case ANNOUNCEMENT_JOINED:
		dnt900_schedule_work(local, NULL, dnt900_refresh_local);
		break;
	case ERROR_PROTOCOL_ARGUMENT:
		dnt900_process_argument_error(local);
		break;
	}

	spin_lock_irqsave(&local->attributes_lock, flags);
	switch (annc[0]) {
	case ANNOUNCEMENT_STARTED:
		dnt900_print_hex(1, annc, local->attributes[announce], ARRAY_SIZE(*local->attributes));
		updates = BIT(announce);
		break;
	case ANNOUNCEMENT_JOINED:
		dnt900_print_hex(6, annc, local->attributes[announce], ARRAY_SIZE(*local->attributes));
		dnt900_print_bytes(3, annc + 2, local->attributes[parent], ARRAY_SIZE(*local->attributes));
		updates = BIT(announce) | BIT(parent);
		break;
	case ANNOUNCEMENT_EXITED:
		dnt900_print_hex(2, annc, local->attributes[announce], ARRAY_SIZE(*local->attributes));
		local->attributes[parent][0] = 0;
		updates = BIT(announce) | BIT(parent);
		break;
	case ANNOUNCEMENT_REMOTE_JOINED:
		dnt900_print_hex(6, annc, local->attributes[announce], ARRAY_SIZE(*local->attributes));
		updates = BIT(announce);
		break;
	case ANNOUNCEMENT_REMOTE_EXITED:
		dnt900_print_hex(4, annc, local->attributes[announce], ARRAY_SIZE(*local->attributes));
		updates = BIT(announce);
		break;
	case ANNOUNCEMENT_BASE_REBOOTED:
		dnt900_print_hex(1, annc, local->attributes[announce], ARRAY_SIZE(*local->attributes));
		updates = BIT(announce);
		break;
	case ANNOUNCEMENT_HEARTBEAT:
		dnt900_print_hex(11, annc, local->attributes[announce], ARRAY_SIZE(*local->attributes));
		updates = BIT(announce);
		break;
	case ANNOUNCEMENT_HEARTBEAT_TIMEOUT:
		dnt900_print_hex(2, annc, local->attributes[announce], ARRAY_SIZE(*local->attributes));
		updates = BIT(announce);
		break;
	case ERROR_UART_OVERFLOW:
	case ERROR_UART_OVERRUN:
	case ERROR_UART_FRAMING:
	case ERROR_HARDWARE:
		dnt900_print_hex(1, annc, local->attributes[announce], ARRAY_SIZE(*local->attributes));
		dnt900_print_hex(1, annc, local->attributes[error], ARRAY_SIZE(*local->attributes));
		updates = BIT(announce) | BIT(error);
		break;
	}
	spin_unlock_irqrestore(&local->attributes_lock, flags);

	for (n = 0; n < ARRAY_SIZE(dnt900_local_attributes); ++n)
		if (test_bit(n, &updates))
			sysfs_notify(&local->dev.kobj, NULL, dnt900_local_attributes[n].attr.name);

	switch (annc[0]) {
	case ANNOUNCEMENT_JOINED:
		err = dnt900_dispatch_to_radio(local, annc + 2, dnt900_radio_matches_mac_address, annc, dnt900_radio_process_announcement);
		break;
	case ANNOUNCEMENT_REMOTE_JOINED:
	case ANNOUNCEMENT_HEARTBEAT:
		err = dnt900_dispatch_to_radio(local, annc + 1, dnt900_radio_matches_mac_address, annc, dnt900_radio_process_announcement);
		break;
	case ANNOUNCEMENT_REMOTE_EXITED:
		dnt900_dispatch_to_radio(local, annc + 1, dnt900_radio_matches_mac_address, annc, dnt900_radio_process_announcement);
		break;
	case ANNOUNCEMENT_HEARTBEAT_TIMEOUT:
		dnt900_dispatch_to_radio(local, annc + 1, dnt900_radio_routes_net_id, annc, dnt900_radio_process_announcement);
		break;
	}
	if (err == -ENODEV)
		dnt900_schedule_work(local, annc + 1, dnt900_add_new_mac_address);
	return 0;
}

static int dnt900_radio_process_announcement(struct dnt900_radio *radio, void *data)
{
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	unsigned char *annc = data;
	unsigned long updates = 0;
	unsigned long flags;
	int n;

	switch (annc[0]) {
	case ANNOUNCEMENT_REMOTE_JOINED:
	case ANNOUNCEMENT_JOINED:
		dnt900_schedule_work(local, radio->mac_address, dnt900_refresh_radio);
		break;
	case ANNOUNCEMENT_REMOTE_EXITED:
	case ANNOUNCEMENT_HEARTBEAT_TIMEOUT:
		dnt900_radio_hangup_ttys(radio);
		break;
	case ANNOUNCEMENT_HEARTBEAT:
		dnt900_radio_check_heartbeat(radio, annc);
		break;
	}

	spin_lock_irqsave(&radio->attributes_lock, flags);
	switch (annc[0]) {
	case ANNOUNCEMENT_HEARTBEAT:
		PRINT_RANGE(radio->attributes[range], annc[10]);
		PRINT_SUCCESS(radio->attributes[success_rate], annc[8]);
		PRINT_RSSI(radio->attributes[beacon_rssi], annc[7]);
		PRINT_RSSI(radio->attributes[parent_rssi], annc[9]);
		updates = BIT(range) | BIT(success_rate) | BIT(beacon_rssi) | BIT(parent_rssi);
		break;
	case ANNOUNCEMENT_JOINED:
	case ANNOUNCEMENT_REMOTE_JOINED:
		PRINT_RANGE(radio->attributes[range], annc[5]);
		updates = BIT(range);
		break;
	}
	spin_unlock_irqrestore(&radio->attributes_lock, flags);

	for (n = 0; n < ARRAY_SIZE(dnt900_radio_attributes); ++n)
		if (test_bit(n, &updates))
			sysfs_notify(&radio->dev.kobj, NULL, dnt900_radio_attributes[n].attr.name);
	return 0;
}

static int dnt900_process_argument_error(struct dnt900_local *local)
{
	struct dnt900_transaction *transaction;

	TRY(mutex_lock_interruptible(&local->transactions_lock));
	list_for_each_entry(transaction, &local->transactions, list) {
		if (completion_done(&transaction->completed))
			continue;
		if (transaction->packet[2] == COMMAND_SET_REGISTER || transaction->packet[2] == COMMAND_SET_REMOTE_REGISTER) {
			transaction->err = -EINVAL;
			complete(&transaction->completed);
			break;
		}
	}
	mutex_unlock(&local->transactions_lock);
	return 0;
}

static int dnt900_process_join_request(struct dnt900_local *local, unsigned char *request)
{
	unsigned long flags;

	spin_lock_irqsave(&local->attributes_lock, flags);
	dnt900_print_bytes(3, request, local->attributes[join_request], ARRAY_SIZE(*local->attributes));
	spin_unlock_irqrestore(&local->attributes_lock, flags);
	sysfs_notify(&local->dev.kobj, NULL, dnt900_local_attributes[join_request].attr.name);
	return 0;
}

static int dnt900_radio_report_rssi(struct dnt900_radio *radio, void *data)
{
	signed char *value = data;
	unsigned long flags;

	switch (*value) {
	case 0x7E:
	case 0x7F:
		break;
	default:
		spin_lock_irqsave(&radio->attributes_lock, flags);
		PRINT_RSSI(radio->attributes[rssi], *value);
		spin_unlock_irqrestore(&radio->attributes_lock, flags);
		sysfs_notify(&radio->dev.kobj, NULL, dnt900_radio_attributes[rssi].attr.name);
	}
	return 0;
}

static int dnt900_radio_check_heartbeat(struct dnt900_radio *radio, void *data)
{
	struct dnt900_local *local = RADIO_TO_LOCAL(radio);
	const unsigned char *annc = data;
	unsigned long flags;
	bool changed = false;

	spin_lock_irqsave(&radio->param_lock, flags);
	changed = radio->params.sys_address[0] != annc[4] || radio->params.sys_address[1] != annc[5] || radio->params.curr_nwk_id != annc[6];
	if (radio->params.sys_address[0] != annc[4]) {
		radio->params.device_mode = annc[4] == 0x00 ? DEVICE_MODE_ROUTER : DEVICE_MODE_REMOTE;
		if (annc[4] == 0x00)
			radio->params.base_mode_net_id = annc[5];
	}
	radio->params.sys_address[0] = annc[4];
	radio->params.sys_address[1] = annc[5];
	radio->params.curr_nwk_id = annc[6];
	spin_unlock_irqrestore(&radio->param_lock, flags);

	if (changed) {
		// TODO: this should be hangup_ttys, but with the old base_mode_net_id
		dnt900_radio_hangup_tty(radio);
		dnt900_schedule_work(local, radio->mac_address, dnt900_refresh_radio);
	}
	return 0;
}

static int dnt900_radio_out(struct dnt900_radio *radio, void *data)
{
	unsigned char *response = data;
	unsigned int len = response[1] - 5;
	struct tty_struct *tty = tty_port_tty_get(&radio->port);

	if (tty) {
		while (len > 0) {
			len -= tty_insert_flip_string(tty, response + 7, len);
			tty_flip_buffer_push(tty);
		}
		tty_kref_put(tty);
	}
	dnt900_radio_report_rssi(radio, response + 6);
	return 0;
}

static void dnt900_schedule_work(struct dnt900_local *local, const unsigned char *address, void (*work_function)(struct work_struct *))
{
	struct dnt900_work *work;

	if (!down_read_trylock(&local->closed_lock))
		return;
	work = kzalloc(sizeof(*work), GFP_KERNEL);
	work->local = local;
	if (address)
		COPY3(work->address, address);
	INIT_WORK(&work->ws, work_function);
	queue_work(local->workqueue, &work->ws);
	up_read(&local->closed_lock);
}

static void dnt900_add_new_mac_address(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);

	dnt900_add_radio(work->local, work->address);
	kfree(work);
}

static void dnt900_add_new_sys_address(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);
	unsigned char mac_address[3];

	if (!dnt900_get_remote_register(work->local, work->address, REG(MacAddress), mac_address))
		dnt900_add_radio(work->local, mac_address);
	kfree(work);
}

static void dnt900_refresh_radio(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);

	dnt900_dispatch_to_radio_no_data(work->local, work->address, dnt900_radio_matches_mac_address, dnt900_radio_get_params);
	kfree(work);
}

static void dnt900_refresh_local(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);

	dnt900_local_get_params(work->local);
	kfree(work);
}

static void dnt900_init_local(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);
	struct dnt900_local *local = work->local;
	struct dnt900_local_params local_params;
	int err;

	UNWIND(err, dnt900_local_add_attributes(local), fail);
	UNWIND(err, dnt900_enter_protocol_mode(local), fail);
	UNWIND(err, dnt900_get_register(local, REG(MacAddress), local->mac_address), fail);
	UNWIND(err, dnt900_local_get_params(local), fail);
	UNWIND(err, dnt900_add_radio(local, local->mac_address), fail);
	dnt900_local_read_params(local, &local_params);
	if (local_params.device_mode == DEVICE_MODE_BASE || local_params.link_status != LINK_STATUS_READY)
		goto success;
	UNWIND(err, dnt900_add_radio(local, local_params.base_mac_address), success);
	goto success;

fail:
	pr_err(LDISC_NAME ": unable to connect to radio module on %s (error %d)\n", dev_name(&local->dev), -err);
success:
	kfree(work);
}

static void dnt900_map_remotes(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);

	dnt900_dispatch_to_radio_no_data(work->local, work->address, dnt900_radio_matches_mac_address, dnt900_radio_map_remotes);
	kfree(work);
}

static irqreturn_t dnt900_cts_handler(int irq, void *dev_id)
{
	struct dnt900_local *local = dev_id;

	if (local->tty)
		gpio_get_value(local->gpio_cts) ? stop_tty(local->tty) : start_tty(local->tty);
	return IRQ_HANDLED;
}

static struct dnt900_local *dnt900_create_local(struct tty_struct *tty)
{
	int err;
	struct dnt900_local *local = kzalloc(sizeof(*local), GFP_KERNEL);

	UNWIND(err, local ? 0 : -ENOMEM, fail_alloc);
	local->tty = tty_kref_get(tty);
	UNWIND(err, local->tty ? 0 : -EPERM, fail_tty_get);
	mutex_init(&local->transactions_lock);
	mutex_init(&local->radios_lock);
	init_rwsem(&local->closed_lock);
	spin_lock_init(&local->param_lock);
	spin_lock_init(&local->tx_fifo_lock);
	spin_lock_init(&local->rx_fifo_lock);
	spin_lock_init(&local->attributes_lock);
	INIT_LIST_HEAD(&local->transactions);
	local->workqueue = create_singlethread_workqueue(tty->name);
	UNWIND(err, local->workqueue ? 0 : -ENOMEM, fail_workqueue);
	local->gpio_cts = gpio_cts; // how to make this per-instance?
	if (local->gpio_cts >= 0) {
		UNWIND(err, gpio_request_one(local->gpio_cts, GPIOF_IN, "/host_cts"), fail_gpio_cts);
		UNWIND(err, request_irq(gpio_to_irq(local->gpio_cts), dnt900_cts_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, LDISC_NAME, local), fail_irq_cts);
	} else if (!C_CRTSCTS(tty))
		pr_warn(LDISC_NAME ": no hardware flow control enabled for %s; risk of data loss\n", tty->name);
	local->dev.devt = MKDEV(0, 0);
	local->dev.class = dnt900_class;
	local->dev.release = dnt900_release_local;
	UNWIND(err, kobject_set_name(&local->dev.kobj, tty->name), fail_name);
	UNWIND(err, device_register(&local->dev), fail_register);
	INIT_KFIFO(local->tx_fifo);
	INIT_KFIFO(local->rx_fifo);
	init_waitqueue_head(&local->tx_queue);
	return local;

fail_register:
fail_name:
	if (local->gpio_cts >= 0)
		free_irq(gpio_to_irq(local->gpio_cts), local);
fail_irq_cts:
	if (local->gpio_cts >= 0)
		gpio_free(local->gpio_cts);
fail_gpio_cts:
	destroy_workqueue(local->workqueue);
fail_workqueue:
	tty_kref_put(local->tty);
fail_tty_get:
	kfree(local);
fail_alloc:
	return ERR_PTR(err);
}

static void dnt900_unregister_local(struct dnt900_local *local)
{
	device_for_each_child(&local->dev, NULL, dnt900_unregister_radio);
	device_unregister(&local->dev);
}

static void dnt900_release_local(struct device *dev)
{
	struct dnt900_local *local = DEV_TO_LOCAL(dev);

	if (local->gpio_cts >= 0)
		gpio_free(local->gpio_cts);
	tty_kref_put(local->tty);
	kfree(local);
}

static int dnt900_ldisc_open(struct tty_struct *tty)
{
	int err;
	struct dnt900_local *local;

	UNWIND(err, tty->ops->write && tty->ops->write_room ? 0 : -EIO, fail_ops);
	local = dnt900_create_local(tty);
	UNWIND(err, IS_ERR(local) ? PTR_ERR(local) : 0, fail_create_local);
	tty->disc_data = local;
	dnt900_schedule_work(local, NULL, dnt900_init_local);
	pr_info(LDISC_NAME ": attached to %s\n", tty->name);
	return 0;

fail_create_local:
fail_ops:
	return err;
}

static void dnt900_ldisc_close(struct tty_struct *tty)
{
	struct dnt900_local *local = TTY_TO_LOCAL(tty);

	down_write(&local->closed_lock);
	dnt900_for_each_radio(local, dnt900_radio_hangup_tty);
	destroy_workqueue(local->workqueue);
	if (local->gpio_cts >= 0)
		free_irq(gpio_to_irq(local->gpio_cts), local);
	tty->disc_data = NULL;
	dnt900_unregister_local(local);
	pr_info(LDISC_NAME ": detached from %s\n", tty->name);
}

static int dnt900_ldisc_hangup(struct tty_struct *tty)
{
	dnt900_ldisc_close(tty);
	return 0;
}

int __init dnt900_init(void)
{
	int err;

	dnt900_class = class_create(THIS_MODULE, CLASS_NAME);
	UNWIND(err, IS_ERR(dnt900_class) ? PTR_ERR(dnt900_class) : 0, fail_class_create);
	dnt900_tty_indices = kcalloc(radios, sizeof(*dnt900_tty_indices), GFP_KERNEL);
	UNWIND(err, dnt900_tty_indices ? 0 : -ENOMEM, fail_alloc_tty_indices);
	dnt900_tty_driver = alloc_tty_driver(radios);
	UNWIND(err, dnt900_tty_driver ? 0 : -ENOMEM, fail_alloc_tty_driver);
	dnt900_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	dnt900_tty_driver->driver_name = TTY_DRIVER_NAME;
	dnt900_tty_driver->name = TTY_DEV_NAME;
	dnt900_tty_driver->major = 0; // allocate automatically
	dnt900_tty_driver->minor_start = 0;
	dnt900_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	dnt900_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	dnt900_tty_driver->init_termios = dnt900_init_termios;
	tty_set_operations(dnt900_tty_driver, &dnt900_tty_ops);
	UNWIND(err, tty_register_driver(dnt900_tty_driver), fail_register_tty_driver);
	dnt900_ldisc_ops.num = n_dnt900;
	UNWIND(err, tty_register_ldisc(n_dnt900, &dnt900_ldisc_ops), fail_register_ldisc);
	pr_info(LDISC_NAME ": module inserted\n");
	return 0;

fail_register_ldisc:
	tty_unregister_driver(dnt900_tty_driver);
fail_register_tty_driver:
	put_tty_driver(dnt900_tty_driver);
fail_alloc_tty_driver:
	kfree(dnt900_tty_indices);
fail_alloc_tty_indices:
	class_destroy(dnt900_class);
fail_class_create:
	return err;
}

void __exit dnt900_exit(void)
{
	tty_unregister_ldisc(n_dnt900);
	tty_unregister_driver(dnt900_tty_driver);
	put_tty_driver(dnt900_tty_driver);
	kfree(dnt900_tty_indices);
	class_destroy(dnt900_class);
	pr_info(LDISC_NAME ": module removed\n");
}

module_init(dnt900_init);
module_exit(dnt900_exit);

MODULE_AUTHOR("Matthew Hollingworth");
MODULE_DESCRIPTION("driver for DNT900 RF module");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.3");
//MODULE_ALIAS_LDISC(N_DNT900);

// Future work:
// 
// TODO: refresh entire subnet when ANNOUNCEMENT_REMOTE_JOINED?
// TODO: add and remove tty port according to whether link is up?
// TODO: hangup tty when TxStatus = 0x03 received in TxDataReply?
// TODO: `parent` attributes can be updated elsewhere?
// TODO: hangup ttys when UcReset attribute is written?
// 
// TODO: in dnt900_radio_drain_fifo, we could just send a single packet per call to get a
//       better round-robin effect when transmitting data to multiple radios (or we could
//       expose a 'priority' attribute to determine how many packets are sent at once)
// TODO: ParentACKQual not working!
// TODO: Would like to issue ExitProtocolMode command when the line discipline is detached.
//       However we can't do this as the tty is not available for use in dnt900_ldisc_close.
// TODO: hangup ttys on ANNOUNCEMENT_BASE_REBOOTED event?
// TODO: add writeable 'remap' attribute to force remap of entire network
