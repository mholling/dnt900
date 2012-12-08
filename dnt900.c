/*
    Line discipline for RFM DNT900 radio transceiver modules.
    Copyright (C) 2012 Matthew Hollingworth.

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
#include <linux/stat.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
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
#include <linux/workqueue.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/kfifo.h>

#define DRIVER_NAME "dnt900"
#define CLASS_NAME "dnt900"

#define LINE() pr_info("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__)

#define MAX_PACKET_SIZE (2+255)

// buffer sizes must be powers of 2
#define RX_BUF_SIZE     (2048)

#define CIRC_INDEX(index, offset, size) (((index) + (offset)) & ((size) - 1))
#define CIRC_OFFSET(index, offset, size) (index) = (((index) + (offset)) & ((size) - 1))

#define TIMEOUT_MS (5000)

#define START_OF_PACKET (0xFB)

#define COMMAND_ENTER_PROTOCOL_MODE (0x00)
#define COMMAND_EXIT_PROTOCOL_MODE  (0x00)
#define COMMAND_SOFTWARE_RESET      (0x02)
#define COMMAND_GET_REGISTER        (0x03)
#define COMMAND_SET_REGISTER        (0x04)
#define COMMAND_TX_DATA             (0x05)
#define COMMAND_DISCOVER            (0x06)
#define COMMAND_GET_REMOTE_REGISTER (0x0A)
#define COMMAND_SET_REMOTE_REGISTER (0x0B)

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

#define EMPTY_PACKET_BYTES (50)

#define ATTR_R  (S_IRUGO)
#define ATTR_W  (S_IWUSR | S_IWGRP)
#define ATTR_RW (S_IRUGO | S_IWUSR | S_IWGRP)

#define EQUAL_ADDRESSES(address1, address2) ( \
	address1[0] == address2[0] && \
	address1[1] == address2[1] && \
	address1[2] == address2[2])

#define RANGE_TO_KMS(range) (4667 * (unsigned char)(range) / 10000)
#define PACKET_SUCCESS_RATE(attempts_x4) (400 / (unsigned char)(attempts_x4))

#define TRY(function_call) do { \
	int error = (function_call); \
	if (error) \
		return error; \
} while (0)
	
#define UNWIND(error, function_call, exit) do { \
	error = (function_call); \
	if (error) \
		goto exit; \
} while (0)

#define COPY3(dest, src) do { \
	dest[0] = src[0]; \
	dest[1] = src[1]; \
	dest[2] = src[2]; \
} while (0)

struct dnt900_message_header {
	char start_of_packet;
	unsigned char length;
	char type;
};

struct dnt900_software_reset_message {
	struct dnt900_message_header header;
	char boot_select;
};

struct dnt900_get_register_message {
	struct dnt900_message_header header;
	char reg;
	char bank;
	char span;
};

struct dnt900_set_register_message {
	struct dnt900_message_header header;
	char reg;
	char bank;
	char span;
	char value[32];
};

struct dnt900_tx_data_message {
	struct dnt900_message_header header;
	char sys_address[3];
	char data[MAX_PACKET_SIZE-6];
};

struct dnt900_discover_message {
	struct dnt900_message_header header;
	char mac_address[3];
};

struct dnt900_get_remote_register_message {
	struct dnt900_message_header header;
	char sys_address[3];
	char reg;
	char bank;
	char span;
};

struct dnt900_set_remote_register_message {
	struct dnt900_message_header header;
	char sys_address[3];
	char reg;
	char bank;
	char span;
	char value[32];
};

struct dnt900_packet {
	struct list_head list;
	struct completion completed;
	char *result;
	int error;
	char *message;
	unsigned int length;
};

struct dnt900_ldisc {
	struct tty_struct *tty;
	int major;
	int minor;
	unsigned gpio_cfg;
	unsigned gpio_cts;
	struct list_head packets;
	struct mutex packets_lock;
	struct mutex devices_lock;
	struct mutex param_lock;
	struct mutex tty_lock;
	struct rw_semaphore shutdown_lock;
	char message[MAX_PACKET_SIZE];
	unsigned int end;
	struct workqueue_struct *workqueue;
	bool use_tree_routing;
	unsigned int slot_size;
};

struct dnt900_device {
	struct cdev cdev;
	struct dnt900_ldisc *ldisc;
	bool is_local;
	struct mutex param_lock;
	struct mutex read_lock;
	struct mutex write_lock;
	wait_queue_head_t queue;
	struct kfifo fifo;
	char buf[RX_BUF_SIZE];
	unsigned int head;
	unsigned int tail;
	char mac_address[3];
	char sys_address[3];
};

struct dnt900_rxdata {
	const char *buf;
	unsigned int len;
};

struct dnt900_register {
	char bank;
	char offset;
	char span;
};

struct dnt900_work {
	struct work_struct ws;
	char mac_address[3];
	struct dnt900_ldisc *ldisc;
};

struct dnt900_attribute {
	struct device_attribute attr;
	struct dnt900_register reg;
	int (*print)(const char *value, char *buf);
	int (*parse)(const char *buf, size_t count, char *value);
};

#define DNT900_ATTR(_name, _mode, _bank, _offset, _span, _print, _parse) { \
	.attr = { \
		.attr = { \
			.name = _name, \
			.mode = _mode \
		}, \
		.show = dnt900_show_attr, \
		.store = dnt900_store_attr \
	}, \
	.reg = { \
		.bank = _bank, \
		.offset = _offset, \
		.span = _span \
	}, \
	.print = _print, \
	.parse = _parse \
}

static int print_bytes(int bytes, const char *value, char *buf);
static int print_1_bytes(const char *value, char *buf);
static int print_2_bytes(const char *value, char *buf);
static int print_3_bytes(const char *value, char *buf);
static int print_4_bytes(const char *value, char *buf);
static int print_hex(int bytes, const char *value, char *buf);
static int print_32_hex(const char *value, char *buf);
static int print_8_ascii(const char *value, char *buf);
static int print_16_ascii(const char *value, char *buf);
static int print_5_macs(const char *value, char *buf);

static int parse_bytes(int bytes, const char *buf, size_t count, char *value);
static int parse_1_bytes(const char *buf, size_t count, char *value);
static int parse_2_bytes(const char *buf, size_t count, char *value);
static int parse_3_bytes(const char *buf, size_t count, char *value);
static int parse_4_bytes(const char *buf, size_t count, char *value);
static int parse_hex(int bytes, const char *buf, size_t count, char *value);
static int parse_16_hex(const char *buf, size_t count, char *value);
static int parse_32_hex(const char *buf, size_t count, char *value);
static int parse_16_ascii(const char *buf, size_t count, char *value);

static int dnt900_add_attributes(struct device *dev);
static void dnt900_remove_attributes(struct device *dev);

static int dnt900_enter_protocol_mode(struct dnt900_ldisc *ldisc);
static int dnt900_exit_protocol_mode(struct dnt900_ldisc *ldisc);
static int dnt900_get_register(struct dnt900_ldisc *ldisc, const struct dnt900_register *reg, char *value);
static int dnt900_get_remote_register(struct dnt900_ldisc *ldisc, const char *sys_address, const struct dnt900_register *reg, char *value);
static int dnt900_set_register(struct dnt900_ldisc *ldisc, const struct dnt900_register *reg, const char *value);
static int dnt900_set_remote_register(struct dnt900_ldisc *ldisc, const char *sys_address, const struct dnt900_register *reg, const char *value);
static int dnt900_discover(struct dnt900_ldisc *ldisc, const char *mac_address, char *sys_address);

static int dnt900_set_sys_address(struct dnt900_device *device, void *data);
static int dnt900_read_sys_address(struct dnt900_device *device, char *sys_address);
static int dnt900_read_slot_size(struct dnt900_ldisc *ldisc, unsigned int *slot_size);
static int dnt900_read_use_tree_routing(struct dnt900_ldisc *ldisc, bool *use_tree_routing);

static int dnt900_device_get_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value);
static int dnt900_device_set_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value);

static ssize_t dnt900_show_attr(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t dnt900_store_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t dnt900_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t dnt900_store_discover(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static int dnt900_cdev_open(struct inode *inode, struct file *filp);
static int dnt900_cdev_release(struct inode *inode, struct file *filp);
static ssize_t dnt900_cdev_read(struct file *filp, char __user *buf, size_t length, loff_t *offp);
static ssize_t dnt900_cdev_write(struct file *filp, const char __user *buf, size_t length, loff_t *offp);

static int dnt900_ldisc_get_params(struct dnt900_ldisc *ldisc);
static int dnt900_device_get_params(struct dnt900_device *device);

static int dnt900_alloc_device(struct dnt900_ldisc *ldisc, const char *mac_address, bool is_local, const char *name);
static int dnt900_alloc_device_remote(struct dnt900_ldisc *ldisc, const char *mac_address);
static int dnt900_alloc_device_local(struct dnt900_ldisc *ldisc, const char *mac_address);
static int dnt900_free_device(struct device *dev, void *unused);
static void dnt900_free_devices(struct dnt900_ldisc *ldisc);

static int dnt900_device_matches_sys_address(struct device *dev, void *data);
static int dnt900_device_matches_mac_address(struct device *dev, void *data);
static int dnt900_device_is_local(struct device *dev, void *data);

static bool dnt900_device_exists(struct dnt900_ldisc *ldisc, const char *mac_address);
static int dnt900_dispatch_to_device(struct dnt900_ldisc *ldisc, void *finder_data, int (*finder)(struct device *, void *), void *action_data, int (*action)(struct dnt900_device *, void *));
static int dnt900_dispatch_to_device_no_data(struct dnt900_ldisc *ldisc, void *finder_data, int (*finder)(struct device *, void *), int (*action)(struct dnt900_device *));
static int dnt900_apply_to_device(struct device *dev, void *data);
static int dnt900_for_each_device(struct dnt900_ldisc *ldisc, int (*action)(struct dnt900_device *));

static int dnt900_send_message(struct dnt900_ldisc *ldisc, void *message, unsigned int arg_length, char type, bool expects_reply, char *result);

static void dnt900_ldisc_receive_buf(struct tty_struct *tty, const unsigned char *cp, char *fp, int count);
static int dnt900_process_reply(struct dnt900_ldisc *ldisc, char command);
static int dnt900_receive_data(struct dnt900_device *device, void *data);
static int dnt900_process_event(struct dnt900_ldisc *ldisc, char event);
static int dnt900_process_announcement(struct dnt900_device *device, void *data);

static void dnt900_schedule_work(struct dnt900_ldisc *ldisc, const char *mac_address, void (*work_function)(struct work_struct *));
static void dnt900_add_new_device(struct work_struct *ws);
static void dnt900_refresh_ldisc(struct work_struct *ws);
static void dnt900_refresh_device(struct work_struct *ws);

static irqreturn_t dnt900_cts_handler(int irq, void *dev_id);

static int dnt900_ldisc_open(struct tty_struct *tty);
static void dnt900_ldisc_close(struct tty_struct *tty);

int __init dnt900_init(void);
void __exit dnt900_exit(void);

static int n_dnt900 = NR_LDISCS - 1;
static int members = 126;
static int gpio_cfg = 25;
static int gpio_cts = 27;

module_param(n_dnt900, int, S_IRUGO);
MODULE_PARM_DESC(n_dnt900, "line discipline number");
module_param(gpio_cfg, int, S_IRUGO);
MODULE_PARM_DESC(gpio_cfg, "GPIO number for /CFG signal");
module_param(gpio_cts, int, S_IRUGO);
MODULE_PARM_DESC(gpio_cts, "GPIO number for /HOST_CTS signal");

static const struct device_attribute dnt900_local_attributes[] = {
	__ATTR(reset,	ATTR_W, NULL, dnt900_store_reset),
	__ATTR(discover, ATTR_W, NULL, dnt900_store_discover)
};

// TODO: hook into sysfs writers when necessary to update device configs, etc.
// (maybe set callbacks on the attribute writers?)

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
	enableRtAcks,
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
	ParentACKQual,
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
	Event_Flags,
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
	MemorySave
};

static const struct dnt900_attribute dnt900_attributes[] = {
	DNT900_ATTR("DeviceMode",         ATTR_RW, 0x00, 0x00, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("RF_DataRate",        ATTR_RW, 0x00, 0x01, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("HopDuration",        ATTR_RW, 0x00, 0x02, 0x02, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("InitialParentNwkID", ATTR_RW, 0x00, 0x04, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("SecurityKey",        ATTR_W,  0x00, 0x05, 0x10, NULL, parse_16_hex),
	DNT900_ATTR("SleepMode",          ATTR_RW, 0x00, 0x15, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("WakeResponseTime",   ATTR_RW, 0x00, 0x16, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("WakeLinkTimeout",    ATTR_RW, 0x00, 0x17, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("TxPower",            ATTR_RW, 0x00, 0x18, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("ExtSyncEnable",      ATTR_RW, 0x00, 0x19, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("DiversityMode",      ATTR_RW, 0x00, 0x1A, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("UserTag",            ATTR_RW, 0x00, 0x1C, 0x10, print_16_ascii, parse_16_ascii),
	DNT900_ATTR("RegDenialDelay",     ATTR_RW, 0x00, 0x2C, 0x02, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("RmtTransDestAddr",   ATTR_RW, 0x00, 0x2E, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("TreeRoutingEn",      ATTR_RW, 0x00, 0x34, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("BaseModeNetID",      ATTR_RW, 0x00, 0x35, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("StaticNetAddr",      ATTR_RW, 0x00, 0x36, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("HeartbeatIntrvl",    ATTR_RW, 0x00, 0x37, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("TreeRoutingSysID",   ATTR_RW, 0x00, 0x39, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("enableRtAcks",       ATTR_RW, 0x00, 0x3A, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("FrequencyBand",      ATTR_RW, 0x01, 0x00, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("AccessMode",         ATTR_RW, 0x01, 0x01, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("BaseSlotSize",       ATTR_RW, 0x01, 0x02, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("LeasePeriod",        ATTR_RW, 0x01, 0x03, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("ARQ_Mode",           ATTR_RW, 0x01, 0x04, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("ARQ_AttemptLimit",   ATTR_RW, 0x01, 0x05, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("MaxSlots",           ATTR_RW, 0x01, 0x06, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("CSMA_Predelay",      ATTR_RW, 0x01, 0x07, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("CSMA_Backoff",       ATTR_RW, 0x01, 0x08, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("MaxPropDelay",       ATTR_RW, 0x01, 0x09, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("LinkDropThreshold",  ATTR_RW, 0x01, 0x0A, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("CSMA_RemtSlotSize",  ATTR_RW, 0x01, 0x0B, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("CSMA_BusyThreshold", ATTR_RW, 0x01, 0x0C, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("RangingInterval",    ATTR_RW, 0x01, 0x0D, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("AuthMode",           ATTR_RW, 0x01, 0x0E, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("P2PReplyTimeout",    ATTR_RW, 0x01, 0x0F, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("MacAddress",         ATTR_R,  0x02, 0x00, 0x03, print_3_bytes, NULL),
	DNT900_ATTR("CurrNwkAddr",        ATTR_R,  0x02, 0x03, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("CurrNwkID",          ATTR_R,  0x02, 0x04, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("CurrRF_DataRate",    ATTR_R,  0x02, 0x05, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("CurrFreqBand",       ATTR_R,  0x02, 0x06, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("LinkStatus",         ATTR_R,  0x02, 0x07, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("RemoteSlotSize",     ATTR_R,  0x02, 0x08, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("TDMA_NumSlots",      ATTR_R,  0x02, 0x09, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("TDMA_CurrSlot",      ATTR_R,  0x02, 0x0B, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("HardwareVersion",    ATTR_R,  0x02, 0x0C, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("FirmwareVersion",    ATTR_R,  0x02, 0x0D, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("FirmwareBuildNum",   ATTR_R,  0x02, 0x0E, 0x02, print_2_bytes, NULL),
	DNT900_ATTR("SuperframeCount",    ATTR_R,  0x02, 0x11, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("RSSI_Idle",          ATTR_R,  0x02, 0x12, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("RSSI_Last",          ATTR_R,  0x02, 0x13, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("CurrTxPower",        ATTR_R,  0x02, 0x14, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("CurrAttemptLimit",   ATTR_R,  0x02, 0x15, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("CurrRangeDelay",     ATTR_R,  0x02, 0x16, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("FirmwareBuildDate",  ATTR_R,  0x02, 0x17, 0x08, print_8_ascii, NULL),
	DNT900_ATTR("FirmwareBuildTime",  ATTR_R,  0x02, 0x1F, 0x08, print_8_ascii, NULL),
	DNT900_ATTR("ModelNumber",        ATTR_R,  0x02, 0x27, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("CurrBaseModeNetID",  ATTR_R,  0x02, 0x28, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("AveRXPwrOvHopSeq",   ATTR_R,  0x02, 0x29, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentACKQual",      ATTR_R,  0x02, 0x2A, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("SerialRate",         ATTR_RW, 0x03, 0x00, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("SerialParams",       ATTR_RW, 0x03, 0x02, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("SerialControls",     ATTR_RW, 0x03, 0x03, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("SPI_Mode",           ATTR_RW, 0x03, 0x04, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("SPI_Divisor",        ATTR_RW, 0x03, 0x05, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("SPI_Options",        ATTR_RW, 0x03, 0x06, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("SPI_MasterCmdLen",   ATTR_RW, 0x03, 0x07, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("SPI_MasterCmdStr",   ATTR_RW, 0x03, 0x08, 0x20, print_32_hex, parse_32_hex),
	DNT900_ATTR("ProtocolMode",       ATTR_RW, 0x04, 0x00, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("ProtocolOptions",    ATTR_RW, 0x04, 0x01, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("TxTimeout",          ATTR_RW, 0x04, 0x02, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("MinPacketLength",    ATTR_RW, 0x04, 0x03, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("AnnounceOptions",    ATTR_RW, 0x04, 0x04, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("TransLinkAnnEn",     ATTR_RW, 0x04, 0x05, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("ProtocolSequenceEn", ATTR_RW, 0x04, 0x06, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("TransPtToPtMode",    ATTR_RW, 0x04, 0x07, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("MaxPktsPerHop",      ATTR_RW, 0x04, 0x08, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO0",              ATTR_RW, 0x05, 0x00, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO1",              ATTR_RW, 0x05, 0x01, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO2",              ATTR_RW, 0x05, 0x02, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO3",              ATTR_RW, 0x05, 0x03, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO4",              ATTR_RW, 0x05, 0x04, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO5",              ATTR_RW, 0x05, 0x05, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("ADC0",               ATTR_R,  0x05, 0x06, 0x02, print_2_bytes, NULL),
	DNT900_ATTR("ADC1",               ATTR_R,  0x05, 0x08, 0x02, print_2_bytes, NULL),
	DNT900_ATTR("ADC2",               ATTR_R,  0x05, 0x0A, 0x02, print_2_bytes, NULL),
	DNT900_ATTR("Event_Flags",        ATTR_R,  0x05, 0x0C, 0x02, print_2_bytes, NULL),
	DNT900_ATTR("PWM0",               ATTR_RW, 0x05, 0x0E, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("PWM1",               ATTR_RW, 0x05, 0x10, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("GPIO_Dir",           ATTR_RW, 0x06, 0x00, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO_Init",          ATTR_RW, 0x06, 0x01, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO_Alt",           ATTR_RW, 0x06, 0x02, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO_Edge_Trigger",  ATTR_RW, 0x06, 0x03, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO_SleepMode",     ATTR_RW, 0x06, 0x04, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO_SleepDir",      ATTR_RW, 0x06, 0x05, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("GPIO_SleepState",    ATTR_RW, 0x06, 0x06, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("PWM0_Init",          ATTR_RW, 0x06, 0x07, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("PWM1_Init",          ATTR_RW, 0x06, 0x09, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("ADC_SampleIntvl",    ATTR_RW, 0x06, 0x0B, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("ADC0_ThresholdLo",   ATTR_RW, 0x06, 0x0D, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("ADC0_ThresholdHi",   ATTR_RW, 0x06, 0x0F, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("ADC1_ThresholdLo",   ATTR_RW, 0x06, 0x11, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("ADC1_ThresholdHi",   ATTR_RW, 0x06, 0x13, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("ADC2_ThresholdLo",   ATTR_RW, 0x06, 0x15, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("ADC2_ThresholdHi",   ATTR_RW, 0x06, 0x17, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("IO_ReportTrigger",   ATTR_RW, 0x06, 0x19, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("IO_ReportInterval",  ATTR_RW, 0x06, 0x1A, 0x04, print_4_bytes, parse_4_bytes),
	DNT900_ATTR("IO_ReportPreDel",    ATTR_RW, 0x06, 0x1E, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("IO_ReportRepeat",    ATTR_RW, 0x06, 0x1F, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("ApprovedAddr00",     ATTR_RW, 0x07, 0x00, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr01",     ATTR_RW, 0x07, 0x03, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr02",     ATTR_RW, 0x07, 0x06, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr03",     ATTR_RW, 0x07, 0x09, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr04",     ATTR_RW, 0x07, 0x0C, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr05",     ATTR_RW, 0x07, 0x0F, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr06",     ATTR_RW, 0x07, 0x12, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr07",     ATTR_RW, 0x07, 0x15, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr08",     ATTR_RW, 0x07, 0x18, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr09",     ATTR_RW, 0x07, 0x1B, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr10",     ATTR_RW, 0x07, 0x1E, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr11",     ATTR_RW, 0x07, 0x21, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr12",     ATTR_RW, 0x07, 0x24, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr13",     ATTR_RW, 0x07, 0x27, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr14",     ATTR_RW, 0x07, 0x2A, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("ApprovedAddr15",     ATTR_RW, 0x07, 0x2D, 0x03, print_3_bytes, parse_3_bytes),
	DNT900_ATTR("BaseNetworkID",      ATTR_R,  0x08, 0x00, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID01",  ATTR_R,  0x08, 0x01, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID02",  ATTR_R,  0x08, 0x02, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID03",  ATTR_R,  0x08, 0x03, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID04",  ATTR_R,  0x08, 0x04, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID05",  ATTR_R,  0x08, 0x05, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID06",  ATTR_R,  0x08, 0x06, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID07",  ATTR_R,  0x08, 0x07, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID08",  ATTR_R,  0x08, 0x08, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID09",  ATTR_R,  0x08, 0x09, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID10",  ATTR_R,  0x08, 0x0A, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID11",  ATTR_R,  0x08, 0x0B, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID12",  ATTR_R,  0x08, 0x0C, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID13",  ATTR_R,  0x08, 0x0D, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID14",  ATTR_R,  0x08, 0x0E, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID15",  ATTR_R,  0x08, 0x0F, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID16",  ATTR_R,  0x08, 0x10, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID17",  ATTR_R,  0x08, 0x11, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID18",  ATTR_R,  0x08, 0x12, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID19",  ATTR_R,  0x08, 0x13, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID20",  ATTR_R,  0x08, 0x14, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID21",  ATTR_R,  0x08, 0x15, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID22",  ATTR_R,  0x08, 0x16, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID23",  ATTR_R,  0x08, 0x17, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID24",  ATTR_R,  0x08, 0x18, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID25",  ATTR_R,  0x08, 0x19, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID26",  ATTR_R,  0x08, 0x1A, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID27",  ATTR_R,  0x08, 0x1B, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID28",  ATTR_R,  0x08, 0x1C, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID29",  ATTR_R,  0x08, 0x1D, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID30",  ATTR_R,  0x08, 0x1E, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID31",  ATTR_R,  0x08, 0x1F, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID32",  ATTR_R,  0x08, 0x20, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID33",  ATTR_R,  0x08, 0x21, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID34",  ATTR_R,  0x08, 0x22, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID35",  ATTR_R,  0x08, 0x23, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID36",  ATTR_R,  0x08, 0x24, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID37",  ATTR_R,  0x08, 0x25, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID38",  ATTR_R,  0x08, 0x26, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID39",  ATTR_R,  0x08, 0x27, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID40",  ATTR_R,  0x08, 0x28, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID41",  ATTR_R,  0x08, 0x29, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID42",  ATTR_R,  0x08, 0x2A, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID43",  ATTR_R,  0x08, 0x2B, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID44",  ATTR_R,  0x08, 0x2C, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID45",  ATTR_R,  0x08, 0x2D, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID46",  ATTR_R,  0x08, 0x2E, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID47",  ATTR_R,  0x08, 0x2F, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID48",  ATTR_R,  0x08, 0x30, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID49",  ATTR_R,  0x08, 0x31, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID50",  ATTR_R,  0x08, 0x32, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID51",  ATTR_R,  0x08, 0x33, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID52",  ATTR_R,  0x08, 0x34, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID53",  ATTR_R,  0x08, 0x35, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID54",  ATTR_R,  0x08, 0x36, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID55",  ATTR_R,  0x08, 0x37, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID56",  ATTR_R,  0x08, 0x38, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID57",  ATTR_R,  0x08, 0x39, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID58",  ATTR_R,  0x08, 0x3A, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID59",  ATTR_R,  0x08, 0x3B, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID60",  ATTR_R,  0x08, 0x3C, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID61",  ATTR_R,  0x08, 0x3D, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID62",  ATTR_R,  0x08, 0x3E, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("ParentNetworkID63",  ATTR_R,  0x08, 0x3F, 0x01, print_1_bytes, NULL),
	DNT900_ATTR("RegMACAddr00",       ATTR_R,  0x09, 0x00, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr01",       ATTR_R,  0x09, 0x01, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr02",       ATTR_R,  0x09, 0x02, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr03",       ATTR_R,  0x09, 0x03, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr04",       ATTR_R,  0x09, 0x04, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr05",       ATTR_R,  0x09, 0x05, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr06",       ATTR_R,  0x09, 0x06, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr07",       ATTR_R,  0x09, 0x07, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr08",       ATTR_R,  0x09, 0x08, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr09",       ATTR_R,  0x09, 0x09, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr10",       ATTR_R,  0x09, 0x0A, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr11",       ATTR_R,  0x09, 0x0B, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr12",       ATTR_R,  0x09, 0x0C, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr13",       ATTR_R,  0x09, 0x0D, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr14",       ATTR_R,  0x09, 0x0E, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr15",       ATTR_R,  0x09, 0x0F, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr16",       ATTR_R,  0x09, 0x10, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr17",       ATTR_R,  0x09, 0x11, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr18",       ATTR_R,  0x09, 0x12, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr19",       ATTR_R,  0x09, 0x13, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr20",       ATTR_R,  0x09, 0x14, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr21",       ATTR_R,  0x09, 0x15, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr22",       ATTR_R,  0x09, 0x16, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr23",       ATTR_R,  0x09, 0x17, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr24",       ATTR_R,  0x09, 0x18, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("RegMACAddr25",       ATTR_R,  0x09, 0x19, 0x0F, print_5_macs, NULL),
	DNT900_ATTR("UcReset",            ATTR_W,  0xFF, 0x00, 0x01, NULL, parse_1_bytes),
	DNT900_ATTR("SleepModeOverride",  ATTR_RW, 0xFF, 0x0C, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("RoutingTableUpd",    ATTR_RW, 0xFF, 0x1C, 0x01, print_1_bytes, parse_1_bytes),
	DNT900_ATTR("DiagSerialRate",     ATTR_RW, 0xFF, 0x20, 0x02, print_2_bytes, parse_2_bytes),
	DNT900_ATTR("MemorySave",         ATTR_W,  0xFF, 0xFF, 0x01, NULL, parse_1_bytes)
};

static struct class *dnt900_class;

static struct file_operations dnt900_cdev_fops = {
	.owner = THIS_MODULE,
	.open = dnt900_cdev_open,
	.release = dnt900_cdev_release,
	.read = dnt900_cdev_read,
	.write = dnt900_cdev_write,
};

static struct tty_ldisc_ops dnt900_ldisc_ops = {
	.magic = TTY_LDISC_MAGIC,
	.name = DRIVER_NAME,
	.open = dnt900_ldisc_open,
	.close = dnt900_ldisc_close,
	.receive_buf = dnt900_ldisc_receive_buf,

	.owner = THIS_MODULE,
};

static int print_bytes(int bytes, const char *value, char *buf)
{
	unsigned int count = scnprintf(buf, PAGE_SIZE, "0x");
	for (; bytes > 0; --bytes)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", value[bytes-1]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	return count;
}

static int print_1_bytes(const char *value, char *buf)
{
	return print_bytes(1, value, buf);
}

static int print_2_bytes(const char *value, char *buf)
{
	return print_bytes(2, value, buf);
}

static int print_3_bytes(const char *value, char *buf)
{
	return print_bytes(3, value, buf);
}

static int print_4_bytes(const char *value, char *buf)
{
	return print_bytes(4, value, buf);
}

static int print_hex(int bytes, const char *value, char *buf)
{
	unsigned int count = scnprintf(buf, PAGE_SIZE, "0x");
	for (; bytes > 0; ++value, --bytes)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", *value);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	return count;
}

static int print_32_hex(const char *value, char *buf)
{
	return print_hex(32, value, buf);
}

static int print_8_ascii(const char *value, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%.8s\n", value);
}

static int print_16_ascii(const char *value, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%.16s\n", value);
}

static int print_5_macs(const char *value, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
		"0x%02X%02X%02X 0x%02X%02X%02X 0x%02X%02X%02X 0x%02X%02X%02X 0x%02X%02X%02X\n",
		value[ 2], value[ 1], value[ 0],
		value[ 5], value[ 4], value[ 3],
		value[ 8], value[ 7], value[ 6],
		value[11], value[10], value[ 9],
		value[14], value[13], value[12]);
}

static int parse_bytes(int bytes, const char *buf, size_t count, char *value)
{
	unsigned long result;
	TRY(kstrtoul(buf, 0, &result));
	if (result >> (8 * bytes))
		return -ERANGE;
	for (; bytes > 0; --bytes, ++value, result >>= 8)
		*value = result & 0xFF;
	return 0;
}

static int parse_1_bytes(const char *buf, size_t count, char *value)
{
	return parse_bytes(1, buf, count, value);
}

static int parse_2_bytes(const char *buf, size_t count, char *value)
{
	return parse_bytes(2, buf, count, value);
}

static int parse_3_bytes(const char *buf, size_t count, char *value)
{
	return parse_bytes(3, buf, count, value);
}

static int parse_4_bytes(const char *buf, size_t count, char *value)
{
	return parse_bytes(4, buf, count, value);
}

static int parse_hex(int bytes, const char *buf, size_t count, char *value)
{
	if (bytes * 2 + 2 != count)
		return -EINVAL;
	if (*buf++ != '0')
		return -EINVAL;
	if (tolower(*buf++) != 'x')
		return -EINVAL;
	for (; bytes > 0; --bytes, ++value) {
		*value = 0;
		for (int n = 0; n < 2; ++n, ++buf) {
			if (!isxdigit(*buf))
				return -EINVAL;
			*value <<= 4;
			*value += isdigit(*buf) ? (*buf - '0') : (tolower(*buf) - 'a' + 10);
		}
	}
	return 0;
}

static int parse_16_hex(const char *buf, size_t count, char *value)
{
	return parse_hex(16, buf, count, value);
}

static int parse_32_hex(const char *buf, size_t count, char *value)
{
	return parse_hex(32, buf, count, value);
}

static int parse_16_ascii(const char *buf, size_t count, char *value)
{
	for (int n = 0; n < 16; ++n)
		value[n] = n < count ? buf[n] : 0;
	return 0;
}

static int dnt900_add_attributes(struct device *dev)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	for (int n = 0; n < ARRAY_SIZE(dnt900_attributes); ++n) {
		int result = device_create_file(dev, &dnt900_attributes[n].attr);
		if (result)
			return result;
	}
	if (device->is_local)
		for (int n = 0; n < ARRAY_SIZE(dnt900_local_attributes); ++n) {
			int result = device_create_file(dev, dnt900_local_attributes + n);
			if (result)
				return result;
		}
	return 0;
}

static void dnt900_remove_attributes(struct device *dev)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	for (int n = 0; n < ARRAY_SIZE(dnt900_attributes); ++n)
		device_remove_file(dev, &dnt900_attributes[n].attr);
	if (device->is_local)
		for (int n = 0; n < ARRAY_SIZE(dnt900_local_attributes); ++n)
			device_remove_file(dev, dnt900_local_attributes + n);
}

static int dnt900_enter_protocol_mode(struct dnt900_ldisc *ldisc)
{
	int error = 0;
	if (ldisc->gpio_cfg >= 0)
		gpio_set_value(ldisc->gpio_cfg, 0);
	else {
		// TODO: test this case
		char message[] = { 0, 0, 0, 'D', 'N', 'T', 'C', 'F', 'G' };
		error = dnt900_send_message(ldisc, message, 6, COMMAND_ENTER_PROTOCOL_MODE, false, NULL);
	}
	// flushing the DNT900 with zeroes seems to be needed for some reason...
	char zeroes[1000] = { 0 };
	ldisc->tty->ops->write(ldisc->tty, zeroes, ARRAY_SIZE(zeroes));
	ldisc->tty->ops->flush_chars(ldisc->tty);
	return error;
}

static int dnt900_exit_protocol_mode(struct dnt900_ldisc *ldisc)
{
	int error = 0;
	if (ldisc->gpio_cfg >= 0)
		gpio_set_value(ldisc->gpio_cfg, 1);
	else {
		char message[] = { 0, 0, 0 };
		error = dnt900_send_message(ldisc, message, 0, COMMAND_EXIT_PROTOCOL_MODE, false, NULL);
	}
	return error;
}

static int dnt900_get_register(struct dnt900_ldisc *ldisc, const struct dnt900_register *reg, char *value)
{
	
	struct dnt900_get_register_message message;
	message.bank = reg->bank;
	message.reg = reg->offset;
	message.span = reg->span;
	return dnt900_send_message(ldisc, &message, 3, COMMAND_GET_REGISTER, true, value);
}

static int dnt900_get_remote_register(struct dnt900_ldisc *ldisc, const char *sys_address, const struct dnt900_register *reg, char *value)
{
	struct dnt900_get_remote_register_message message;
	COPY3(message.sys_address, sys_address);
	message.bank = reg->bank;
	message.reg = reg->offset;
	message.span = reg->span;
	return dnt900_send_message(ldisc, &message, 6, COMMAND_GET_REMOTE_REGISTER, true, value);
}

static int dnt900_set_register(struct dnt900_ldisc *ldisc, const struct dnt900_register *reg, const char *value)
{
	struct dnt900_set_register_message message;
	message.bank = reg->bank;
	message.reg = reg->offset;
	message.span = reg->span;
	memcpy(&message.value, value, reg->span);
	bool expects_reply = reg->bank != 0xFF || (reg->offset != 0x00 && (reg->offset != 0xFF || *value != 0x02));
	return dnt900_send_message(ldisc, &message, 3 + reg->span, COMMAND_SET_REGISTER, expects_reply, NULL);
}

static int dnt900_set_remote_register(struct dnt900_ldisc *ldisc, const char *sys_address, const struct dnt900_register *reg, const char *value)
{
	struct dnt900_set_remote_register_message message;
	COPY3(message.sys_address, sys_address);
	message.bank = reg->bank;
	message.reg = reg->offset;
	message.span = reg->span;
	memcpy(&message.value, value, reg->span);
	bool expects_reply = reg->bank != 0xFF || (reg->offset != 0x00 && (reg->offset != 0xFF || *value != 0x02));
	return dnt900_send_message(ldisc, &message, 6 + reg->span, COMMAND_SET_REMOTE_REGISTER, expects_reply, NULL);
}

static int dnt900_discover(struct dnt900_ldisc *ldisc, const char *mac_address, char *sys_address)
{
	bool use_tree_routing;
	TRY(dnt900_read_use_tree_routing(ldisc, &use_tree_routing));
	if (use_tree_routing) {
		struct dnt900_discover_message message;
		COPY3(message.mac_address, mac_address);
		int error = dnt900_send_message(ldisc, &message, 3, COMMAND_DISCOVER, true, sys_address);
		return error ? -ENODEV : 0;
	} else {
		COPY3(sys_address, mac_address);
		return 0;
	}
}

static int dnt900_set_sys_address(struct dnt900_device *device, void *data)
{
	const char *sys_address = data;
	TRY(mutex_lock_interruptible(&device->param_lock));
	COPY3(device->sys_address, sys_address);
	mutex_unlock(&device->param_lock);
	return 0;
}

static int dnt900_read_sys_address(struct dnt900_device *device, char *sys_address)
{
	TRY(mutex_lock_interruptible(&device->param_lock));
	COPY3(sys_address, device->sys_address);
	mutex_unlock(&device->param_lock);
	return 0;
}

static int dnt900_read_slot_size(struct dnt900_ldisc *ldisc, unsigned int *slot_size)
{
	// TODO: Is this expensive (given that it is called for every TxData packet)?
	TRY(mutex_lock_interruptible(&ldisc->param_lock));
	*slot_size = ldisc->slot_size;
	mutex_unlock(&ldisc->param_lock);
	return 0;
}

static int dnt900_read_use_tree_routing(struct dnt900_ldisc *ldisc, bool *use_tree_routing)
{
	TRY(mutex_lock_interruptible(&ldisc->param_lock));
	*use_tree_routing = ldisc->use_tree_routing;
	mutex_unlock(&ldisc->param_lock);
	return 0;
}

static int dnt900_device_get_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value)
{
	if (device->is_local)
		return dnt900_get_register(device->ldisc, reg, value);
	char sys_address[3];
	TRY(dnt900_read_sys_address(device, sys_address));
	return dnt900_get_remote_register(device->ldisc, sys_address, reg, value);
}

static int dnt900_device_set_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value)
{
	if (device->is_local)
		return dnt900_set_register(device->ldisc, reg, value);
	char sys_address[3];
	TRY(dnt900_read_sys_address(device, sys_address));
	return dnt900_set_remote_register(device->ldisc, sys_address, reg, value);
}

static ssize_t dnt900_show_attr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	struct dnt900_attribute *attribute = container_of(attr, struct dnt900_attribute, attr);
	char value[32];
	int error = dnt900_device_get_register(device, &attribute->reg, value);
	return error ? error : attribute->print(value, buf);
}

static ssize_t dnt900_store_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	struct dnt900_attribute *attribute = container_of(attr, struct dnt900_attribute, attr);
	char value[32];
	TRY(attribute->parse(buf, count, value));
	TRY(dnt900_device_set_register(device, &attribute->reg, value));
	return count;
}

static ssize_t dnt900_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	struct dnt900_software_reset_message message;
	message.boot_select = 0;
	TRY(dnt900_send_message(device->ldisc, &message, 1, COMMAND_SOFTWARE_RESET, true, NULL));
	return count;
}

static ssize_t dnt900_store_discover(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int mac_address_int;
	char mac_address[3];
	struct dnt900_device *device = dev_get_drvdata(dev);
	struct dnt900_ldisc *ldisc = device->ldisc;
	TRY(kstrtouint(buf, 16, &mac_address_int));
	for (int n = 0; n < 3; ++n, mac_address_int >>= 8)
		mac_address[n] = mac_address_int & 0xFF;
	if (mac_address_int)
		return -EINVAL;
	TRY(dnt900_alloc_device_remote(ldisc, mac_address));
	return count;
}

static int dnt900_cdev_open(struct inode *inode, struct file *filp)
{
	struct dnt900_device *device = container_of(inode->i_cdev, struct dnt900_device, cdev);
	int error = 0;
	
	if (device->is_local && (filp->f_mode & FMODE_WRITE))
		return -EPERM;
	if (filp->f_mode & FMODE_READ)
		UNWIND(error, !mutex_trylock(&device->read_lock), fail_lock_read);
	if (filp->f_mode & FMODE_WRITE)
		UNWIND(error, !mutex_trylock(&device->write_lock), fail_lock_write);
	filp->private_data = device;
	return 0;
	
fail_lock_write:
	if (filp->f_mode & FMODE_READ)
		mutex_unlock(&device->read_lock);
fail_lock_read:
	return -EBUSY;
}

static int dnt900_cdev_release(struct inode *inode, struct file *filp)
{
	struct dnt900_device *device = container_of(inode->i_cdev, struct dnt900_device, cdev);
	if (filp->f_mode & FMODE_READ)
		mutex_unlock(&device->read_lock);
	if (filp->f_mode & FMODE_WRITE)
		mutex_unlock(&device->write_lock);
	return 0;
}

static ssize_t dnt900_cdev_read(struct file *filp, char __user *buf, size_t length, loff_t *offp)
{
	struct dnt900_device *device = filp->private_data;
	if (filp->f_flags & O_NONBLOCK && kfifo_is_empty(&device->fifo))
		return -EAGAIN;
	TRY(wait_event_interruptible(device->queue, !kfifo_is_empty(&device->fifo)));
	unsigned int copied;
	TRY(kfifo_to_user(&device->fifo, buf, length, &copied));
	*offp += copied;
	return copied;
}

static ssize_t dnt900_cdev_write(struct file *filp, const char __user *buf, size_t length, loff_t *offp)
{
	if (!length)
		return length;
	// TODO: implement a write fifo for each device? (or, per tty) if so, why?
	// TODO: how to handle blocking/non-blocking here when the tty is being overrun?
	struct dnt900_device *device = filp->private_data;
	struct dnt900_tx_data_message message;
	TRY(dnt900_read_sys_address(device, message.sys_address));
	unsigned int slot_size; // TODO: check case where slot_size < 6
	TRY(dnt900_read_slot_size(device->ldisc, &slot_size));
	unsigned long count = slot_size - 6 < length ? slot_size - 6 : length;
	if (copy_from_user(message.data, buf, count))
		return -EFAULT;
	TRY(dnt900_send_message(device->ldisc, &message, 3 + count, COMMAND_TX_DATA, true, NULL));
	*offp += count;
	return count;
}

static int dnt900_ldisc_get_params(struct dnt900_ldisc *ldisc)
{
	char announce_options, protocol_options, auth_mode, device_mode, slot_size, tree_routing_en;
	TRY(dnt900_get_register(ldisc, &dnt900_attributes[AnnounceOptions].reg, &announce_options));
	TRY(dnt900_get_register(ldisc, &dnt900_attributes[ProtocolOptions].reg, &protocol_options));
	TRY(dnt900_get_register(ldisc, &dnt900_attributes[AuthMode].reg, &auth_mode));
	if ((announce_options & 0x03) != 0x03)
		pr_err("set radio AnnounceOptions register to 0x07 for correct driver operation\n");
	if ((protocol_options & 0x05) != 0x05)
		pr_err("set radio ProtocolOptions register to 0x05 for correct driver operation\n");
	if (auth_mode == 0x02)
		pr_warn("AuthMode register is set to 0x02 but host-based authentication is not supported\n");
	TRY(dnt900_get_register(ldisc, &dnt900_attributes[TreeRoutingEn].reg, &tree_routing_en));
	TRY(dnt900_get_register(ldisc, &dnt900_attributes[DeviceMode].reg, &device_mode));
	TRY(dnt900_get_register(ldisc, &dnt900_attributes[device_mode == 0x01 ? BaseSlotSize : RemoteSlotSize].reg, &slot_size));
	TRY(mutex_lock_interruptible(&ldisc->param_lock));
	ldisc->slot_size = (unsigned char)slot_size;
	ldisc->use_tree_routing = tree_routing_en == 0x01;
	mutex_unlock(&ldisc->param_lock);
	return 0;
}

static int dnt900_device_get_params(struct dnt900_device *device)
{
	if (device->is_local)
		return 0;
	char sys_address[3];
	TRY(dnt900_discover(device->ldisc, device->mac_address, sys_address));
	TRY(mutex_lock_interruptible(&device->param_lock));
	COPY3(device->sys_address, sys_address);
	mutex_unlock(&device->param_lock);
	char mac_address[3];
	TRY(dnt900_device_get_register(device, &dnt900_attributes[MacAddress].reg, mac_address));
	return 0;
}

static int dnt900_alloc_device(struct dnt900_ldisc *ldisc, const char *mac_address, bool is_local, const char *name)
{
	TRY(mutex_lock_interruptible(&ldisc->devices_lock));
	int error = 0;
	if (dnt900_device_exists(ldisc, mac_address)) {
		error = -EEXIST;
		goto fail_exists;
	}
	struct dnt900_device *device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		error = -ENOMEM;
		goto fail_alloc;
	}
	device->ldisc = ldisc;
	device->is_local = is_local;
	COPY3(device->mac_address, mac_address);
	mutex_init(&device->param_lock);
	mutex_init(&device->read_lock);
	mutex_init(&device->write_lock);
	init_waitqueue_head(&device->queue);
	kfifo_init(&device->fifo, &device->buf, RX_BUF_SIZE);
	UNWIND(error, dnt900_device_get_params(device), fail_get_params);
	dev_t devt = MKDEV(ldisc->major, ldisc->minor++);
	struct device *dev = device_create(dnt900_class, ldisc->tty->dev, devt, device, name);
	if (IS_ERR(dev)) {
		error = PTR_ERR(dev);
		goto fail_dev_create;
	}
	UNWIND(error, dnt900_add_attributes(dev), fail_add_attributes);
	cdev_init(&device->cdev, &dnt900_cdev_fops);
	device->cdev.owner = THIS_MODULE;
	UNWIND(error, cdev_add(&device->cdev, devt, 1), fail_cdev_add);
	// TODO: set cdev permissions to read-only if is_local?
	goto success;
	
	cdev_del(&device->cdev);
fail_cdev_add:
	dnt900_remove_attributes(dev);
fail_add_attributes:
	device_unregister(dev);
fail_dev_create:
fail_get_params:
	kfree(device);
fail_alloc:
fail_exists:
success:
	mutex_unlock(&ldisc->devices_lock);
	return error;
}

static int dnt900_alloc_device_remote(struct dnt900_ldisc *ldisc, const char *mac_address)
{
	char name[80];
	snprintf(name, ARRAY_SIZE(name), "%s.0x%02X%02X%02X", ldisc->tty->name, mac_address[2], mac_address[1], mac_address[0]);
	return dnt900_alloc_device(ldisc, mac_address, false, name);
}

static int dnt900_alloc_device_local(struct dnt900_ldisc *ldisc, const char *mac_address)
{
	char name[80];
	snprintf(name, ARRAY_SIZE(name), "%s.local", ldisc->tty->name);
	return dnt900_alloc_device(ldisc, mac_address, true, name);
}

static int dnt900_free_device(struct device *dev, void *unused)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	cdev_del(&device->cdev);
	dnt900_remove_attributes(dev);
	device_unregister(dev);
	kfree(device);
	return 0;
}

static void dnt900_free_devices(struct dnt900_ldisc *ldisc)
{
	device_for_each_child(ldisc->tty->dev, NULL, dnt900_free_device);
}

static int dnt900_device_matches_sys_address(struct device *dev, void *data)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	const char *sys_address = data;
	char device_sys_address[3];
	int error = dnt900_read_sys_address(device, device_sys_address);
	return !error && EQUAL_ADDRESSES(device_sys_address, sys_address);
}

static int dnt900_device_matches_mac_address(struct device *dev, void *data)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	const char *mac_address = data;
	return EQUAL_ADDRESSES(device->mac_address, mac_address);
}

static int dnt900_device_is_local(struct device *dev, void *data)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	return device->is_local;
}

static bool dnt900_device_exists(struct dnt900_ldisc *ldisc, const char *mac_address)
{
	return device_for_each_child(ldisc->tty->dev, (void *)mac_address, dnt900_device_matches_mac_address);
}

static int dnt900_dispatch_to_device(struct dnt900_ldisc *ldisc, void *finder_data, int (*finder)(struct device *, void *), void *action_data, int (*action)(struct dnt900_device *, void *))
{
	int error;
	struct device *dev = device_find_child(ldisc->tty->dev, finder_data, finder);
	if (dev) {
		struct dnt900_device *device = dev_get_drvdata(dev);
		error = action(device, action_data);
		put_device(dev);
	} else 
		error = -ENODEV;
	return error;
}

static int dnt900_dispatch_to_device_no_data(struct dnt900_ldisc *ldisc, void *finder_data, int (*finder)(struct device *, void *), int (*action)(struct dnt900_device *))
{
	int error;
	struct device *dev = device_find_child(ldisc->tty->dev, finder_data, finder);
	if (dev) {
		struct dnt900_device *device = dev_get_drvdata(dev);
		error = action(device);
		put_device(dev);
	} else 
		error = -ENODEV;
	return error;
}

static int dnt900_apply_to_device(struct device *dev, void *data)
{
	int (*action)(struct dnt900_device *) = data;
	struct dnt900_device *device = dev_get_drvdata(dev);
	return action(device);
}

static int dnt900_for_each_device(struct dnt900_ldisc *ldisc, int (*action)(struct dnt900_device *))
{
	return device_for_each_child(ldisc->tty->dev, action, dnt900_apply_to_device);
}

static int dnt900_send_message(struct dnt900_ldisc *ldisc, void *message, unsigned int arg_length, char type, bool expects_reply, char *result)
{
	int error = 0;
	struct dnt900_packet packet;
	struct dnt900_message_header *header = message;
	unsigned int count = 3 + arg_length;
	
	INIT_LIST_HEAD(&packet.list);
	init_completion(&packet.completed);
	packet.result = result;
	packet.error = 0;
	packet.message = message;
	header->start_of_packet = START_OF_PACKET;
	header->length = 1 + arg_length;
	header->type = type;
	
	TRY(mutex_lock_interruptible(&ldisc->packets_lock));
	list_add_tail(&packet.list, &ldisc->packets);
	mutex_unlock(&ldisc->packets_lock);
	
	const char *buf = message;
	
	UNWIND(error, mutex_lock_interruptible(&ldisc->tty_lock), exit);
	for (int sent; count > 0; count -= sent, buf += sent) {
		sent = ldisc->tty->ops->write(ldisc->tty, buf, count);
		ldisc->tty->ops->flush_chars(ldisc->tty);
		// TODO: error checking?
		// TODO: implement non-blocking option using tty->ops->write_room?
	}
	mutex_unlock(&ldisc->tty_lock);
	
	if (!expects_reply)
		goto exit;
	
	long remaining = wait_for_completion_interruptible_timeout(&packet.completed, msecs_to_jiffies(TIMEOUT_MS));
	error = !remaining ? -ETIMEDOUT : remaining < 0 ? remaining : 0;
	if (error)
		goto exit;
	return packet.error;
	
exit:
	mutex_lock(&ldisc->packets_lock); // TODO: use TRY and _interrupible?
	list_del(&packet.list);
	mutex_unlock(&ldisc->packets_lock);
	return error;
}

static void dnt900_ldisc_receive_buf(struct tty_struct *tty, const unsigned char *cp, char *fp, int count)
{
	struct dnt900_ldisc *ldisc = tty->disc_data;
	
	while (count > 0) {
		// TODO: not testing flag bytes (fp) for now
		if (ldisc->end == 0)
			for (; count > 0 && *cp != START_OF_PACKET; --count, ++cp)
				;
		for (; count > 0 && ldisc->end < 2; --count, ++cp, ++ldisc->end)
			ldisc->message[ldisc->end] = *cp;
		if (ldisc->end < 2)
			break;
		const unsigned int length = 2 + (unsigned char)ldisc->message[1];
		for (; count > 0 && ldisc->end < length; --count, ++cp, ++ldisc->end)
			ldisc->message[ldisc->end] = *cp;
		if (ldisc->end == length) { // TODO: need to also check length > 2?
			const char type = ldisc->message[2];
			// TODO: need to process possible errors returned by
			// dnt900_process_... below:
			if (type & TYPE_REPLY)
				dnt900_process_reply(ldisc, type & MASK_COMMAND);
			if (type & TYPE_EVENT)
				dnt900_process_event(ldisc, type);
			ldisc->end = 0;
		}
	}
}

static int dnt900_process_reply(struct dnt900_ldisc *ldisc, char command)
{
	TRY(mutex_lock_interruptible(&ldisc->packets_lock));
	// TODO: do we have to hold the mutex for this whole loop?
	struct dnt900_packet *packet;
	list_for_each_entry(packet, &ldisc->packets, list) {
		char tx_status = 0;
		if (packet->message[2] != command)
			continue;
		switch (command) {
		case COMMAND_TX_DATA:
			// match Addr:
			if (packet->message[3] != ldisc->message[4] 
			 || packet->message[4] != ldisc->message[5] 
			 || packet->message[5] != ldisc->message[6])
				continue;
			tx_status = ldisc->message[3];
			break;
		case COMMAND_GET_REGISTER:
			// match Reg, Bank, span:
			if (packet->message[3] != ldisc->message[3] 
			 || packet->message[4] != ldisc->message[4] 
			 || packet->message[5] != ldisc->message[5])
				continue;
			// TODO: check packet->message[5] + 6 == length?
			for (int n = 0; n < packet->message[5]; ++n)
				packet->result[n] = ldisc->message[6 + n];
			break;
		case COMMAND_SET_REGISTER:
			break;
		case COMMAND_DISCOVER:
			// match MacAddr:
			if (packet->message[3] != ldisc->message[4] 
			 || packet->message[4] != ldisc->message[5] 
			 || packet->message[5] != ldisc->message[6])
				continue;
			tx_status = ldisc->message[3];
			if (tx_status == STATUS_ACKNOWLEDGED)
				for (int n = 0; n < 3; ++n)
					packet->result[n] = ldisc->message[7 + n];
			break;
		case COMMAND_GET_REMOTE_REGISTER:
			// match Addr:
			if (packet->message[3] != ldisc->message[4] 
			 || packet->message[4] != ldisc->message[5] 
			 || packet->message[5] != ldisc->message[6])
				continue;
			tx_status = ldisc->message[3];
			if (tx_status == STATUS_ACKNOWLEDGED) {
				// match Reg, Bank, span:
				if (packet->message[6] != ldisc->message[8] 
				 || packet->message[7] != ldisc->message[9] 
				 || packet->message[8] != ldisc->message[10])
					continue;
				for (int n = 0; n < packet->message[8]; ++n)
					packet->result[n] = ldisc->message[11 + n];
			}
			break;
		case COMMAND_SET_REMOTE_REGISTER:
			// match Addr:
			if (packet->message[3] != ldisc->message[4] 
			 || packet->message[4] != ldisc->message[5] 
			 || packet->message[5] != ldisc->message[6])
				continue;
			tx_status = ldisc->message[3];
			break;
		case COMMAND_SOFTWARE_RESET:
			break;
		case COMMAND_ENTER_PROTOCOL_MODE:
			break;
		}
		switch (tx_status) {
		case STATUS_ACKNOWLEDGED:
			packet->error = 0;
			break;
		case STATUS_HOLDING_FOR_FLOW:
			packet->error = -ETIMEDOUT;
			break;
		case STATUS_NOT_ACKNOWLEDGED:
		case STATUS_NOT_LINKED:
		default:
			packet->error = -ECOMM;
		}
		list_del(&packet->list);
		complete(&packet->completed);
		break;
	}
	mutex_unlock(&ldisc->packets_lock);
	return 0;
}

static int dnt900_process_event(struct dnt900_ldisc *ldisc, char event)
{
	char sys_address[3];
	struct dnt900_rxdata rxdata;
	char announcement[11];
	int error = 0;
	
	switch (event) {
	case EVENT_RX_DATA:
		for (int offset = 3; offset < 6; ++offset)
			sys_address[offset-3] = ldisc->message[offset];
		rxdata.buf = ldisc->message + 6;
		rxdata.len = ldisc->end - 6;
		error = dnt900_dispatch_to_device(ldisc, sys_address, dnt900_device_matches_sys_address, &rxdata, dnt900_receive_data);
		// TODO: add new device if it doesn't exist? You would have to retrieve its MAC first,
		// and also do this in a workqueue job. The first packets of received data would likely
		// be discarded.
		break;
	case EVENT_ANNOUNCE:
		for (int offset = 3; offset < ldisc->end && offset < 3 + ARRAY_SIZE(announcement); ++offset)
			announcement[offset-3] = ldisc->message[offset];
		error = dnt900_dispatch_to_device(ldisc, NULL, dnt900_device_is_local, announcement, dnt900_process_announcement);
		break;
	case EVENT_RX_EVENT:
		// unimplemented for now (used for automatic I/O reporting)
		break;
	case EVENT_JOIN_REQUEST:
		// unimplemented for now (use for host-based authentication)
		break;
	}
	return error;
}

static int dnt900_receive_data(struct dnt900_device *device, void *data)
{
	struct dnt900_rxdata *rxdata = data;
	unsigned int copied = kfifo_in(&device->fifo, rxdata->buf, rxdata->len);
	wake_up_interruptible(&device->queue);
	return copied == rxdata->len ? 0 : -ENOSPC; // TODO: better error code? (or even needed?)
}

static int dnt900_process_announcement(struct dnt900_device *device, void *data)
{
	char *annc = data;
	char message[512];
	struct dnt900_rxdata rxdata = { .buf = message, .len = 0 };
	struct dnt900_ldisc *ldisc = device->ldisc;
	
	switch (annc[0]) {
	case ANNOUNCEMENT_STARTED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"completed startup initialization\n");
		dnt900_schedule_work(ldisc, NULL, dnt900_refresh_ldisc);
		break;
	case ANNOUNCEMENT_JOINED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"joined network:\n" \
			"    network ID 0x%02X\n" \
			"    base MAC address 0x%02X%02X%02X\n" \
			"    range %d km\n", \
			annc[1], annc[4], annc[3], annc[2], RANGE_TO_KMS(annc[5]));
		if (dnt900_device_exists(ldisc, annc + 2))
			dnt900_schedule_work(ldisc, annc + 2, dnt900_refresh_device);
		else
			dnt900_schedule_work(ldisc, annc + 2, dnt900_add_new_device);
		break;
	case ANNOUNCEMENT_EXITED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"exited network:\n" \
			"    network ID 0x%02X\n", \
			annc[1]);
		break;
	case ANNOUNCEMENT_REMOTE_JOINED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"radio joined:\n" \
			"    MAC address 0x%02X%02X%02X\n" \
			"    range %d km\n", \
			annc[3], annc[2], annc[1], RANGE_TO_KMS(annc[5]));
		if (dnt900_device_exists(ldisc, annc + 1))
			dnt900_schedule_work(ldisc, annc + 1, dnt900_refresh_device);
		else
			dnt900_schedule_work(ldisc, annc + 1, dnt900_add_new_device);
		break;
	case ANNOUNCEMENT_REMOTE_EXITED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"radio exited:\n" \
			"    MAC address 0x%02X%02X%02X\n", \
			annc[3], annc[2], annc[1]);
		break;
	case ANNOUNCEMENT_BASE_REBOOTED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"base rebooted\n");
		break;
	case ANNOUNCEMENT_HEARTBEAT:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"received radio heartbeat:\n" \
			"    MAC address 0x%02X%02X%02X\n" \
			"    network address 0x%02X\n" \
			"    network ID 0x%02X\n" \
			"    parent network ID 0x%02X\n" \
			"    received RSSI %ddBm\n" \
			"    reported RSSI %ddBm\n" \
			"    packet success rate %d%%\n" \
			"    range %d km\n", \
			annc[3], annc[2], annc[1], annc[4], annc[5], annc[6], \
			(signed char)annc[7], (signed char)annc[9], \
			PACKET_SUCCESS_RATE(annc[8]), RANGE_TO_KMS(annc[10]));
		char sys_address[] = { annc[4], annc[5], 0xFF };
		TRY(dnt900_dispatch_to_device(ldisc, annc + 1, dnt900_device_matches_mac_address, sys_address, dnt900_set_sys_address));
		break;
	case ANNOUNCEMENT_HEARTBEAT_TIMEOUT:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"router heartbeat timed out:\n" \
			"    network ID 0x%02X\n",
			annc[1]);
		break;
	case ERROR_PROTOCOL_TYPE:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), "protocol error: invalid message type\n");
		break;
	case ERROR_PROTOCOL_ARGUMENT:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), "protocol error: invalid argument\n");
		break;
	case ERROR_PROTOCOL_GENERAL:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), "protocol error: general error\n");
		break;
	case ERROR_PROTOCOL_TIMEOUT:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), "protocol error: parser timeout\n");
		break;
	case ERROR_PROTOCOL_READONLY:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), "protocol error: register is read-only\n");
		break;
	case ERROR_UART_OVERFLOW:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), "UART error: receive buffer overflow\n");
		break;
	case ERROR_UART_OVERRUN:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), "UART error: receive overrun\n");
		break;
	case ERROR_UART_FRAMING:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), "UART error: framing error\n");
		break;
	case ERROR_HARDWARE:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), "hardware error\n");
		break;
	default:
		return 0;
	}
	return dnt900_receive_data(device, &rxdata);
}

static void dnt900_schedule_work(struct dnt900_ldisc *ldisc, const char *mac_address, void (*work_function)(struct work_struct *))
{
	if (!down_read_trylock(&ldisc->shutdown_lock))
		return;
	struct dnt900_work *work = kzalloc(sizeof(*work), GFP_KERNEL);
	work->ldisc = ldisc;
	if (mac_address)
		COPY3(work->mac_address, mac_address);
	INIT_WORK(&work->ws, work_function);
	queue_work(ldisc->workqueue, &work->ws);
	up_read(&ldisc->shutdown_lock);
}

static void dnt900_add_new_device(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);
	dnt900_alloc_device_remote(work->ldisc, work->mac_address);
	kfree((void *)work);
}

static void dnt900_refresh_ldisc(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);
	dnt900_ldisc_get_params(work->ldisc);
	dnt900_for_each_device(work->ldisc, dnt900_device_get_params);
	kfree((void *)work);
}

static void dnt900_refresh_device(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);
	dnt900_dispatch_to_device_no_data(work->ldisc, work->mac_address, dnt900_device_matches_mac_address, dnt900_device_get_params);
	kfree((void *)work);
}

static void dnt900_init_ldisc(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);
	int error;
	UNWIND(error, dnt900_enter_protocol_mode(work->ldisc), fail);
	UNWIND(error, dnt900_ldisc_get_params(work->ldisc), fail);
	char mac_address[3];
	UNWIND(error, dnt900_get_register(work->ldisc, &dnt900_attributes[MacAddress].reg, mac_address), fail);
	UNWIND(error, dnt900_alloc_device_local(work->ldisc, mac_address), fail);
	goto success;
	
fail:
	pr_err("could not connect to dnt900 module on %s (error %d)\n", work->ldisc->tty->name, -error);
success:
	kfree((void *)work);
}

static irqreturn_t dnt900_cts_handler(int irq, void *dev_id)
{
	struct dnt900_ldisc *ldisc = dev_id;
	gpio_get_value(ldisc->gpio_cts) ? stop_tty(ldisc->tty) : start_tty(ldisc->tty);
	return IRQ_HANDLED;
}

static int dnt900_ldisc_open(struct tty_struct *tty)
{
	int error = 0;
	
	if (!tty->ops->write || ! tty->ops->flush_chars)
		return -EPERM;
	
	struct dnt900_ldisc *ldisc = kzalloc(sizeof(*ldisc), GFP_KERNEL);
	if (!ldisc)
		return -ENOMEM;
	
	ldisc->tty = tty_kref_get(tty);
	if (!ldisc->tty) {
		error = -EPERM;
		goto fail_tty_get;
	}
	tty->disc_data = ldisc;
	
	mutex_init(&ldisc->packets_lock);
	mutex_init(&ldisc->devices_lock);
	mutex_init(&ldisc->param_lock);
	mutex_init(&ldisc->tty_lock);
	init_rwsem(&ldisc->shutdown_lock);
	INIT_LIST_HEAD(&ldisc->packets);
	
	ldisc->workqueue = create_singlethread_workqueue(DRIVER_NAME);
	if (!ldisc->workqueue) {
		error = -ENOMEM;
		goto fail_workqueue;
	}

	dev_t devt;
	UNWIND(error, alloc_chrdev_region(&devt, 0, members, DRIVER_NAME), fail_alloc_chrdev_region);
	ldisc->major = MAJOR(devt);
	ldisc->minor = MINOR(devt);
	
	ldisc->gpio_cfg = gpio_cfg; // how to make this per-instance?
	if (ldisc->gpio_cfg >= 0)
		UNWIND(error, gpio_request_one(ldisc->gpio_cfg, GPIOF_OUT_INIT_HIGH, "/cfg"), fail_gpio_cfg);
	
	ldisc->gpio_cts = gpio_cts; // how to make this per-instance?
	if (ldisc->gpio_cts >= 0) {
		UNWIND(error, gpio_request_one(ldisc->gpio_cts, GPIOF_IN, "/host_cts"), fail_gpio_cts);
		UNWIND(error, request_irq(gpio_to_irq(ldisc->gpio_cts), dnt900_cts_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, DRIVER_NAME, ldisc), fail_irq_cts);
	}
	
	dnt900_schedule_work(ldisc, NULL, dnt900_init_ldisc);
	goto success;
	
fail_irq_cts:
	if (ldisc->gpio_cts >= 0)
		gpio_free(ldisc->gpio_cts);
fail_gpio_cts:
	if (ldisc->gpio_cfg >= 0)
		gpio_free(ldisc->gpio_cfg);
fail_gpio_cfg:
	tty->disc_data = NULL;
	unregister_chrdev_region(MKDEV(ldisc->major, 0), members);
fail_alloc_chrdev_region:
	destroy_workqueue(ldisc->workqueue);
fail_workqueue:
	tty_kref_put(ldisc->tty);
fail_tty_get:
	kfree(ldisc);
success:
	return error;
}

static void dnt900_ldisc_close(struct tty_struct *tty)
{
	struct dnt900_ldisc *ldisc = tty->disc_data;

	// TODO: code to sleep the dnt900 here?
	down_write(&ldisc->shutdown_lock);
	destroy_workqueue(ldisc->workqueue);
	dnt900_free_devices(ldisc);
	dnt900_exit_protocol_mode(ldisc);
	if (ldisc->gpio_cts >= 0)
		free_irq(gpio_to_irq(ldisc->gpio_cts), ldisc);
	if (ldisc->gpio_cts >= 0)
		gpio_free(ldisc->gpio_cts);
	if (ldisc->gpio_cfg >= 0)
		gpio_free(ldisc->gpio_cfg);
	tty->disc_data = NULL;
	unregister_chrdev_region(MKDEV(ldisc->major, 0), members);
	tty_kref_put(ldisc->tty);
	kfree(ldisc);
}

int __init dnt900_init(void)
{
	dnt900_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(dnt900_class))
		return PTR_ERR(dnt900_class);
	int error = 0;
	dnt900_ldisc_ops.num = n_dnt900;
	UNWIND(error, tty_register_ldisc(n_dnt900, &dnt900_ldisc_ops), fail_register);
	pr_info("inserting dnt900 module\n");
	return 0;
	
fail_register:
	class_destroy(dnt900_class);
	return error;
}

void __exit dnt900_exit(void)
{
	pr_info("removing dnt900 module\n");
	tty_unregister_ldisc(n_dnt900);
	class_destroy(dnt900_class);
}

module_init(dnt900_init);
module_exit(dnt900_exit);

MODULE_AUTHOR("Matthew Hollingworth");
MODULE_DESCRIPTION("driver for DNT900 RF module");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

// TODO: use dev_error() etc
// TODO: fix error where module unloading breaks if any of the char devices are still open
// TODO: move tty.local char device functions into the tty char device itself. would requre
//       dnt900_for_each_device to be changed to dnt900_for_each_remote, etc.
