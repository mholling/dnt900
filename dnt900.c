#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/printk.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/list.h>
#include <linux/spinlock.h>
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

#define MAX_PACKET_SIZE (2+255)

// buffer sizes must be powers of 2
#define DRIVER_BUF_SIZE (512)
#define RX_BUF_SIZE     (2048)

#define CIRC_INDEX(index, offset, size) (((index) + (offset)) & ((size) - 1))
#define CIRC_OFFSET(index, offset, size) (index) = (((index) + (offset)) & ((size) - 1))

#define START_OF_PACKET (0xFB)

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

#define RANGE_TO_METRES(range) (46671 * (unsigned char)(range) / 100)
#define PACKET_SUCCESS_RATE(attempts_x4) (400 / (unsigned char)(attempts_x4))

#define TRY(function_call) do { \
	int error = function_call; \
	if (error) \
		return error; \
} while (0)
	
#define UNWIND(error, function_call, exit) do { \
	error = function_call; \
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
	struct spi_transfer transfer;
	struct spi_message message;
	struct list_head list;
	struct completion completed;
	char *result;
	int error;
	void *payload;
	int expects_reply;
};

struct dnt900_driver {
	struct spi_device *spi;
	int major;
	int minor;
	unsigned gpio_cfg;
	unsigned gpio_avl;
	unsigned gpio_cts;
	struct list_head queued_packets;
	struct list_head sent_packets;
	spinlock_t queue_lock;
	struct mutex devices_lock;
	struct mutex param_lock;
	struct rw_semaphore shutdown_lock;
	char buf[DRIVER_BUF_SIZE + MAX_PACKET_SIZE];
	unsigned int head;
	unsigned int tail;
	struct dnt900_packet empty_packet;
	struct workqueue_struct *workqueue;
	int use_tree_routing;
	unsigned int slot_size;
};

struct dnt900_device {
	struct cdev cdev;
	struct dnt900_driver *driver;
	int is_local;
	struct mutex buf_lock;
	struct mutex param_lock;
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
	struct dnt900_driver *driver;
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

static int dnt900_get_register(struct dnt900_driver *driver, const struct dnt900_register *reg, char *value);
static int dnt900_get_remote_register(struct dnt900_driver *driver, const char *sys_address, const struct dnt900_register *reg, char *value);
static int dnt900_set_register(struct dnt900_driver *driver, const struct dnt900_register *reg, const char *value);
static int dnt900_set_remote_register(struct dnt900_driver *driver, const char *sys_address, const struct dnt900_register *reg, const char *value);
static int dnt900_discover(struct dnt900_driver *driver, const char *mac_address, char *sys_address);

static int dnt900_read_sys_address(struct dnt900_device *device, char *sys_address);
static int dnt900_read_slot_size(struct dnt900_driver *driver, unsigned int *slot_size);
static int dnt900_read_use_tree_routing(struct dnt900_driver *driver, int *use_tree_routing);

static int dnt900_get_device_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value);
static int dnt900_set_device_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value);

static ssize_t dnt900_show_attr(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t dnt900_store_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t dnt900_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t dnt900_store_discover(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static int dnt900_open(struct inode *inode, struct file *filp);
static ssize_t dnt900_read(struct file *filp, char __user *buf, size_t length, loff_t *offp);
static ssize_t dnt900_write(struct file *filp, const char __user *buf, size_t length, loff_t *offp);

static int dnt900_get_driver_params(struct dnt900_driver *driver);
static int dnt900_get_device_params(struct dnt900_device *device);

static int dnt900_alloc_device(struct dnt900_driver *driver, const char *mac_address, int is_local, const char *name);
static int dnt900_alloc_device_remote(struct dnt900_driver *driver, const char *mac_address);
static int dnt900_alloc_device_local(struct dnt900_driver *driver, const char *mac_address);
static int dnt900_free_device(struct device *dev, void *unused);
static void dnt900_free_devices(struct dnt900_driver *driver);

static int dnt900_device_matches_sys_address(struct device *dev, void *data);
static int dnt900_device_matches_mac_address(struct device *dev, void *data);
static int dnt900_device_is_local(struct device *dev, void *data);

static int dnt900_device_exists(struct dnt900_driver *driver, const char *mac_address);
static int dnt900_dispatch_to_device(struct dnt900_driver *driver, void *finder_data, int (*finder)(struct device *, void *), void *action_data, int (*action)(struct dnt900_device *, void *));
static int dnt900_dispatch_to_device_no_data(struct dnt900_driver *driver, void *finder_data, int (*finder)(struct device *, void *), int (*action)(struct dnt900_device *));
static int dnt900_apply_to_device(struct device *dev, void *data);
static int dnt900_for_each_device(struct dnt900_driver *driver, int (*action)(struct dnt900_device *));

static void dnt900_init_packet(struct dnt900_driver *driver, struct dnt900_packet *packet, const void *tx_buf, unsigned int len, int expects_reply, char *result);
static int dnt900_send_message(struct dnt900_driver *driver, void *payload, unsigned int arg_length, char type, int expects_reply, char *result);
static void dnt900_send_queued_packets(struct dnt900_driver *driver);
static void dnt900_complete_packet(void *context);

static void dnt900_process_reply(struct dnt900_driver *driver, char command);
static int dnt900_receive_data(struct dnt900_device *device, void *data);
static void dnt900_process_event(struct dnt900_driver *driver, char event, unsigned int length);
static int dnt900_process_announcement(struct dnt900_device *device, void *data);

static void dnt900_schedule_work(struct dnt900_driver *driver, const char *mac_address, void (*work_function)(struct work_struct *));
static void dnt900_add_new_device(struct work_struct *ws);
static void dnt900_refresh_driver(struct work_struct *ws);
static void dnt900_refresh_device(struct work_struct *ws);
static int dnt900_set_sys_address(struct dnt900_device *device, void *data);

static irqreturn_t dnt900_avl_handler(int irq, void *dev_id);
static irqreturn_t dnt900_cts_handler(int irq, void *dev_id);
static int dnt900_probe(struct spi_device *spi);
static int dnt900_remove(struct spi_device *spi);

int __init dnt900_init(void);
void __exit dnt900_exit(void);

static int members = 126;
static int spi_hz = 80640;
static int gpio_cfg = 25;
static int gpio_avl = 4;
static int gpio_cts = 27;

module_param(spi_hz, int, S_IRUGO);
MODULE_PARM_DESC(spi_hz, "SPI clock frequency in Hertz");
module_param(gpio_cfg, int, S_IRUGO);
MODULE_PARM_DESC(gpio_cfg, "GPIO number for /CFG signal");
module_param(gpio_avl, int, S_IRUGO);
MODULE_PARM_DESC(gpio_avl, "GPIO number for SPI_RX_AVL signal");
module_param(gpio_cts, int, S_IRUGO);
MODULE_PARM_DESC(gpio_cts, "GPIO number for /HOST_CTS signal");

static const char driver_name[] = "dnt900";
static const char class_name[] = "dnt900";
static const char local_name[] = "dnt900.local";
static const char remote_name_template[] = "dnt900.0x%02X%02X%02X";

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

static struct file_operations dnt900_fops = {
	.owner = THIS_MODULE,
	.open = dnt900_open,
	.read = dnt900_read,
	.write = dnt900_write
};

static struct spi_driver dnt900_spi_driver = {
	.driver = {
		.name = driver_name,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = dnt900_probe,
	.remove = __devexit_p(dnt900_remove),
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

static int dnt900_get_register(struct dnt900_driver *driver, const struct dnt900_register *reg, char *value)
{
	struct dnt900_get_register_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	message->bank = reg->bank;
	message->reg = reg->offset;
	message->span = reg->span;
	int error = dnt900_send_message(driver, message, 3, COMMAND_GET_REGISTER, true, value);
	kfree(message);
	return error;
}

static int dnt900_get_remote_register(struct dnt900_driver *driver, const char *sys_address, const struct dnt900_register *reg, char *value)
{
	struct dnt900_get_remote_register_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	COPY3(message->sys_address, sys_address);
	message->bank = reg->bank;
	message->reg = reg->offset;
	message->span = reg->span;
	int error = dnt900_send_message(driver, message, 6, COMMAND_GET_REMOTE_REGISTER, true, value);
	kfree(message);
	return error;
}

static int dnt900_set_register(struct dnt900_driver *driver, const struct dnt900_register *reg, const char *value)
{
	struct dnt900_set_register_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	message->bank = reg->bank;
	message->reg = reg->offset;
	message->span = reg->span;
	memcpy(&message->value, value, reg->span);
	int expects_reply = reg->bank != 0xFF || (reg->offset != 0x00 && (reg->offset != 0xFF || *value != 0x02));
	int error = dnt900_send_message(driver, message, 3 + reg->span, COMMAND_SET_REGISTER, expects_reply, NULL);
	kfree(message);
	return error;
}

static int dnt900_set_remote_register(struct dnt900_driver *driver, const char *sys_address, const struct dnt900_register *reg, const char *value)
{
	struct dnt900_set_remote_register_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	COPY3(message->sys_address, sys_address);
	message->bank = reg->bank;
	message->reg = reg->offset;
	message->span = reg->span;
	memcpy(&message->value, value, reg->span);
	int expects_reply = reg->bank != 0xFF || (reg->offset != 0x00 && (reg->offset != 0xFF || *value != 0x02));
	int error = dnt900_send_message(driver, message, 6 + reg->span, COMMAND_SET_REMOTE_REGISTER, expects_reply, NULL);
	kfree(message);
	return error;
}

static int dnt900_discover(struct dnt900_driver *driver, const char *mac_address, char *sys_address)
{
	int use_tree_routing;
	TRY(dnt900_read_use_tree_routing(driver, &use_tree_routing));
	if (use_tree_routing) {
		struct dnt900_discover_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
		if (!message)
			return -ENOMEM;
		COPY3(message->mac_address, mac_address);
		int error = dnt900_send_message(driver, message, 3, COMMAND_DISCOVER, true, sys_address);
		kfree(message);
		return error ? -ENODEV : 0;
	} else {
		COPY3(sys_address, mac_address);
		return 0;
	}
}

static int dnt900_read_sys_address(struct dnt900_device *device, char *sys_address)
{
	TRY(mutex_lock_interruptible(&device->param_lock));
	COPY3(sys_address, device->sys_address);
	mutex_unlock(&device->param_lock);
	return 0;
}

static int dnt900_read_slot_size(struct dnt900_driver *driver, unsigned int *slot_size)
{
	// TODO: Is this expensive (given that it is called for every TxData packet)?
	TRY(mutex_lock_interruptible(&driver->param_lock));
	*slot_size = driver->slot_size;
	mutex_unlock(&driver->param_lock);
	return 0;
}

static int dnt900_read_use_tree_routing(struct dnt900_driver *driver, int *use_tree_routing)
{
	TRY(mutex_lock_interruptible(&driver->param_lock));
	*use_tree_routing = driver->use_tree_routing;
	mutex_unlock(&driver->param_lock);
	return 0;
}

static int dnt900_get_device_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value)
{
	if (device->is_local)
		return dnt900_get_register(device->driver, reg, value);
	char sys_address[3];
	TRY(dnt900_read_sys_address(device, sys_address));
	return dnt900_get_remote_register(device->driver, sys_address, reg, value);
}

static int dnt900_set_device_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value)
{
	if (device->is_local)
		return dnt900_set_register(device->driver, reg, value);
	char sys_address[3];
	TRY(dnt900_read_sys_address(device, sys_address));
	return dnt900_set_remote_register(device->driver, sys_address, reg, value);
}

static ssize_t dnt900_show_attr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	struct dnt900_attribute *attribute = container_of(attr, struct dnt900_attribute, attr);
	char value[32];
	int error = dnt900_get_device_register(device, &attribute->reg, value);
	return error < 0 ? error : attribute->print(value, buf);
}

static ssize_t dnt900_store_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	struct dnt900_attribute *attribute = container_of(attr, struct dnt900_attribute, attr);
	char value[32];
	TRY(attribute->parse(buf, count, value));
	TRY(dnt900_set_device_register(device, &attribute->reg, value));
	return count;
}

static ssize_t dnt900_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dnt900_device *device = dev_get_drvdata(dev);
	struct dnt900_software_reset_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	message->boot_select = 0;
	int error = dnt900_send_message(device->driver, message, 1, COMMAND_SOFTWARE_RESET, true, NULL);
	kfree(message);
	return error ? error : count;
}

static ssize_t dnt900_store_discover(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int mac_address_int;
	char mac_address[3];
	struct dnt900_device *device = dev_get_drvdata(dev);
	struct dnt900_driver *driver = device->driver;
	TRY(kstrtouint(buf, 16, &mac_address_int));
	for (int n = 0; n < 3; ++n, mac_address_int >>= 8)
		mac_address[n] = mac_address_int & 0xFF;
	if (mac_address_int)
		return -EINVAL;
	TRY(dnt900_alloc_device_remote(driver, mac_address));
	return count;
}

static int dnt900_open(struct inode *inode, struct file *filp)
{
	struct dnt900_device *device = container_of(inode->i_cdev, struct dnt900_device, cdev);
	filp->private_data = device;
	return 0;
}

static ssize_t dnt900_read(struct file *filp, char __user *buf, size_t length, loff_t *offp)
{
	struct dnt900_device *device = filp->private_data;
	
	if (!mutex_trylock(&device->buf_lock))
		return 0;
	unsigned int available = (device->head > device->tail ? device->head : RX_BUF_SIZE) - device->tail;
	unsigned int count = available > length ? length : available;
	unsigned int copied = count - copy_to_user(buf, device->buf + device->tail, count);
	CIRC_OFFSET(device->tail, count, RX_BUF_SIZE);
	mutex_unlock(&device->buf_lock);
	
	if (count && !copied)
		return -EFAULT;
	*offp += copied;
	return copied;
}

static ssize_t dnt900_write(struct file *filp, const char __user *buf, size_t length, loff_t *offp)
{
	struct dnt900_device *device = filp->private_data;
	
	if (device->is_local)
		return -EPERM;
	struct dnt900_tx_data_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	unsigned int copied = 0;
	int error;
	UNWIND(error, dnt900_read_sys_address(device, message->sys_address), fail);
	
	unsigned int slot_size;
	UNWIND(error, dnt900_read_slot_size(device->driver, &slot_size), fail);
	unsigned int count = slot_size - 6 < length ? slot_size - 6 : length;
	copied = count - copy_from_user(message->data, buf, count);
	error = count && !copied ? -EFAULT : dnt900_send_message(device->driver, message, 3 + copied, COMMAND_TX_DATA, true, NULL);
fail:
	kfree(message);
	*offp += copied;
	return error ? error : copied;
}

static int dnt900_get_driver_params(struct dnt900_driver *driver)
{
	char announce_options, protocol_options, auth_mode, device_mode, slot_size, tree_routing_en;
	TRY(dnt900_get_register(driver, &dnt900_attributes[AnnounceOptions].reg, &announce_options));
	TRY(dnt900_get_register(driver, &dnt900_attributes[ProtocolOptions].reg, &protocol_options));
	TRY(dnt900_get_register(driver, &dnt900_attributes[AuthMode].reg, &auth_mode));
	if ((announce_options & 0x03) != 0x03)
		pr_err("set radio AnnounceOptions register to 0x07 for correct driver operation\n");
	if ((protocol_options & 0x05) != 0x05)
		pr_err("set radio ProtocolOptions register to 0x05 for correct driver operation\n");
	if (auth_mode == 0x02)
		pr_warn("AuthMode register is set to 0x02 but host-based authentication is not supported\n");
	TRY(dnt900_get_register(driver, &dnt900_attributes[TreeRoutingEn].reg, &tree_routing_en));
	TRY(dnt900_get_register(driver, &dnt900_attributes[DeviceMode].reg, &device_mode));
	TRY(dnt900_get_register(driver, &dnt900_attributes[device_mode == 0x01 ? BaseSlotSize : RemoteSlotSize].reg, &slot_size));
	TRY(mutex_lock_interruptible(&driver->param_lock));
	driver->slot_size = (unsigned char)slot_size;
	driver->use_tree_routing = tree_routing_en == 0x01;
	mutex_unlock(&driver->param_lock);
	return 0;
}

static int dnt900_get_device_params(struct dnt900_device *device)
{
	if (device->is_local)
		return 0;
	char sys_address[3];
	TRY(dnt900_discover(device->driver, device->mac_address, sys_address));
	TRY(mutex_lock_interruptible(&device->param_lock));
	COPY3(device->sys_address, sys_address);
	mutex_unlock(&device->param_lock);
	return 0;
}

static int dnt900_alloc_device(struct dnt900_driver *driver, const char *mac_address, int is_local, const char *name)
{
	TRY(mutex_lock_interruptible(&driver->devices_lock));
	int error = 0;
	if (dnt900_device_exists(driver, mac_address)) {
		error = -EEXIST;
		goto fail_exists;
	}
	struct dnt900_device *device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		error = -ENOMEM;
		goto fail_alloc;
	}
	device->driver = driver;
	device->is_local = is_local;
	COPY3(device->mac_address, mac_address);
	mutex_init(&device->buf_lock);
	mutex_init(&device->param_lock);
	UNWIND(error, dnt900_get_device_params(device), fail_get_params);
	dev_t devt = MKDEV(driver->major, driver->minor++);
	struct device *dev = device_create(dnt900_class, &driver->spi->dev, devt, device, name);
	if (IS_ERR(dev)) {
		error = PTR_ERR(dev);
		goto fail_dev_create;
	}
	UNWIND(error, dnt900_add_attributes(dev), fail_add_attributes);
	cdev_init(&device->cdev, &dnt900_fops);
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
	mutex_unlock(&driver->devices_lock);
	return error;
}

static int dnt900_alloc_device_remote(struct dnt900_driver *driver, const char *mac_address)
{
	char name[32];
	snprintf(name, ARRAY_SIZE(name), remote_name_template, mac_address[2], mac_address[1], mac_address[0]);
	return dnt900_alloc_device(driver, mac_address, false, name);
}

static int dnt900_alloc_device_local(struct dnt900_driver *driver, const char *mac_address)
{
	return dnt900_alloc_device(driver, mac_address, true, local_name);
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

static void dnt900_free_devices(struct dnt900_driver *driver)
{
	device_for_each_child(&driver->spi->dev, NULL, dnt900_free_device);
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

static int dnt900_device_exists(struct dnt900_driver *driver, const char *mac_address)
{
	return device_for_each_child(&driver->spi->dev, (void *)mac_address, dnt900_device_matches_mac_address);
}

static int dnt900_dispatch_to_device(struct dnt900_driver *driver, void *finder_data, int (*finder)(struct device *, void *), void *action_data, int (*action)(struct dnt900_device *, void *))
{
	int error;
	struct device *dev = device_find_child(&driver->spi->dev, finder_data, finder);
	if (dev) {
		struct dnt900_device *device = dev_get_drvdata(dev);
		error = action(device, action_data);
		put_device(dev);
	} else 
		error = -ENODEV;
	return error;
}

static int dnt900_dispatch_to_device_no_data(struct dnt900_driver *driver, void *finder_data, int (*finder)(struct device *, void *), int (*action)(struct dnt900_device *))
{
	int error;
	struct device *dev = device_find_child(&driver->spi->dev, finder_data, finder);
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

static int dnt900_for_each_device(struct dnt900_driver *driver, int (*action)(struct dnt900_device *))
{
	return device_for_each_child(&driver->spi->dev, action, dnt900_apply_to_device);
}

static void dnt900_init_packet(struct dnt900_driver *driver, struct dnt900_packet *packet, const void *tx_buf, unsigned int len, int expects_reply, char *result)
{
	INIT_LIST_HEAD(&packet->list);
	init_completion(&packet->completed);
	packet->result = result;
	packet->error = 0;
	packet->expects_reply = expects_reply;
	INIT_LIST_HEAD(&packet->message.transfers);
	packet->message.context = driver;
	packet->message.complete = dnt900_complete_packet;
	packet->transfer.len = len;
	packet->transfer.tx_buf = tx_buf;
	spi_message_add_tail(&packet->transfer, &packet->message);
}

static int dnt900_send_message(struct dnt900_driver *driver, void *payload, unsigned int arg_length, char type, int expects_reply, char *result)
{
	unsigned long flags;
	struct dnt900_packet packet;
	struct dnt900_message_header *header = payload;
	
	dnt900_init_packet(driver, &packet, payload, 3 + arg_length, expects_reply, result);
	header->start_of_packet = START_OF_PACKET;
	header->length = 1 + arg_length;
	header->type = type;
	
	spin_lock_irqsave(&driver->queue_lock, flags);
	list_add_tail(&packet.list, &driver->queued_packets);
	spin_unlock_irqrestore(&driver->queue_lock, flags);
	dnt900_send_queued_packets(driver);
	// TODO: use wait_for_completion_interruptible_timeout to timeout if taking too long?
	int interrupted = wait_for_completion_interruptible(&packet.completed);
	if (interrupted) {
		spin_lock_irqsave(&driver->queue_lock, flags);
		list_del(&packet.list);
		spin_unlock_irqrestore(&driver->queue_lock, flags);
		return interrupted;
	}
	return packet.error;
}

static void dnt900_send_queued_packets(struct dnt900_driver *driver)
{
	unsigned long flags;
	
	// TODO: reduce time spent with spin lock...
	spin_lock_irqsave(&driver->queue_lock, flags);
	if (gpio_get_value(driver->gpio_avl) && (list_empty(&driver->queued_packets) || gpio_get_value(driver->gpio_cts)))
		list_add(&driver->empty_packet.list, &driver->queued_packets);
	if (!list_empty(&driver->queued_packets)) {
		struct dnt900_packet *packet = list_first_entry(&driver->queued_packets, struct dnt900_packet, list);
		// spinlock for accessing driver->head ??
		packet->transfer.rx_buf = driver->buf + driver->head;
		CIRC_OFFSET(driver->head, packet->transfer.len, DRIVER_BUF_SIZE);
		spi_async(driver->spi, &packet->message);
	}
	spin_unlock_irqrestore(&driver->queue_lock, flags);
}

static void dnt900_complete_packet(void *context)
{
	struct dnt900_driver *driver = context;
	unsigned long flags;
	
	spin_lock_irqsave(&driver->queue_lock, flags);
	struct dnt900_packet *packet = list_first_entry(&driver->queued_packets, struct dnt900_packet, list);
	if (!packet->message.status && packet->transfer.tx_buf && packet->expects_reply)
		list_move_tail(&packet->list, &driver->sent_packets);
	else
		list_del(&packet->list);
	spin_unlock_irqrestore(&driver->queue_lock, flags);
	
	char * const buf_end = driver->buf + DRIVER_BUF_SIZE;
	char * const transfer_end = packet->transfer.rx_buf + packet->transfer.len;
	if (transfer_end > buf_end)
		memcpy(driver->buf, buf_end, transfer_end - buf_end);
	
	if (packet->message.status || !packet->expects_reply) {
		packet->error = packet->message.status;
		complete(&packet->completed);
	}
	
	// pr_info("scanning received data...\n");
	// while (driver->head != driver->tail) {
	// 	pr_info("[%03i]: %02x\n", driver->tail, driver->buf[driver->tail]);
	// 	CIRC_OFFSET(driver->tail, 1, DRIVER_BUF_SIZE);
	// }
	
	while (driver->head != driver->tail) {
		for (; driver->tail != driver->head && driver->buf[driver->tail] != START_OF_PACKET; CIRC_OFFSET(driver->tail, 1, DRIVER_BUF_SIZE))
			;
		if (driver->head == driver->tail)
			break;
		
		const unsigned int remaining = CIRC_INDEX(DRIVER_BUF_SIZE + driver->head - driver->tail, 0, DRIVER_BUF_SIZE);
		const unsigned int length = driver->buf[CIRC_INDEX(driver->tail, 1, DRIVER_BUF_SIZE)] + 2;
		if (remaining < 2 || length >= remaining)
			break;

		const char type = driver->buf[CIRC_INDEX(driver->tail, 2, DRIVER_BUF_SIZE)];
		if (type & TYPE_REPLY)
			dnt900_process_reply(driver, type & MASK_COMMAND);
		if (type & TYPE_EVENT)
			dnt900_process_event(driver, type, length);
		
		CIRC_OFFSET(driver->tail, length, DRIVER_BUF_SIZE);
	}
	
	dnt900_send_queued_packets(driver);
}

static void dnt900_process_reply(struct dnt900_driver *driver, char command)
{
	struct dnt900_packet *packet;
	list_for_each_entry(packet, &driver->sent_packets, list) {
		const char * const buf = packet->transfer.tx_buf;
		char tx_status = 0;
		if (buf[2] != command)
			continue;
		switch (command) {
		case COMMAND_SOFTWARE_RESET:
			break;
		case COMMAND_GET_REGISTER:
			// match Reg, Bank, span:
			if (buf[3] != driver->buf[CIRC_INDEX(driver->tail, 3, DRIVER_BUF_SIZE)] 
			 || buf[4] != driver->buf[CIRC_INDEX(driver->tail, 4, DRIVER_BUF_SIZE)] 
			 || buf[5] != driver->buf[CIRC_INDEX(driver->tail, 5, DRIVER_BUF_SIZE)])
				continue;
			// TODO: check buf[5] + 6 == length?
			for (int n = 0; n < buf[5]; ++n)
				packet->result[n] = driver->buf[CIRC_INDEX(driver->tail, 6 + n, DRIVER_BUF_SIZE)];
			break;
		case COMMAND_SET_REGISTER:
			break;
		case COMMAND_TX_DATA:
			// match Addr:
			if (buf[3] != driver->buf[CIRC_INDEX(driver->tail, 4, DRIVER_BUF_SIZE)] 
			 || buf[4] != driver->buf[CIRC_INDEX(driver->tail, 5, DRIVER_BUF_SIZE)] 
			 || buf[5] != driver->buf[CIRC_INDEX(driver->tail, 6, DRIVER_BUF_SIZE)])
				continue;
			tx_status = driver->buf[CIRC_INDEX(driver->tail, 3, DRIVER_BUF_SIZE)];
			break;
		case COMMAND_DISCOVER:
			// match MacAddr:
			if (buf[3] != driver->buf[CIRC_INDEX(driver->tail, 4, DRIVER_BUF_SIZE)] 
			 || buf[4] != driver->buf[CIRC_INDEX(driver->tail, 5, DRIVER_BUF_SIZE)] 
			 || buf[5] != driver->buf[CIRC_INDEX(driver->tail, 6, DRIVER_BUF_SIZE)])
				continue;
			tx_status = driver->buf[CIRC_INDEX(driver->tail, 3, DRIVER_BUF_SIZE)];
			if (tx_status == STATUS_ACKNOWLEDGED)
				for (int n = 0; n < 3; ++n)
					packet->result[n] = driver->buf[CIRC_INDEX(driver->tail, 7 + n, DRIVER_BUF_SIZE)];
			break;
		case COMMAND_GET_REMOTE_REGISTER:
			// match Addr:
			if (buf[3] != driver->buf[CIRC_INDEX(driver->tail, 4, DRIVER_BUF_SIZE)] 
			 || buf[4] != driver->buf[CIRC_INDEX(driver->tail, 5, DRIVER_BUF_SIZE)] 
			 || buf[5] != driver->buf[CIRC_INDEX(driver->tail, 6, DRIVER_BUF_SIZE)])
				continue;
			if (driver->buf[CIRC_INDEX(driver->tail, 3, DRIVER_BUF_SIZE)] == STATUS_ACKNOWLEDGED) {
				// match Reg, Bank, span:
				if (buf[6] != driver->buf[CIRC_INDEX(driver->tail, 8, DRIVER_BUF_SIZE)] 
				 || buf[7] != driver->buf[CIRC_INDEX(driver->tail, 9, DRIVER_BUF_SIZE)] 
				 || buf[8] != driver->buf[CIRC_INDEX(driver->tail, 10, DRIVER_BUF_SIZE)])
					continue;
				for (int n = 0; n < buf[8]; ++n)
					packet->result[n] = driver->buf[CIRC_INDEX(driver->tail, 11 + n, DRIVER_BUF_SIZE)];
				tx_status = STATUS_ACKNOWLEDGED;
			} else
				tx_status = driver->buf[CIRC_INDEX(driver->tail, 3, DRIVER_BUF_SIZE)];
			break;
		case COMMAND_SET_REMOTE_REGISTER:
			// match Addr:
			if (buf[3] != driver->buf[CIRC_INDEX(driver->tail, 4, DRIVER_BUF_SIZE)] 
			 || buf[4] != driver->buf[CIRC_INDEX(driver->tail, 5, DRIVER_BUF_SIZE)] 
			 || buf[5] != driver->buf[CIRC_INDEX(driver->tail, 6, DRIVER_BUF_SIZE)])
				continue;
			tx_status = driver->buf[CIRC_INDEX(driver->tail, 3, DRIVER_BUF_SIZE)];
			break;
		}
		switch (tx_status) {
		case STATUS_ACKNOWLEDGED:
			packet->error = 0;
		case STATUS_HOLDING_FOR_FLOW:
			packet->error = -ETIMEDOUT;
		case STATUS_NOT_ACKNOWLEDGED:
		case STATUS_NOT_LINKED:
		default:
			packet->error = -ECOMM;
		}
		list_del(&packet->list);
		complete(&packet->completed);
		break;
	}
}

static void dnt900_process_event(struct dnt900_driver *driver, char event, unsigned int length)
{
	char sys_address[3];
	unsigned int start, end;
	struct dnt900_rxdata rxdata;
	char announcement[11];
	switch (event) {
	case EVENT_RX_DATA:
		for (int offset = 3; offset < 6; ++offset)
			sys_address[offset-3] = driver->buf[CIRC_INDEX(driver->tail, offset, DRIVER_BUF_SIZE)];
		start = CIRC_INDEX(driver->tail,      7, DRIVER_BUF_SIZE);
		end   = CIRC_INDEX(driver->tail, length, DRIVER_BUF_SIZE);
		
		rxdata.buf = driver->buf + start;
		rxdata.len = (end < start ? DRIVER_BUF_SIZE : end) - start;
		dnt900_dispatch_to_device(driver, sys_address, dnt900_device_matches_sys_address, &rxdata, dnt900_receive_data);
		
		if (end < start) {
			rxdata.buf = driver->buf;
			rxdata.len = end;
			dnt900_dispatch_to_device(driver, sys_address, dnt900_device_matches_sys_address, &rxdata, dnt900_receive_data);
		}
		// TODO: add new device if it doesn't exist? You would have to retrieve its MAC first,
		// and also do this in a workqueue job. The first packets of received data would likely
		// be discarded.
		break;
	case EVENT_ANNOUNCE:
		start = 3;
		end = min(length, start + ARRAY_SIZE(announcement));
		for (int offset = start; offset < end; ++offset)
			announcement[offset-start] = driver->buf[CIRC_INDEX(driver->tail, offset, DRIVER_BUF_SIZE)];
		dnt900_dispatch_to_device(driver, NULL, dnt900_device_is_local, announcement, dnt900_process_announcement);
		break;
	case EVENT_RX_EVENT:
		// unimplemented for now (used for automatic I/O reporting)
		break;
	case EVENT_JOIN_REQUEST:
		// unimplemented for now (used for host-based authentication)
		break;
	}
}

static int dnt900_receive_data(struct dnt900_device *device, void *data)
{
	struct dnt900_rxdata *rxdata = data;
	
	TRY(mutex_lock_interruptible(&device->buf_lock));
	for (; rxdata->len > 0; ++rxdata->buf, --rxdata->len) {
		device->buf[device->head] = *rxdata->buf;
		CIRC_OFFSET(device->head, 1, RX_BUF_SIZE);
		if (device->head == device->tail)
			CIRC_OFFSET(device->tail, 1, RX_BUF_SIZE);
	}
	mutex_unlock(&device->buf_lock);
	return 0;
}

static int dnt900_process_announcement(struct dnt900_device *device, void *data)
{
	char *annc = data;
	char message[512];
	struct dnt900_rxdata rxdata = { .buf = message };
	struct dnt900_driver *driver = device->driver;
	
	switch (annc[0]) {
	case ANNOUNCEMENT_STARTED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"completed startup initialization\n");
		dnt900_schedule_work(driver, device->mac_address, dnt900_refresh_driver);
		break;
	case ANNOUNCEMENT_JOINED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"joined network:\n" \
			"    network ID 0x%02X\n" \
			"    base MAC address 0x%02X%02X%02X\n" \
			"    range %i metres\n", \
			annc[1], annc[4], annc[3], annc[2], RANGE_TO_METRES(annc[5]));
		if (dnt900_device_exists(driver, annc + 2))
			dnt900_schedule_work(driver, annc + 2, dnt900_refresh_device);
		else
			dnt900_schedule_work(driver, annc + 2, dnt900_add_new_device);
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
			"    range %i metres\n", \
			annc[3], annc[2], annc[1], RANGE_TO_METRES(annc[5]));
		if (dnt900_device_exists(driver, annc + 1))
			dnt900_schedule_work(driver, annc + 1, dnt900_refresh_device);
		else
			dnt900_schedule_work(driver, annc + 1, dnt900_add_new_device);
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
			"    received RSSI %idBm\n" \
			"    reported RSSI %idBm\n" \
			"    packet success rate %i%%\n" \
			"    range %i metres\n", \
			annc[3], annc[2], annc[1], annc[4], annc[5], annc[6], \
			(signed char)annc[7], (signed char)annc[9], \
			PACKET_SUCCESS_RATE(annc[8]), RANGE_TO_METRES(annc[10]));
		char sys_address[] = { annc[4], annc[5], 0xFF };
		dnt900_dispatch_to_device(driver, annc + 1, dnt900_device_matches_mac_address, sys_address, dnt900_set_sys_address);
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
	dnt900_receive_data(device, &rxdata);
	return 0;
}

static void dnt900_schedule_work(struct dnt900_driver *driver, const char *mac_address, void (*work_function)(struct work_struct *))
{
	if (!down_read_trylock(&driver->shutdown_lock))
		return;
	struct dnt900_work *work = kmalloc(sizeof(*work), GFP_KERNEL);
	COPY3(work->mac_address, mac_address);
	work->driver = driver;
	INIT_WORK(&work->ws, work_function);
	queue_work(driver->workqueue, &work->ws);
	up_read(&driver->shutdown_lock);
}

static void dnt900_add_new_device(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);
	dnt900_alloc_device_remote(work->driver, work->mac_address);
	kfree((void *)work);
}

static void dnt900_refresh_driver(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);
	dnt900_get_driver_params(work->driver);
	dnt900_for_each_device(work->driver, dnt900_get_device_params);
	kfree((void *)work);
}

static void dnt900_refresh_device(struct work_struct *ws)
{
	struct dnt900_work *work = container_of(ws, struct dnt900_work, ws);
	dnt900_dispatch_to_device_no_data(work->driver, work->mac_address, dnt900_device_matches_mac_address, dnt900_get_device_params);
	kfree((void *)work);
}

static int dnt900_set_sys_address(struct dnt900_device *device, void *data)
{
	const char *sys_address = data;
	TRY(mutex_lock_interruptible(&device->param_lock));
	COPY3(device->sys_address, sys_address);
	mutex_unlock(&device->param_lock);
	return 0;
}

static irqreturn_t dnt900_avl_handler(int irq, void *dev_id)
{
	struct spi_device *spi = dev_id;
	struct dnt900_driver *driver = spi_get_drvdata(spi);
	dnt900_send_queued_packets(driver);
	return IRQ_HANDLED;
}

static irqreturn_t dnt900_cts_handler(int irq, void *dev_id)
{
	struct spi_device *spi = dev_id;
	struct dnt900_driver *driver = spi_get_drvdata(spi);
	dnt900_send_queued_packets(driver);
	return IRQ_HANDLED;
}

static int dnt900_probe(struct spi_device *spi)
{
	int error = 0;

	struct dnt900_driver *driver = kzalloc(sizeof(*driver), GFP_KERNEL);
	if (!driver) {
		error = -ENOMEM;
		goto fail_driver_alloc;
	}
	
	driver->spi = spi_dev_get(spi);
	if (!driver->spi) {
		error = -EPERM;
		goto fail_spi_get;
	}
	mutex_init(&driver->devices_lock);
	mutex_init(&driver->param_lock);
	init_rwsem(&driver->shutdown_lock);
	spin_lock_init(&driver->queue_lock);
	INIT_LIST_HEAD(&driver->queued_packets);
	INIT_LIST_HEAD(&driver->sent_packets);
	
	driver->workqueue = create_singlethread_workqueue(driver_name);
	if (!driver->workqueue) {
		error = -ENOMEM;
		goto fail_workqueue;
	}

	dev_t devt;
	UNWIND(error, alloc_chrdev_region(&devt, 0, members, driver_name), fail_alloc_chrdev_region);
	driver->major = MAJOR(devt);
	driver->minor = MINOR(devt);

	spi_set_drvdata(spi, driver);

	spi->max_speed_hz = spi_hz;
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	UNWIND(error, spi_setup(spi), fail_spi_setup);
	
	dnt900_init_packet(driver, &driver->empty_packet, NULL, EMPTY_PACKET_BYTES, false, NULL);
	
	driver->gpio_cfg = gpio_cfg; // how to make this per-instance?
	driver->gpio_avl = gpio_avl; // how to make this per-instance?
	driver->gpio_cts = gpio_cts; // how to make this per-instance?
	
	UNWIND(error, gpio_request_one(driver->gpio_cfg, GPIOF_OUT_INIT_HIGH, "/cfg"), fail_gpio_cfg);
	UNWIND(error, gpio_request_one(driver->gpio_avl, GPIOF_IN, "rx_avl"), fail_gpio_avl);
	UNWIND(error, gpio_request_one(driver->gpio_cts, GPIOF_IN, "/host_cts"), fail_gpio_cts);
	gpio_set_value(driver->gpio_cfg, 0); // enter protocol mode
	
	UNWIND(error, dnt900_get_driver_params(driver), fail_get_params);

	char local_mac_address[3];
	UNWIND(error, dnt900_get_register(driver, &dnt900_attributes[MacAddress].reg, local_mac_address), fail_mac_address);
	UNWIND(error, dnt900_alloc_device_local(driver, local_mac_address), fail_alloc_device);
	
	UNWIND(error, request_irq(gpio_to_irq(driver->gpio_avl), dnt900_avl_handler, IRQF_TRIGGER_RISING, driver_name, spi), fail_irq_avl);
	UNWIND(error, request_irq(gpio_to_irq(driver->gpio_cts), dnt900_cts_handler, IRQF_TRIGGER_FALLING, driver_name, spi), fail_irq_cts);
	goto success;
	
	free_irq(gpio_to_irq(driver->gpio_cts), driver);
fail_irq_cts:
	free_irq(gpio_to_irq(driver->gpio_avl), driver);
fail_irq_avl:
fail_alloc_device:
fail_mac_address:
fail_get_params:
	down_write(&driver->shutdown_lock);
	dnt900_free_devices(driver);
	gpio_free(driver->gpio_cts);
fail_gpio_cts:
	gpio_free(driver->gpio_avl);
fail_gpio_avl:
	gpio_free(driver->gpio_cfg);
fail_gpio_cfg:
fail_spi_setup:
	unregister_chrdev_region(MKDEV(driver->major, 0), members);
fail_alloc_chrdev_region:
	destroy_workqueue(driver->workqueue);
fail_workqueue:
	spi_dev_put(spi);
fail_spi_get:
	kfree(driver);
fail_driver_alloc:
success:
	return error;
}

static int dnt900_remove(struct spi_device *spi)
{
	struct dnt900_driver *driver = spi_get_drvdata(spi);

	// TODO: code to sleep the dnt900 here?
	
	free_irq(gpio_to_irq(driver->gpio_avl), spi);
	free_irq(gpio_to_irq(driver->gpio_cts), spi);
	down_write(&driver->shutdown_lock);
	destroy_workqueue(driver->workqueue);
	dnt900_free_devices(driver);
	gpio_free(driver->gpio_cts);
	gpio_free(driver->gpio_avl);
	gpio_free(driver->gpio_cfg);
	unregister_chrdev_region(MKDEV(driver->major, 0), members);
	spi_dev_put(driver->spi);
	kfree(driver);
	return 0;
}

int __init dnt900_init(void)
{
	dnt900_class = class_create(THIS_MODULE, class_name);
	if (IS_ERR(dnt900_class))
		return PTR_ERR(dnt900_class);
	int error;
	UNWIND(error, spi_register_driver(&dnt900_spi_driver), fail_register);
	pr_info("inserting dnt900 module\n");
	return 0;
	
fail_register:
	class_destroy(dnt900_class);
	return error;
}

void __exit dnt900_exit(void)
{
	pr_info("removing dnt900 module\n");
	spi_unregister_driver(&dnt900_spi_driver);
	class_destroy(dnt900_class);
}

module_init(dnt900_init);
module_exit(dnt900_exit);

MODULE_AUTHOR("Matthew Hollingworth");
MODULE_DESCRIPTION("SPI driver for DNT900 RF module");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

// TODO: use dev_error() etc
