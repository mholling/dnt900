#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/stddef.h>
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
// #include <linux/proc_fs.h>
// #include <linux/fcntl.h>
// #include <asm/system.h>

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

#define TYPE_COMMAND (0x0F)
#define TYPE_REPLY   (0x10)
#define TYPE_EVENT   (0x20)

#define STATUS_ACKNOWLEDGED     (0x00)
#define STATUS_NOT_ACKNOWLEDGED (0x01)
#define STATUS_NOT_LINKED       (0x02)
#define STATUS_HOLDING_FOR_FLOW (0x03)

#define ANNOUNCEMENT_STARTED       (0xA0)
#define ANNOUNCEMENT_REMOTE_JOINED (0xA2)
#define ANNOUNCEMENT_JOINED        (0xA3)
#define ANNOUNCEMENT_EXITED        (0xA4)
#define ANNOUNCEMENT_BASE_REBOOTED (0xA5)
#define ANNOUNCEMENT_REMOTE_EXITED (0xA7)
#define ANNOUNCEMENT_HEARTBEAT     (0xA8)

#define ERROR_PROTOCOL_TYPE     (0xE0)
#define ERROR_PROTOCOL_ARGUMENT (0xE1)
#define ERROR_PROTOCOL_GENERAL  (0xE2)
#define ERROR_PROTOCOL_TIMEOUT  (0xE3)
#define ERROR_PROTOCOL_READONLY (0xE4)
#define ERROR_UART_OVERFLOW     (0xE8)
#define ERROR_UART_OVERRUN      (0xE9)
#define ERROR_UART_FRAMING      (0xEA)
#define ERROR_HARDWARE          (0xEE)

#define RANGE_TO_METRES(range) (46671 * (unsigned char)(range) / 100)

#define EMPTY_PACKET_BYTES (8)

#define ATTR_R  (S_IRUGO)
#define ATTR_W  (S_IWUSR | S_IWGRP)
#define ATTR_RW (S_IRUGO | S_IWUSR | S_IWGRP)

#define EQUAL_ADDRESSES(address1, address2) \
	address1[0] == address2[0] && \
	address1[1] == address2[1] && \
	address1[2] == address2[2]

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
	spinlock_t lock;
	char buf[DRIVER_BUF_SIZE + MAX_PACKET_SIZE];
	unsigned int head;
	unsigned int tail;
	struct dnt900_packet empty_packet;
	
	char tree_routing_en;
};

#define IS_TREE_ROUTING(driver) ((driver)->tree_routing_en == 1)

struct dnt900_device {
	struct cdev cdev;
	struct dnt900_driver *driver;
	int is_local;
	struct mutex lock;
	char buf[RX_BUF_SIZE];
	unsigned int head;
	unsigned int tail;
	
	char sys_address[3];
	char mac_address[3];
	char device_mode;
	char slot_size;
	char protocol_options;
	char announce_options;
};

#define IS_BASE(device)   ((device)->device_mode == 1)
#define IS_REMOTE(device) ((device)->device_mode == 0)
#define IS_ROUTER(device) ((device)->device_mode == 3)

#define TX_REPLY_ENABLED(device) ((device)->protocol_options & 0x04)
#define ANNOUNCE_ENABLED(device) ((device)->protocol_options & 0x01)

#define ANNOUNCE_ERRORS(device) ((device)->announce_options & 0x04)
#define ANNOUNCE_INIT(device)   ((device)->announce_options & 0x01)
#define ANNOUNCE_LINK(device)   ((device)->announce_options & 0x02)

struct dnt900_rxdata {
	const char *buf;
	unsigned int len;
};

struct dnt900_register {
	char bank;
	char offset;
	char span;
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

static int dnt900_get_device_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value);
static int dnt900_set_device_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value);

static ssize_t dnt900_show_attr(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t dnt900_store_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t dnt900_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t dnt900_store_discover(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static int dnt900_open(struct inode *inode, struct file *filp);
static ssize_t dnt900_read(struct file *filp, char __user *buf, size_t length, loff_t *offp);
static ssize_t dnt900_write(struct file *filp, const char __user *buf, size_t length, loff_t *offp);

static int dnt900_configure_tree_routing(struct dnt900_driver *driver);
static int dnt900_refresh_device(struct dnt900_device *device);

static int dnt900_alloc_device(struct dnt900_driver *driver, int is_local, const char *name);
static int dnt900_free_device(struct device *dev, void *unused);
static void dnt900_free_devices(struct dnt900_driver *driver);

static int dnt900_device_matches_sys_address(struct device *dev, void *data);
static int dnt900_device_matches_mac_address(struct device *dev, void *data);
static int dnt900_device_is_local(struct device *dev, void *data);

static int dnt900_device_exists(struct dnt900_driver *driver, const char *mac_address);
static int dnt900_dispatch_to_device(struct dnt900_driver *driver, void *finder_data, int (*finder)(struct device *, void *), void *action_data, int (*action)(struct dnt900_device *, void *));

static int dnt900_receive_data(struct dnt900_device *device, void *data);
static int dnt900_process_announcement(struct dnt900_device *device, void *data);

static void dnt900_init_packet(struct dnt900_driver *driver, struct dnt900_packet *packet, const void *tx_buf, unsigned int len, char *result);
static int dnt900_send_message(struct dnt900_driver *driver, void *payload, unsigned int arg_length, char type, char *result);
static void dnt900_send_queued_packets(struct dnt900_driver *driver);
static void dnt900_complete_packet(void *context);

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
	int count = scnprintf(buf, PAGE_SIZE, "0x");
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
	int error = kstrtoul(buf, 0, &result);
	if (error)
		return error;
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
		int n;
		for (*value = 0, n = 0; n < 2; ++n, ++buf) {
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
	int n;
	for (n = 0; n < 16; ++n)
		value[n] = n < count ? buf[n] : 0;
	return 0;
}

static int dnt900_add_attributes(struct device *dev)
{
	int n;
	struct dnt900_device *device = dev_get_drvdata(dev);
	for (n = 0; n < ARRAY_SIZE(dnt900_attributes); ++n) {
		int result = device_create_file(dev, &dnt900_attributes[n].attr);
		if (result)
			return result;
	}
	if (device->is_local)
		for (n = 0; n < ARRAY_SIZE(dnt900_local_attributes); ++n) {
			int result = device_create_file(dev, dnt900_local_attributes + n);
			if (result)
				return result;
		}
	return 0;
}

static void dnt900_remove_attributes(struct device *dev)
{
	int n;
	struct dnt900_device *device = dev_get_drvdata(dev);
	for (n = 0; n < ARRAY_SIZE(dnt900_attributes); ++n)
		device_remove_file(dev, &dnt900_attributes[n].attr);
	if (device->is_local)
		for (n = 0; n < ARRAY_SIZE(dnt900_local_attributes); ++n)
			device_remove_file(dev, dnt900_local_attributes + n);
}

static int dnt900_get_register(struct dnt900_driver *driver, const struct dnt900_register *reg, char *value)
{
	int error;
	struct dnt900_get_register_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	message->bank = reg->bank;
	message->reg = reg->offset;
	message->span = reg->span;
	error = dnt900_send_message(driver, message, 3, COMMAND_GET_REGISTER, value);
	kfree(message);
	return error;
}

static int dnt900_get_remote_register(struct dnt900_driver *driver, const char *sys_address, const struct dnt900_register *reg, char *value)
{
	int error;
	struct dnt900_get_remote_register_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	memcpy(message->sys_address, sys_address, 3);
	message->bank = reg->bank;
	message->reg = reg->offset;
	message->span = reg->span;
	error = dnt900_send_message(driver, message, 6, COMMAND_GET_REMOTE_REGISTER, value);
	kfree(message);
	return error;
}

static int dnt900_set_register(struct dnt900_driver *driver, const struct dnt900_register *reg, const char *value)
{
	int error;
	struct dnt900_set_register_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	message->bank = reg->bank;
	message->reg = reg->offset;
	message->span = reg->span;
	memcpy(&message->value, value, reg->span);
	error = dnt900_send_message(driver, message, 3 + reg->span, COMMAND_SET_REGISTER, NULL);
	kfree(message);
	return error;
}

static int dnt900_set_remote_register(struct dnt900_driver *driver, const char *sys_address, const struct dnt900_register *reg, const char *value)
{
	int error;
	struct dnt900_set_remote_register_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	memcpy(message->sys_address, sys_address, 3);
	message->bank = reg->bank;
	message->reg = reg->offset;
	message->span = reg->span;
	memcpy(&message->value, value, reg->span);
	error = dnt900_send_message(driver, message, 6 + reg->span, COMMAND_SET_REMOTE_REGISTER, NULL);
	kfree(message);
	return error;
}

static int dnt900_discover(struct dnt900_driver *driver, const char *mac_address, char *sys_address)
{
	if (IS_TREE_ROUTING(driver)) {
		int error;
		struct dnt900_discover_message *message;
		message = kzalloc(sizeof(*message), GFP_KERNEL);
		if (!message)
			return -ENOMEM;
		memcpy(message->mac_address, mac_address, 3);
		error = dnt900_send_message(driver, message, 3, COMMAND_DISCOVER, sys_address);
		kfree(message);
		return error < 0 ? -ENODEV : 0;
	} else {
		memcpy(sys_address, mac_address, 3);
		return 0;
	}
}

static int dnt900_get_device_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value)
{
	return device->is_local ?
		dnt900_get_register(device->driver, reg, value) :
		dnt900_get_remote_register(device->driver, device->sys_address, reg, value);
}

static int dnt900_set_device_register(struct dnt900_device *device, const struct dnt900_register *reg, char *value)
{
	return device->is_local ?
		dnt900_set_register(device->driver, reg, value) :
		dnt900_set_remote_register(device->driver, device->sys_address, reg, value);
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
	int error = attribute->parse(buf, count, value);
	if (error)
		return error;
	error = dnt900_set_device_register(device, &attribute->reg, value);
	return error < 0 ? error : count;
}

static ssize_t dnt900_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int error;
	struct dnt900_device *device = dev_get_drvdata(dev);
	struct dnt900_software_reset_message *message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	message->boot_select = 0;
	error = dnt900_send_message(device->driver, message, 1, COMMAND_SOFTWARE_RESET, NULL);
	kfree(message);
	return error ? error : count;
}

static ssize_t dnt900_store_discover(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int error;
	unsigned int mac_address_int;
	char mac_address[3];
	char name[32];
	int byte;
	struct dnt900_device *device = dev_get_drvdata(dev);
	struct dnt900_driver *driver = device->driver;
	error = kstrtouint(buf, 16, &mac_address_int);
	if (error)
		return error;
	for (byte = 0; byte < 3; ++byte, mac_address_int >>= 8)
		mac_address[byte] = mac_address_int & 0xFF;
	if (mac_address_int)
		return -EINVAL;
	if (dnt900_device_exists(driver, mac_address))
		return -EEXIST;
	snprintf(name, ARRAY_SIZE(name), remote_name_template, mac_address[2], mac_address[1], mac_address[0]);
	error = dnt900_alloc_device(driver, false, name);
	return error < 0 ? error : count;
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
	unsigned int available, count, copied;
	
	if (!mutex_trylock(&device->lock))
		return 0;
	available = (device->head > device->tail ? device->head : RX_BUF_SIZE) - device->tail;
	count = available > length ? length : available;
	copied = count - copy_to_user(buf, device->buf + device->tail, count);
	CIRC_OFFSET(device->tail, count, RX_BUF_SIZE);
	mutex_unlock(&device->lock);
	
	if (count && !copied)
		return -EFAULT;
	*offp += copied;
	return copied;
}

static ssize_t dnt900_write(struct file *filp, const char __user *buf, size_t length, loff_t *offp)
{
	int error;
	struct dnt900_device *device = filp->private_data;
	struct dnt900_tx_data_message *message;
	unsigned int count, copied;
	
	if (device->is_local)
		return -EPERM;
	message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	memcpy(message->sys_address, device->sys_address, 3);
	count = device->slot_size > length ? length : device->slot_size;
	copied = count - copy_from_user(message->data, buf, count);
	if (count && !copied)
		error = -EFAULT;
	else
		error = dnt900_send_message(device->driver, message, 3 + copied, COMMAND_TX_DATA, NULL);
	kfree(message);
	*offp += copied;
	return error < 0 ? error : copied;
}

static int dnt900_configure_tree_routing(struct dnt900_driver *driver)
{
	return dnt900_get_register(driver, &dnt900_attributes[TreeRoutingEn].reg, &driver->tree_routing_en);
}

static int dnt900_refresh_device(struct dnt900_device *device)
{
	struct param { char *value; const struct dnt900_register *reg; };
	struct param params[] = {
		{ .value = device->mac_address,       .reg = &dnt900_attributes[MacAddress].reg },
		{ .value = &device->device_mode,      .reg = &dnt900_attributes[DeviceMode].reg },
		{ .value = &device->protocol_options, .reg = &dnt900_attributes[ProtocolOptions].reg },
		{ .value = &device->announce_options, .reg = &dnt900_attributes[AnnounceOptions].reg },
		{ .value = &device->slot_size,        .reg = &dnt900_attributes[IS_BASE(device) ? BaseSlotSize : RemoteSlotSize].reg }
	};
	int error, index;
	
	// TODO: use a mutex here?
	for (error = 0, index = 0; !error && index < ARRAY_SIZE(params); ++index)
		error = dnt900_get_device_register(device, params[index].reg, params[index].value);
	if (error)
		return error;
	error = dnt900_discover(device->driver, device->mac_address, device->sys_address);
	return error;
}

static int dnt900_alloc_device(struct dnt900_driver *driver, int is_local, const char *name)
{
	int error;
	struct device *dev;
	dev_t devt;
	
	struct dnt900_device *device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		error = -ENOMEM;
		goto fail_device_alloc;
	}
	device->driver = driver;
	device->is_local = is_local;
	mutex_init(&device->lock);
	devt = MKDEV(driver->major, driver->minor++);
	dev = device_create(dnt900_class, &driver->spi->dev, devt, device, name);
	if (IS_ERR(dev)) {
		error = PTR_ERR(dev);
		goto fail_device_create;
	}
	error = dnt900_refresh_device(device);
	if (error)
		goto fail_refresh_device;
	error = dnt900_add_attributes(dev);
	if (error)
		goto fail_add_attributes;
	cdev_init(&device->cdev, &dnt900_fops);
	device->cdev.owner = THIS_MODULE;
	error = cdev_add(&device->cdev, devt, 1);
	if (error)
		goto fail_cdev_add;
	// TODO: set cdev permissions to read-only if is_local?
	return 0;
	
	cdev_del(&device->cdev);
fail_cdev_add:
	dnt900_remove_attributes(dev);
fail_add_attributes:
fail_refresh_device:
	device_unregister(dev);
fail_device_create:
	kfree(device);
fail_device_alloc:
	return error;
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
	return EQUAL_ADDRESSES(device->sys_address, sys_address);
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
	return device_find_child(&driver->spi->dev, (void *)mac_address, dnt900_device_matches_mac_address) != 0;
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
		error = -ENODEV; // TODO: if no device is found we should probably add it!
	return error;
}

static int dnt900_receive_data(struct dnt900_device *device, void *data)
{
	struct dnt900_rxdata *rxdata = data;
	
	int error = mutex_lock_interruptible(&device->lock);
	if (error)
		return error;
	for (; rxdata->len > 0; ++rxdata->buf, --rxdata->len) {
		device->buf[device->head] = *rxdata->buf;
		CIRC_OFFSET(device->head, 1, RX_BUF_SIZE);
		if (device->head == device->tail)
			CIRC_OFFSET(device->tail, 1, RX_BUF_SIZE);
	}
	mutex_unlock(&device->lock);
	return 0;
}

static int dnt900_process_announcement(struct dnt900_device *device, void *data)
{
	char *annc = data;
	char message[512];
	struct dnt900_rxdata rxdata = { .buf = message };
	
	switch (annc[0]) {
	case ANNOUNCEMENT_STARTED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"completed startup initialization\n");
		break;
	case ANNOUNCEMENT_REMOTE_JOINED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"radio joined:\n" \
			"    MAC address 0x%02X%02X%02X\n" \
			"    range %i metres\n", \
			annc[3], annc[2], annc[1], RANGE_TO_METRES(annc[5]));
		break;
	case ANNOUNCEMENT_JOINED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"joined network:\n" \
			"    network ID 0x%02X\n" \
			"    base MAC address 0x%02X%02X%02X\n" \
			"    range %i metres\n", \
			annc[1], annc[4], annc[3], annc[2], RANGE_TO_METRES(annc[5]));
		break;
	case ANNOUNCEMENT_EXITED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"exited network:\n" \
			"    network ID 0x%02X\n", \
			annc[1]);
		break;
	case ANNOUNCEMENT_BASE_REBOOTED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"base rebooted\n");
		break;
	case ANNOUNCEMENT_REMOTE_EXITED:
		rxdata.len = scnprintf(message, ARRAY_SIZE(message), \
			"radio exited:\n" \
			"    MAC address 0x%02X%02X%02X\n", \
			annc[3], annc[2], annc[1]);
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
			"    range %i metres\n", \
			annc[3], annc[2], annc[1], annc[4], annc[5], annc[6], (signed char)annc[7], (signed char)annc[9], RANGE_TO_METRES(annc[10]));
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

static void dnt900_init_packet(struct dnt900_driver *driver, struct dnt900_packet *packet, const void *tx_buf, unsigned int len, char *result)
{
	INIT_LIST_HEAD(&packet->list);
	init_completion(&packet->completed);
	packet->result = result;
	packet->error = 0;
	INIT_LIST_HEAD(&packet->message.transfers);
	packet->message.context = driver;
	packet->message.complete = dnt900_complete_packet;
	packet->transfer.len = len;
	packet->transfer.tx_buf = tx_buf;
	spi_message_add_tail(&packet->transfer, &packet->message);
}

static int dnt900_send_message(struct dnt900_driver *driver, void *payload, unsigned int arg_length, char type, char *result)
{
	int interrupted;
	unsigned long flags;
	struct dnt900_packet packet;
	struct dnt900_message_header *header = payload;
	
	dnt900_init_packet(driver, &packet, payload, 3 + arg_length, result);
	header->start_of_packet = START_OF_PACKET;
	header->length = 1 + arg_length;
	header->type = type;
	
	spin_lock_irqsave(&driver->lock, flags);
	list_add_tail(&packet.list, &driver->queued_packets);
	spin_unlock_irqrestore(&driver->lock, flags);
	dnt900_send_queued_packets(driver);
	// TODO: use wait_for_completion_interruptible_timeout to timeout if taking too long?
	interrupted = wait_for_completion_interruptible(&packet.completed);
	if (interrupted) {
		spin_lock_irqsave(&driver->lock, flags);
		list_del(&packet.list);
		spin_unlock_irqrestore(&driver->lock, flags);
		return interrupted;
	}
	return packet.error;
}

static void dnt900_send_queued_packets(struct dnt900_driver *driver)
{
	unsigned long flags;
	
	// TODO: reduce time spent with spin lock...
	spin_lock_irqsave(&driver->lock, flags);
	if (gpio_get_value(driver->gpio_avl) && (list_empty(&driver->queued_packets) || gpio_get_value(driver->gpio_cts)))
		list_add(&driver->empty_packet.list, &driver->queued_packets);
	if (!list_empty(&driver->queued_packets)) {
		struct dnt900_packet *packet = list_first_entry(&driver->queued_packets, struct dnt900_packet, list);
		// spinlock for accessing driver->head ??
		packet->transfer.rx_buf = driver->buf + driver->head;
		CIRC_OFFSET(driver->head, packet->transfer.len, DRIVER_BUF_SIZE);
		spi_async(driver->spi, &packet->message);
	}
	spin_unlock_irqrestore(&driver->lock, flags);
}

static void dnt900_complete_packet(void *context)
{
	struct dnt900_driver *driver = context;
	struct dnt900_packet *packet;
	char * const buf_end = driver->buf + DRIVER_BUF_SIZE;
	char * transfer_end; // TODO: use C99!
	unsigned long flags;
	
	spin_lock_irqsave(&driver->lock, flags);
	packet = list_first_entry(&driver->queued_packets, struct dnt900_packet, list);
	if (!packet->message.status && packet->transfer.tx_buf)
		list_move_tail(&packet->list, &driver->sent_packets);
	else
		list_del(&packet->list);
	spin_unlock_irqrestore(&driver->lock, flags);
	
	transfer_end = packet->transfer.rx_buf + packet->transfer.len;
	if (transfer_end > buf_end)
		memcpy(driver->buf, buf_end, transfer_end - buf_end);
	
	if (packet->message.status) {
		packet->error = packet->message.status;
		complete(&packet->completed);
	}
	
	printk(KERN_INFO "scanning received data...\n");
	while (driver->head != driver->tail) {
		printk(KERN_INFO "[%03i]: %02x\n", driver->tail, driver->buf[driver->tail]);
		CIRC_OFFSET(driver->tail, 1, DRIVER_BUF_SIZE);
	}
	
	while (driver->head != driver->tail) {
		unsigned int remaining, length;
		char type;
		
		for (; driver->tail != driver->head && driver->buf[driver->tail] != START_OF_PACKET; CIRC_OFFSET(driver->tail, 1, DRIVER_BUF_SIZE))
			;
		
		if (driver->head == driver->tail)
			break;
		
		remaining = CIRC_INDEX(DRIVER_BUF_SIZE + driver->head - driver->tail, 0, DRIVER_BUF_SIZE);
		length = driver->buf[CIRC_INDEX(driver->tail, 1, DRIVER_BUF_SIZE)] + 2;
		type = driver->buf[CIRC_INDEX(driver->tail, 2, DRIVER_BUF_SIZE)];
		
		if (remaining < 2 || length >= remaining)
			break;
		
		if (type & TYPE_REPLY) {
			const char command = type & TYPE_COMMAND;
			struct dnt900_packet *packet;
			list_for_each_entry(packet, &driver->sent_packets, list) {
				const char * const buf = packet->transfer.tx_buf;
				int index;
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
					for (index = 0; index < buf[5]; ++index)
						packet->result[index] = driver->buf[CIRC_INDEX(driver->tail, 6 + index, DRIVER_BUF_SIZE)];
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
						for (index = 0; index < 3; ++index)
							packet->result[index] = driver->buf[CIRC_INDEX(driver->tail, 7 + index, DRIVER_BUF_SIZE)];
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
						for (index = 0; index < buf[8]; ++index)
							packet->result[index] = driver->buf[CIRC_INDEX(driver->tail, 11 + index, DRIVER_BUF_SIZE)];
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
		
		if (type & TYPE_EVENT) {
			char sys_address[3];
			unsigned int offset, start, end;
			struct dnt900_rxdata rxdata;
			char announcement[11];
			switch (type) {
			case EVENT_RX_DATA:
				for (offset = 3; offset < 6; ++offset)
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
				break;
			case EVENT_ANNOUNCE:
				start = 3;
				end = min(length, start + ARRAY_SIZE(announcement));
				for (offset = start; offset < end; ++offset)
					announcement[offset-start] = driver->buf[CIRC_INDEX(driver->tail, offset, DRIVER_BUF_SIZE)];
				dnt900_dispatch_to_device(driver, NULL, dnt900_device_is_local, announcement, dnt900_process_announcement);
				break;
			case EVENT_RX_EVENT:
				// unimplemented for now
				break;
			case EVENT_JOIN_REQUEST:
				// unimplemented for now
				break;
			}
		}
		
		CIRC_OFFSET(driver->tail, length, DRIVER_BUF_SIZE);
	}
	
	dnt900_send_queued_packets(driver);
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
	dev_t devt;

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
	spin_lock_init(&driver->lock);
	INIT_LIST_HEAD(&driver->queued_packets);
	INIT_LIST_HEAD(&driver->sent_packets);

	error = alloc_chrdev_region(&devt, 0, members, driver_name);
	if (error)
		goto fail_alloc_chrdev_region;
	driver->major = MAJOR(devt);
	driver->minor = MINOR(devt);

	spi_set_drvdata(spi, driver);

	spi->max_speed_hz = spi_hz;
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	error = spi_setup(spi);
	if (error)
		goto fail_spi_setup;
	
	dnt900_init_packet(driver, &driver->empty_packet, NULL, EMPTY_PACKET_BYTES, NULL);
	
	driver->gpio_cfg = gpio_cfg; // how to make this per-instance?
	driver->gpio_avl = gpio_avl; // how to make this per-instance?
	driver->gpio_cts = gpio_cts; // how to make this per-instance?
	
	error = gpio_request_one(driver->gpio_cfg, GPIOF_OUT_INIT_HIGH, "/cfg");
	if (error)
		goto fail_gpio_cfg;
	error = gpio_request_one(driver->gpio_avl, GPIOF_IN, "rx_avl");
	if (error)
		goto fail_gpio_avl;
	error = gpio_request_one(driver->gpio_cts, GPIOF_IN, "/host_cts");
	if (error)
		goto fail_gpio_cts;
	gpio_set_value(driver->gpio_cfg, 0); // enter protocol mode
	
	error = dnt900_configure_tree_routing(driver);
	if (error)
		goto fail_tree_routing;
	error = dnt900_alloc_device(driver, true, local_name);
	if (error)
		goto fail_alloc_device;
	
	error = request_irq(gpio_to_irq(driver->gpio_avl), dnt900_avl_handler, IRQF_TRIGGER_RISING, driver_name, spi);
	if (error)
		goto fail_irq_avl;
	error = request_irq(gpio_to_irq(driver->gpio_cts), dnt900_cts_handler, IRQF_TRIGGER_FALLING, driver_name, spi);
	if (error)
		goto fail_irq_cts;
	return 0;
	
	free_irq(gpio_to_irq(driver->gpio_cts), driver);
fail_irq_cts:
	free_irq(gpio_to_irq(driver->gpio_avl), driver);
fail_irq_avl:
fail_alloc_device:
fail_tree_routing:
	gpio_free(driver->gpio_cts);
fail_gpio_cts:
	gpio_free(driver->gpio_avl);
fail_gpio_avl:
	gpio_free(driver->gpio_cfg);
fail_gpio_cfg:
fail_spi_setup:
	unregister_chrdev_region(MKDEV(driver->major, 0), members);
fail_alloc_chrdev_region:
	spi_dev_put(spi);
fail_spi_get:
	kfree(driver);
fail_driver_alloc:
	return error;
}

static int dnt900_remove(struct spi_device *spi)
{
	struct dnt900_driver *driver = spi_get_drvdata(spi);

	// TODO: code to sleep the dnt900 here?
	free_irq(gpio_to_irq(driver->gpio_avl), spi);
	free_irq(gpio_to_irq(driver->gpio_cts), spi);
	gpio_free(driver->gpio_cts);
	gpio_free(driver->gpio_avl);
	gpio_free(driver->gpio_cfg);
	dnt900_free_devices(driver);
	unregister_chrdev_region(MKDEV(driver->major, 0), members);
	spi_dev_put(driver->spi);
	kfree(driver);
	return 0;
}

int __init dnt900_init(void)
{
	int error;
	
	dnt900_class = class_create(THIS_MODULE, class_name);
	if (IS_ERR(dnt900_class))
		return PTR_ERR(dnt900_class);
	error = spi_register_driver(&dnt900_spi_driver);
	if (error)
		goto fail_register;
	printk(KERN_INFO "inserting dnt900 module\n");
	return 0;
	
fail_register:
	class_destroy(dnt900_class);
	return error;
}

void __exit dnt900_exit(void)
{
	printk(KERN_INFO "removing dnt900 module\n");
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
