#include <linux/kernel.h>
#include <linux/module.h>
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

#define MAX_PACKET_SIZE (258)

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

#define EMPTY_PACKET_BYTES (8)

#define ATTR_R  (S_IRUGO)
#define ATTR_W  (S_IWUSR | S_IWGRP)
#define ATTR_RW (S_IRUGO | S_IWUSR | S_IWGRP)

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
static const char remote_name_template[] = "dnt900.0x%06X";

static struct class *dnt900_class;

struct dnt900_packet {
	struct spi_transfer transfer;
	struct spi_message message;
	struct list_head list;
	struct completion completed;
	char *result;
	int status;
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
	int head;
	int tail;

	struct dnt900_packet empty_packet;
};

struct dnt900_device {
	struct cdev cdev;
	int is_local;
	unsigned int mac_address;
	unsigned int sys_address;

	struct mutex lock;
	
	char buf[RX_BUF_SIZE];
	int head;
	int tail;
};

struct dnt900_attribute {
	struct device_attribute attr;
	char bank;
	char offset;
	char span;
	int (*print)(const char *value, char *buf);
	int (*parse)(const char *buf, size_t count, char *value);
};

static ssize_t dnt900_show_attr(struct device *, struct device_attribute *, char *);
static ssize_t dnt900_store_attr(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t dnt900_reset(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t dnt900_discover(struct device *, struct device_attribute *, const char *, size_t);

static struct device_attribute dnt900_local_attributes[] = {
	__ATTR(reset,	ATTR_W, NULL, dnt900_reset),
	__ATTR(discover, ATTR_W, NULL, dnt900_discover)
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
	.bank = _bank, \
	.offset = _offset, \
	.span = _span, \
	.print = _print, \
	.parse = _parse \
}

static int print_bytes(int bytes, const char *value, char *buf)
{
	int count = scnprintf(buf, PAGE_SIZE, "0x");
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

static struct dnt900_attribute dnt900_attributes[] = {
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

static int dnt900_add_attributes(struct device *dev) {
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

static void dnt900_remove_attributes(struct device *dev) {
	int n;
	struct dnt900_device *device = dev_get_drvdata(dev);
	for (n = 0; n < ARRAY_SIZE(dnt900_attributes); ++n)
		device_remove_file(dev, &dnt900_attributes[n].attr);
	if (device->is_local)
		for (n = 0; n < ARRAY_SIZE(dnt900_local_attributes); ++n)
			device_remove_file(dev, dnt900_local_attributes + n);
}

static int dnt900_send_command(struct dnt900_driver *driver, const char *buf, char *result);
static int dnt900_get_register(struct dnt900_driver *driver, char bank, char offset, char span, char *value);
static int dnt900_set_register(struct dnt900_driver *driver, char bank, char offset, char span, const char *value);

static ssize_t dnt900_show_attr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev->parent);
	struct dnt900_driver *driver = spi_get_drvdata(spi);
	struct dnt900_attribute *attribute = container_of(attr, struct dnt900_attribute, attr);
	char value[32];
	int error = dnt900_get_register(driver, attribute->offset, attribute->bank, attribute->span, value);
	return error < 0 ? error : attribute->print(value, buf);
}

static ssize_t dnt900_store_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev->parent);
	struct dnt900_driver *driver = spi_get_drvdata(spi);
	struct dnt900_attribute *attribute = container_of(attr, struct dnt900_attribute, attr);
	char value[32];
	int error = attribute->parse(buf, count, value);
	if (error)
		return error;
	error = dnt900_set_register(driver, attribute->offset, attribute->bank, attribute->span, value);
	return error < 0 ? error : count;
}

static ssize_t dnt900_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev->parent);
	struct dnt900_driver *driver = spi_get_drvdata(spi);
	char cmd[4] = { START_OF_PACKET, 2, COMMAND_SOFTWARE_RESET, 0 };
	dnt900_send_command(driver, cmd, NULL);
	return count;
}

static int dnt900_alloc_device(struct dnt900_driver *driver, unsigned int mac_address, unsigned int sys_address, int is_local);

static ssize_t dnt900_discover(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev->parent);
	struct dnt900_driver *driver = spi_get_drvdata(spi);
	unsigned int mac_address;
	unsigned int sys_address;
	
	int error = kstrtouint(buf, 16, &mac_address);
	if (error)
		return -EINVAL;
	if (mac_address & ~0xFFFFFF)
		return -EINVAL;
	if (false /* TODO: send Discover message to verify existence and set sys_address */)
		return -ENODEV;
	if (false /* TODO: check mac address does not correspond to the local dnt900 */)
		return -EEXIST;
	if (false /* TODO: check that the device has not already been discovered and registered */)
		return -EEXIST;
		
	error = dnt900_alloc_device(driver, mac_address, sys_address, 0);
	
	return error ? error : count;
}

static struct file_operations dnt900_fops;

static int dnt900_alloc_device(struct dnt900_driver *driver, unsigned int mac_address, unsigned int sys_address, int is_local)
{
	int error = 0;
	struct device *dev;
	dev_t devt;

	struct dnt900_device *device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		error = -ENOMEM;
		goto fail_device_alloc;
	}

	device->mac_address = mac_address;
	device->sys_address = sys_address;
	device->is_local = is_local;
	
	mutex_init(&device->lock);

	devt = MKDEV(driver->major, driver->minor++);
	dev = is_local ? // TODO: make name using sprintf?
		device_create(dnt900_class, &driver->spi->dev, devt, device, local_name) :
		device_create(dnt900_class, &driver->spi->dev, devt, device, remote_name_template, mac_address);
	if (IS_ERR(dev)) {
		error = PTR_ERR(dev);
		goto fail_device_create;
	}

	error = dnt900_add_attributes(dev);
	if (error)
		goto fail_add_attributes;

	cdev_init(&device->cdev, &dnt900_fops);
	error = cdev_add(&device->cdev, devt, 1);
	if (error)
		goto fail_cdev_add;

	return 0;

fail_cdev_add:
	dnt900_remove_attributes(dev);
fail_add_attributes:
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
	unsigned int *sys_address = data;
	return device->sys_address == *sys_address;
}

struct dnt900_rxdata {
	const char *buf;
	int len;
};

int dnt900_receive_data(struct device *dev, void *data) {
	struct dnt900_rxdata *rxdata = data;
	struct dnt900_device *device = dev_get_drvdata(dev);
	int n;
	
	int error = mutex_lock_interruptible(&device->lock);
	if (error)
		return error;
	// TODO: can do this copy more efficiently using memcpy()
	for (n = 0; n < rxdata->len; ++n) {
		device->buf[device->head] = rxdata->buf[n];
		CIRC_OFFSET(device->head, 1, RX_BUF_SIZE);
		if (device->head == device->tail)
			CIRC_OFFSET(device->tail, 1, RX_BUF_SIZE);
	}
	mutex_unlock(&device->lock);
	return 0;
}

static int dnt900_dispatch_to_device(struct dnt900_driver *driver, unsigned int sys_address, void *data, int (*operation)(struct device *, void *))
{
	int error;
	struct device *dev = device_find_child(&driver->spi->dev, &sys_address, dnt900_device_matches_sys_address);
	if (dev) {
		error = operation(dev, data);
		put_device(dev);
	} else 
		error = -ENODEV; // TODO: if no device is found we should probably add it!
	return error;
}

static void dnt900_complete_packet(void *);

static void dnt900_init_packet(struct dnt900_packet *packet, struct dnt900_driver *driver, unsigned len, const void *tx_buf, char *result)
{
	memset(packet, 0, sizeof(*packet));
	INIT_LIST_HEAD(&packet->message.transfers);
	packet->message.context = driver;
	packet->message.complete = dnt900_complete_packet;
	packet->transfer.len = len;
	packet->transfer.tx_buf = tx_buf;
	spi_message_add_tail(&packet->transfer, &packet->message);
	INIT_LIST_HEAD(&packet->list);
	init_completion(&packet->completed);
	packet->result = result;
	packet->status = 0;
}

static int dnt900_errno_from_txstatus(char txstatus)
{
	switch (txstatus) {
	case STATUS_ACKNOWLEDGED:
		return 0;
	case STATUS_HOLDING_FOR_FLOW:
		return -ETIMEDOUT;
	case STATUS_NOT_ACKNOWLEDGED:
	case STATUS_NOT_LINKED:
	default:
		return -ECOMM;
	}
}

static void dnt900_send_queued_packets(struct dnt900_driver *driver)
{
	unsigned long flags;

	spin_lock_irqsave(&driver->lock, flags);
	if (gpio_get_value(driver->gpio_avl) && (list_empty(&driver->queued_packets) || gpio_get_value(driver->gpio_cts))) {
		dnt900_init_packet(&driver->empty_packet, driver, EMPTY_PACKET_BYTES, NULL, NULL);
		list_add(&driver->empty_packet.list, &driver->queued_packets);
	}
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
	const int head = driver->head;
	int tail = driver->tail;
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
		packet->status = packet->message.status;
		complete(&packet->completed);
	}

	printk(KERN_INFO "scanning received data...\n");
	while (head != tail) {
		printk(KERN_INFO "[%03i]: %02x\n", tail, driver->buf[tail]);
		CIRC_OFFSET(tail, 1, DRIVER_BUF_SIZE);
	}

	while (head != tail) {
		int bytes, length, type;

		for (; tail != head && driver->buf[tail] != START_OF_PACKET; CIRC_OFFSET(tail, 1, DRIVER_BUF_SIZE))
			;

		if (head == tail)
			break;

		// bytes = (DRIVER_BUF_SIZE + head - tail) % DRIVER_BUF_SIZE;
		bytes = CIRC_INDEX(DRIVER_BUF_SIZE - tail, head, DRIVER_BUF_SIZE);
		length = driver->buf[CIRC_INDEX(tail, 1, DRIVER_BUF_SIZE)] + 2;
		type = driver->buf[CIRC_INDEX(tail, 2, DRIVER_BUF_SIZE)];

		if (bytes < 2 || length >= bytes)
			break;

		if (type & TYPE_REPLY) {
			const int command = type & TYPE_COMMAND;
			struct dnt900_packet *packet;
			list_for_each_entry(packet, &driver->sent_packets, list) {
				const char * const buf = packet->transfer.tx_buf;
				int index;
				if (buf[2] != command)
					continue;
				switch (command) {
				case COMMAND_SOFTWARE_RESET:
					packet->status = 0;
					break;
				case COMMAND_GET_REGISTER:
					// match Reg, Bank, span:
					if (buf[3] != driver->buf[CIRC_INDEX(tail, 3, DRIVER_BUF_SIZE)] 
					 || buf[4] != driver->buf[CIRC_INDEX(tail, 4, DRIVER_BUF_SIZE)] 
					 || buf[5] != driver->buf[CIRC_INDEX(tail, 5, DRIVER_BUF_SIZE)])
						continue;
					// TODO: check buf[5] + 6 == length?
					for (index = 0; index < buf[5]; ++index)
						packet->result[index] = driver->buf[CIRC_INDEX(tail, 6 + index, DRIVER_BUF_SIZE)];
					packet->status = 0;
					break;
				case COMMAND_SET_REGISTER:
					packet->status = 0;
					break;
				case COMMAND_TX_DATA:
					// match Addr:
					if (buf[3] != driver->buf[CIRC_INDEX(tail, 4, DRIVER_BUF_SIZE)] 
					 || buf[4] != driver->buf[CIRC_INDEX(tail, 5, DRIVER_BUF_SIZE)] 
					 || buf[5] != driver->buf[CIRC_INDEX(tail, 6, DRIVER_BUF_SIZE)])
						continue;
					packet->status = dnt900_errno_from_txstatus(driver->buf[CIRC_INDEX(tail, 3, DRIVER_BUF_SIZE)]);
					break;
				case COMMAND_DISCOVER:
					// match MacAddr:
					if (buf[3] != driver->buf[CIRC_INDEX(tail, 4, DRIVER_BUF_SIZE)] 
					 || buf[4] != driver->buf[CIRC_INDEX(tail, 5, DRIVER_BUF_SIZE)] 
					 || buf[5] != driver->buf[CIRC_INDEX(tail, 6, DRIVER_BUF_SIZE)])
						continue;
					packet->status = dnt900_errno_from_txstatus(driver->buf[CIRC_INDEX(tail, 3, DRIVER_BUF_SIZE)]);
					if (!packet->status)
						for (index = 0; index < 3; ++index)
							packet->result[index] = driver->buf[CIRC_INDEX(tail, 7 + index, DRIVER_BUF_SIZE)];
					break;
				case COMMAND_GET_REMOTE_REGISTER:
					// match Addr:
					if (buf[3] != driver->buf[CIRC_INDEX(tail, 4, DRIVER_BUF_SIZE)] 
					 || buf[4] != driver->buf[CIRC_INDEX(tail, 5, DRIVER_BUF_SIZE)] 
					 || buf[5] != driver->buf[CIRC_INDEX(tail, 6, DRIVER_BUF_SIZE)])
						continue;
					if (driver->buf[CIRC_INDEX(tail, 3, DRIVER_BUF_SIZE)] == STATUS_ACKNOWLEDGED) {
						// match Reg, Bank, span:
						if (buf[6] != driver->buf[CIRC_INDEX(tail, 8, DRIVER_BUF_SIZE)] 
						 || buf[7] != driver->buf[CIRC_INDEX(tail, 9, DRIVER_BUF_SIZE)] 
						 || buf[8] != driver->buf[CIRC_INDEX(tail, 10, DRIVER_BUF_SIZE)])
							continue;
						for (index = 0; index < buf[8]; ++index)
							packet->result[index] = driver->buf[CIRC_INDEX(tail, 11 + index, DRIVER_BUF_SIZE)];
						packet->status = 0;
					} else
						packet->status = dnt900_errno_from_txstatus(driver->buf[CIRC_INDEX(tail, 3, DRIVER_BUF_SIZE)]);
					break;
				case COMMAND_SET_REMOTE_REGISTER:
					// match Addr:
					if (buf[3] != driver->buf[CIRC_INDEX(tail, 4, DRIVER_BUF_SIZE)] 
					 || buf[4] != driver->buf[CIRC_INDEX(tail, 5, DRIVER_BUF_SIZE)] 
					 || buf[5] != driver->buf[CIRC_INDEX(tail, 6, DRIVER_BUF_SIZE)])
						continue;
					packet->status = dnt900_errno_from_txstatus(driver->buf[CIRC_INDEX(tail, 3, DRIVER_BUF_SIZE)]);
					break;
				}
				list_del(&packet->list);
				complete(&packet->completed);
				break;
			}
		}

		if (type & TYPE_EVENT) {
			unsigned int sys_address;
			int offset, start, end;
			struct dnt900_rxdata rxdata;
			switch (type) {
			case EVENT_RX_DATA:
				for (sys_address = 0, offset = 5; offset > 2; sys_address <<= 8, --offset)
					sys_address += driver->buf[CIRC_INDEX(tail, offset, DRIVER_BUF_SIZE)];
				start = CIRC_INDEX(tail,      7, DRIVER_BUF_SIZE);
				end   = CIRC_INDEX(tail, length, DRIVER_BUF_SIZE);
				
				rxdata.buf = driver->buf + start;
				rxdata.len = (end < start ? DRIVER_BUF_SIZE : end) - start;
				dnt900_dispatch_to_device(driver, sys_address, &rxdata, dnt900_receive_data);
				
				if (end < start) {
					rxdata.buf = driver->buf;
					rxdata.len = end;
					dnt900_dispatch_to_device(driver, sys_address, &rxdata, dnt900_receive_data);
				}
				break;
			case EVENT_ANNOUNCE:
				// stringify the announcement and send to buffer for the local device
				break;
			case EVENT_RX_EVENT:
				// ignore for now
				break;
			case EVENT_JOIN_REQUEST:
				// ignore for now
				break;
			}
		}

		CIRC_OFFSET(tail, length, DRIVER_BUF_SIZE);
	}

	driver->tail = tail;
	dnt900_send_queued_packets(driver);
}

static int dnt900_send_command(struct dnt900_driver *driver, const char *buf, char *result)
{
	struct dnt900_packet packet;
	int error;
	unsigned long flags;

	dnt900_init_packet(&packet, driver, buf[1] + 2, buf, result);
	spin_lock_irqsave(&driver->lock, flags);
	list_add_tail(&packet.list, &driver->queued_packets);
	spin_unlock_irqrestore(&driver->lock, flags);
	dnt900_send_queued_packets(driver);
	// TODO: use wait_for_completion_interruptible_timeout to timeout if taking too long?
	error = wait_for_completion_interruptible(&packet.completed);
	// TODO: remove packet from queue if interrupted
	// TODO: implement dnt900_destroy_packet to do this...
	return error < 0 ? error : packet.status;
}

static int dnt900_get_register(struct dnt900_driver *driver, char bank, char offset, char span, char *value)
{
	// TODO: cmd buffer should be DMA-safe i.e. allocated on the
	// heap using kmalloc, not on the stack as here:
	char cmd[6] = { START_OF_PACKET, 4, COMMAND_GET_REGISTER, offset, bank, span };
	return dnt900_send_command(driver, cmd, value);
}

static int dnt900_set_register(struct dnt900_driver *driver, char bank, char offset, char span, const char *value)
{
	// TODO: cmd buffer should be DMA-safe i.e. allocated on the
	// heap using kmalloc, not on the stack as here:
	char cmd[6 + 32] = { START_OF_PACKET, 4, COMMAND_GET_REGISTER, offset, bank, span };
	int n;
	for (n = 0; n < span; ++n)
		cmd[6 + n] = value[n];
	return dnt900_send_command(driver, cmd, NULL);
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
	int available, count, copied;
	
	if (!mutex_trylock(&device->lock)) // TODO: use mutex_lock_interruptible?
		return 0;
	available = (device->head > device->tail ? device->head : RX_BUF_SIZE) - device->tail;
	count = available > length ? length : available;
	copied = count - copy_to_user(buf, device->buf + device->tail, count);
	CIRC_OFFSET(device->tail, count, RX_BUF_SIZE);
	mutex_unlock(&device->lock);
	
	return count && !copied ? -EFAULT : copied;
}

static ssize_t dnt900_write(struct file *filp, const char __user *buf, size_t length, loff_t *offp)
{
	// struct dnt900_device *device = filp->private_data;
	// TODO: pull a packet's worth of data from buf and send TxData message
	return length;
}

static struct file_operations dnt900_fops = {
	.owner = THIS_MODULE,
	.open = dnt900_open,
	.read = dnt900_read,
	.write = dnt900_write
};

static irqreturn_t dnt900_avl_handler(int irq, void *dev_id)
{
	struct spi_device *spi = dev_id;
	struct dnt900_driver *driver = spi_get_drvdata(spi);
	dnt900_send_queued_packets(driver);
	return IRQ_HANDLED;
}

static irqreturn_t dnt900_cts_handler(int irq, void *dev_id)
{
	// struct spi_device *spi = dev_id;
	// struct dnt900_driver *driver = spi_get_drvdata(spi);
	// dnt900_send_queued_packets(driver);
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

	driver->spi = spi;
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

	error = dnt900_alloc_device(driver, 0x000000, 0x000000, 1); // TODO: get local mac address & sys address?
	if (error)
		goto fail_alloc_device;
	 
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

	error = request_irq(gpio_to_irq(driver->gpio_avl), dnt900_avl_handler, IRQF_DISABLED | IRQF_TRIGGER_RISING, driver_name, spi);
	if (error)
		goto fail_irq_avl;
	error = request_irq(gpio_to_irq(driver->gpio_cts), dnt900_cts_handler, IRQF_DISABLED | IRQF_TRIGGER_FALLING, driver_name, spi);
	if (error)
		goto fail_irq_cts;

	// code to verify dnt900 device is present?
	// code to initialise dnt900 device?
	// device->mac_address = XXX; // obtain from dnt900 via protocol message

	return 0;

fail_irq_cts:
	free_irq(gpio_to_irq(driver->gpio_avl), driver);
fail_irq_avl:
	gpio_free(driver->gpio_cts);
fail_gpio_cts:
	gpio_free(driver->gpio_avl);
fail_gpio_avl:
	gpio_free(driver->gpio_cfg);
fail_gpio_cfg:
	dnt900_free_devices(driver);
fail_alloc_device:
fail_spi_setup:
	unregister_chrdev_region(MKDEV(driver->major, 0), members);
fail_alloc_chrdev_region:
	kfree(driver);
fail_driver_alloc:
	return error;
}

static int dnt900_remove(struct spi_device *spi)
{
	struct dnt900_driver *driver = spi_get_drvdata(spi);

	// code to sleep the dnt900 here?
	free_irq(gpio_to_irq(driver->gpio_avl), spi);
	free_irq(gpio_to_irq(driver->gpio_cts), spi);
	gpio_free(driver->gpio_cts);
	gpio_free(driver->gpio_avl);
	gpio_free(driver->gpio_cfg);
	dnt900_free_devices(driver);
	unregister_chrdev_region(MKDEV(driver->major, 0), members);
	kfree(driver);
	return 0;
}

static struct spi_driver dnt900_driver = {
	.driver = {
		.name = driver_name,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = dnt900_probe,
	.remove = __devexit_p(dnt900_remove),
};

int __init dnt900_init(void)
{
	// TODO: error handling!
	printk(KERN_INFO "inserting dnt900 module\n");
	dnt900_class = class_create(THIS_MODULE, class_name);
	// dnt900_class->dev_attrs = dnt900_attribs;
	spi_register_driver(&dnt900_driver);
	return 0;
}

void __exit dnt900_exit(void)
{
	printk(KERN_INFO "removing dnt900 module\n");
	spi_unregister_driver(&dnt900_driver);
	class_destroy(dnt900_class);
}

module_init(dnt900_init);
module_exit(dnt900_exit);

MODULE_AUTHOR("Matthew Hollingworth");
MODULE_DESCRIPTION("SPI driver for DNT900 RF module");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
