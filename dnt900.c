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
#include <linux/semaphore.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
// #include <linux/proc_fs.h>
// #include <linux/fcntl.h>
// #include <asm/system.h>
// #include <asm/uaccess.h>

#define RX_BUFFER_SIZE (512)
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

#define EMPTY_PACKET_BYTES (32)

#define CIRC_OFFSET(index, offset, end) ((tail) + (offset) % (end))

static int members = 126;
static int spi_hz = 38400;
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
static const char remote_name[] = "dnt900.0x%06X";

static struct class *dnt900_class;

struct dnt900_packet {
    struct spi_device *spi;
    struct spi_transfer transfer;
    struct spi_message message;
    struct list_head list;
    struct semaphore completed;
    char *result;
    int status;
};

struct dnt900_driver {
    int major;
    int minor;
    
    unsigned gpio_cfg;
    unsigned gpio_avl;
    unsigned gpio_cts;
    
    struct list_head unsent_packets;
    struct list_head sent_packets;
    
    spinlock_t lock;
    
    char rx_buf[RX_BUFFER_SIZE];
    int rx_head;
    int rx_tail;
    int rx_end;
    
    struct dnt900_packet empty_packet;
};

struct dnt900_device {
    struct device *dev;
    struct cdev cdev;
    int is_local;
    unsigned int mac_address;
};

static irqreturn_t dnt900_avl_handler(int, void *);
static irqreturn_t dnt900_cts_handler(int, void *);

static int dnt900_probe(struct spi_device *spi)
{
    int error = 0;
    struct dnt900_driver *driver;
    struct dnt900_device *device;
    dev_t devt;
    
    driver = kzalloc(sizeof(*driver), GFP_KERNEL);
    if (!driver) {
        error = -ENOMEM;
        goto fail_driver_alloc;
    }
    
    spin_lock_init(&driver->lock);
    
    INIT_LIST_HEAD(&driver->unsent_packets);
    INIT_LIST_HEAD(&driver->sent_packets);
    
    driver->rx_end = RX_BUFFER_SIZE;
    
    error = alloc_chrdev_region(&devt, 0, members, driver_name);
    if (error) goto fail_alloc_chrdev_region;
    driver->major = MAJOR(devt);
    driver->minor = MINOR(devt);

    spi_set_drvdata(spi, driver);
    
    spi->max_speed_hz = spi_hz;
    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 8;
    error = spi_setup(spi);
    if (error) goto fail_spi_setup;
    
    device = kzalloc(sizeof(*device), GFP_KERNEL);
    if (!device) {
        error = -ENOMEM;
        goto fail_device_alloc;
    }
    
    // code to verify dnt900 device is present
    // code to claim and configure GPIOs for /CFG and SPI_RX_AVL
    // code to initialise dnt900 device
    
    // device->mac_address = XXX; // obtain from dnt900 via protocol message
    device->is_local = 1;
    device->dev = device_create(dnt900_class, &spi->dev, MKDEV(0, 0), device, local_name);
    if (IS_ERR(device->dev)) {
        error = PTR_ERR(device->dev);
        goto fail_device_create;
    } 
     
    driver->gpio_cfg = gpio_cfg; // how to make this per-instance?
    driver->gpio_avl = gpio_avl; // how to make this per-instance?
    
    error = gpio_request_one(driver->gpio_cfg, GPIOF_OUT_INIT_HIGH, "/cfg");
    if (error) goto fail_gpio_cfg;
    error = gpio_request_one(driver->gpio_avl, GPIOF_IN, "rx_avl");
    if (error) goto fail_gpio_avl;
    error = gpio_request_one(driver->gpio_cts, GPIOF_IN, "/host_cts");
    if (error) goto fail_gpio_cts;
    error = request_irq(gpio_to_irq(driver->gpio_avl), dnt900_avl_handler, IRQF_DISABLED | IRQF_TRIGGER_RISING, driver_name, spi);
    if (error) goto fail_irq_avl;
    error = request_irq(gpio_to_irq(driver->gpio_cts), dnt900_cts_handler, IRQF_DISABLED | IRQF_TRIGGER_FALLING, driver_name, spi);
    if (error) goto fail_irq_cts;
    
    gpio_set_value(driver->gpio_cfg, 0); // enter protocol mode
    
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
    device_unregister(device->dev);
fail_device_create:
    kfree(device);
fail_device_alloc:
fail_spi_setup:
    unregister_chrdev_region(MKDEV(driver->major, 0), members);
fail_alloc_chrdev_region:
    kfree(driver);
fail_driver_alloc:
    return error;
}

static int dnt900_remove_child(struct device *child, void *unused)
{
    struct dnt900_device *device = dev_get_drvdata(child);
    if (!device->is_local)
        cdev_del(&device->cdev);
    device_unregister(child);
    kfree(device);
    return 0;
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
    device_for_each_child(&spi->dev, NULL, dnt900_remove_child);
    unregister_chrdev_region(MKDEV(driver->major, 0), members);
    kfree(driver);
    return 0;
}

static void dnt900_packet_complete(void *);

static void dnt900_init_packet(struct dnt900_packet *packet, struct spi_device *spi, unsigned len, const void *tx_buf, char *result)
{
    memset(packet, 0, sizeof(*packet));
    INIT_LIST_HEAD(&packet->message.transfers);
    packet->message.context = packet;
    packet->message.complete = dnt900_packet_complete;
    packet->transfer.len = len;
    packet->transfer.tx_buf = tx_buf;
    packet->transfer.cs_change = 1;
    spi_message_add_tail(&packet->transfer, &packet->message);
    INIT_LIST_HEAD(&packet->list);
    sema_init(&packet->completed, 0);
    packet->spi = spi;
    packet->result = result;
    packet->status = 0;
}

static void dnt900_send_queued_packets(struct spi_device *spi)
{
    struct dnt900_driver *driver = spi_get_drvdata(spi);
    struct dnt900_packet *packet;
    unsigned long flags;
    
    printk(KERN_INFO "checking /CTS\n");
    if (gpio_get_value(driver->gpio_cts) != 0)
        return;
    printk(KERN_INFO "/CTS asserted\n");
    
    printk(KERN_INFO "checking for a packet to send\n");
    spin_lock_irqsave(&driver->lock, flags);
    if (list_empty(&driver->unsent_packets) && gpio_get_value(driver->gpio_avl)) {
        dnt900_init_packet(&driver->empty_packet, spi, EMPTY_PACKET_BYTES, NULL, NULL);
        printk(KERN_INFO "inserting empty packet\n");
        list_add_tail(&driver->empty_packet.list, &driver->unsent_packets);
    }
    packet = list_empty(&driver->unsent_packets) ? NULL : list_first_entry(&driver->unsent_packets, struct dnt900_packet, list);
    spin_unlock_irqrestore(&driver->lock, flags);
    
    if (packet) {
        printk(KERN_INFO "found a packet\n");
        
        if (driver->rx_head + packet->transfer.len < RX_BUFFER_SIZE) {
            packet->transfer.rx_buf = driver->rx_buf + driver->rx_head;
            driver->rx_head += packet->transfer.len;
        } else {
            driver->rx_end = driver->rx_head;
            driver->rx_head = packet->transfer.len;
            packet->transfer.rx_buf = driver->rx_buf;
        }
        
        spi_async(spi, &packet->message);
        printk(KERN_INFO "queued packet to send on SPI\n");
    }
}

static void dnt900_packet_complete(void *context)
{
    struct dnt900_packet *packet = context;
    struct spi_device *spi = packet->spi;
    struct dnt900_driver *driver = spi_get_drvdata(spi);
    const int head = driver->rx_head;
    int end = driver->rx_end;
    int tail = driver->rx_tail;
    unsigned long flags;
    printk(KERN_INFO "packet sent\n");
    
    // TODO: check packet->message.actual_length agains packet->transfer.len?
    
    spin_lock_irqsave(&driver->lock, flags);
    if (packet->message.status == 0 && packet->transfer.tx_buf)
        list_move_tail(&packet->list, &driver->sent_packets);
    else
        list_del(&packet->list);
    spin_unlock_irqrestore(&driver->lock, flags);
    printk(KERN_INFO "packet moved/deleted\n");
    
    if (packet->message.status != 0) {
        packet->status = packet->message.status;
        up(&packet->completed);
    }
    
    printk(KERN_INFO "consuming packets...\n");
    while (head != tail) {
        int bytes, length, type;
        
        printk(KERN_INFO "reading rx_buf\n");
        for (; tail != head && driver->rx_buf[tail] != START_OF_PACKET; tail = CIRC_OFFSET(tail, 1, end))
            ;
        printk(KERN_INFO "reading complete\n");
        
        if (head == tail)
            break;
        printk(KERN_INFO "found START_OF_PACKET\n");
        break;
        
        printk(KERN_INFO "found start of packet\n");
        bytes = head > tail ? head - tail : end - tail + head;
        length = driver->rx_buf[CIRC_OFFSET(tail, 1, end)] + 2;
        type = driver->rx_buf[CIRC_OFFSET(tail, 2, end)];
        
        if (bytes < 2 || length >= bytes)
            break;
        
        printk(KERN_INFO "found whole packet\n");
        if (type & TYPE_REPLY) {
            const int command = type & TYPE_COMMAND;
            struct dnt900_packet *packet;
            list_for_each_entry(packet, &driver->sent_packets, list) {
                const char *tx_buf = packet->transfer.tx_buf;
                int index;
                printk(KERN_INFO "checking against a sent packet\n");
                if (tx_buf[2] != command)
                    continue;
                switch (command) {
                    
                case COMMAND_SOFTWARE_RESET:
                    packet->status = 0;
                    break;
                    
                case COMMAND_GET_REGISTER:
                    // match Reg, Bank, span:
                    if (tx_buf[3] != driver->rx_buf[CIRC_OFFSET(tail, 3, end)] \
                     || tx_buf[4] != driver->rx_buf[CIRC_OFFSET(tail, 4, end)] \
                     || tx_buf[5] != driver->rx_buf[CIRC_OFFSET(tail, 5, end)])
                        continue;
                    for (index = 0; index < tx_buf[5]; ++index)
                        packet->result[index] = driver->rx_buf[CIRC_OFFSET(tail, 6 + index, end)];
                    packet->status = 0;
                    break;
                    
                case COMMAND_SET_REGISTER:
                    packet->status = 0;
                    break;
                    
                case COMMAND_TX_DATA:
                    // match Addr:
                    if (tx_buf[3] != driver->rx_buf[CIRC_OFFSET(tail, 4, end)] \
                     || tx_buf[4] != driver->rx_buf[CIRC_OFFSET(tail, 5, end)] \
                     || tx_buf[5] != driver->rx_buf[CIRC_OFFSET(tail, 6, end)])
                        continue;
                    packet->status = driver->rx_buf[CIRC_OFFSET(tail, 3, end)];
                    break;
                    
                case COMMAND_DISCOVER:
                    // match MacAddr:
                    if (tx_buf[3] != driver->rx_buf[CIRC_OFFSET(tail, 4, end)] \
                     || tx_buf[4] != driver->rx_buf[CIRC_OFFSET(tail, 5, end)] \
                     || tx_buf[5] != driver->rx_buf[CIRC_OFFSET(tail, 6, end)])
                        continue; 
                    packet->status = driver->rx_buf[CIRC_OFFSET(tail, 3, end)];
                    if (packet->status == STATUS_ACKNOWLEDGED)
                        for (index = 0; index < 3; ++index)
                            packet->result[index] = driver->rx_buf[CIRC_OFFSET(tail, 7 + index, end)];
                    break;
                    
                case COMMAND_GET_REMOTE_REGISTER:
                    // match Addr:
                    if (tx_buf[3] != driver->rx_buf[CIRC_OFFSET(tail, 4, end)] \
                     || tx_buf[4] != driver->rx_buf[CIRC_OFFSET(tail, 5, end)] \
                     || tx_buf[5] != driver->rx_buf[CIRC_OFFSET(tail, 6, end)])
                        continue;
                    packet->status = driver->rx_buf[CIRC_OFFSET(tail, 3, end)];
                    if (packet->status == STATUS_ACKNOWLEDGED)
                        for (index = 0; index < tx_buf[8]; ++index)
                            packet->result[index] = driver->rx_buf[CIRC_OFFSET(tail, 11 + index, end)];
                    break;
                    
                case COMMAND_SET_REMOTE_REGISTER:
                    // match Addr:
                    if (tx_buf[3] != driver->rx_buf[CIRC_OFFSET(tail, 4, end)] \
                     || tx_buf[4] != driver->rx_buf[CIRC_OFFSET(tail, 5, end)] \
                     || tx_buf[5] != driver->rx_buf[CIRC_OFFSET(tail, 6, end)])
                        continue;
                    packet->status = driver->rx_buf[CIRC_OFFSET(tail, 3, end)];
                    break;
                    
                }
                printk(KERN_INFO "matched a sent packet\n");
                list_del(&packet->list);
                up(&packet->completed); // TODO: can we do this in interrupt context?
                break;
            }
        }
        
        if (type & TYPE_EVENT) {
            // handle events
        }
        
        tail += length;
        if (tail >= end) {
            tail -= end;
            end = RX_BUFFER_SIZE;
        }
    }
    printk(KERN_INFO "all packets consumed\n");
    
    driver->rx_tail = tail;
    driver->rx_end = end;
    
    dnt900_send_queued_packets(spi);
}

static irqreturn_t dnt900_avl_handler(int irq, void *dev_id)
{
    struct spi_device *spi = dev_id;
    dnt900_send_queued_packets(spi);
    return IRQ_HANDLED;
}

static irqreturn_t dnt900_cts_handler(int irq, void *dev_id)
{
    struct spi_device *spi = dev_id;
    dnt900_send_queued_packets(spi);
    return IRQ_HANDLED;    
}

static int dnt900_send_command(struct spi_device *spi, char *buf, char *result)
{
    struct dnt900_driver *driver = spi_get_drvdata(spi);
    struct dnt900_packet packet;
    int interrupted;
    unsigned long flags;
    
    dnt900_init_packet(&packet, spi, buf[1] + 2, buf, result);
    spin_lock_irqsave(&driver->lock, flags);
    list_add_tail(&packet.list, &driver->unsent_packets);
    spin_unlock_irqrestore(&driver->lock, flags);
    dnt900_send_queued_packets(spi);
    interrupted = down_interruptible(&packet.completed);
    return interrupted ? interrupted : packet.status;
}

static int dnt900_get_register(struct spi_device *spi, char reg, char bank, char span, char *result)
{
    char cmd[6] = { START_OF_PACKET, 4, COMMAND_GET_REGISTER, reg, bank, span };
    return dnt900_send_command(spi, cmd, result);
}

// static ssize_t dnt900_read_attribute(struct device *dev, struct device_attribute *attr, char *buf)
// {
//     struct spi_device *spi = to_spi_device(dev->parent);
//     char reg, bank, span;
//     char result[32];
//     // TODO: code to match attribute name and set reg/bank/span accordingly (use gperf?)
//     dnt900_get_register(spi, reg, bank, span, result);
//     // TODO: code to convert result to string
//     return XXX;
// }

static ssize_t dnt900_test(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct spi_device *spi = to_spi_device(dev->parent);
    char result[32];
    dnt900_get_register(spi, 0x00, 0x01, 1, result);
    return sprintf(buf, "0x%2X", result[0]);
}

static ssize_t dnt900_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct spi_device *spi = to_spi_device(dev->parent);
    char cmd[4] = { START_OF_PACKET, 2, COMMAND_SOFTWARE_RESET, 0 };
    dnt900_send_command(spi, cmd, NULL);
    return count;
}

static int dnt900_open(struct inode *inode, struct file *filp)
{
    struct dnt900_device *device = container_of(inode->i_cdev, struct dnt900_device, cdev);
    filp->private_data = device;
    return 0;
}

static ssize_t dnt900_read(struct file *filp, char __user *buf, size_t count, loff_t *offp)
{
    // struct dnt900_device *device = filp->private_data;
    // send TxData packet
    return 0;
}

static ssize_t dnt900_write(struct file *filp, const char __user *buf, size_t count, loff_t *offp)
{
    // struct dnt900_device *device = filp->private_data;
    // pull data from buffer
    return count;
}

static struct file_operations dnt900_fops = {
    .owner = THIS_MODULE,
    .open = dnt900_open,
    .read = dnt900_read,
    .write = dnt900_write
};

static ssize_t dnt900_discover(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int error = 0;
    struct spi_device *spi = to_spi_device(dev->parent);
    struct dnt900_driver *driver = spi_get_drvdata(spi);
    struct dnt900_device *device;
    unsigned int mac_address;
    dev_t devt;
    
    if (sscanf(buf, "0x%6x\n", &mac_address) != 1 || (mac_address & ~0xFFFFFF) != 0)
        goto fail_input;
    if (false /* TODO: send Discover message to verify existence */)
        goto fail_discovery;
    if (false /* TODO: check mac address corresponds to a remote (not the local dnt900) */)
        goto fail_nonlocal;
        
    device = kzalloc(sizeof(*device), GFP_KERNEL);
    if (!device) {
        error = -ENOMEM;
        goto fail_device_alloc;
    }
    
    device->mac_address = mac_address;
    device->is_local = 0;
    devt = MKDEV(driver->major, driver->minor++);
    device->dev = device_create(dnt900_class, &spi->dev, devt, device, remote_name, mac_address);
    if (IS_ERR(device->dev)) {
        error = PTR_ERR(device->dev);
        goto fail_device_create;
    }

    cdev_init(&device->cdev, &dnt900_fops);
    error = cdev_add(&device->cdev, devt, 1);
    if (error) goto fail_cdev_add;
    
    return count;
    
fail_cdev_add:
    device_unregister(device->dev);
fail_device_create:
    kfree(device);
fail_device_alloc:
fail_nonlocal:
fail_discovery:
fail_input:
    return count;
}

static struct device_attribute dnt900_attribs[] = {
    __ATTR(reset, S_IWUSR|S_IWGRP, NULL, dnt900_reset),
    __ATTR(discover, S_IWUSR|S_IWGRP, NULL, dnt900_discover),
    __ATTR(test, S_IRUSR|S_IRGRP, dnt900_test, NULL),
    __ATTR_NULL
};

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
    dnt900_class = class_create(THIS_MODULE, class_name);
    dnt900_class->dev_attrs = dnt900_attribs;
    spi_register_driver(&dnt900_driver);
    printk(KERN_INFO "inserting dnt900 module\n");
    return 0;
}

void __exit dnt900_exit(void)
{
    spi_unregister_driver(&dnt900_driver);
    class_destroy(dnt900_class);
    printk(KERN_INFO "removing dnt900 module\n");
}

module_init(dnt900_init);
module_exit(dnt900_exit);

MODULE_AUTHOR("Matthew Hollingworth");
MODULE_DESCRIPTION("SPI driver for DNT900 RF module");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
