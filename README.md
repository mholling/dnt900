Summary
=======

This code implements a Linux driver (technically, a [tty line discipline](http://en.wikipedia.org/wiki/Line_discipline)) for use with the [DNT900](http://www.rfm.com/products/spec_sheet.php?record=DNT900P) series of 900 MHz radio transceiver modules, produced by [RFM](http://www.rfm.com/). The line discipline allows you to connect to the radio module via serial port. Each radio present on the DNT900 network is then presented to user-space as a character device to which data can be written and from which data can be read. The configuration registers for each radio are also presented as attribute files in sysfs, enabling the configuration of each radio to be read and changed.

The DNT900 series includes the DNT900P and the DNT900C. This software is also likely to be compatible with the 2.4GHz [DNT2400](http://www.rfm.com/products/spec_sheet.php?record=DNT2400P) series (DNT2400P and DNT2400C), since they appear to use the same communication protocol and registers.

Building the Module
===================

To build the module, your linux system should be set up for kernel development. This is probably as simple as installing the `kernel-devel` package, or its equivalent in your distribution.

All going well, running the provided makefile should result in a sucessfully built kernel module called `dnt900.ko`:

    $ make
    make -C /lib/modules/3.2.27-13-ARCH+/build M=/home/matthew/dnt900 modules
    make[1]: Entering directory `/usr/src/linux-3.2.27-13-ARCH+'
      CC [M]  /home/matthew/dnt900/dnt900.o
      Building modules, stage 2.
      MODPOST 1 modules
      CC      /home/matthew/dnt900/dnt900.mod.o
      LD [M]  /home/matthew/dnt900/dnt900.ko
    make[1]: Leaving directory `/usr/src/linux-3.2.27-13-ARCH+'

Loading the module is then a simple matter:

    $ sudo insmod dnt900.ko

Usage
=====

To use the line discipline, you must first attach it to the serial port to which your DNT900 radio is connected. First identify the name of your serial device. For example, if you are using the USB connection on the development kit, the serial device will probably be named `/dev/ttyUSB0`. (You can monitor `dmesg --follow` as you plug it in to determine the device name.)

Apply power to the radio and use the `ldattach` utility to attach the line discipline (which defaults to number 29) to the serial port as follows:

    $ ldattach -8n1 -s 9600 29 /dev/ttyUSB0

These options specify 8 bits, no parity, one stop bit, 9600 bps, which are the default serial settings for the DNT900; adjust as appropriate to your settings.

Once the line discipline has been successfully attached to your serial port, you can monitor dmesg to see when radios are detected and added. (Any warnings and error messages will also show up here.)

When you are done with the radio, kill the line discipline and (optionally) remove the module:

    $ killall ldattach
    $ sudo rmmod dnt900

In the following sections, we use as example a local DNT900 radio with MAC address `0x00165F`, attached to our computer via `/dev/ttyAMA0`. The radio is configured as a base and connects to a remote radio with MAC address `0x00165E`.

Setting Configuration Registers
===============================

The configuration registers of each radio on the network may be accessed via the sysfs file system, usually mounted at `/sys`. Each radio network is represented as a virtual device under the name of its serial port:

    $ ls -l /sys/devices/virtual/dnt900/ttyAMA0/
    total 0
    --w--w--w- 1 root root 4096 Dec 25 15:11 discover
    drwxr-xr-x 2 root root    0 Dec 25 15:11 power
    --w--w--w- 1 root root 4096 Dec 25 15:11 reset
    lrwxrwxrwx 1 root root    0 Dec 25 15:00 subsystem -> ../../../../class/dnt900
    drwxr-xr-x 3 root root    0 Dec 25 15:00 ttyAMA0.0x00165E
    drwxr-xr-x 3 root root    0 Dec 25 15:00 ttyAMA0.0x00165F
    lrwxrwxrwx 1 root root    0 Dec 25 15:11 ttyAMA0.local -> ttyAMA0.0x00165F
    -rw-r--r-- 1 root root 4096 Dec 25 15:00 uevent

This directory lists each radio on the network under its corresponding MAC address (e.g. `ttyAMA0.0x00165E` in this example). A symlink (`ttyAMA0.local`) for the locally connected radio is also listed for convenience.

The configuration registers for a radio are listed within its named subdirectory. These registers are described in detail in the [DNT900 Series Integration Guide](http://www.rfm.com/products/data/dnt900dk_manual.pdf).

    $ ls -l /sys/devices/virtual/dnt900/ttyAMA0/ttyAMA0.0x00165E/
    total 0
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ADC0
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ADC0_ThresholdHi
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ADC0_ThresholdLo
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ADC1
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ADC1_ThresholdHi
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ADC1_ThresholdLo
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ADC2
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ADC2_ThresholdHi
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ADC2_ThresholdLo
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ADC_SampleIntvl
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ARQ_AttemptLimit
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ARQ_Mode
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 AccessMode
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 AnnounceOptions
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr00
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr01
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr02
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr03
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr04
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr05
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr06
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr07
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr08
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr09
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr10
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr11
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr12
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr13
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr14
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ApprovedAddr15
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 AuthMode
    -r--r--r-- 1 root root 4096 Dec 25 15:20 AveRXPwrOvHopSeq
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 BaseModeNetID
    -r--r--r-- 1 root root 4096 Dec 25 15:20 BaseNetworkID
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 BaseSlotSize
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 CSMA_Backoff
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 CSMA_BusyThreshold
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 CSMA_Predelay
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 CSMA_RemtSlotSize
    -r--r--r-- 1 root root 4096 Dec 25 15:20 CurrAttemptLimit
    -r--r--r-- 1 root root 4096 Dec 25 15:20 CurrBaseModeNetID
    -r--r--r-- 1 root root 4096 Dec 25 15:20 CurrFreqBand
    -r--r--r-- 1 root root 4096 Dec 25 15:20 CurrNwkAddr
    -r--r--r-- 1 root root 4096 Dec 25 15:20 CurrNwkID
    -r--r--r-- 1 root root 4096 Dec 25 15:20 CurrRF_DataRate
    -r--r--r-- 1 root root 4096 Dec 25 15:20 CurrRangeDelay
    -r--r--r-- 1 root root 4096 Dec 25 15:20 CurrTxPower
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 DeviceMode
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 DiagSerialRate
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 DiversityMode
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 EnableRtAcks
    -r--r--r-- 1 root root 4096 Dec 25 15:20 Event_Flags
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ExtSyncEnable
    -r--r--r-- 1 root root 4096 Dec 25 15:20 FirmwareBuildDate
    -r--r--r-- 1 root root 4096 Dec 25 15:20 FirmwareBuildNum
    -r--r--r-- 1 root root 4096 Dec 25 15:20 FirmwareBuildTime
    -r--r--r-- 1 root root 4096 Dec 25 15:20 FirmwareVersion
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 FrequencyBand
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO0
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO1
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO2
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO3
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO4
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO5
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO_Alt
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO_Dir
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO_Edge_Trigger
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO_Init
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO_SleepDir
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO_SleepMode
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 GPIO_SleepState
    -r--r--r-- 1 root root 4096 Dec 25 15:20 HardwareVersion
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 HeartbeatIntrvl
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 HopDuration
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 IO_ReportInterval
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 IO_ReportPreDel
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 IO_ReportRepeat
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 IO_ReportTrigger
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 InitialParentNwkID
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 LeasePeriod
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 LinkDropThreshold
    -r--r--r-- 1 root root 4096 Dec 25 15:20 LinkStatus
    -r--r--r-- 1 root root 4096 Dec 25 15:20 MacAddress
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 MaxPktsPerHop
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 MaxPropDelay
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 MaxSlots
    --w--w--w- 1 root root 4096 Dec 25 15:20 MemorySave
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 MinPacketLength
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ModelNumber
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 P2PReplyTimeout
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 PWM0
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 PWM0_Init
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 PWM1
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 PWM1_Init
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentACKQual
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID01
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID02
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID03
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID04
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID05
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID06
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID07
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID08
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID09
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID10
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID11
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID12
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID13
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID14
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID15
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID16
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID17
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID18
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID19
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID20
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID21
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID22
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID23
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID24
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID25
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID26
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID27
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID28
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID29
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID30
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID31
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID32
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID33
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID34
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID35
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID36
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID37
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID38
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID39
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID40
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID41
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID42
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID43
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID44
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID45
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID46
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID47
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID48
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID49
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID50
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID51
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID52
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID53
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID54
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID55
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID56
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID57
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID58
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID59
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID60
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID61
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID62
    -r--r--r-- 1 root root 4096 Dec 25 15:20 ParentNetworkID63
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ProtocolMode
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ProtocolOptions
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 ProtocolSequenceEn
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 RF_DataRate
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RSSI_Idle
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RSSI_Last
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 RangingInterval
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 RegDenialDelay
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr00
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr01
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr02
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr03
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr04
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr05
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr06
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr07
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr08
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr09
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr10
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr11
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr12
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr13
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr14
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr15
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr16
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr17
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr18
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr19
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr20
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr21
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr22
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr23
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr24
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RegMACAddr25
    -r--r--r-- 1 root root 4096 Dec 25 15:20 RemoteSlotSize
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 RmtTransDestAddr
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 RoutingTableUpd
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 SPI_Divisor
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 SPI_MasterCmdLen
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 SPI_MasterCmdStr
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 SPI_Mode
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 SPI_Options
    --w--w--w- 1 root root 4096 Dec 25 15:20 SecurityKey
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 SerialControls
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 SerialParams
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 SerialRate
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 SleepMode
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 SleepModeOverride
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 StaticNetAddr
    -r--r--r-- 1 root root 4096 Dec 25 15:20 SuperframeCount
    -r--r--r-- 1 root root 4096 Dec 25 15:20 TDMA_CurrSlot
    -r--r--r-- 1 root root 4096 Dec 25 15:20 TDMA_NumSlots
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 TransLinkAnnEn
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 TransPtToPtMode
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 TreeRoutingEn
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 TreeRoutingSysID
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 TxPower
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 TxTimeout
    --w--w--w- 1 root root 4096 Dec 25 15:20 UcReset
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 UserTag
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 WakeLinkTimeout
    -rw-rw-rw- 1 root root 4096 Dec 25 15:20 WakeResponseTime
    -r--r--r-- 1 root root 4096 Dec 25 15:20 dev
    lrwxrwxrwx 1 root root    0 Dec 25 15:20 device -> ../../ttyAMA0
    drwxr-xr-x 2 root root    0 Dec 25 15:20 power
    lrwxrwxrwx 1 root root    0 Dec 25 15:00 subsystem -> ../../../../../class/dnt900
    -rw-r--r-- 1 root root 4096 Dec 25 15:00 uevent

Reading an attribute file initiates a radio transmission to retrieve the value of the register from the corresponding radio. For example, to read the current value of `ADC0` as a hex word:

    $ cat /sys/devices/virtual/dnt900/ttyAMA0/ttyAMA0.0x00165E/ADC0
    0x01E2

For registers which are writeable, changing their value is as simple as writing a new value to the attribute file. For example, to configure `GPIO1` as an output and set its value high:

    $ echo 0x02 > /sys/devices/virtual/dnt900/ttyAMA0/ttyAMA0.0x00165E/GPIO_Dir
    $ echo 0x01 > /sys/devices/virtual/dnt900/ttyAMA0/ttyAMA0.0x00165E/GPIO1

When you wish your configuration registers changes to be permanent, you will need to save them using the `MemorySave` register:

    $ echo 0x01 > /sys/devices/virtual/dnt900/ttyAMA0/ttyAMA0.0x00165E/MemorySave

Note that changing some configuration registers relating to the serial port or radio network may cause a temporary loss of connection to the radio. For example, changing the serial rate on the local radio to 115200 would result in the serial connection being lost, requiring that the line discipline be dropped and reattached at the new rate:

    $ echo 0x0004 > /sys/devices/virtual/dnt900/ttyAMA0/ttyAMA0.0x00165F/SerialRate 
    -bash: echo: write error: Connection timed out
    $ killall ldattach
    $ ldattach -8n1 -s 115200 29 /dev/ttyAMA0

(Note that a `MemorySave` would then be required to make this change permanent.)

Finally, two system attributes, `discover` and `reset`, are also available. Radios on the network are usually discovered and added automatically, however if this fails for any reason a radio may be added manually by writing its MAC address to the `discover` attribute file:

    $ echo 0x00165A > /sys/devices/virtual/dnt900/ttyAMA0/discover

A software reset may be issued to the local radio by writing (anything) to the `reset` attribute file:

    $ echo 1 > /sys/devices/virtual/dnt900/ttyAMA0/reset

Note that issuing a software reset may cause the radio to fail to restart properly, requiring a power cycle. This is due to the DNT900's power-on reset requirements, which state that the `RADIO_TXD` pin must remain low for 10ms after a reset. Depending on your serial port and driver, this requirement may not be met. (Specifically, this can occur if your serial port sets a pull-up on its receive line.) The USB interface to the development kit does not exhibit this problem.

Transmitting and Receiving Data
===============================

Data transmissions to and from remote radios are implemented using a character device for each radio on the network. Loading the line discipline causes the network to be interrogated for information on all radios in the network. In our example, we get the following:

    $ ls -l /dev/ttyAMA0*
    crw-rw-rw- 1 root uucp 204, 64 Jan  1  1970 /dev/ttyAMA0
    crw-rw-rw- 1 root root 250,  1 Dec 25 15:00 /dev/ttyAMA0.0x00165E
    crw-rw-rw- 1 root root 250,  0 Dec 25 15:00 /dev/ttyAMA0.0x00165F

We can send data to the remote radio by writing to `/dev/ttyAMA0.0x00165E`. For example:

    $ echo "hello, world" > /dev/ttyAMA0.0x00165E

We may also use the same file to read any data sent by the remote radio:

    $ cat /dev/ttyAMA0.0x00165E

Since there is no data transmission to and from the local radio, its character device is used for transmitting broadcast messages (which are sent to all radios on the network). For example, we can broadcast data to all remotes as follows:

    $ echo "some broadcast message" > /dev/ttyAMA0.0x00165F

This same file is used to reveice event messages posted by the local radio. In our example, the following messages are emitted when a remote radio joins the network (in tree routing mode) and begins transmitting heartbeats:

    $ cat /dev/ttyAMA0.0x00165F 
    - event: remote joined
      code: 0xA2
      MAC address: 0x00165E
      range: 0 km
    - event: received heartbeat
      code: 0xA8
      MAC address: 0x00165E
      network address: 0x00
      network ID: 0x01
      parent network ID: 0x00
      received RSSI: -39 dBm
      reported RSSI: -57 dBm
      packet success rate: 100%
      range: 0 km

Whilst human-readable, these messages are in [YAML](http://en.wikipedia.org/wiki/YAML) format and should be easy to parse in a user-space application.

Flow Control
============

Per the [DNT900 manual](http://www.rfm.com/products/data/dnt900dk_manual.pdf), it is highly recommended that hardware flow control be used if you intend to use your radio network for high-volume data transmission. (This means connecting the `/HOST_CTS` signal on the radio to a `/CTS` line on your serial port). If you are using hardware flow control, you should enable it before loading the line discipline, as follows:

    $ stty -F /dev/ttyUSB0 crtscts

It is also possible to enable 'out-of-band' `/HOST_CTS` flow control using a GPIO, should your serial port not include flow control signals. You can specify the GPIO to be used with a module parameter (see below).

Module Parameters
=================

Several module parameters are available:

    $ modinfo --parameters dnt900.ko 
    radios:maximum number of radios (int)
    n_dnt900:line discipline number (int)
    gpio_cts:GPIO number for /HOST_CTS signal (int)

You can specify the maximum number of radios allowed using the `radios` parameter (default = 255). You can specify a line discipline number to be used with the `n_dnt900` parameter (default = 29); the linux kernel allows at most 30 line disciplines, the first 17 of which are already in use. If you have connected the radio's `/HOST_CTS` to a GPIO for flow control, set the number of that GPIO using `gpio_cts`.

For example, to connect the DNT900 to `/dev/ttyAMA0` on a [Raspberry Pi](http://www.raspberrypi.org/) using a line discipline number of 20 and GPIO27 as `/HOST_CTS`:

    $ sudo insmod dnt900.ko n_dnt900=20 gpio_cts=27
    $ ldattach -8n1 -s 115200 20 /dev/ttyAMA0

Caveats
=======

As far as possible, this module takes a 'hands-off' approach to the attached radio. It does not independently alter any of the radio's configuration registers. However, a small number of the registers do affect the operation of the software.

* `ProtocolOptions` must have bit 0 set in order to enable protocol message announcements (which are used by the software).
* `AnnounceOptions` must be set to 0x07 to enable all announcement types.
* Unless you have grounded the radio's `/CFG` pin, the `ProtocolSequenceEn` register must be set to 0x02. (It is by default). This allows the *EnterProtocolMode* ASCII command string to be used to switch the radio to protocol mode.
* Bit 2 of `ProtocolOptions` affects data transmission from the local radio. If bit 2 is set, *TxReply* messages are generated by the radio and these checked for acknowledgment of transmitted packets. If the bit is clear, packet acknowledgements are not checked. This yields faster data transmission rates, but possibly at the expense of data integrity. (This is only likely to be a problem in a congested network or when the link with the remote is poor. The `ARQ_Mode` register may also be of interest in this situation.)

Communication with the radio is in protocol mode. Ideally you should select this mode permanently by configuring the radio's `ProtocolMode` register:

    $ echo 0x01 > /sys/devices/virtual/dnt900/ttyAMA0/ttyAMA0.0x00165F/ProtocolMode
    $ echo 0x01 > /sys/devices/virtual/dnt900/ttyAMA0/ttyAMA0.0x00165F/MemorySave

(Alternatively, you can ground the radio's `/CFG` pin in your hardware design.) This is not compulsory as the *EnterProtocolMode* command is also issued. However, unless you do so, use of an external reset source such as a reset button will render the radio connection unresponsive.

Finally, if the local radio is configured as a remote in *TDMA Dynamic Slots* mode, data transmission may not succeed if new remotes join the network. (This is due to dynamic changes in the remote slot size in this access mode.)

Release History
===============

* 27/12/2012: version 0.1 (initial release): undoubtedly, not bug-free
  * 6/1/2013: version 0.1.1: improved handling of radio resets; removed gpio_cfg parameter; fixed bug preventing operation of remote radio in tree routing mode.
  * 8/4/2013: version 0.1.2: fixed bug whereby signals were ignored during device writes.
