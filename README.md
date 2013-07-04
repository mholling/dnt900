Summary
=======

This code implements a Linux driver (technically, a [tty line discipline](http://en.wikipedia.org/wiki/Line_discipline)) for use with the [DNT900](http://www.rfm.com/products/spec_sheet.php?record=DNT900P) series of 900 MHz radio transceiver modules, produced by [RFM](http://www.rfm.com/). The line discipline allows you to connect to the radio module via serial port. Each radio present on the DNT900 network is then presented to user-space as a new tty device to which data can be written and from which data can be read. The configuration registers for each radio are also presented as attribute files in sysfs, enabling the configuration of each radio to be read and changed.

The DNT900 series includes the DNT900P and the DNT900C. This software is also likely to be compatible with the 2.4GHz [DNT2400](http://www.rfm.com/products/spec_sheet.php?record=DNT2400P) series (DNT2400P and DNT2400C), since they appear to use the same communication protocol and registers.

Building the Module
===================

To build the module, your linux system should be set up for kernel development. This is probably as simple as installing the `kernel-devel` package, or its equivalent in your distribution.

All going well, running the provided makefile should result in a sucessfully built kernel module called `dnt900.ko`:

    $ make
    make -C /lib/modules/3.6.11-8-ARCH+/build M=/home/matthew/dnt900 modules
    make[1]: Entering directory `/usr/src/linux-3.6.11-8-ARCH+'
      CC [M]  /home/matthew/dnt900/dnt900.o
      Building modules, stage 2.
      MODPOST 1 modules
      CC      /home/matthew/dnt900/dnt900.mod.o
      LD [M]  /home/matthew/dnt900/dnt900.ko
    make[1]: Leaving directory `/usr/src/linux-3.6.11-8-ARCH+'

Loading the module is then a simple matter:

    $ sudo insmod dnt900.ko

Usage
=====

To use the line discipline, you must first attach it to the serial port to which your DNT900 radio is connected. First identify the name of your serial device. For example, if you are using the USB connection on the development kit, the serial device will probably be named `/dev/ttyUSB0`. (You can monitor `dmesg --follow` as you plug it in to determine the device name.)

Apply power to the radio and use the `ldattach` utility to attach the line discipline (which defaults to number 29) to the serial port as follows:

    $ ldattach -8n1 -s 9600 29 /dev/ttyUSB0

These options specify 8 bits, no parity, one stop bit, 9600 bps, which are the default serial settings for the DNT900; adjust as appropriate to your settings.

Once the line discipline has been successfully attached to your serial port, you can monitor `dmesg` to see when radios are detected and added. (Any warnings and error messages will also show up here.)

When you are done with the radio, kill the line discipline and (optionally) remove the module:

    $ killall ldattach
    $ sudo rmmod dnt900

In the following sections, we use as example a local DNT900 radio with MAC address `0x00165F`, attached to our computer via `/dev/ttyAMA0`. The radio is configured as a base and connects to a remote radio with MAC address `0x00165E`.

Setting Configuration Registers
===============================

The configuration registers of each radio on the network may be accessed via the sysfs file system, usually mounted at `/sys`. Each radio network is represented as a virtual device under the name of its serial port:

    $ ls -l /sys/class/dnt900/ttyAMA0/
    total 0
    drwxr-xr-x 3 root root    0 Apr 16 15:25 0x00165E
    drwxr-xr-x 4 root root    0 Apr 16 15:25 0x00165F
    --w------- 1 root root 4096 Apr 16 15:25 discover
    lrwxrwxrwx 1 root root    0 Apr 16 15:25 local -> 0x00165F
    drwxr-xr-x 2 root root    0 Apr 16 15:25 power
    --w------- 1 root root 4096 Apr 16 15:25 reset
    lrwxrwxrwx 1 root root    0 Apr 16 15:25 subsystem -> ../../../../class/dnt900
    -rw-r--r-- 1 root root 4096 Apr 16 15:25 uevent    

This directory lists each radio on the network under its corresponding MAC address (`0x00165E` and `0x00165F` in this example). A symlink (`local`) for the locally connected radio is also listed for convenience.

The configuration registers for a radio are listed within its named subdirectory. These registers are described in detail in the [DNT900 Series Integration Guide](http://www.rfm.com/products/data/dnt900dk_manual.pdf).

    $ ls -l /sys/class/dnt900/ttyAMA0/0x00165E/
    total 0
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ADC0
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ADC0_ThresholdHi
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ADC0_ThresholdLo
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ADC1
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ADC1_ThresholdHi
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ADC1_ThresholdLo
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ADC2
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ADC2_ThresholdHi
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ADC2_ThresholdLo
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ADC_SampleIntvl
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ARQ_AttemptLimit
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ARQ_Mode
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 AccessMode
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 AnnounceOptions
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr00
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr01
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr02
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr03
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr04
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr05
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr06
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr07
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr08
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr09
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr10
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr11
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr12
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr13
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr14
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ApprovedAddr15
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 AuthMode
    -r--r--r-- 1 root root 4096 Apr 16 15:29 AveRXPwrOvHopSeq
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 BaseModeNetID
    -r--r--r-- 1 root root 4096 Apr 16 15:29 BaseNetworkID
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 BaseSlotSize
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 CSMA_Backoff
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 CSMA_BusyThreshold
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 CSMA_Predelay
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 CSMA_RemtSlotSize
    -r--r--r-- 1 root root 4096 Apr 16 15:29 CurrAttemptLimit
    -r--r--r-- 1 root root 4096 Apr 16 15:29 CurrBaseModeNetID
    -r--r--r-- 1 root root 4096 Apr 16 15:29 CurrFreqBand
    -r--r--r-- 1 root root 4096 Apr 16 15:29 CurrNwkAddr
    -r--r--r-- 1 root root 4096 Apr 16 15:29 CurrNwkID
    -r--r--r-- 1 root root 4096 Apr 16 15:29 CurrRF_DataRate
    -r--r--r-- 1 root root 4096 Apr 16 15:29 CurrRangeDelay
    -r--r--r-- 1 root root 4096 Apr 16 15:29 CurrTxPower
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 DeviceMode
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 DiagSerialRate
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 DiversityMode
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 EnableRtAcks
    -r--r--r-- 1 root root 4096 Apr 16 15:29 Event_Flags
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ExtSyncEnable
    -r--r--r-- 1 root root 4096 Apr 16 15:29 FirmwareBuildDate
    -r--r--r-- 1 root root 4096 Apr 16 15:29 FirmwareBuildNum
    -r--r--r-- 1 root root 4096 Apr 16 15:29 FirmwareBuildTime
    -r--r--r-- 1 root root 4096 Apr 16 15:29 FirmwareVersion
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 FrequencyBand
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO0
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO1
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO2
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO3
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO4
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO5
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO_Alt
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO_Dir
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO_Edge_Trigger
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO_Init
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO_SleepDir
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO_SleepMode
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 GPIO_SleepState
    -r--r--r-- 1 root root 4096 Apr 16 15:29 HardwareVersion
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 HeartbeatIntrvl
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 HopDuration
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 IO_ReportInterval
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 IO_ReportPreDel
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 IO_ReportRepeat
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 IO_ReportTrigger
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 InitialParentNwkID
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 LeasePeriod
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 LinkDropThreshold
    -r--r--r-- 1 root root 4096 Apr 16 15:29 LinkStatus
    -r--r--r-- 1 root root 4096 Apr 16 15:29 MacAddress
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 MaxPktsPerHop
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 MaxPropDelay
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 MaxSlots
    --w------- 1 root root 4096 Apr 16 15:29 MemorySave
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 MinPacketLength
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ModelNumber
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 P2PReplyTimeout
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 PWM0
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 PWM0_Init
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 PWM1
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 PWM1_Init
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID01
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID02
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID03
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID04
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID05
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID06
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID07
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID08
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID09
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID10
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID11
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID12
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID13
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID14
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID15
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID16
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID17
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID18
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID19
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID20
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID21
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID22
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID23
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID24
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID25
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID26
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID27
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID28
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID29
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID30
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID31
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID32
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID33
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID34
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID35
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID36
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID37
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID38
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID39
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID40
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID41
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID42
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID43
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID44
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID45
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID46
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID47
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID48
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID49
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID50
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID51
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID52
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID53
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID54
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID55
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID56
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID57
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID58
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID59
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID60
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID61
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID62
    -r--r--r-- 1 root root 4096 Apr 16 15:29 ParentNetworkID63
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ProtocolMode
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ProtocolOptions
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 ProtocolSequenceEn
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 RF_DataRate
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RSSI_Idle
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RSSI_Last
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 RangingInterval
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 RegDenialDelay
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr00
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr01
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr02
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr03
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr04
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr05
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr06
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr07
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr08
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr09
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr10
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr11
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr12
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr13
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr14
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr15
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr16
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr17
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr18
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr19
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr20
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr21
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr22
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr23
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr24
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RegMACAddr25
    -r--r--r-- 1 root root 4096 Apr 16 15:29 RemoteSlotSize
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 RmtTransDestAddr
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 RoutingTableUpd
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 SPI_Divisor
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 SPI_MasterCmdLen
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 SPI_MasterCmdStr
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 SPI_Mode
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 SPI_Options
    --w------- 1 root root 4096 Apr 16 15:29 SecurityKey
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 SerialControls
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 SerialParams
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 SerialRate
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 SleepMode
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 SleepModeOverride
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 StaticNetAddr
    -r--r--r-- 1 root root 4096 Apr 16 15:29 SuperframeCount
    -r--r--r-- 1 root root 4096 Apr 16 15:29 TDMA_CurrSlot
    -r--r--r-- 1 root root 4096 Apr 16 15:29 TDMA_NumSlots
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 TransLinkAnnEn
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 TransPtToPtMode
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 TreeRoutingEn
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 TreeRoutingSysID
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 TxPower
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 TxTimeout
    --w------- 1 root root 4096 Apr 16 15:29 UcReset
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 UserTag
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 WakeLinkTimeout
    -rw-r--r-- 1 root root 4096 Apr 16 15:29 WakeResponseTime
    lrwxrwxrwx 1 root root    0 Apr 16 15:29 device -> ../../ttyAMA0
    drwxr-xr-x 2 root root    0 Apr 16 15:29 power
    lrwxrwxrwx 1 root root    0 Apr 16 15:29 subsystem -> ../../../../../class/dnt900
    -rw-r--r-- 1 root root 4096 Apr 16 15:25 uevent

Reading an attribute file initiates a radio transmission to retrieve the value of the register from the corresponding radio. For example, to read the current value of `ADC0` as a hex word:

    $ cat /sys/class/dnt900/ttyAMA0/0x00165E/ADC0
    0x01E2

For registers which are writeable, changing their value is as simple as writing a new value to the attribute file. For example, to configure `GPIO1` as an output and set its value high:

    $ echo 0x02 > /sys/class/dnt900/ttyAMA0/0x00165E/GPIO_Dir
    $ echo 0x01 > /sys/class/dnt900/ttyAMA0/0x00165E/GPIO1

When you wish your configuration registers changes to be permanent, you will need to save them using the `MemorySave` register:

    $ echo 0x01 > /sys/class/dnt900/ttyAMA0/0x00165E/MemorySave

Note that changing some configuration registers relating to the serial port or radio network may cause a temporary loss of connection to the radio. For example, changing the serial rate on the local radio to 115200 would result in the serial connection being lost, requiring that the line discipline be dropped and reattached at the new rate:

    $ echo 0x0004 > /sys/class/dnt900/ttyAMA0/0x00165F/SerialRate 
    -bash: echo: write error: Connection timed out
    $ killall ldattach
    $ ldattach -8n1 -s 115200 29 /dev/ttyAMA0

(Note that a `MemorySave` would then be required to make this change permanent.)

Finally, two system attributes, `discover` and `reset`, are also available. Radios on the network are usually discovered and added automatically, however if this fails for any reason a radio may be added manually by writing its MAC address to the `discover` attribute file:

    $ echo 0x00165A > /sys/class/dnt900/ttyAMA0/discover

A software reset may be issued to the local radio by writing (anything) to the `reset` attribute file:

    $ echo 1 > /sys/class/dnt900/ttyAMA0/reset

Note that issuing a software reset may cause the radio to fail to restart properly, requiring a power cycle. This is due to the DNT900's power-on reset requirements, which state that the `RADIO_TXD` pin must remain low for 10ms after a reset. Depending on your serial port and driver, this requirement may not be met. (Specifically, this can occur if your serial port sets a pull-up on its receive line.) The USB interface to the development kit does not exhibit this problem.

Transmitting and Receiving Data
===============================

Data transmissions to and from remote radios are implemented using a virtual tty device for each non-local radio on the network. Loading the line discipline causes the network to be interrogated for information on all radios in the network. In our example, we get the following:

    $ ls -l /dev/ttyDNT*
    crw-rw-rw- 1 root uucp 248,  0 Apr 16 15:25 /dev/ttyDNT0

Here, `/dev/ttyDNT0` is a new tty representing the remote radio with MAC address `0x00165E`. We can send data to the remote radio by writing to the new tty. For example:

    $ echo "hello, world" > /dev/ttyDNT0

We may also use the same file to read any data sent by the remote radio:

    $ cat /dev/ttyDNT0

Since there is no data transmission to and from the local radio, the original tty is used for transmitting broadcast messages (which are sent to all radios on the network, at a considerably slower rate). For example, we can broadcast data to all remotes as follows:

    $ echo "some broadcast message" > /dev/ttyAMA0

This original tty is also used to receive event messages posted by the local radio. In our example, the following messages are emitted when a remote radio joins the network (in tree routing mode) and begins transmitting heartbeats:

    $ cat /dev/ttyAMA0 
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

udev Rules
==========

It is suggested to add [udev rules](http://www.reactivated.net/writing_udev_rules.html) to create more meaningful names for the remote ttys. Any of the radio register attributes (in particular, the MAC address) can be used in the rules. `udevadm` is helpful in writing a suitable rule:

    $ udevadm info --attribute-walk /sys/class/tty/ttyDNT0

For example, a rule which creates a symlink for each remote radio, named for its MAC address, is as follows:

    $ cat /etc/udev/rules.d/99-dnt900.rules
    SUBSYSTEMS=="dnt900" ATTRS{MacAddress}=="0x*" MODE="0666" SYMLINK+="$attr{MacAddress}"
    
    $ ls -l /dev/0x*
    lrwxrwxrwx 1 root root 9 Apr 16 15:25 /dev/0x00165E -> ttyDNT0

(Individual MAC addresses could also be used to give custom names to individual radios.)

Flow Control
============

Per the [DNT900 manual](http://www.rfm.com/products/data/dnt900dk_manual.pdf), it is *highly* recommended that hardware flow control be used if you intend to use your radio network for high-volume data transmission. (This means connecting the `/HOST_CTS` signal on the radio to a `/CTS` line on your serial port). If you are using hardware flow control, you should enable it before loading the line discipline, as follows:

    $ stty -F /dev/ttyAMA0 crtscts

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
* Bit 2 of `ProtocolOptions` determines whether the local radio issues *TxDataReply* packets, which include a transmission status byte for transmitted data packets. However these status bytes are currently ignored by the software.

Communication with the local radio is in protocol mode. You may wish to select this mode permanently by configuring the radio's `ProtocolMode` register:

    $ echo 0x01 > /sys/class/dnt900/ttyAMA0/0x00165F/ProtocolMode
    $ echo 0x01 > /sys/class/dnt900/ttyAMA0/0x00165F/MemorySave

(Alternatively, you can ground the radio's `/CFG` pin in your hardware design.) This is not compulsory as the *EnterProtocolMode* command is also issued. However, unless you do so, use of an external reset source such as a reset button or a remote reset command will render the radio connection unresponsive.

As an observation, reading radio registers (via sysfs attribute files) can be very slow when data is also being transmitted via the radio (the radio appears to prioritise data packets over register queries).

Finally, if the local radio is configured as a remote in *TDMA Dynamic Slots* mode, data transmission may not succeed if new remotes join the network. (This is due to dynamic changes in the remote slot size in this access mode.)

Release History
===============

* 27/12/2012: version 0.1 (initial release): undoubtedly, not bug-free
  * 6/1/2013: version 0.1.1: improved handling of radio resets; removed gpio_cfg parameter; fixed bug preventing operation of remote radio in tree routing mode.
  * 8/4/2013: version 0.1.2: fixed bug whereby signals were ignored during device writes.
* 16/4/2013: version 0.2: partial rewrite to represent remote radios as ttys instead of character devices.
  * 17/4/2013: version 0.2.1: fixed bug which caused an infinite loop when tty sends non-normal flag byte.
  * 20/4/2013: version 0.2.2: added tty hangups on shutdown and when radios leave network.
  * 4/7/2013: HEAD: new Makefile; added flush_buffer and ioctl for line discipline; changed tty driver to avoid shutdown bug.
