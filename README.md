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

Configuration Registers
=======================

The configuration registers of each radio on the network may be accessed via the sysfs file system, usually mounted at `/sys`. Each radio network is represented as a virtual device under the name of its serial port:

    $ ls -l /sys/devices/virtual/dnt900/ttyAMA0/
    total 0
    drwxr-xr-x 3 root root    0 Apr 16 15:25 0x00165E
    drwxr-xr-x 4 root root    0 Apr 16 15:25 0x00165F
    -r--r--r-- 1 root root 4096 Apr 16 15:25 announce
    --w------- 1 root root 4096 Apr 16 15:25 discover
    -r--r--r-- 1 root root 4096 Apr 16 11:06 error
    lrwxrwxrwx 1 root root    0 Apr 16 15:25 local -> 0x00165F
    -r--r--r-- 1 root root 4096 Apr 16 11:06 parent
    drwxr-xr-x 2 root root    0 Apr 16 15:25 power
    --w------- 1 root root 4096 Apr 16 15:25 reset
    lrwxrwxrwx 1 root root    0 Apr 16 15:25 subsystem -> ../../../../class/dnt900
    -rw-r--r-- 1 root root 4096 Apr 16 15:25 uevent    

This directory lists each radio on the network under its corresponding MAC address (`0x00165E` and `0x00165F` in this example). A symlink (`local`) for the locally connected radio is also listed for convenience.

The configuration registers for a radio are listed within its named subdirectory. These registers are described in detail in the [DNT900 Series Integration Guide](http://www.rfm.com/products/data/dnt900dk_manual.pdf).

    $ ls -l /sys/devices/virtual/dnt900/ttyAMA0/0x00165F/
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
    -r--r--r-- 1 root root 4096 Apr 16 15:29 EventFlags
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

    $ cat /sys/devices/virtual/dnt900/ttyAMA0/0x00165E/ADC0
    0x01E2

For registers which are writeable, changing their value is as simple as writing a new value to the attribute file. For example, to configure `GPIO1` as an output and set its value high:

    $ echo 0x02 > /sys/devices/virtual/dnt900/ttyAMA0/0x00165E/GPIO_Dir
    $ echo 0x01 > /sys/devices/virtual/dnt900/ttyAMA0/0x00165E/GPIO1

When you wish your configuration registers changes to be permanent, you will need to save them using the `MemorySave` register:

    $ echo 0x01 > /sys/devices/virtual/dnt900/ttyAMA0/0x00165E/MemorySave

Note that changing some configuration registers relating to the serial port or radio network may cause a temporary loss of connection to the radio. For example, changing the serial rate on the local radio to 115200 would result in the serial connection being lost. After writing the value, interrupt with `ctrl-c` then drop and reattach the line discipline at the new rate:

    $ echo 0x0004 > /sys/devices/virtual/dnt900/ttyAMA0/0x00165F/SerialRate 
    ^Cbash: echo: write error: Interrupted system call
    $ killall ldattach
    $ ldattach -8n1 -s 115200 29 /dev/ttyAMA0

(Note that a `MemorySave` would then be required to make this change permanent.)

As a general comment, you should expect that reading and writing attribute files may be slow (many seconds) when data is being concurrently transmitted.

Transmitting and Receiving Data
===============================

Data transmissions to and from remote radios are implemented using a virtual tty device for each non-local radio on the network. Loading the line discipline causes the network to be interrogated for information on all radios in the network. In our example, we get the following:

    $ ls -l /dev/ttyDNT*
    crw-rw-rw- 1 root uucp 248,  0 Apr 16 15:25 /dev/ttyDNT0

Here, `/dev/ttyDNT0` is a new tty representing the remote radio with MAC address `0x00165E`. We can send data to the remote radio by writing to the new tty. For example:

    $ echo "hello, world" > /dev/ttyDNT0

We may also use the same file to read any data sent by the remote radio:

    $ cat /dev/ttyDNT0

These virtual ttys are fully implemented, and may be used by any linux application which uses a tty. In particular, the virtual tty can be used to host a Point-to-Point Protocol (PPP) connection, allowing you to establish a TCP/IP network connection over the radio link.

Since there is no data transmission to and from the local radio, the original tty is used for transmitting broadcast messages (which are sent to all radios on the network, at a considerably slower rate). For example, we can broadcast data to all remotes as follows:

    $ echo "some broadcast message" > /dev/ttyAMA0

Remote Radio Attributes
=======================

Your radio receives status information from remote radios while it operates. This information is available to read in a few extra sysfs attributes.

If the radio is operating as a base, it receives hearbeat packets from remote radios. Information from these heartbeats are presented in the following attribute files:

    $ cat /sys/devices/virtual/dnt900/ttyAMA0/0x00165E/beacon_rssi
    -63
    $ cat /sys/devices/virtual/dnt900/ttyAMA0/0x00165E/parent_rssi
    -62
    $ cat /sys/devices/virtual/dnt900/ttyAMA0/0x00165E/success_rate
    100

(Heartbeats are described on page 41 and elsewhere in the [DNT900 manual](http://www.rfm.com/products/data/dnt900dk_manual.pdf).) Units for `beacon_rssi`, `parent_rssi` and `success_rate` are dBm, dBm and percent, respectively.

Range and signal strength (RSSI) information is also available when data is received from remote radios:

    $ cat /sys/devices/virtual/dnt900/ttyAMA0/0x00165E/rssi
    -66
    $ cat /sys/devices/virtual/dnt900/ttyAMA0/0x00165E/range
    933

Units for `range` are metres, although it is in fact a coarse measurement, with increments of about 466 metres. Depending on your radio network configuration, not all radios will have this data available, as indicated by an empty read.

The above attributes are pollable, allowing them to be monitored by your application for updates as they occur. (See below.)

A final `leave` attribute is available for remote radios. This attribute implements the `RemoteLeave` command. Write a number of seconds to the attribute to remove the radio from the network for that amount of time. For example, to force radio `0x00165E` to leave the network for ten minutes:

    # echo 600 > /sys/devices/virtual/dnt900/ttyAMA0/0x00165E/leave

(This will only be effective when the local radio is a base or router, and the remote radio is a child.)

Local Radio Attributes
======================

Some additional attributes are available for the local radio:

    $ ls -l /sys/devices/virtual/dnt900/ttyAMA0/
    ...
    -r--r--r-- 1 root root 4096 Aug  5 11:56 announce
    --w------- 1 root root 4096 Aug  5 11:56 discover
    -r--r--r-- 1 root root 4096 Aug  5 11:56 error
    --w------- 1 root root 4096 Aug  5 12:01 join_deny
    --w------- 1 root root 4096 Aug  5 12:03 join_permit
    -r--r--r-- 1 root root 4096 Aug  5 11:51 join_request
    -r--r--r-- 1 root root 4096 Aug  5 11:56 parent
    --w------- 1 root root 4096 Aug  5 11:56 remap
    --w------- 1 root root 4096 Aug  5 11:56 reset
    ...

The `announce` attribute contains the latest announcement from the radio, as a hex string. DNT900 announcements are detailed on pages 41-42 of the [DNT900 manual](http://www.rfm.com/products/data/dnt900dk_manual.pdf). For example, after radio `0x00165E` transmits a heartbeat, the following is output:

    $ cat /sys/devices/virtual/dnt900/ttyAMA0/announce
    0xA85E1600000100C004C002

The `error` contains the most recent error code, if any, that has been received. An empty attribute indicates no error and is normally the case. Error codes are in the range 0xE0 to 0xEE and could be helpful for diagnosing problems.

The `parent` attribute contains the MAC address of the router or base to which the radio is connected. (It is empty if the radio is acting as a base or is not linked to a network.)

When host-based authentication is used (`AuthMode` value of 0x02), the `join_request` attribute announces the MAC addresses of radios seeking to join the network. Respond to these requests by writing that MAC address to `join_permit` to allow the radio to join, or to `join_deny` to deny the request. For example:

    # cat /sys/devices/virtual/dnt900/ttyAMA0/join_request
    0x00165E
    # echo 0x00165E > /sys/devices/virtual/dnt900/ttyAMA0/join_permit

(The `announce`, `error`, `parent` and `join_request` attribute files are all pollable.)

Radios on the network are usually discovered and added automatically when the line discipline is loaded. However, when operating on a remote or router, the line discipline is not always able to detect radios which subsequently link to the network. Such a radio may be added manually by writing its MAC address to the `discover` attribute file:

    # echo 0x00165A > /sys/devices/virtual/dnt900/ttyAMA0/discover

Alternatively, you can remap the entire radio network to find new radios by using the `remap` attribute:

    # echo 1 > /sys/devices/virtual/dnt900/ttyAMA0/remap

Finally, a software reset may be issued to the local radio by writing (anything) to the `reset` attribute file:

    # echo 1 > /sys/devices/virtual/dnt900/ttyAMA0/reset

Note that issuing a software reset may cause the radio to fail to restart properly, requiring a power cycle. This is due to the DNT900's power-on reset requirements, which state that the `RADIO_TXD` pin must remain low for 10ms after a reset. Depending on your serial port and driver, this requirement may not be met. (Specifically, this can occur if your serial port sets a pull-up on its receive line.) The USB interface to the development kit does not exhibit this problem.

Receiving I/O Reports
=====================

Instead of actively querying the I/O registers of a remote radio, you can configure the remote to send I/O reports to the base radio automatically. (See pages 55-58 of the [DNT900 manual](http://www.rfm.com/products/data/dnt900dk_manual.pdf).) These reports can be triggered on a periodic basis, or by certain conditions (ADC threshold, GPIO edge). The report consists of register values for the six GPIOs and three ADCs. If you are using the I/O reporting facility, the reported values are available using the following attribute files:

    $ ls -l /sys/devices/virtual/dnt900/ttyAMA0/0x00165E/report_*
    -r--r--r-- 1 root root 4096 Jul 31 21:52 report_ADC0
    -r--r--r-- 1 root root 4096 Jul 31 21:58 report_ADC1
    -r--r--r-- 1 root root 4096 Jul 31 21:58 report_ADC2
    -r--r--r-- 1 root root 4096 Jul 31 21:58 report_EventFlags
    -r--r--r-- 1 root root 4096 Jul 31 21:58 report_GPIO0
    -r--r--r-- 1 root root 4096 Jul 31 21:58 report_GPIO1
    -r--r--r-- 1 root root 4096 Jul 31 21:58 report_GPIO2
    -r--r--r-- 1 root root 4096 Jul 31 21:58 report_GPIO3
    -r--r--r-- 1 root root 4096 Jul 31 21:58 report_GPIO4
    -r--r--r-- 1 root root 4096 Jul 31 21:58 report_GPIO5

Each of these attribute files contains the most recently reported value for its corresponding ADC or GPIO. The files can be polled by your application in order to process new values as they arrive from the remote radio.

Polling Attributes
==================

Some attribute files above are described as *pollable*. This means you can monitor changes to their values using the `poll()` function in C. For example, to monitor a radio signal strength and perform an action (say, update a graph or log) each time the value changes, something like the following C snippet would be suitable:

    char attr[10];
    int fd = open("/sys/devices/virtual/dnt900/ttyAMA0/0x00165E/rssi", O_RDONLY);
    struct pollfd ufds = { .fd = fd, .events = POLLPRI | POLLERR };
    do {
        poll(&ufds, 1, -1);
        memset(attr, 0, 10);
        read(fd, attr, 10);
        // ... process RSSI value in attr
        lseek(fd, 0, SEEK_SET);
    } while (1);

Multiple attributes can be simultaneously monitored this way. Other languages, including [Python](http://docs.python.org/dev/library/select.html#select.poll), also expose the `poll()` function.

udev Rules
==========

It is suggested to add [udev rules](http://www.reactivated.net/writing_udev_rules.html) to create more meaningful names for the remote ttys. Any of the radio register attributes (in particular, the MAC address) can be used in the rules. `udevadm` is helpful in writing a suitable rule:

    $ udevadm info --attribute-walk /dev/ttyDNT0

For example, a rule which creates a symlink for each remote radio, named for its MAC address, is as follows:

    $ cat /etc/udev/rules.d/99-dnt900.rules
    SUBSYSTEMS=="dnt900" ATTRS{MacAddress}=="0x*" MODE="0666" SYMLINK+="$attr{MacAddress}"
    
    $ ls -l /dev/0x*
    lrwxrwxrwx 1 root root 9 Apr 16 15:25 /dev/0x00165E -> ttyDNT0

Flow Control
============

Per the [DNT900 manual](http://www.rfm.com/products/data/dnt900dk_manual.pdf), it is **highly** recommended that hardware flow control be used if you intend to use your radio network for high-volume data transmission. (This means connecting the `/HOST_CTS` signal on the radio to a `/CTS` line on your serial port). If you are using hardware flow control, you should enable it before loading the line discipline, as follows:

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
* Bit 2 of `ProtocolOptions` determines whether the local radio issues *TxDataReply* packets, which include a transmission status byte for transmitted data packets. If enabled, these packets help when using dynamic ttys.

A small amount of state information is kept for radios; in particular, the values of the `DeviceMode`, `TreeRoutingEn`, `BaseModeNetId` and `EnableRtAcks` registers are held. After configuring these registers, it is recommended to save their values and reload the line discipline to ensure consistency.

Communication with the local radio is in protocol mode. For best operation, you should select this mode permanently by configuring the radio's `ProtocolMode` register:

    $ echo 0x01 > /sys/devices/virtual/dnt900/ttyAMA0/0x00165F/ProtocolMode
    $ echo 0x01 > /sys/devices/virtual/dnt900/ttyAMA0/0x00165F/MemorySave

(Alternatively, you can ground the radio's `/CFG` pin in your hardware design.) This is not compulsory as the *EnterProtocolMode* command is also issued. However, unless you do so, use of an external reset source such as a reset button or a remote reset command will render the radio connection unresponsive.

Reading radio registers (via sysfs attribute files) can be very slow when data is also being transmitted, since the queued data packets must first empty into the radio before the register query is seen.

Finally, if the local radio is configured as a remote in *TDMA Dynamic Slots* mode, data transmission may not succeed if new remotes join the network. (This is due to dynamic changes in the remote slot size in this access mode.)

Release History
===============

* 27/12/2012: version 0.1 (initial release): undoubtedly, not bug-free
  * 6/1/2013: version 0.1.1: improved handling of radio resets; removed gpio_cfg parameter; fixed bug preventing operation of remote radio in tree routing mode.
  * 8/4/2013: version 0.1.2: fixed bug whereby signals were ignored during device writes.
* 16/4/2013: version 0.2: partial rewrite to represent remote radios as ttys instead of character devices.
  * 17/4/2013: version 0.2.1: fixed bug which caused an infinite loop when tty sends non-normal flag byte.
  * 20/4/2013: version 0.2.2: added tty hangups on shutdown and when radios leave network.
  * 4/7/2013: version 0.2.3: new Makefile; added flush_buffer and ioctl for line discipline; changed tty driver to avoid shutdown bug.
  * 22/7/2013: version 0.2.4: reduced internal buffer sizes; fixed attribute timeout issues; fixed bug wherein tty minor number was not correct.
* 7/8/2013: version 0.3: added pollable attributes for announcements, I/O reports, RSSI, range, and heartbeats; support for host-based authentication; handled invalid argument errors.
  * 24/8/2013: version 0.3.1: fixed bug whereby kernel could hang on module unload when radio tty open; added remote leave attribute; added network remap attribute; implemented carrier up/down functions for radio ttys.
