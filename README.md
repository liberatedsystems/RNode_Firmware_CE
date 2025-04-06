# RNode Firmware - Community Edition

This is the community maintained fork of the open firmware which powers RNode devices. It has been created to continue to expand development and support for more hardware devices, as the upstream repository is no longer accepting PRs for new hardware support. The original repository by Mark Qvist can be found [here](https://github.com/markqvist/RNode_Firmware).

An RNode is an open, free and unrestricted digital radio transceiver. It enables anyone to send and receive any kind of data over both short and very long distances. RNodes can be used with many different kinds of programs and systems, but they are especially well suited for use with [Reticulum](https://reticulum.network).

RNode is not a product, and not any *one* specific device in particular. It is a system that is easy to replicate across space and time, that produces highly functional communications tools, which respects user autonomy and empowers individuals and communities to protect their sovereignty, privacy and ability to communicate and exchange data and ideas freely.

<img src="Documentation/images/rnv21_bgp.webp" width="100%">

<center><i>An RNode made from readily available and cheap parts, in a durable 3D printed case</i></center><br/><br/>

The RNode system is primarily software, which *transforms* different kinds of available hardware devices into functional, physical RNodes, which can then be used to solve a wide range of communications tasks. Such RNodes can be modified and built to suit the specific time, locale and environment they need to exist in.

## Latest Release

The latest release, installable through `rnodeconf`, is version `1.73`. This release brings the following changes:

- Added multiple interface support - for supported boards you may now use more than one radio modem at once! Currently the only supported board is the RAK4631 with the right hardware.
- Fixes for various issues with the SX1280 - data rates are now at expected speeds overall.
- Added PA calculations for the LoRa1280F27 - this allows for the TX power to be set accurately for this modem.
- Fixes for OLED compilation errors as a result of commit 055083f.
- Added switching graphics to the display - shows statistics from up to 2 modems simultaneously.
- Fix TNC EEPROM settings not being saved - courtesy of @attermann
- Fix ESP32 linker errors - BSP version is now fixed at 2.0.17, using the older crosstool-ng linker from previous versions (2021r1)

You must have at least version `2.1.3` of `rnodeconf` installed to update your RNode Firmware install to version `1.73`. Get it by updating the `rns` package to at least version `0.6.4`.

## Supported products and boards

### Products
| Name | Manufacturer | Link | Transceiver | MCU | Description |
| :---: | :---: | :---: | :---: | :---: | :---: |
| Handheld v2.x RNodes | [Mark Qvist](https://unsigned.io) | [Buy here](https://unsigned.io/shop/product/handheld-rnode) | SX1276 | ESP32 |
| openCom XL | [Liberated Embedded Systems](https://liberatedsystems.co.uk) | [Buy here](https://store.liberatedsystems.co.uk/product/opencom-xl/) | SX1262 & SX1280 | nRF52 | Supports utilisation of both modems at once |

### Homebrew devices
| Board name | Link | Transceiver | MCU | Description |
| :--- | :---: | :---: | :---: | :---: |
| RAK4631 | [Buy here](https://store.rakwireless.com/products/rak4631-lpwan-node?m=5&h=wisblock-core) | SX1262 | nRF52 |
| LilyGO T-BEAM v1.1 | [Buy here](https://www.lilygo.cc/products/t-beam-v1-1-esp32-lora-module) | SX1276/8 or SX1262 | ESP32 |
| LilyGO T-Beam Supreme | [Buy here](https://lilygo.cc/products/t-beam-supreme) | SX1262 | ESP32-S3 |
| LilyGO LoRa32 v1.0 | [Buy here](https://www.lilygo.cc/products/lora32-v1-0) | SX1276/8 | ESP32 |
| LilyGO LoRa32 v2.0 | No link | SX1276/8 | ESP32 | Discontinued? |
| LilyGO LoRa32 v2.1 |  [Buy here](https://www.lilygo.cc/products/lora3) | SX1276/8 | ESP32 | With and without TCXO |
| Heltec LoRa32 v2 | No link | SX1276/8 | ESP32 | Discontinued? |
| Heltec LoRa32 v3 | [Buy here](https://heltec.org/project/wifi-lora-32-v3/) | SX1262 | ESP32 |
| LilyGo T3S3 v1.0 | [Buy here](https://lilygo.cc/products/t3s3-v1-0) | SX1262 or SX1276 or SX1280 | ESP32-S3 |
| Heltec T114 v2.0 | [Buy here](https://heltec.org/project/mesh-node-t114/) | SX1262 | nRF52 |
| Homebrew ESP32 boards | | Any supported | ESP32 | This can be any board with an Adafruit Feather (or generic) ESP32 chip |

It's easy to create your own RNodes from one of the supported development boards and devices. If a device or board you want to use is not yet supported, you are welcome to [join the effort](Documentation/CONTRIBUTING.md) and help create a board definition and pin mapping for it!

<!--| LilyGO T-Deck | [Buy here](https://lilygo.cc/products/t-deck) | SX1262 | ESP32-S3 |-->
<!--| Heltec Wireless Paper | [Buy here](https://heltec.org/project/wireless-paper/) | SX1262 | ESP32-S3 | -->

<!--<img src="Documentation/images/devboards_1.webp" width="100%"/>-->

## Supported Transceiver Modules
The RNode Firmware supports all transceiver modules based on the following chips:
* Semtech SX1262
* Semtech SX1276
* Semtech SX1278
* Semtech SX1280

These also must have an **SPI interface** and **DIO_0 (sometimes called DIO_1)** pin connected to the MCU directly.

## One Tool, Many Uses

The RNode design is meant to flexible and hackable. At it's core, it is a low-power, but extremely long-range digital radio transceiver. Coupled with Reticulum, it provides encrypted and secure communications.

Depending on configuration, it can be used for local networking purposes, or to send data over very long distances. Once you have an RNode, there is a wide variety of possible uses:

- As a network adapter for [Reticulum](https://reticulum.network)
- Messaging using [Sideband](https://unsigned.io/software/Sideband.html)
- Information sharing and communication using [Nomad Network](https://unsigned.io/software/Nomad_Network.html)
- LoRa-based [KISS-compatible amateur radio TNC](https://unsigned.io/guides/2020_05_03_using_rnodes_with_amateur_radio_software.html)
- LoRa development platform
- [Packet sniffer](https://unsigned.io/software/LoRaMon.html) for LoRa networks
- Long range [Ethernet and IP network interface](https://unsigned.io/guides/2020_05_27_ethernet-and-ip-over-packet-radio-tncs.html) on Linux
- As a general-purpose long-range data radio

## Types & Performance

RNodes can be made in many different configurations, and can use many different radio bands, but they will generally operate in the **433 MHz**, **868 MHz**, **915 MHZ** and **2.4 GHz** bands. They will usually offer configurable on-air data speeds between just a **few hundred bits per second**, up to **hundreds of kilobits per second**. Maximum output power will depend on the transceiver and PA setup used, but will generally lie between **17 dbm (50mW)** and **27 dBm (500mW)**.

The RNode system has been designed to allow reliable systems for basic human communications, over very wide areas, while using very little power, being cheap to build, free to operate, and near impossible to censor.

While **speeds are lower** than WiFi, typical communication **ranges are many times higher**. Several kilometers can be acheived with usable bitrates, even in urban areas, and over **100 kilometers** can be achieved in line-of-sight conditions.

## A Self-Replicating System

If you notice the presence of a circularity in the naming of the system as a whole, and the physical devices, it is no coincidence. Every RNode contains the seeds necessary to reproduce the system, the [RNode Bootstrap Console](https://unsigned.io/rnode_bootstrap_console), which is hosted locally on every RNode, and can be activated and accesses at any time - no Internet required.

The designs, guides and software stored within allows users to create more RNodes, and even to bootstrap entire communications networks, completely independently of existing infrastructure, or in situations where infrastructure has become unreliable or is broken.

<img src="Documentation/images/126dcfe92fb7.webp" width="100%"/>

<center><i>Where there is no Internet, RNodes will still communicate</i></center><br/><br/>

The production of one particular RNode device is not an end, but the potential starting point of a new branch of devices on the tree of the RNode system as a whole.

This tree fits into the larger biome of Free & Open Communications Systems, which I hope that you - by using communications tools like RNode - will help grow and prosper.


## Getting Started Fast
You can download and flash the firmware to all the supported boards using [rnodeconf](https://github.com/markqvist/Reticulum). All firmware releases are handled and installed directly through the `rnodeconf` utility, which is included in the `rns` package. It can be installed via `pip`:

```
# Install rnodeconf via rns package
pip install rns --upgrade

# Install the firmware on a board with the install guide
rnodeconf --autoinstall
```

For most of the supported device types, it is also possible to use [Liam Cottle's Web-based RNode Flasher](https://liamcottle.github.io/rnode-flasher/). This option may be easier if you're not familiar with using a command line interface.

For more detailed instruction and in-depth guides, you can have a look at some of these resources:

- Create a [basic RNode from readily available development boards](https://unsigned.io/guides/2022_01_25_installing-rnode-firmware-on-supported-devices.html)
- Follow a complete build recipe for [making a handheld RNode](https://unsigned.io/guides/2023_01_14_Making_A_Handheld_RNode.html), like the one pictured above
- Learn the basics on how to [create and build your own RNode designs](https://unsigned.io/guides/2022_01_26_how-to-make-your-own-rnodes.html) from scratch
- Once you've got the hang of it, start building RNodes for your community, or [even for selling them](https://unsigned.io/sell_rnodes.html)

## Support development
### Contributing
You can contribute features and board support to the project if you wish. Please see [here](Documentation/CONTRIBUTING.md).

### Hardware donations
If you would like to see support added for a board which you possess, you may donate it to myself, Jacob Eva, so that I can implement support for it into this project. There will be no official timescale given for implementation however, but I will try my best when I have time :) Please [contact me (scroll to the bottom)](https://liberatedsystems.co.uk/about) if you wish to donate hardware to the project.

### Purchasing products
You can support the development of the RNode Firmware CE project by purchasing RNodes (to come in the future) and other products from [Liberated Embedded systems](https://liberatedsystems.co.uk), my online business. For more information on my business, please see [here](https://liberatedsystems.co.uk/about/).

From time to time the creator of the [Reticulum](https://reticulum.network) project, Mark Qvist, produces his own RNodes and sells them through his [shop](https://unsigned.io/shop). You may support the development of open, free and private communications systems through purchasing these from him.

### Monetary donations
I, the maintainer of this fork, am currently not accepting monetary donations. These are instead better directed to [Mark Qvist](https://unsigned.io), who created the original repository this one is based on, and is the core developer of the [Reticulum](https://reticulum.network) project, which is what the RNode Firmware was designed to supplement as a network adapter.
You can help support the continued development of open, free and private communications systems by donating to him via one of the following channels:

- Monero:
  ```
  84FpY1QbxHcgdseePYNmhTHcrgMX4nFfBYtz2GKYToqHVVhJp8Eaw1Z1EedRnKD19b3B8NiLCGVxzKV17UMmmeEsCrPyA5w
  ```
- Ethereum
  ```
  0xFDabC71AC4c0C78C95aDDDe3B4FA19d6273c5E73
  ```
- Bitcoin
  ```
  35G9uWVzrpJJibzUwpNUQGQNFzLirhrYAH
  ```
- Ko-Fi: https://ko-fi.com/markqvist

## FAQ
Please see [here](Documentation/FAQ.md).

## License & Use
The upstream RNode Firmware is Copyright © 2024 Mark Qvist / [unsigned.io](https://unsigned.io).
The modified RNode Firmware CE (community edition) is Copyright © Jacob Eva / [Liberated Embedded Systems](https://liberatedsystems.co.uk) and is made available under the **GNU General Public License v3.0**.

The source code includes an SX1276 driver that is released under MIT License, and Copyright © 2018 Sandeep Mistry / Mark Qvist. The SX126x and SX128x drivers are adaptations of this original driver.

You can obtain the source code from [my business Git instance](https://git.liberatedsystems.co.uk/jacob.eva/RNode_Firmware_CE) or [GitHub](https://github.com/liberatedsystems/RNode_Firmware_CE).

Every RNode which supports the console functionality also includes an internal copy of it's own firmware source code, that can be downloaded through the [RNode Bootstrap Console](https://unsigned.io/rnode_bootstrap_console), by putting the RNode into Console Mode (which can be activated by holding the user button for 10 seconds and releasing it).

The RNode Ecosystem is free and non-proprietary, and actively seeks to distribute it's ownership and control. If you want to build RNodes for commercial purposes, including selling them, you must do so adhering to the Open Source licenses that the various parts of the RNode project is released under, and under your own responsibility.

If you distribute or modify this work, you **must** adhere to the terms of the GPLv3, including, but not limited to, providing up-to-date source code upon distribution, displaying appropriate copyright and license notices in prominent positions of all conveyed works, and making users aware of their rights to the software under the GPLv3.
