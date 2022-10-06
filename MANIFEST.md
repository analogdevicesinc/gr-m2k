title: gr-m2k
brief: GNU Radio blocks for ADALM2000
tags:
  - IIO
  - ADALM2000
author:
  - Adrian Suciu
  - Alexandra Trifan
  - Teo Perisanu
copyright_owner:
  - Analog Devices Inc.
dependencies:
  - gnuradio
  - libm2k
repo: https://github.com/analogdevicesinc/gr-m2k
stable_release: HEAD
icon:
---

The ADALM2000 (M2K) Active Learning Module is an affordable USB-powered measurement unit. With 12-bit ADCs and DACs running at 100 MSPS, the ADALM2000 brings the power of high performance lab equipment to the palm of your hand, enabling electrical engineering students and hobbyists to explore signals and systems into the tens of MHz without the cost and bulk associated with traditional lab gear. 

gr-m2k contains blocks that represent all major components of ADALM2000. These GNU Radio blocks are build around ADI's libm2k library. Libm2k can be used for encoding and decoding digital signals, communicating with a wide variety of chips, using some well known protocols, such as SPI, I²C or UART. 
The gr-m2k blocks offer the possibility of interfacing with a variety of peripherals, in order to use the ADALM2000 as a master to configure/use them.
