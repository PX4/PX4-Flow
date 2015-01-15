## PX4Flow Firmware

[![Build Status](https://travis-ci.org/PX4/Flow.svg?branch=master)](https://travis-ci.org/PX4/Flow)

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/PX4/Firmware?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

PX4 FLOW is a smart camera processing optical flow directly on the camera module. It is optimized for processing and outputs images only for development purposes. Its main output is a UART or I2C stream of flow measurements at ~400 Hz.

Project:
http://px4.io/modules/px4flow

Dev guide / toolchain installation:
http://px4.io/dev/px4flow

To build, run:
```
  make
```

To flash via the PX4 bootloader (first run this command, then connect the board):
```
  make upload-usb
```
