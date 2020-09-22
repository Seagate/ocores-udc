# OpenCores USB Device Controller (UDC) Driver

This repository contains the Linux driver for the [OpenCores USB 1.1 Device function](https://opencores.org/projects/usbhostslave).
The driver has been tested with Linux 4.14.x and 5.4.x (other versions may be supported but have not been tested).

## Requirements

To build the provided Linux driver, you'll need to build it against a Linux source.
If your plan is to build and install it in your local system, make sure you have the kernel headers installed for your current kernel.
For instance, to install the kernel headers in a debian-based system you would do:

```
sudo apt-get install kernel-headers-$(uname -r)
```

## Build & Install

To build and install in your system:

```
make
sudo make install
```

To build and install against a custom kernel:

```
make -C /path/to/kernel/source M=${PWD}
make -C /path/to/kernel/source M=${PWD} modules_install
```

## Acknowledgment

During the development of this driver, Xilinx UDC driver was used as a reference driver (`udc-xilinx.c`),
as well as a previous version of the this same driver created by David W. Miller \<david.w.miller@seagate.com\>.

## License

Copyright © 2020 by Seagate Technology, LLC \<opensource@seagate.com\>.
Released under [GPLv2 License](LICENSE).
