KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

install: default
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

.PHONY: clean
clean:
	@rm -f *.o *.ko *.a *.mod *.mod.c .*.cmd modules.order Module.symvers
