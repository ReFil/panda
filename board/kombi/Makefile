PROJ_NAME = panda
CFLAGS = -g -Wall -Wextra -Wstrict-prototypes -Werror

CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4
CFLAGS += -mhard-float -DSTM32F4 -DSTM32F413xx -mfpu=fpv4-sp-d16 -fsingle-precision-constant -DKOMBI
STARTUP_FILE = startup_stm32f413xx

CFLAGS += -I ../inc -I ../ -I ../../ -nostdlib -fno-builtin -std=gnu11 -Os

CFLAGS += -T../stm32_flash.ld

DFU_UTIL = "dfu-util"

#COMPILER_PATH = /home/batman/Downloads/gcc-arm-none-eabi-9-2020-q2-update/bin/
CC = $(COMPILER_PATH)arm-none-eabi-gcc
OBJCOPY = $(COMPILER_PATH)arm-none-eabi-objcopy
OBJDUMP = $(COMPILER_PATH)arm-none-eabi-objdump

CERT = ../../certs/debug
CFLAGS += "-DALLOW_DEBUG"



DEPDIR = ../generated_dependencies
$(shell mkdir -p -m 777 $(DEPDIR) >/dev/null)
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.Td
POSTCOMPILE = @mv -f $(DEPDIR)/$*.Td $(DEPDIR)/$*.d && touch $@

# this no longer pushes the bootstub
canflash: obj/$(PROJ_NAME).bin
	../../tests/ctrls/enter_canloader.py $<

usbflash: obj/$(PROJ_NAME).bin
	../../tests/ibst/enter_canloader.py; sleep 0.5
	PYTHONPATH=../../ python -c "from python import Panda; p = [x for x in [Panda(x) for x in Panda.list()] if x.bootstub]; assert(len(p)==1); p[0].flash('obj/$(PROJ_NAME).bin', reconnect=False)"

recover: obj/bootstub.bin obj/$(PROJ_NAME).bin
	../tools/enter_download_mode.py $<
	sleep 1.0
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08004000 -D obj/$(PROJ_NAME).bin
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08000000:leave -D obj/bootstub.bin

recover_can: obj/bootstub.bin obj/$(PROJ_NAME).bin
	../../tests/ibst/enter_canloader.py --recover; sleep 0.5
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08004000 -D obj/$(PROJ_NAME).bin
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08000000:leave -D obj/bootstub.bin

include ../../common/version.mk

obj/cert.h: ../../crypto/getcertheader.py
	../../crypto/getcertheader.py ../../certs/debug.pub ../../certs/release.pub > $@

obj/main.o: main.c ../*.h
	mkdir -p obj
	$(CC) $(CFLAGS) -o $@ -c $<

obj/bootstub.o: ../bootstub.c ../*.h obj/gitversion.h obj/cert.h
	mkdir -p obj
	mkdir -p ../obj
	cp obj/gitversion.h ../obj/gitversion.h
	cp obj/cert.h  ../obj/cert.h
	$(CC) $(CFLAGS) -o $@ -c $<

obj/$(STARTUP_FILE).o: ../$(STARTUP_FILE).s
	$(CC) $(CFLAGS) -o $@ -c $<

obj/%.o: ../../crypto/%.c
	$(CC) $(CFLAGS) -o $@ -c $<

obj/$(PROJ_NAME).bin: obj/$(STARTUP_FILE).o obj/main.o
  # hack
	$(CC) -Wl,--section-start,.isr_vector=0x8004000 $(CFLAGS) -o obj/$(PROJ_NAME).elf $^
	$(OBJCOPY) -v -O binary obj/$(PROJ_NAME).elf obj/code.bin
	SETLEN=1 ../../crypto/sign.py obj/code.bin $@ $(CERT)

obj/bootstub.bin: obj/$(STARTUP_FILE).o obj/bootstub.o obj/sha.o obj/rsa.o
	$(CC) $(CFLAGS) -o obj/bootstub.$(PROJ_NAME).elf $^
	$(OBJCOPY) -v -O binary obj/bootstub.$(PROJ_NAME).elf $@

clean:
	rm -f obj/*
