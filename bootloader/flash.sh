#!/bin/sh
FMICRO=m328p
PORT=/dev/ttyUSB0
PRGM=stk500v2
OPTIBOOT=../../optiboot/optiboot/bootloaders/optiboot/optiboot_atmega328.hex
if [ ! -f $OPTIBOOT ]; then
    echo "Bootloader file doesn't exist"
	exit 1
fi
avrdude -p $FMICRO -c $PRGM -P $PORT -e &&
avrdude -p $FMICRO -c $PRGM -P $PORT -U flash:w:$OPTIBOOT:a &&
avrdude -p $FMICRO -c $PRGM -P $PORT -U lfuse:w:0xF7:m -U hfuse:w:0xDE:m -U efuse:w:0x05:m &&
avrdude -p $FMICRO -c $PRGM -P $PORT -U lock:w:0x2F
