CPUFREQ=16000000L
FLASHDEV=/dev/ttyU0
RS485ADR=0x02

all: ulm2003rs485.hex

ulm2003rs485.bin: src/ulm2003rs485.c

	avr-gcc -Wall -Os -mmcu=atmega328p -DF_CPU=$(CPUFREQ) -DRS485_ADDRESS=$(RS485ADR) -o ulm2003rs485.bin src/ulm2003rs485.c

ulm2003rs485.hex: ulm2003rs485.bin

	avr-size -t ulm2003rs485.bin
	avr-objcopy -j .text -j .data -O ihex ulm2003rs485.bin ulm2003rs485.hex

flash: ulm2003rs485.hex

	sudo chmod 666 $(FLASHDEV)
	avrdude -v -p atmega328p -c arduino -P $(FLASHDEV) -b 57600 -D -U flash:w:ulm2003rs485.hex:i

framac: src/ulm2003rs485.c

	-rm framacreport.csv
	frama-c -wp-verbose 0 -wp -rte -wp-rte -wp-dynamic -wp-timeout 300 -cpp-extra-args="-I/usr/home/tsp/framaclib/ -DF_CPU=16000000L -D__AVR_ATmega328P__ -DFRAMAC_SKIP" ulm2003rs485.c -then -no-unicode -report -report-csv framacreport.csv

clean:

	-rm *.bin

cleanall: clean

	-rm *.hex

.PHONY: all clean cleanall
