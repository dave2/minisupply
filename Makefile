# Copyright (C) 2013 David Zanetti
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; version 2 of the License.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along
#  with this program; if not, write to the Free Software Foundation, Inc.,
#  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

# Main project Makefile

CC=/usr/bin/avr-gcc
GITDEF := -D'GITVER="$(shell git show --format=%H | head -1)"'
DATEDEF := -D'BUILDDATE="$(shell date +%Y%m%d%H%M%S)"'
#DEBUG += -DDEBUG_FW_UPDATE
DEBUG += -DDEBUG_ADC
CFLAGS= $(DEBUG) $(GITDEF) $(DATEDEF) --std=c99 -Os -funroll-loops -Wa,-adhlns=$(<:.c=.lst) -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wstrict-prototypes -Wall -mcall-prologues -I.
# because headers for 256d3 are wrong, but don't matter while compiling
CFLAGS += -mmcu=atxmega192d3
OBJ2HEX=/usr/bin/objcopy
TARGET=minisupply
AVRFLAGS = -p x256d3 -c avrisp2 -P usb
AVRDUDE = /usr/local/bin/avrdude $(AVRFLAGS)
#AVRFLAGS = -g -j usb -p atmega1284p
#AVRDUDE = avarice $(AVRFLAGS)

# objects for various subsystems
# main loop
OBJ += main.o
# libraries
OBJ += ringbuffer.o serial.o adc.o

#LDFLAGS = -Wl,-Map=$(TARGET).map -Wl,--cref -Wl,-u,vfprintf -lprintf_flt -lm
LDFLAGS = -Wl,-Map=$(TARGET).map -Wl,--cref

all : $(TARGET).hex $(TARGET).eep

program : $(TARGET).hex $(TARGET).eep
	$(AVRDUDE) -U application:w:$(TARGET).hex

image : $(TARGET).bin

%.bin : %.hex
	$(OBJ2HEX) -I ihex -O binary --pad-to 0x18000 --gap-fill 0xff $< $@

eeprom : $(TARGET).eep
	$(AVRDUDE) -U eeprom:w:$(TARGET).eep

%.o : %.c %.h global.h Makefile
	$(CC) -c $(CFLAGS) $< -o $@

#%.obj : %.o
#	$(CC) $(CFLAGS) $< -o $@

.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf : $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) --output $@ $(LDFLAGS)

%.hex : %.elf
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

%.eep : %.elf
	$(OBJ2HEX) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O ihex $< $@

clean :
	rm -f *.hex *.elf *.eep *.o *.map *.lst

reset :
	$(AVRDUDE)
