# BlueSense2 makefile.
#
# This is a newer makefile that generates dependencies in a .dep directory
# It can also be modified to generate bootloaders.
#
# ---- BOOTLOADER ----
# If a file called bootloader.hex is available, then programming will flash
# the main firmware, and if space is available the bootloader. 
# If no space is available for the bootloader, only the main firmware is programmed.
#
# The bootloader is assumed to take the maximum size of 8192 bytes.
#
# ---- FUSES ----
#
# This makefile automatically programs the fuses as needed. 
# If a bootloader is flashed, the fuses are set to boot at the bootloader address. 
# If a bootloader is not flashed (not available, or firmware too large), the fuses
# are set to boot at address 0000.
#
# The fuses without bootloader must be: 
# 	Extended: FF
# 	High: D1
# 	Low: FF
#
# The fuses with bootloader must be: 
# 	Extended: FF
# 	High: D0
# 	Low: FF



# ----------------------------------------------------------------------------
# Author: Peter Fleury
# based on WinAVR Makefile Template written by Eric B. Weddington, Jörg Wunsch, et al.
#
# Adjust F_CPU below to the clock frequency in Mhz of your AVR target
# Adjust BOOTLOADER_ADDRESS to your AVR target
#
#----------------------------------------------------------------------------
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make coff = Convert ELF to AVR COFF.
#
# make extcoff = Convert ELF to AVR Extended COFF.
#
# make program = Download the hex file to the device, using avrdude.
#                Please customize the avrdude settings below first!
#
# make debug = Start either simulavr or avarice as specified for debugging, 
#              with avr-gdb or avr-insight as the front end for debugging.
#
# make filename.s = Just compile filename.c into the assembler code only.
#
# make filename.i = Create a preprocessed source file for use in submitting
#                   bug reports to the GCC project.
#
# To rebuild project do "make clean" then "make all".
#----------------------------------------------------------------------------
#	<MLS> = Mark Sproul msproul-at-skychariot.com


# MCU name
MCU = atmega1284p

FSIZ = 122880

# Processor frequency.
#     This will define a symbol, F_CPU, in all source code files equal to the 
#     processor frequency. You can then use this symbol in your source code to 
#     calculate timings. Do NOT tack on a 'UL' at the end, this will be done
#     automatically to create a 32-bit value in your source code.
#F_CPU = 16000000


# Bootloader
# Address is in bytes. Size in bootloader is specified in words
# Please adjust if using a different AVR
# 0x0e00*2=0x1C00 for ATmega8  512 words Boot Size
# 0xFC00*2=0x1F800 for ATmega128  1024 words Boot Size
# 0xF800*2=0x1F000 for ATmega1280
# 0xF000*2=0x1E000 for ATmega1280
#BOOTLOADER_ADDRESS = 1E000
#BOOTLOADER_ADDRESS = 1E000



# Output format. (can be srec, ihex, binary)
FORMAT = ihex


# Target file name (without extension).
TARGET = main

# Secondary hex file to program; e.g. bootloader or main
SECONDARYHEX = bootloader

# Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = 2
#OPT = 3		# ok




# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).c
# If there is more than one source file, append them above, or modify and
# uncomment the following:
# SRC += ../common/core.c
# SRC += ../common/global.c
SRC += bluesense-bsp/serial.c
#SRC += bluesense-bsp/serial0.c
SRC += bluesense-bsp/serial1.c
SRC += megalol/adc.c
SRC += megalol/i2c.c
#SRC += megalol/i2c_poll.c
SRC += megalol/i2c_int.c
#SRC += megalol/menu.c
SRC += bluesense-bsp/ds3232.c
SRC += bluesense-bsp/rn41.c
SRC += bluesense-bsp/ltc2942.c
SRC += bluesense-bsp/pkt.c
SRC += bluesense-bsp/mpu.c
SRC += bluesense-bsp/mpu_config.c
SRC += bluesense-bsp/mpu_geometry.c
SRC += bluesense-bsp/mpu-usart0.c
#SRC += bluesense-bsp/mpu-common.c
SRC += bluesense-bsp/mode_mputest.c
#SRC += mpu_test.c
SRC += bluesense-bsp/spi.c
SRC += bluesense-bsp/sd.c
SRC += bluesense-bsp/sd_bit.c
SRC += bluesense-bsp/sd_int.c
SRC += bluesense-bsp/test_sd.c
SRC += bluesense-bsp/init.c
SRC += helper/helper.c
SRC += helper/helper2.c
SRC += bluesense-bsp/uiconfig.c
SRC += bluesense-bsp/pio.c
SRC += bluesense-bsp/power.c
SRC += bluesense-bsp/isr.c
#SRC += bluesense-bsp/gfx/lcd.c
#SRC += bluesense-bsp/gfx/fb.c
#SRC += bluesense-bsp/gfx/fnt4x6.c
#SRC += mode_demo.c
#SRC += mode_demo_clock.c
SRC += bluesense-bsp/dbg.c
SRC += megalol/circbuf.c
SRC += bluesense-bsp/spi-usart0.c
SRC += megalol/wait.c
SRC += megalol/i2c_internal.c
SRC += bluesense-bsp/system.c
SRC += bluesense-bsp/system-extra.c
#SRC += stk500.c
SRC += bluesense-bsp/boot.c
SRC += bluesense-bsp/interface.c
SRC += bluesense-bsp/command.c
SRC += bluesense-bsp/commandset.c
SRC += bluesense-bsp/mode_sample.c
SRC += bluesense-bsp/mode_sample_adc.c
SRC += bluesense-bsp/mode_sample_motion.c
SRC += bluesense-bsp/mode.c
SRC += bluesense-bsp/mode_global.c
SRC += bluesense-bsp/mode_coulomb.c
SRC += bluesense-bsp/mode_teststream.c
SRC += bluesense-bsp/mode_testsync.c
SRC += bluesense-bsp/mode_sd.c
SRC += bluesense-bsp/mode_idle.c
SRC += bluesense-bsp/mode_adcfast.c
#SRC += mode_bench.c
SRC += bluesense-bsp/mode_bt.c
#SRC += bluesense-bsp/mode_motionrecog.c
#SRC += bluesense-bsp/mode_motionsample.c
SRC += bluesense-bsp/ufat.c
#SRC += bluesense-bsp/MadgwickAHRS_fixed.c
SRC += bluesense-bsp/MadgwickAHRS_float.c
#SRC += bluesense-bsp/MadgwickAHRS.c
SRC += bluesense-bsp/mathfix.c
SRC += bluesense-bsp/a3d.c
SRC += bluesense-bsp/test.c
SRC += bluesense-bsp/global.c




# List Assembler source files here.
# Make them always end in a capital .S.  Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC =
#ASRC = asmfunc.S
#ASRC = bluesense-bsp/num.S
ASRC = helper/helper_num.S
ASRC += helper/numeric.S
ASRC += bluesense-bsp/mpu_helper.S
#ASRC += bluesense-bsp/softspi.S
#ASRC += bluesense-bsp/gfx/lcd_spi.S
#ASRC = softspi.S
ASRC += bluesense-bsp/spi-usart0-isr.S
#ASRC += megalol/wait_s.S


# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
EXTRAINCDIRS = megalol
EXTRAINCDIRS += helper
EXTRAINCDIRS += bluesense-bsp
EXTRAINCDIRS += bluesense-bsp/gfx



#---------------- Compiler Options ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS) $(CINCS)
CFLAGS += -O$(OPT)
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -fno-jump-tables
CFLAGS += -Wall 
#CFLAGS += -Wstrict-prototypes
CFLAGS += -Wa,-adhlns=$(<:.c=.lst)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CSTANDARD)

CFLAGS += -Werror=implicit-function-declaration
CFLAGS += -Wall

# Dan: remove unused functions with compile flag -ffunction-sections and linker flag -Wl,-gc-sections
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections



# Compiler flag to set the C Standard level.
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
#CSTANDARD = -std=gnu99
CSTANDARD = -std=gnu++11


# Place -D or -U options here
CDEFS = 
#CDEFS = -DF_CPU=$(F_CPU)UL

# BlueSense2 compile-time parameters
# HWVER: hardware version
# ENABLE_SERIAL0: enable uart0 interface
# ENABLE_SERIAL1: enable uart1 interface

# BlueSense2: define the hardware revision below
# Available versions are: 1, 4, 5, 6, 7, 9. Version 7 is compatible with version 8.
#
#CDEFS += -DHWVER=4
#CDEFS += -DHWVER=5
#CDEFS += -DHWVER=6
CDEFS += -DHWVER=7
#CDEFS += -DHWVER=9

# BlueSense2: define firmware options below
CDEFS += -DENABLE_SERIAL0=0
CDEFS += -DENABLE_SERIAL1=1
CDEFS += -DENABLE_I2CINTERRUPT
#CDEFS += -DENABLE_I2CPOLL
CDEFS += -DFIXEDPOINTQUATERNION=0
CDEFS += -DFIXEDPOINTQUATERNIONSHIFT=0
CDEFS += -DENABLEQUATERNION=1
CDEFS += -DENABLEGFXDEMO=0
CDEFS += -DENABLEMODECOULOMB=0
CDEFS += -DDBG_TIMERELATEDTEST=0
# Set DDBG_RN41TERMINAL=1 to activate the Bluetooth terminal
CDEFS += -DDBG_RN41TERMINAL=0
CDEFS += -DBOOTLOADER=0
# Set NOBLUETOOTHINIT=1 to prevent Bluetooth initialisation - keeps connection ongoing despite reset. For normal use set NOBLUETOOTHINIT=0.
CDEFS += -DNOBLUETOOTHINIT=0
# Set BLUETOOTH_POWERSAVE to: 0=default (high power); 1=lower power
CDEFS += -DBLUETOOTH_POWERSAVE=0
# UFAT_INCLUDENODENAME: 1=include the 4 digits of the node in the log file (e.g LOG-8554.### with ### the log number); 0=log file is called LOG-0000.### (with ### the log number)
CDEFS += -DUFAT_INCLUDENODENAME=0

CDEFS += -D__DELAY_BACKWARD_COMPATIBLE__

#CFLAGS += -minit-stack=0x1234






# Place -I options here
CINCS =




# Debugging format.
#     Native formats for AVR-GCC's -g are dwarf-2 [default] or stabs.
#     AVR Studio 4.10 requires dwarf-2.
#     AVR [Extended] COFF format requires stabs, plus an avr-objcopy run.
DEBUG = 
#DEBUG = dwarf-2










#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -ahlms:    create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs 


#---------------- Library Options ----------------
# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

# If this is left blank, then it will use the Standard printf version.
#PRINTF_LIB = 
#PRINTF_LIB = $(PRINTF_LIB_MIN)
PRINTF_LIB = $(PRINTF_LIB_FLOAT)


# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min

# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

# If this is left blank, then it will use the Standard scanf version.
SCANF_LIB = 
#SCANF_LIB = $(SCANF_LIB_MIN)
#SCANF_LIB = $(SCANF_LIB_FLOAT)


MATH_LIB = -lm



#---------------- External Memory Options ----------------

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#EXTMEMOPTS = -Wl,--defsym=__heap_start=0x801100,--defsym=__heap_end=0x80ffff

EXTMEMOPTS =




#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += $(EXTMEMOPTS)
LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)

#---------------- Dan's library simplification ---------------
#LDFLAGS += -Wl,-u,_delay_ms -l_delay_ms_boot
#---------------- Dan's library simplification ---------------
# Dan: remove unused functions with compile flag -ffunction-sections and linker flag -Wl,-gc-sections
LDFLAGS += -Wl,-gc-sections




#--------------- bootloader linker Options -------
# BOOTLOADER_ADDRESS (=Start of Boot Loader section in bytes - not words) is defined above.
#LDFLAGS += -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS) -nostartfiles -nodefaultlibs
#LDFLAGS += -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS) -nostartfiles
#LDFLAGS += -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS)
#LDFLAGS += -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS) -nodefaultlibs

#---------------- Programming Options (avrdude) ----------------

# Programming hardware: alf avr910 avrisp bascom bsd 
# dt006 pavr picoweb pony-stk200 sp12 stk200 stk500
#
# Type: avrdude -c ?
# to get a full listing.
#
# AVRDUDE_PROGRAMMER = stk500v2
AVRDUDE_PROGRAMMER = avrispmkii
AVRDUDE_PROGRAMMER_BL = stk500v2

AVRDUDE_WRITE_BIT_CLOCK = 2.5MHz

# com1 = serial port. Use lpt1 to connect to parallel port.
AVRDUDE_PORT = usb		# programmer connected to serial device
AVRDUDE_PORT_BL = com1	# programmer connected to serial device


AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
AVRDUDE_WRITE_SECONDARY = -U flash:w:$(SECONDARYHEX).hex -D

#AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep


# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> 
# to submit bug reports.
#AVRDUDE_VERBOSE = -v -v

AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) -B $(AVRDUDE_WRITE_BIT_CLOCK)
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)

AVRDUDE_FLAGS_BL = -p $(MCU) -P $(AVRDUDE_PORT_BL) -c $(AVRDUDE_PROGRAMMER_BL)
# Disable verify and disable chip erase
AVRDUDE_FLAGS_BL += -D -V
AVRDUDE_FLAGS_BL += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS_BL += $(AVRDUDE_ERASE_COUNTER)



#---------------- Debugging Options ----------------

# For simulavr only - target MCU frequency.
DEBUG_MFREQ = $(F_CPU)

# Set the DEBUG_UI to either gdb or insight.
# DEBUG_UI = gdb
DEBUG_UI = insight

# Set the debugging back-end to either avarice, simulavr.
DEBUG_BACKEND = avarice
#DEBUG_BACKEND = simulavr

# GDB Init Filename.
GDBINIT_FILE = __avr_gdbinit

# When using avarice settings for the JTAG
JTAG_DEV = /dev/com1

# Debugging port used to communicate between GDB / avarice / simulavr.
DEBUG_PORT = 4242

# Debugging host used to communicate between GDB / avarice / simulavr, normally
#     just set to localhost unless doing some sort of crazy debugging when 
#     avarice is running on a different computer.
DEBUG_HOST = localhost



#============================================================================


# Define programs and commands.
SHELL = sh
#CC = avr-gcc
CC = avr-g++
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AVRDUDE = avrdude
REMOVE = rm -f
COPY = cp
WINSHELL = cmd


# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_SIZE_AFTER2 = "Size after (total flash usage)":
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:




# Define all object files.
OBJ = $(SRC:.c=.o) $(ASRC:.S=.o) 

# Define all listing files.
LST = $(SRC:.c=.lst) $(ASRC:.S=.lst) 


# Compiler flags to generate dependency files.
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d


# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)




# Default target.
all: begin gccversion sizebefore build sizeafter sizeafter2 end

build: elf hex eep lss sym
#build:  hex eep lss sym

elf: $(TARGET).elf
hex: $(TARGET).hex
eep: $(TARGET).eep
lss: $(TARGET).lss 
sym: $(TARGET).sym



# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin:
	@echo
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)
	@echo


# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
HEXSIZETOT = $(SIZE) $(TARGET).hex | sed '2q;d' | grep -Eo --color  '^\s*\w+\s*\w+\s*' | grep -Eo --color '\s*\w+\s*$$' | grep -Eo --color '\w+'
#ELFSIZE = $(SIZE) --format=avr --mcu=$(MCU) $(TARGET).elf
ELFSIZE = $(SIZE) -A $(TARGET).elf

sizebefore:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); \
	2>/dev/null; echo; fi

sizeafter:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); \
	2>/dev/null; echo; fi

sizeafter2:
	@if test -f $(TARGET).hex; then echo; echo $(MSG_SIZE_AFTER2); $(HEXSIZETOT); \
	2>/dev/null; echo; fi

# Display compiler version information.
gccversion : 
	@$(CC) --version



# Program the device.  
# Programs the firmware and an optional bootloader (in bootloader.hex) if available and space sufficient. 
# If the bootloader is flashed, the fuses are set to enable it and the lock bit protects it.
program: $(TARGET).hex $(TARGET).eep sizeafter2
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM)
	@sleep .1
	@echo Programming bootloader, if available and space sufficient
	$(eval HEXSIZETOTVAR := $(shell $(HEXSIZETOT)))
	@echo Firmware size is $(HEXSIZETOTVAR), maximum size to allow for bootloader is 122880
	@if test $(HEXSIZETOTVAR) -gt 122880; then echo The firmware is too large for a bootloader, no bootloader will be flashed.; \
	echo Flashing fuses;$(AVRDUDE) $(AVRDUDE_FLAGS) -U hfuse:w:0xd1:m -U lfuse:w:0xff:m -U efuse:w:0xff:m -U lock:w:0xff:m; fi	
	@if test $(HEXSIZETOTVAR) -le 122880; then echo Space sufficient for a bootloader.; \
	if test -f $(SECONDARYHEX).hex; then echo Flashing bootloader;$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_SECONDARY);\
	echo Flashing fuses;$(AVRDUDE) $(AVRDUDE_FLAGS) -U hfuse:w:0xd0:m -U lfuse:w:0xff:m -U efuse:w:0xff:m -U lock:w:0xef:m; fi;\
	if test ! -f $(SECONDARYHEX).hex; then echo No bootloader to flash;\
	echo Flashing fuses;$(AVRDUDE) $(AVRDUDE_FLAGS) -U hfuse:w:0xd1:m -U lfuse:w:0xff:m -U efuse:w:0xff:m -U lock:w:0xff:m; fi;\
	fi
	
# Program the device with the STK500 bootloader
programbl: $(TARGET).hex $(TARGET).eep sizeafter2
	$(eval HEXSIZETOTVAR := $(shell $(HEXSIZETOT)))
	@echo Firmware size is $(HEXSIZETOTVAR), maximum size to allow for bootloader is 122880
	@if test $(HEXSIZETOTVAR) -gt 122880; then echo The firmware is too large for a bootloader. Cannot flash.; fi
	@if test $(HEXSIZETOTVAR) -le 122880; then echo Flashing firmware; \
	$(AVRDUDE) $(AVRDUDE_FLAGS_BL) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM); fi
	
	
	
	


# Generate avr-gdb config/init file which does the following:
#     define the reset signal, load the target file, connect to target, and set 
#     a breakpoint at main().
gdb-config: 
	@$(REMOVE) $(GDBINIT_FILE)
	@echo define reset >> $(GDBINIT_FILE)
	@echo SIGNAL SIGHUP >> $(GDBINIT_FILE)
	@echo end >> $(GDBINIT_FILE)
	@echo file $(TARGET).elf >> $(GDBINIT_FILE)
	@echo target remote $(DEBUG_HOST):$(DEBUG_PORT)  >> $(GDBINIT_FILE)
ifeq ($(DEBUG_BACKEND),simulavr)
	@echo load  >> $(GDBINIT_FILE)
endif	
	@echo break main >> $(GDBINIT_FILE)
	
debug: gdb-config $(TARGET).elf
ifeq ($(DEBUG_BACKEND), avarice)
	@echo Starting AVaRICE - Press enter when "waiting to connect" message displays.
	@$(WINSHELL) /c start avarice --jtag $(JTAG_DEV) --erase --program --file \
	$(TARGET).elf $(DEBUG_HOST):$(DEBUG_PORT)
	@$(WINSHELL) /c pause
	
else
	@$(WINSHELL) /c start simulavr --gdbserver --device $(MCU) --clock-freq \
	$(DEBUG_MFREQ) --port $(DEBUG_PORT)
endif
	@$(WINSHELL) /c start avr-$(DEBUG_UI) --command=$(GDBINIT_FILE)
	



# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000 



coff: $(TARGET).elf
	@echo
	@echo $(MSG_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-avr $< $(TARGET).cof


extcoff: $(TARGET).elf
	@echo
	@echo $(MSG_EXTENDED_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-ext-avr $< $(TARGET).cof


# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

%.eep: %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@



# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $^ --output $@ $(LDFLAGS)


# Compile: create object files from C source files.
%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@ 


# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

# Create preprocessed source for use in sending a bug report.
%.i : %.c
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) $< -o $@ 


# Target: clean project.
clean: begin clean_list end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).eep
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).cof
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).a90
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(OBJ)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) .dep/*


.PHONY: toto2
toto2:
	$(eval FIRMWARESIZE = $(shell avr-size main.elf | sed '2q;d' | grep -Eo --color  '^\s*\w+\s*' | grep -Eo --color '\w+'))

getfirwmaresize2:
	$(eval FIRMWARESIZE2 = $(shell avr-size main.hex | sed '2q;d' | grep -Eo --color  '^\s*\w+\s*\w+\s*' | grep -Eo --color '\s*\w+\s*$' | grep -Eo --color '\w+'))

printfirmwaresize: getfirwmaresize
	@echo Firmware size is $(FIRMWARESIZE)

toto: getfirwmaresize printfirmwaresize
	@echo hello world
	# FSIZ: must have the size of the .text section in bytes
	@echo Firmware size: $(FIRMWARESIZE)
	@if test $(FIRMWARESIZE) -gt 122880; then echo Error: firmware too large for bootloader; fi
	@if test $(FIRMWARESIZE) -le 122880; then echo OK to flash firwmare and bootloader; fi


# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex eep lss sym coff extcoff \
clean clean_list program debug gdb-config

