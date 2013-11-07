################################################################################
## Defines
################################################################################

# Paths
UPLDR_FILES_PATH	= built/uploader_files
IMAGES_PATH			= built/images

# Project specifications
SERIALDEV		= /dev/ttyS0
CLOCKFREQ		= 12000
TARGET			= sfp_tester
TERMINA			= ltser
BAUDRATE		= 19200
#BAUDRATE		= 115200

# Modules, objects and dependences
SRC = $(wildcard hw-access/*.c)
SRC += $(wildcard cfg/*.c)
SRC += $(wildcard cli/*.c)
OBJ = $(SRC:.c=.o)
MODULES			= main.o $(UPLDR_FILES_PATH)/crt.o $(OBJ)
INCLUDE_PATHS	= -Icli -Icfg -Ifault-inject -Ihw-access -I.

# C compiler name, assembler and linker
CC		= arm-elf-gcc
LD		= arm-elf-gcc
AS		= arm-elf-as
AFLAGS	= -mapcs-32 -mcpu=arm7tdmi
CFLAGS	= -Wall -O2 -mcpu=arm7tdmi-s -D BAUDRATE=$(BAUDRATE) $(INCLUDE_PATHS)
LFLAGS	= -nostartfiles

# Flags for PC compilation
CCPC	= gcc

# Debug mode
ifeq ($(DEBUG_EN),true)
DEBUG_FLAG = -DDEBUG_MODE
endif


################################################################################
## Compilation
################################################################################
all: $(TARGET).hex

# Convert elf file to hex file
%.hex: %.elf
	arm-elf-objcopy -O ihex $(IMAGES_PATH)/$< $(IMAGES_PATH)/$@

# Call C compiler
%.o: %.c $(HEADERS)
	$(CC) -c $(CFLAGS) $(DEBUG_FLAG) -o $@ $<

# Compile assembly modules
%.o: %.S
	$(AS) $(AFLAGS) -o $@ $<

# Call linker/loader to get together the modules, generating the elf file
$(TARGET).elf: $(MODULES)
	$(LD) $(LFLAGS) -T$(UPLDR_FILES_PATH)/lpc2378_flash.ld -o $(IMAGES_PATH)/$(TARGET).elf $(MODULES)

$(TARGET)r.elf: $(MODULES)
	$(LD) $(LFLAGS) -T$(UPLDR_FILES_PATH)/lpc2378_ram.ld -o $(IMAGES_PATH)/$(TARGET)r.elf $(MODULES)

# Call the terminal and execute the program on RAM memory (it requires mon23)
tser: $(TARGET)r.hex
	$(TERMINAL) $(SERIALDEV) b=$(BAUDRATE) $(TARGET)r.hex

tseru: $(TARGET)r.hex
	$(TERMINAL) /dev/ttyUSB0 b=$(BAUDRATE) $(TARGET)r.hex


################################################################################
## Use 'make isp' to program memory in flash memory
################################################################################
isp: $(TARGET).hex
	lpc21isp $(IMAGES_PATH)/$(TARGET).hex $(SERIALDEV) $(BAUDRATE) $(CLOCKFREQ)
ispu: $(TARGET).hex
	lpc21isp $(IMAGES_PATH)/$(TARGET).hex /dev/ttyUSB0 $(BAUDRATE) $(CLOCKFREQ)
isp1: $(TARGET).hex
	lpc21isp $(IMAGES_PATH)/$(TARGET).hex com1 $(BAUDRATE) $(CLOCKFREQ)
isp2: $(TARGET).hex
	lpc21isp $(IMAGES_PATH)/$(TARGET).hex com2 $(BAUDRATE) $(CLOCKFREQ)
isp3: $(TARGET).hex
	lpc21isp $(IMAGES_PATH)/$(TARGET).hex com3 $(BAUDRATE) $(CLOCKFREQ)
isp4: $(TARGET).hex
	lpc21isp $(IMAGES_PATH)/$(TARGET).hex com4 $(BAUDRATE) $(CLOCKFREQ)
isp5: $(TARGET).hex
	lpc21isp $(IMAGES_PATH)/$(TARGET).hex com5 $(BAUDRATE) $(CLOCKFREQ)


################################################################################
## Specific PC compilation
################################################################################
pc:
	make pc -C hw-access
	make pc -C cfg
	make pc -C cli

	$(CCPC) -DPC_COMPILATION $(INCLUDE_PATHS) -o $(IMAGES_PATH)/$(TARGET).pc main.c $(OBJ)


################################################################################
## Clean project
################################################################################
clean:
	-rm -rf *.o *.elf *~ *.bin *.map *.hex
	make clean -C hw-access
	make clean -C cfg
	make clean -C cli

