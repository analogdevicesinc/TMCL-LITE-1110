# Hey Emacs, this is a -*- makefile -*-
#
# On command line:
#
# make hex CPU=MK20DX128  Make software for modules with MK20DX128 CPU
#
# make hex CPU=GD32F425   Make aoftware for modules with GD32F425 CPU
#
# make clean  Clean project
#
# Add BOOT_LOADER=TMCM-NO-BL to make a version for use without bootloader.
#
# Execute "make clean" before using a different CPU or BOOT_LOADER option.
#

# Default options
BOOT_LOADER ?= TMCM-BL
CPU ?= MK20DX128

# Toolchain prefix (i.e arm-elf- -> arm-elf-gcc.exe)
#TCHAIN_PREFIX = arm-eabi-
#TCHAIN_PREFIX = arm-elf-
TCHAIN_PREFIX = arm-none-eabi-
REMOVE_CMD=rm
#REMOVE_CMD=cs-rm


# YES enables -mthumb option to flags for source-files listed 
# in SRC and CPPSRC and -mthumb-interwork option for all source
USE_THUMB_MODE = YES
#USE_THUMB_MODE = NO

# MCU name, submodel and board
# - MCU used for compiler-option (-mcpu)
# - SUBMDL used for linker-script name (-T) and passed as define
# - BOARD just passed as define (optional)
MCU      = cortex-m4
ifeq ($(CPU), MK20DX128)
  SUBMDL   = MK20DX128
  CHIP     = $(SUBMDL)
else ifeq ($(CPU), GD32F425)
  SUBMDL   = GD32F425
  CHIP     = $(SUBMDL)
endif


# *** This example only supports "ROM_RUN" ***
# RUN_MODE is passed as define and used for the linker-script filename,
# the user has to implement the necessary operations for 
# the used mode(s) (i.e. no copy of .data, remapping)
# Create ROM-Image
RUN_MODE=ROM_RUN
# Create RAM-Image
#RUN_MODE=RAM_RUN

# Exception-vectors placement option is just passed as define,
# the user has to implement the necessary operations (i.e. remapping)
# Exception vectors in ROM:
VECTOR_TABLE_LOCATION=VECT_TAB_ROM
# Exception vectors in RAM:
#VECTOR_TABLE_LOCATION=VECT_TAB_RAM

# Directory for output files (lst, obj, dep, elf, sym, map, hex, bin etc.)
OUTDIR = $(RUN_MODE)

# Target file name (without extension).
TARGET = stepRockerMiniTMCL

# Paths to Libraries
ifeq ($(CPU), MK20DX128)
  LIBDIR = ./lib_mk
  LIBSRCDIR = ./lib_mk/src
  LIBINCDIR = ./lib_mk/inc
  INCLUDE_DIRS = -I$(LIBDIR)
else ifeq ($(CPU), GD32F425)
  LIBDIR = ./lib_gd
  LIBSRCDIR = ./lib_gd/src
  LIBINCDIR = ./lib_gd/inc
  INCLUDE_DIRS = -I$(LIBDIR)
endif

# List C source files here. (C dependencies are automatically generated.)
# use file-extension c for "c-only"-files
## Our Application:
SRC = stepRocker.c SysTick.c Commands.c Globals.c \
      TMC429.c TMC262.c SysControl.c Eeprom.c

ifeq ($(CPU), MK20DX128)
  SRC += IO_mk.c
  SRC += RS485_mk.c
  SRC += Can_mk.c
  SRC += USB_mk.c
  SRC += SPI_mk.c
endif

ifeq ($(CPU), GD32F425)
  SRC += IO_gd.c
  SRC += RS485_gd.c
  SRC += Can_gd.c
  SRC += USB_gd.c
  SRC += SPI_gd.c
  SRC += gd32f4xx_hw.c
endif

ifeq ($(CPU), MK20DX128)
  #used parts of the Kinetis library
  SRC += $(LIBSRCDIR)/kinetis_sysinit.c
  SRC += $(LIBSRCDIR)/nvic-m4.c

  SRC += $(LIBSRCDIR)/usb/CDC1.c
  SRC += $(LIBSRCDIR)/usb/CS1.c
  SRC += $(LIBSRCDIR)/usb/Rx1.c
  SRC += $(LIBSRCDIR)/usb/Tx1.c
  SRC += $(LIBSRCDIR)/usb/usb_cdc.c
  SRC += $(LIBSRCDIR)/usb/usb_cdc_pstn.c
  SRC += $(LIBSRCDIR)/usb/usb_class.c
  SRC += $(LIBSRCDIR)/usb/usb_dci_kinetis.c
  SRC += $(LIBSRCDIR)/usb/usb_descriptor.c
  SRC += $(LIBSRCDIR)/usb/usb_driver.c
  SRC += $(LIBSRCDIR)/usb/usb_framework.c
  SRC += $(LIBSRCDIR)/usb/USB0.c
  SRC += $(LIBSRCDIR)/usb/USB1.c
  SRC += $(LIBSRCDIR)/usb/wdt_kinetis.c
endif

ifeq ($(CPU), GD32F425)
  #used parts of the GigaDevice library
	SRC += $(LIBSRCDIR)/system_gd32f4xx.c
	SRC += $(LIBSRCDIR)/gd32f4xx_adc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_can.c
	SRC += $(LIBSRCDIR)/gd32f4xx_crc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_ctc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_dac.c
	SRC += $(LIBSRCDIR)/gd32f4xx_dbg.c
	SRC += $(LIBSRCDIR)/gd32f4xx_dci.c
	SRC += $(LIBSRCDIR)/gd32f4xx_dma.c
	SRC += $(LIBSRCDIR)/gd32f4xx_enet.c
	SRC += $(LIBSRCDIR)/gd32f4xx_exmc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_exti.c
	SRC += $(LIBSRCDIR)/gd32f4xx_fmc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_fwdgt.c
	SRC += $(LIBSRCDIR)/gd32f4xx_gpio.c
	SRC += $(LIBSRCDIR)/gd32f4xx_i2c.c
	SRC += $(LIBSRCDIR)/gd32f4xx_ipa.c
	SRC += $(LIBSRCDIR)/gd32f4xx_iref.c
	SRC += $(LIBSRCDIR)/gd32f4xx_misc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_pmu.c
	SRC += $(LIBSRCDIR)/gd32f4xx_rcu.c
	SRC += $(LIBSRCDIR)/gd32f4xx_rtc.c
	SRC += $(LIBSRCDIR)/gd32f4xx_sdio.c
	SRC += $(LIBSRCDIR)/gd32f4xx_spi.c
	SRC += $(LIBSRCDIR)/gd32f4xx_syscfg.c
	SRC += $(LIBSRCDIR)/gd32f4xx_timer.c
	SRC += $(LIBSRCDIR)/gd32f4xx_tli.c
	SRC += $(LIBSRCDIR)/gd32f4xx_trng.c
	SRC += $(LIBSRCDIR)/gd32f4xx_usart.c
	SRC += $(LIBSRCDIR)/gd32f4xx_wwdgt.c
	SRC += $(LIBSRCDIR)/usb/cdc_acm_core.c
	SRC += $(LIBSRCDIR)/usb/drv_usbd_int.c
	SRC += $(LIBSRCDIR)/usb/drv_usb_core.c
	SRC += $(LIBSRCDIR)/usb/drv_usb_dev.c
	SRC += $(LIBSRCDIR)/usb/usbd_core.c
	SRC += $(LIBSRCDIR)/usb/usbd_enum.c
	SRC += $(LIBSRCDIR)/usb/usbd_transc.c
endif

# List C source files here which must be compiled in ARM-Mode (no -mthumb).
# use file-extension c for "c-only"-files
## just for testing, timer.c could be compiled in thumb-mode too
SRCARM = 

# List C++ source files here.
# use file-extension .cpp for C++-files (not .C)
CPPSRC = 

# List C++ source files here which must be compiled in ARM-Mode.
# use file-extension .cpp for C++-files (not .C)
#CPPSRCARM = $(TARGET).cpp
CPPSRCARM = 

# List Assembler source files here.
# Make them always end in a capital .S. Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ifeq ($(CPU), MK20DX128)
  ASRC = $(LIBSRCDIR)/startup.S
else ifeq ($(CPU), GD32F425)
  ASRC =  $(LIBSRCDIR)/startup_gd32f405_425_gas.S
endif

# List Assembler source files here which must be assembled in ARM-Mode..
ASRCARM = 

# List any extra directories to look for include files here.
#    Each directory must be seperated by a space.
EXTRAINCDIRS  = $(LIBINCDIR) $(LIBINCDIR)/usb

# List any extra directories to look for library files here.
# Also add directories where the linker should search for
# includes from linker-script to the list
#     Each directory must be seperated by a space.
EXTRA_LIBDIRS = $(LIBDIR)

# Extra libraries
#    Each library-name must be seperated by a space.
#    i.e. to link with libxyz.a, libabc.a and libefsl.a: 
#    EXTRA_LIBS = xyz abc efsl
EXTRA_LIBS =

# Path to Linker-Scripts
LINKERSCRIPTPATH = .

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s
#OPT = 2
#OPT = 3
#OPT = 0

# Output format. (can be ihex or binary or both)
#  binary to create a load-image in raw-binary format i.e. for SAM-BA, 
#  ihex to create a load-image in Intel hex format i.e. for lpc21isp
LOADFORMAT = ihex

# Debugging format.
#DEBUG = stabs
DEBUG = dwarf-2

# Place project-specific -D (define) and/or 
# -U options for C here.
ifeq ($(BOOT_LOADER), TMCM-BL)
CDEFS += -DBOOTLOADER
endif

# Place project-specific -D and/or -U options for 
# Assembler with preprocessor here.
#ADEFS = -DUSE_IRQ_ASM_WRAPPER
ADEFS = -D__ASSEMBLY__

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

#-----

ifdef VECTOR_TABLE_LOCATION
CDEFS += -D$(VECTOR_TABLE_LOCATION)
ADEFS += -D$(VECTOR_TABLE_LOCATION)
endif

CDEFS += -D$(RUN_MODE) -D$(CHIP) -D__USE_CMSIS
ADEFS += -D$(RUN_MODE) -D$(CHIP)


# Compiler flags.

ifeq ($(USE_THUMB_MODE),YES)
THUMB    = -mthumb
### no for CM3 THUMB_IW = -mthumb-interwork
else 
THUMB    = 
THUMB_IW = 
endif

#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#
# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)
CFLAGS =  -g$(DEBUG)
CFLAGS += -O$(OPT)
CFLAGS += -mcpu=$(MCU) $(THUMB_IW) 
CFLAGS += $(CDEFS)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.
# when using ".ramfunc"s without longcall:
##CFLAGS += -mlong-calls
# -mapcs-frame is important if gcc's interrupt attributes are used
# (at least from my eabi tests), not needed if assembler-wrapper is used 
##CFLAGS += -mapcs-frame 
##CFLAGS += -fomit-frame-pointer
#CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wall -Wextra
CFLAGS += -Wimplicit -Wcast-align -Wpointer-arith
CFLAGS += -Wredundant-decls -Wshadow -Wcast-qual -Wcast-align
#CFLAGS += -pedantic
CFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(OUTDIR)/dep/$(@F).d

# flags only for C
CONLYFLAGS += -Wnested-externs 
CONLYFLAGS += $(CSTANDARD)

# flags only for C++ (arm-elf-g++)
CPPFLAGS = -fno-rtti -fno-exceptions
CPPFLAGS = 

# Assembler flags.
#  -Wa,...:    tell GCC to pass this to the assembler.
#  -ahlns:     create listing
#  -g$(DEBUG): have the assembler create line number information
ASFLAGS  = -mcpu=$(MCU) $(THUMB_IW) -I. -x assembler-with-cpp
ASFLAGS += $(ADEFS)
ASFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
ASFLAGS += -Wa,-g$(DEBUG)
ASFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

MATH_LIB = -lm

# Link with the GNU C++ stdlib.
#CPLUSPLUS_LIB = -lstdc++

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,--gc-sections,-Map=$(OUTDIR)/$(TARGET).map,-cref
#LDFLAGS += -u,Reset_Handler
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += -lc
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS))
LDFLAGS += $(MATH_LIB)
LDFLAGS += $(CPLUSPLUS_LIB)
LDFLAGS += -lc -lgcc 
LDFLAGS += $(INCLUDE_DIRS)

# Set linker-script name depending on selected run-mode and submodel name
ifeq ($(BOOT_LOADER),TMCM-BL)
LDFLAGS +=-T $(LINKERSCRIPTPATH)/$(SUBMDL)-TMCM.ld
else
LDFLAGS +=-T $(LINKERSCRIPTPATH)/$(SUBMDL).ld
endif


# Define programs and commands.
SHELL   = sh
CC      = $(TCHAIN_PREFIX)gcc
CPP     = $(TCHAIN_PREFIX)g++
AR      = $(TCHAIN_PREFIX)ar
OBJCOPY = $(TCHAIN_PREFIX)objcopy
OBJDUMP = $(TCHAIN_PREFIX)objdump
SIZE    = $(TCHAIN_PREFIX)size
NM      = $(TCHAIN_PREFIX)nm
##COPY    = cp
REMOVE  = $(REMOVE_CMD) -f

# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = "-------- begin (mode: $(RUN_MODE)) --------"
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after build:
MSG_LOAD_FILE = Creating load file:
MSG_EXTENDED_LISTING = Creating Extended Listing/Disassembly:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = "**** Linking :"
MSG_COMPILING = "**** Compiling C :"
MSG_COMPILING_ARM = "**** Compiling C (ARM-only):"
MSG_COMPILINGCPP = "Compiling C++ :"
MSG_COMPILINGCPP_ARM = "Compiling C++ (ARM-only):"
MSG_ASSEMBLING = "**** Assembling:"
MSG_ASSEMBLING_ARM = "****Assembling (ARM-only):"
MSG_CLEANING = Cleaning project:
MSG_FORMATERROR = Can not handle output-format
MSG_LPC21_RESETREMINDER = You may have to bring the target in bootloader-mode now.
MSG_ASMFROMC = "Creating asm-File from C-Source:"
MSG_ASMFROMC_ARM = "Creating asm-File from C-Source (ARM-only):"

# List of all source files.
ALLSRC     = $(ASRCARM) $(ASRC) $(SRCARM) $(SRC) $(CPPSRCARM) $(CPPSRC)
# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))

# Define all object files.
ALLOBJ     = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))

elf: $(OUTDIR)/$(TARGET).elf
lss: $(OUTDIR)/$(TARGET).lss 
sym: $(OUTDIR)/$(TARGET).sym
hex: $(OUTDIR)/$(TARGET).hex
bin: $(OUTDIR)/$(TARGET).bin

# Default target.
#all: begin gccversion sizebefore build sizeafter finished end
all: begin gccversion build sizeafter finished end

ifeq ($(LOADFORMAT),ihex)
build: elf hex lss sym
else 
ifeq ($(LOADFORMAT),binary)
build: elf bin lss sym
else 
ifeq ($(LOADFORMAT),both)
build: elf hex bin lss sym
else 
$(error "$(MSG_FORMATERROR) $(FORMAT)")
endif
endif
endif


# Eye candy.
begin:
##	@echo
	@echo $(MSG_BEGIN)

finished:
##	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)
##	@echo

# Display sizes of sections.
ELFSIZE = $(SIZE) -A  $(OUTDIR)/$(TARGET).elf
##ELFSIZE = $(SIZE) --format=Berkeley --common $(OUTDIR)/$(TARGET).elf
sizebefore:
#	@if [ -f  $(OUTDIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
#	@if [ -f  $(OUTDIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi
	@echo $(MSG_SIZE_AFTER)
	$(ELFSIZE)
	
# Display compiler version information.
gccversion : 
	@$(CC) --version
#	@echo $(ALLOBJ)

# Create final output file (.hex) from ELF output file.
%.hex: %.elf
##	@echo
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O ihex $< $@
	
# Create final output file (.bin) from ELF output file.
%.bin: %.elf
##	@echo
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O binary $< $@

# Create extended listing file/disassambly from ELF output file.
# using objdump testing: option -C
%.lss: %.elf
##	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -C -r $< > $@
#	$(OBJDUMP) -x -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
##	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(ALLOBJ)
%.elf:  $(ALLOBJ)
	@echo
	@echo $(MSG_LINKING) $@
# use $(CC) for C-only projects or $(CPP) for C++-projects:
	$(CC) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)
#	$(CPP) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)


# Assemble: create object files from assembler source files.
define ASSEMBLE_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_ASSEMBLING) $$< "->" $$@
	$(CC) -c $(THUMB) $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRC), $(eval $(call ASSEMBLE_TEMPLATE, $(src)))) 

# Assemble: create object files from assembler source files. ARM-only
define ASSEMBLE_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_ASSEMBLING_ARM) $$< "->" $$@
	$(CC) -c $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRCARM), $(eval $(call ASSEMBLE_ARM_TEMPLATE, $(src)))) 


# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_COMPILING) $$< "->" $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRC), $(eval $(call COMPILE_C_TEMPLATE, $(src)))) 

# Compile: create object files from C source files. ARM-only
define COMPILE_C_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_COMPILING_ARM) $$< "->" $$@
	$(CC) -c $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRCARM), $(eval $(call COMPILE_C_ARM_TEMPLATE, $(src)))) 


# Compile: create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_COMPILINGCPP) $$< "->" $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRC), $(eval $(call COMPILE_CPP_TEMPLATE, $(src)))) 

# Compile: create object files from C++ source files. ARM-only
define COMPILE_CPP_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $(MSG_COMPILINGCPP_ARM) $$< "->" $$@
	$(CC) -c $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRCARM), $(eval $(call COMPILE_CPP_ARM_TEMPLATE, $(src)))) 


# Compile: create assembler files from C source files. ARM/Thumb
$(SRC:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC) $< to $@
	$(CC) $(THUMB) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Compile: create assembler files from C source files. ARM only
$(SRCARM:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC_ARM) $< to $@
	$(CC) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Target: clean project.
clean: begin clean_list finished end

clean_list :
##	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(OUTDIR)/$(TARGET).map
	$(REMOVE) $(OUTDIR)/$(TARGET).elf
	$(REMOVE) $(OUTDIR)/$(TARGET).hex
	$(REMOVE) $(OUTDIR)/$(TARGET).bin
	$(REMOVE) $(OUTDIR)/$(TARGET).sym
	$(REMOVE) $(OUTDIR)/$(TARGET).lss
	$(REMOVE) $(OUTDIR)/*.lst
	$(REMOVE) $(OUTDIR)/*.o
	$(REMOVE) $(OUTDIR)/*.lst
	$(REMOVE) $(OUTDIR)/dep/*.d


# Create output files directory
## $(shell mkdir $(OUTDIR) 2>/dev/null)
$(shell mkdir $(OUTDIR) 2>NUL)

# Include the dependency files.
##-include $(shell mkdir $(OUTDIR)/dep 2>/dev/null) $(wildcard $(OUTDIR)/dep/*)
-include $(shell mkdir $(OUTDIR)\dep 2>NUL) $(wildcard $(OUTDIR)/dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex bin lss sym clean clean_list

 