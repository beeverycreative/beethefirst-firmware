
# Hey Emacs, this is a -*- makefile -*-
#
# WinARM template makefile 
# by Martin Thomas, Kaiserslautern, Germany 
# <eversmith(at)heizung-thomas(dot)de>
#
# Released to the Public Domain
# Please read the make user manual!
#
# The user-configuration is based on the WinAVR makefile-template
# written by Eric B. Weddington, J�rg Wunsch, et al. but the internal
# handling used hoere is very different.
# This makefile can also be used with the GNU tools included in
# Yagarto, GNUARM or the Codesourcery packages. It should work
# on Unix/Linux-Systems too. Just a rather up-to-date GNU make is
# needed.
#
#
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make program = Upload load-image to the device
#
# make filename.s = Just compile filename.c into the assembler code only
#
# make filename.o = Create object filename.o from filename.c (using CFLAGS)
#
# To rebuild project do "make clean" then "make all".
#
# Changes:
# - 17. Feb. 2005  - added thumb-interwork support (mth)
# - 28. Apr. 2005  - added C++ support (mth)
# - 29. Arp. 2005  - changed handling for lst-Filename (mth)
# -  1. Nov. 2005  - exception-vector placement options (mth)
# - 15. Nov. 2005  - added library-search-path (EXTRA_LIB...) (mth)
# -  2. Dec. 2005  - fixed ihex and binary file extensions (mth)
# - 22. Feb. 2006  - added AT91LIBNOWARN setting (mth)
# - 19. Apr. 2006  - option FLASH_TOOL (mth)
# - 23. Jun. 2006  - option USE_THUMB_MODE -> THUMB/THUMB_IW
# -  3. Aug. 2006  - added -ffunction-sections -fdata-sections to CFLAGS
#                    and --gc-sections to LDFLAGS. Only available for gcc >=4 (mth)
#                    (needs appropriate linker-script, remove them when using a
#                    "simple" linker-script)
# -  4. Aug. 2006  - pass SUBMDL-define to frontend (mth)
# - 11. Nov. 2006  - FLASH_TOOL-config, TCHAIN-config (mth)
# - 28. Mar. 2007  - remove .dep-Directory with rm -r -f and force "no error" (mth)
# - 24. Apr. 2007  - added "both" option for format (.bin and .hex) (mth)
# - 20. Aug. 2007  - extraincdirs in asflags, passing a "board"-define (mth)
# - 13. Sep. 2007  - create assembler from c-sources fixed (make foo.s for foo.c) (mth)
#                  - IMGEXT no longer used and removed (mth)
#                  - moved some entries (mth)
# - 25. Oct. 2007  - reverted 20070328-change (b/o "race condition" with 
#                    make clean all or when called from Eclipse) (mth)
#                  - removed "for flash" from objdump message-string (mth)
#                  - added same remarks (mth)
# - 30. Oct. 2007  - Support for an output-directory with all files 
#                    created during "make all". (mth)
#                  - modified targets which creates assembler (lower-case s) 
#                    from C-source using make <.c-file w/o ext.>.s (mth)
#                  - removed redundant/unused defines, overall cleanup (mth)
# - 10. Nov. 2007  - renamed TCHAIN to TCHAIN_PREFIX, other minor cleanup (mth)
# - 13. Mar. 2008  - renamed FORMAT to LOADFORMAT, edited some comments/messages (mth)
# - 13. Apr. 2009  - OpenOCD options for batch-programming (make program) (mth)
# -  1. May  2009  - replaced SUBMDL with CHIP (mth)
# - 15. Jul. 2009  - ComSpec environment-variable to select host-OS, should 
#                    increase compatibility (mth)
# -  1. Sep. 2009  - rename ROM_RUN->FLASH_RUN, VECT_TAB_ROM->VECT_TAB_FLASH (mth)
# - 11. Sep. 2009  - new target to create output directories. attempt for better
#                    "Win32 only" support (without "Unix"-shell and -tools) (mth)
#                    This is much faster on Win32 then MSYS/MinGW or Cygwin. (mth)
# -  5. Dec. 2009  - automatic selection of gcc or g++ for linking. g++ used when C++ 
#                    source-files are listed. -nostartfiles not used for C++ (mth)
# - 16. May. 2010  - on Win32 use set instead of env to get environment, removed
#                    flashing with uVision, added BOARD to CDEFS and ADEFS, phony
#                    targets listed in one line, removed setting for output-format
#                    and always create both (.hex and .bin), added debug-format gdb,
#                    rename AT91LIBNOWARN to DISABLESPECIALWARNINGS, removed target 
#                    finished, remark on rm/cs-rm, other minor clean-ups, tested
#                    with GNU make version 3.81 from CS and Cygwin (mth)
# - 13. Jun. 2010  - Trigger build when non-source files have changed 
#                    (see BUILDONCHANGE). (mth)

#Define Firmware Version
FW_VERSION = 10.5.14

#Define Config UID
CFG_UID = 28

# Toolchain prefix (arm-elf- -> arm-elf-gcc.exe)
TCHAIN_PREFIX = arm-none-eabi-
#TCHAIN_PREFIX = arm-eabi-
#TCHAIN_PREFIX = arm-elf-
# cs-rm is a standard GNU rm which gets installed with CS G++ lite.
# Just the filename is different. Use REMOVE_CMD=rm in environments
# with rm (Linux, BSD, msys, Cygwin etc.)
REMOVE_CMD=rm
#REMOVE_CMD=cs-rm

# YES enables -mthumb option to flags for source-files listed 
# in SRC and CPPSRC and -mthumb-interwork option for all source
USE_THUMB_MODE = YES
#USE_THUMB_MODE = NO

# MCU name, submodel and board
# - MCU used for compiler-option (-mcpu)
# - SUBMDL used for linker-script name (-T) and passed as define
# - BOARD just passed as define (don't used '-' characters)
MCU      = cortex-m3
CHIP     = LPC1758
BOARD    = USER

# *** This example only supports "FLASH_RUN" ***
# RUN_MODE is passed as define and used for the linker-script filename,
# the user has to implement the necessary operations for 
# the used mode (i.e. no copy of .data, remapping...)
# Create FLASH-Image
RUN_MODE=FLASH_RUN
# Create RAM-Image
#RUN_MODE=RAM_RUN

# *** This example only supports "VECT_TAB_FLASH" ***
# Exception-vectors placement option is just passed as define,
# the user has to implement the necessary operations (i.e. remapping)
# Exception vectors in FLASH:
VECTOR_TABLE_LOCATION=VECT_TAB_FLASH
# Exception vectors in RAM:
#VECTOR_TABLE_LOCATION=VECT_TAB_RAM

# Directory for output files (lst, obj, dep, elf, sym, map, hex, bin etc.)
OUTDIR = $(RUN_MODE)
BINDIR = releases

# Target file name (without extension).
TARGET = project

# application dir and source
APPDIR = beethefirst
APPSRC = \
	$(APPDIR)/gcode_parse.c \
	$(APPDIR)/gcode_process.c \
	$(APPDIR)/beethefirst.c \
	$(APPDIR)/temp.c \
	$(APPDIR)/config.c \
	$(APPDIR)/endstops.c \
	$(APPDIR)/geometry.c \
	$(APPDIR)/bootloader.c \
	$(APPDIR)/pwm.c \
	$(APPDIR)/pause.c \
	$(APPDIR)/lights.c \
	$(APPDIR)/fans.c \
	$(APPDIR)/MovementController.c \
	$(APPDIR)/ExpBoard.c \
	app/grbl/planner.c \
	app/grbl/stepper.c

# Utility variables
APPLIBDIR    = libraries
INCLUDES = $(APPLIBDIR)/Core/CMSIS/Include \
	 $(APPLIBDIR)/LPCUSB/inc \
	 $(APPLIBDIR)/Core/Device/NXP/LPC17xx/Include/ \
	 $(APPLIBDIR)/Drivers/include \
	 $(APPLIBDIR)/FatFs/ \
	 $(APPLIBDIR)/R2C2/ \
	 $(APPLIBDIR)/iap/ \
	 app/grbl/ \
	 $(APPDIR)

# List C source files here. (C dependencies are automatically generated.)
# use file-extension c for "c-only"-files
SRC = \
	$(wildcard $(APPLIBDIR)/LPCUSB/src/*.c) \
	$(APPLIBDIR)/Core/Device/NXP/LPC17xx/Source/system_LPC17xx.c \
	$(APPLIBDIR)/Drivers/source/debug_frmwrk.c \
	$(wildcard $(APPLIBDIR)/Drivers/source/lpc17xx_*.c) \
	$(APPLIBDIR)/R2C2/ios.c \
	$(APPLIBDIR)/R2C2/usb.c \
	$(APPLIBDIR)/R2C2/serial_fifo.c \
	$(APPLIBDIR)/R2C2/serial.c \
	$(APPLIBDIR)/R2C2/sermsg.c \
	$(APPLIBDIR)/R2C2/sersendf.c \
	$(APPLIBDIR)/R2C2/timer.c \
	$(APPLIBDIR)/R2C2/adc.c \
	$(APPLIBDIR)/R2C2/spi.c \
	$(APPLIBDIR)/R2C2/sd.c \
	$(APPLIBDIR)/R2C2/buzzer.c \
	$(APPLIBDIR)/FatFs/ff.c \
	$(APPLIBDIR)/FatFs/fattime.c \
	$(APPLIBDIR)/iap/sbl_iap.c \
	$(APPSRC) \
	main.c \
	skel.c
	
	#$(APPLIBDIR)/FatFs/option/ccsbcs.c \
	
# List C source files here which must be compiled in ARM-Mode (no -mthumb).
# use file-extension c for "c-only"-files
## just for testing, timer.c classcould be compiled in thumb-mode too
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
ASRC = startup.S

# List Assembler source files here which must be assembled in ARM-Mode.
ASRCARM  = 

# Place project-specific -D (define) and/or -U options for C here.
CDEFS = 
# enables parameter-checking in NXP's library
#CDEFS += -DDEBUG
#CDEFS += -DWITH_USB_MS=1

# Place project-specific -D and/or -U options for 
# Assembler with preprocessor here.
ADEFS = 

# List any extra directories to look for include files here.
#    Each directory must be seperated by a space.
EXTRAINCDIRS  = $(INCLUDES)

# Extra libraries
#    Each library-name must be seperated by a space.
#    i.e. to link with libxyz.a, libabc.a and libefsl.a: 
#     EXTRA_LIBS = xyz abc efsl
#    for newlib-lpc (file: libnewlibc-lpc.a):
#EXTRA_LIBS = newlib-lpc
EXTRA_LIBS = 

# List non-source files which should trigger build here
#    Typically the Makefile and selected header-files
#    Entries must be seperated by a space.
BUILDONCHANGE = Makefile 

# Path to linker-scripts (see -T option)
LINKERSCRIPTPATH = .

# List any directories with files included from linker-scripts.
#    Each directory must be seperated by a space.
LINKERSCRIPTINC  = .

# List any extra directories to look for library files here.
#     Each directory must be seperated by a space.
EXTRA_LIBDIRS = 

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = Turn off optimization. Reduce compilation time and make debugging
#     produce the expected results.
# 1 = The compiler tries to reduce code size and execution time, without
#     performing any optimizations that take a great deal of compilation time.
# 2 = GCC performs nearly all supported optimizations that do not involve a 
#     space-speed tradeoff. As compared to -O1, this option increases
#     both compilation time and the performance of the generated code.
# 3 = Optimize yet more. Turns on -finline-functions and more.
# s = -Os enables all -O2 optimizations that do not typically increase code
#     size.
# (See gcc manual for further information)
OPT = 2
#OPT = 1
#OPT = 2
#OPT = 3
#OPT = s

# Debugging format.
#DEBUG = stabs
#DEBUG = dwarf-2
#DEBUG = gdb

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

# Flash programming tool
#FLASH_TOOL = OPENOCD
FLASH_TOOL = LPC21ISP
#FLASH_TOOL = R2C2

# Some warnings can be disabled by this setting 
# (useful for the old single-file AT91-lib) 
#  yes - disable some warnings
#  no  - keep default settings
DISABLESPECIALWARNINGS = yes
#DISABLESPECIALWARNINGS = no


# ---------------------------------------------------------------------------
# Options for lpc21isp by Martin Maurer 
# lpc21isp only supports NXP LPC and Analog ADuC ARMs though the 
# integrated uart-bootloader (ISP)
#
# Settings and variables:
LPC21ISP = ./LPC_ISP_183/lpc21isp_183/lpc21isp
LPC21ISP_FLASHFILE = $(OUTDIR)/$(TARGET).bin
LPC21ISP_PORT = /dev/ttyUSB1
LPC21ISP_BAUD = 230400
LPC21ISP_XTAL = 12000
# other options:
# -debug: verbose output
# -control: enter bootloader via RS232 DTR/RTS (only if hardware 
#           supports this feature - see NXP AppNote)
LPC21ISP_OPTIONS = -control
LPC21ISP_OPTIONS += -bin
LPC21ISP_OPTIONS += -wipe
#LPC21ISP_OPTIONS += -debug
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Options for OpenOCD flash-programming
# see openocd.pdf/openocd.texi for further information
#
OOCD_LOADFILE+=$(OUTDIR)/$(TARGET).elf
# if OpenOCD is in the $PATH just set OPENOCDEXE=openocd
OOCD_EXE=./OpenOCD/bin/openocd
# debug level
OOCD_CL=-d0
#OOCD_CL=-d2
# interface and board/target settings (using the OOCD target-library here)
##OOCD_CL+=-f interface/jtagkey.cfg
OOCD_CL+=-f interface/jtagkey2.cfg
OOCD_CL+=-f LPC1766.cfg
# set JTAG frequency
#OOCD_CL+= -c "adapter_khz 500"
# initialize
OOCD_CL+=-c init
# enable "fast mode" - can be disabled for tests
## OOCD_CL+=-c "fast enable"
# show the targets
OOCD_CL+=-c targets
# commands to prepare flash-write
OOCD_CL+= -c "reset init"
#OOCD_CL+= -c "soft_reset_halt"
# disable polling (optional)
OOCD_CL+= -c "poll off"
# flash-write and -verify
#OOCD_CL+=-c "mww 0x400FC040 1"
#OOCD_CL+=-c "mdw 0x400FC040"
OOCD_CL+=-c "flash write_image erase $(OOCD_LOADFILE)"
#OOCD_CL+=-c "mdw 0x400FC040"
OOCD_CL+=-c "verify_image $(OOCD_LOADFILE)"
#OOCD_CL+=-c "mdw 0x400FC040"
# reset target
OOCD_CL+=-c "reset run"
# terminate OOCD after programming
OOCD_CL+=-c shutdown
# ---------------------------------------------------------------------------


ifdef VECTOR_TABLE_LOCATION
CDEFS += -D$(VECTOR_TABLE_LOCATION)
ADEFS += -D$(VECTOR_TABLE_LOCATION)
endif

CDEFS += -D$(RUN_MODE) -D$(CHIP) -D$(BOARD)
ADEFS += -D$(RUN_MODE) -D$(CHIP) -D$(BOARD)


# Compiler flags.

ifeq ($(USE_THUMB_MODE),YES)
THUMB    = -mthumb
THUMB_IW = -mthumb-interwork
else 
THUMB    = 
THUMB_IW = 
endif

#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#
# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)
CFLAGS =  -g
CFLAGS += -O$(OPT)
CFLAGS += -mcpu=$(MCU) $(THUMB_IW) 
CFLAGS += $(CDEFS)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.
# when using ".ramfunc"s without attribute longcall:
#CFLAGS += -mlong-calls
# -mapcs-frame is important if gcc's interrupt attributes are used
# (at least from my eabi tests), not needed if assembler-wrappers are used 
#CFLAGS += -mapcs-frame 
#CFLAGS += -fomit-frame-pointer
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wall -Wextra
#CFLAGS += -pedantic
CFLAGS += -Wimplicit -Wcast-align -Wpointer-arith
CFLAGS += -Wredundant-decls -Wshadow -Wcast-qual -Wcast-align
CFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
# Compiler flags to generate dependency files:
#CFLAGS += -MD -MP -MF $(OUTDIR)/dep/$(@F).d 
CFLAGS += -MMD -MP -MF $(OUTDIR)/dep/$(@F).d
##testing##CFLAGS += -MF"$@" -MG -MM -MP -MT"$@" -MT"$(<:.c=.o)"
#CFLAGS += -fno-dwarf2-cfi-asm
#CFLAGS += -fno-strict-aliasing -Wno-nested-externs

# flags only for C
CONLYFLAGS += -Wnested-externs 
CONLYFLAGS += $(CSTANDARD)

ifeq ($(DISABLESPECIALWARNINGS),yes)
CFLAGS += -Wno-cast-qual
CONLYFLAGS += -Wno-missing-prototypes 
CONLYFLAGS += -Wno-strict-prototypes
CONLYFLAGS += -Wno-missing-declarations
endif

# flags only for C++ (arm-*-g++)
CPPFLAGS = -fno-rtti -fno-exceptions


# Assembler flags.
#  -Wa,...:    tell GCC to pass this to the assembler.
#  -ahlns:     create listing
#  -g$(DEBUG): have the assembler create line number information
ASFLAGS  = -mcpu=$(MCU) $(THUMB_IW) -I. -x assembler-with-cpp -g
ASFLAGS += -D__ASSEMBLY__ $(ADEFS)
ASFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
ASFLAGS += -Wa,-g$(DEBUG)
ASFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

# Link with the GNU C++ stdlib.
CPLUSPLUS_LIB = -lstdc++
#CPLUSPLUS_LIB += -lsupc++

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(OUTDIR)/$(TARGET).map,--cref,--gc-sections
LDFLAGS += -lgcc -lc -lm 
#LDFLAGS += $(CPLUSPLUS_LIB)
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += $(patsubst %,-L%,$(LINKERSCRIPTINC))
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS))

# Set linker-script name depending on selected run-mode and chip
ifeq ($(RUN_MODE),RAM_RUN)
LDFLAGS +=-T$(LINKERSCRIPTPATH)/$(CHIP)_ram.ld
else 
LDFLAGS +=-T$(LINKERSCRIPTPATH)/$(CHIP)_flash.ld
endif

# Autodetect environment
SHELL   = sh
ifneq ($(or $(COMSPEC), $(ComSpec)),)
$(info COMSPEC detected $(COMSPEC) $(ComSpec))
ifeq ($(findstring cygdrive,$(shell set)),)
SHELL:=$(or $(COMSPEC),$(ComSpec))
SHELL_IS_WIN32=1
else
$(info cygwin detected)
#override user-setting since cygwin has rm
REMOVE_CMD:=rm
endif
else
#most probaly a Unix/Linux/BSD system which should have rm
REMOVE_CMD:=rm
endif
$(info SHELL is $(SHELL), REMOVE_CMD is $(REMOVE_CMD))

# Define programs and commands.
CC      = $(TCHAIN_PREFIX)gcc
CPP     = $(TCHAIN_PREFIX)g++
AR      = $(TCHAIN_PREFIX)ar
OBJCOPY = $(TCHAIN_PREFIX)objcopy
OBJDUMP = $(TCHAIN_PREFIX)objdump
SIZE    = $(TCHAIN_PREFIX)size
NM      = $(TCHAIN_PREFIX)nm
REMOVE  = $(REMOVE_CMD) -f

# Define Messages
# English
MSG_BEGIN = --------  begin, mode: $(RUN_MODE)  --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before:
MSG_SIZE_AFTER = Size after build:
MSG_LOAD_FILE =        " COPY     "
MSG_EXTENDED_LISTING = Creating Extended Listing/Disassembly:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING =          "  LD      "
MSG_COMPILING =        "  CC      "
MSG_COMPILING_ARM =    "  CC      "
MSG_COMPILINGCPP =     "  CPP     "
MSG_COMPILINGCPP_ARM = "  CPP     "
MSG_ASSEMBLING =       "  AS      "
MSG_ASSEMBLING_ARM =   "  AS      "
MSG_CLEANING = Cleaning project:
MSG_LPC21_RESETREMINDER = You may have to bring the target in bootloader-mode now.
MSG_ASMFROMC = "Creating asm-File from C-Source:"
MSG_ASMFROMC_ARM = "Creating asm-File from C-Source (ARM-only):"

# List of all source files.
ALLSRC     = $(ASRCARM) $(ASRC) $(SRCARM) $(SRC) $(CPPSRCARM) $(CPPSRC)
# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))

# Define all object files.
ALLOBJ     = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))

# Define all listing files (used for make clean).
LSTFILES   = $(addprefix $(OUTDIR)/, $(addsuffix .lst, $(ALLSRCBASE)))
# Define all depedency-files (used for make clean).
DEPFILES   = $(addprefix $(OUTDIR)/dep/, $(addsuffix .o.d, $(ALLSRCBASE)))

# Default target.
#all: begin createdirs gccversion build sizeafter end
#all: btf msftbtf btfold btfplus btfme btfschool btfplusbatt
all:

#Debug UART BTF+
debugPlus: CFLAGS += -DDEBUG_UART
debugPlus: btfplus

# BTF
btf: CFLAGS += -DBTF
btf: CFLAGS += -DFW_V='"BEEVC-BEETHEFIRST-$(FW_VERSION)"' -DCFG_UID=$(CFG_UID)
btf: begin createdirs gccversion build sizeafter copyBinBTF end

# BTF_OLD
btfold: CFLAGS += -DBTF_OLD
btfold: CFLAGS += -DFW_V='"BEEVC-BEETHEFIRST0-$(FW_VERSION)"' -DCFG_UID=$(CFG_UID)
btfold: begin createdirs gccversion build sizeafter copyBinBTF_OLD end

# BTF_PLUS
btfplus: CFLAGS += -DBTF_PLUS
btfplus: CFLAGS += -DFW_V='"BEEVC-BEETHEFIRST_PLUS-$(FW_VERSION)"' -DCFG_UID=$(CFG_UID)
btfplus: begin createdirs gccversion build sizeafter copyBinBTF_PLUS end

# BTF_PLUS_BATT
btfplusbatt: CFLAGS += -DBTF_PLUS_BATT
btfplusbatt: CFLAGS += -DFW_V='"BEEVC-BEETHEFIRST_PLUS_A-$(FW_VERSION)"' -DCFG_UID=$(CFG_UID)
btfplusbatt: begin createdirs gccversion build sizeafter copyBinBTF_PLUS_Batt end

# BTF_ME
btfme: CFLAGS += -DBTF_ME
btfme: CFLAGS += -DFW_V='"BEEVC-BEEME-$(FW_VERSION)"' -DCFG_UID=$(CFG_UID)
btfme: begin createdirs gccversion build sizeafter copyBinBTF_ME end


# BTF_IS
btfschool: CFLAGS += -DBTF_SCHOOL
btfschool: CFLAGS += -DFW_V='"BEEVC-BEEINSCHOOL-$(FW_VERSION)"' -DCFG_UID=$(CFG_UID)
btfschool: begin createdirs gccversion build sizeafter copyBinBTF_SCHOOL end

# BTF_IS_BATT
btfschoolbatt: CFLAGS += -DBTF_SCHOOL_BATT
btfschoolbatt: CFLAGS += -DFW_V='"BEEVC-BEEINSCHOOL_A-$(FW_VERSION)"' -DCFG_UID=$(CFG_UID)
btfschoolbatt: begin createdirs gccversion build sizeafter copyBinBTF_SCHOOL_BATT end

#

# Target for the build-sequence.
build: elf lss sym hex bin

elf: $(OUTDIR)/$(TARGET).elf
lss: $(OUTDIR)/$(TARGET).lss 
sym: $(OUTDIR)/$(TARGET).sym
hex: $(OUTDIR)/$(TARGET).hex
bin: $(OUTDIR)/$(TARGET).bin

# Create output directories.
ifdef SHELL_IS_WIN32
createdirs:
	-@md $(OUTDIR) >NUL 2>&1 || echo "" >NUL
	-@md $(OUTDIR)\dep >NUL 2>&1 || echo "" >NUL
	-@md $(BINDIR) >NUL 2>&1 || echo "" >NUL
else
createdirs:
	-@mkdir $(OUTDIR) 2>/dev/null || echo "" >/dev/null
	-@mkdir $(OUTDIR)/dep 2>/dev/null || echo "" >/dev/null
	-@mkdir $(BINDIR) 2>/dev/null || echo "" >/dev/null

#Copy bin files to bin directory
copyBinBTF_OLD:
	cp $(OUTDIR)/$(TARGET).bin $(BINDIR)/BEEVC-BEETHEFIRST0-Firmware-$(FW_VERSION).BIN
		
copyBinBTF:
	cp $(OUTDIR)/$(TARGET).bin $(BINDIR)/BEEVC-BEETHEFIRST-Firmware-$(FW_VERSION).BIN
	cp $(OUTDIR)/$(TARGET).bin ~/git/BEEcom/BTF
	
copyBinBTF_PLUS:
	cp $(OUTDIR)/$(TARGET).bin $(BINDIR)/BEEVC-BEETHEFIRST-PLUS-Firmware-$(FW_VERSION).BIN
	cp $(OUTDIR)/$(TARGET).bin ~/git/BEEcom/BTF

copyBinBTF_PLUS_Batt:
	cp $(OUTDIR)/$(TARGET).bin $(BINDIR)/BEEVC-BEETHEFIRST-PLUS-A-Firmware-$(FW_VERSION).BIN
	cp $(OUTDIR)/$(TARGET).bin ~/git/BEEcom/BTF
	
copyBinBTF_ME:
	cp $(OUTDIR)/$(TARGET).bin $(BINDIR)/BEEVC-BEEME-Firmware-$(FW_VERSION).BIN
	cp $(OUTDIR)/$(TARGET).bin ~/git/BEEcom/BTF

copyBinBTF_SCHOOL:
	cp $(OUTDIR)/$(TARGET).bin $(BINDIR)/BEEVC-BEEINSCHOOL-Firmware-$(FW_VERSION).BIN
	cp $(OUTDIR)/$(TARGET).bin ~/git/BEEcom/BTF
	
copyBinBTF_SCHOOL_BATT:
	cp $(OUTDIR)/$(TARGET).bin $(BINDIR)/BEEVC-BEEINSCHOOL-A-Firmware-$(FW_VERSION).BIN
	cp $(OUTDIR)/$(TARGET).bin ~/git/BEEcom/BTF

endif

# Eye candy.
begin:
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)


# Display sizes of sections.
ELFSIZE = $(SIZE) -A $(OUTDIR)/$(TARGET).elf | egrep '^(section|Total|\.(text|data|bss|stack|heap))'
sizebefore:
#	@if [ -f  $(OUTDIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
#	@if [ -f  $(OUTDIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi
	@echo $(MSG_SIZE_AFTER)
	@$(ELFSIZE)

# Display compiler version information.
gccversion : 
	@$(CC) --version

# Program the device.
ifeq ($(FLASH_TOOL),OPENOCD)
# Program the device with Dominic Rath's OPENOCD in "batch-mode"
program: $(OUTDIR)/$(TARGET).elf
	@echo "Programming with OPENOCD"
ifdef SHELL_IS_WIN32 
	$(subst /,\,$(OOCD_EXE)) $(OOCD_CL)
else
	$(OOCD_EXE) $(OOCD_CL)
endif
else ifeq ($(FLASH_TOOL),LPC21ISP)
# Program the device using lpc21isp (for NXP2k and ADuC UART bootloader)
program: $(OUTDIR)/$(TARGET).hex
	@echo $(MSG_LPC21_RESETREMINDER)
	-$(LPC21ISP) $(LPC21ISP_OPTIONS) $(LPC21ISP_FLASHFILE) $(LPC21ISP_PORT) $(LPC21ISP_BAUD) $(LPC21ISP_XTAL)
else ifeq ($(FLASH_TOOL),R2C2)
program: $(OUTDIR)/$(TARGET).bin
	mount /mnt/r2c2
	cp $(OUTDIR)/$(TARGET).bin /mnt/r2c2/firmware.bin
	eject /mnt/r2c2
endif

# Create final output file in ihex format from ELF output file (.hex).
%.hex: %.elf
	@echo "  HEX   " $@
	@$(OBJCOPY) -O ihex $< $@

# Create final output file in raw binary format from ELF output file (.bin)
%.bin: %.elf
	@echo "  BIN   " $@
	@$(OBJCOPY) -O binary $< $@

# Create extended listing file/disassambly from ELF output file.
# using objdump (testing: option -C)
%.lss: %.elf
	@echo "  LIST  " $@
	@$(OBJDUMP) -h -S -C -r $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo "  SYM   " $@
	@$(NM) -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(ALLOBJ)
%.elf:  $(ALLOBJ) $(BUILDONCHANGE)
	@echo "  LINK  " $@
# use $(CC) for C-only projects or $(CPP) for C++-projects:
ifeq "$(strip $(CPPSRC)$(CPPARM))" ""
	@$(CC) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ -nostartfiles $(LDFLAGS)
else
	@$(CPP) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)
endif


# Assemble: create object files from assembler source files.
define ASSEMBLE_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) $(BUILDONCHANGE)
	@echo "  AS    " $$@
	@$(CC) -c $(THUMB) $$(ASFLAGS) $$< -o $$@
endef
$(foreach src, $(ASRC), $(eval $(call ASSEMBLE_TEMPLATE, $(src)))) 

# Assemble: create object files from assembler source files. ARM-only
define ASSEMBLE_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) $(BUILDONCHANGE)
	@echo $(MSG_ASSEMBLING_ARM) $$< to $$@
	$(CC) -c $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRCARM), $(eval $(call ASSEMBLE_ARM_TEMPLATE, $(src)))) 


# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) $(BUILDONCHANGE)
	@echo "  CC    " $$@
	@$(CC) -c $(THUMB) $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@
endef
$(foreach src, $(SRC), $(eval $(call COMPILE_C_TEMPLATE, $(src)))) 

# Compile: create object files from C source files. ARM-only
define COMPILE_C_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) $(BUILDONCHANGE)
	@echo $(MSG_COMPILING_ARM) $$< to $$@
	$(CC) -c $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRCARM), $(eval $(call COMPILE_C_ARM_TEMPLATE, $(src)))) 


# Compile: create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) $(BUILDONCHANGE)
	@echo $(MSG_COMPILINGCPP) $$< to $$@
	$(CPP) -c $(THUMB) $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRC), $(eval $(call COMPILE_CPP_TEMPLATE, $(src)))) 

# Compile: create object files from C++ source files. ARM-only
define COMPILE_CPP_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1) $(BUILDONCHANGE)
	@echo $(MSG_COMPILINGCPP_ARM) $$< to $$@
	$(CPP) -c $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRCARM), $(eval $(call COMPILE_CPP_ARM_TEMPLATE, $(src)))) 


# Compile: create assembler files from C source files. ARM/Thumb
$(SRC:.c=.s) : %.s : %.c $(BUILDONCHANGE)
	@echo $(MSG_ASMFROMC) $< to $@
	$(CC) $(THUMB) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Compile: create assembler files from C source files. ARM only
$(SRCARM:.c=.s) : %.s : %.c $(BUILDONCHANGE)
	@echo $(MSG_ASMFROMC_ARM) $< to $@
	$(CC) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Target: clean project.
clean: begin clean_list end

clean_list :
	@echo $(MSG_CLEANING)
	@$(REMOVE) $(OUTDIR)/$(TARGET).map
	@$(REMOVE) $(OUTDIR)/$(TARGET).elf
	@$(REMOVE) $(OUTDIR)/$(TARGET).hex
	@$(REMOVE) $(OUTDIR)/$(TARGET).bin
	@$(REMOVE) $(OUTDIR)/$(TARGET).sym
	@$(REMOVE) $(OUTDIR)/$(TARGET).lss
	@$(REMOVE) $(ALLOBJ)
	@$(REMOVE) $(LSTFILES)
	@$(REMOVE) $(DEPFILES)
	@$(REMOVE) $(SRC:.c=.s)
	@$(REMOVE) $(SRCARM:.c=.s)
	@$(REMOVE) $(CPPSRC:.cpp=.s)
	@$(REMOVE) $(CPPSRCARM:.cpp=.s)
	#@$(REMOVE) $(BINDIR)/*

# Include the dependency files.
-include $(wildcard $(OUTDIR)/dep/*)

# Listing of phony targets.
.PHONY : all begin end sizebefore sizeafter gccversion build elf hex bin lss sym clean clean_list program createdirs

