# File: makefile
# Author: Justin Garcia
# ---------------------
# Simple makefile for avr-libc compilation.

# ---------------------------------------------- #
# ----------------- Variables ------------------ #
# ---------------------------------------------- #

# --------- Target Specific Options ------------ #
TARGET =

MODULES = lib fatfs cc3000 vs1053

DIRAVR = ~/Code/atmega328p
DIRINC = $(DIRAVR) $(addprefix $(DIRAVR)/, $(MODULES))

SRC = $(TARGET).c \
	$(foreach d, $(DIRINC), $(wildcard $d/*.c))

# -------------- Compiler Options -------------- #
CC = avr-gcc

DIROBJ = .obj
#OBJ = $(patsubst %.c, $(DIROBJ)/%.o, $(notdir $(SRC)))
OBJ = $(SRC:%.c=%.o)

CC_MCU = atmega328p
CSTD = -std=c99
OPT = s
CINC = $(foreach d, $(DIRINC), -I $d)

CFLAGS = -mmcu=$(CC_MCU)
CFLAGS += $(CSTD)
CFLAGS += -O$(OPT)
CFLAGS += -Wall
CFLAGS += -I. $(CINC)

# ------------- Dependency Options ------------- #
DIRDEP = .dep
DEP = $(patsubst %.c, $(DIROBJ)/%.d, $(notdir $(SRC)))
df = $(DIRDEP)/$(*F)

DEP_FLAGS = -MM

# ------------- Object Copy Options ------------ #
OBJCOPY = avr-objcopy

OBJCOPY_MEM_SECTIONS = -j .text -j .data
OBJCOPY_FORMAT = ihex

OBJCOPY_FLAGS = $(OBJCOPY_MEM_SECTIONS)
OBJCOPY_FLAGS += -O $(OBJCOPY_FORMAT)

# ------------- Programming Options ------------ #
AVRDUDE	= avrdude

AVRDUDE_MCU = m328p
AVRDUDE_PROGRAMMER = usbtiny
AVRDUDE_VERBOSE = -v
AVRDUDE_ERASE = -e

AVRDUDE_FLAGS = -p $(AVRDUDE_MCU)
AVRDUDE_FLAGS += -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE)

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex

# ------------------ Commands ------------------ #
COPY = cp -f
EDIT = sed
FORMAT = fmt
MKDIR = mkdir -p
MOVE = mv -f
REMOVE = rm -f
RMDIR = rmdir
SUDO = sudo

# ------------------ Messages ------------------ #
MSG_BEGIN = -------- begin --------
MSG_COMPILE = Compiling:
MSG_LINK = Linking:
MSG_FLASH = Creating load file for Flash:
MSG_PROGRAM = Programming MCU:
MSG_CLEAN = Cleaning project:
MSG_END = -------- end --------

# ---------------------------------------------- #
# -------------- Additional Rules -------------- #
# ---------------------------------------------- #

all : begin build end

build : $(TARGET).hex

begin :
	@echo
	@echo $(MSG_BEGIN)

end :
	@echo $(MSG_END)
	@echo

$(OBJ) : | $(DIROBJ)

$(DIROBJ) :
	@$(MKDIR) $@

$(DEP) : | $(DIRDEP)

$(DIRDEP) :
	@$(MKDIR) $@

%.o : %.c
	@echo $(MSG_COMPILE)
	@$(CC) $(CFLAGS) $(DEP_FLAGS) $*.c > $(df).d
	@$(MOVE) $(df).d $(df).d.tmp
	@$(EDIT) -e 's|.*:|$*.o:|' < $(df).d.tmp > $(df).d
	@$(EDIT) -e 's/.*://' -e 's/\\$$//' < $(df).d.tmp | $(FORMAT) -1 | \
		$(EDIT) -e 's/^ *//' -e 's/$$/:/' >> $(df).d
	@$(REMOVE) $(df).d.tmp
	$(CC) $(CFLAGS) -o $(DIROBJ)/$(@F) -c $<

-include $(DEP)

$(TARGET).elf : $(OBJ)
	@echo $(MSG_LINK)
	$(CC) $(CFLAGS) -o $@ $(addprefix $(DIROBJ)/, $(^F))

$(TARGET).hex : $(TARGET).elf
	@echo $(MSG_FLASH)
	$(OBJCOPY) $(OBJCOPY_FLAGS) $^ $@

program : $(TARGET).hex
	@echo $(MSG_PROGRAM)
	$(SUDO) $(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)

clean :
	@echo
	@echo $(MSG_CLEAN)
	$(REMOVE) *.hex
	$(REMOVE) *.elf
	$(REMOVE) .obj/*
	$(RMDIR) .obj
	$(REMOVE) .dep/*
	$(RMDIR) .dep

.PHONY : all begin build end program clean
