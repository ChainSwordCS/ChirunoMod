#---------------------------------------------------------------------------------
.SUFFIXES:
#---------------------------------------------------------------------------------

ifeq ($(strip $(DEVKITARM)),)
$(error "Please set DEVKITARM in your environment. export DEVKITARM=<path to>devkitARM")
endif

TOPDIR ?= $(CURDIR)
include $(DEVKITARM)/3ds_rules

#---------------------------------------------------------------------------------
# TARGET is the name of the output
# BUILD is the directory where object files & intermediate files will be placed
# SOURCES is a list of directories containing source code
# DATA is a list of directories containing data files
# INCLUDES is a list of directories containing header files
#
# NO_SMDH: if set to anything, no SMDH file is generated.
# APP_TITLE is the name of the app stored in the SMDH file (Optional)
# APP_DESCRIPTION is the description of the app stored in the SMDH file (Optional)
# APP_AUTHOR is the author of the app stored in the SMDH file (Optional)
# ICON is the filename of the icon (.png), relative to the project folder.
#   If not set, it attempts to use one of the following (in this order):
#     - <Project name>.png
#     - icon.png
#     - <libctru folder>/default_icon.png
#---------------------------------------------------------------------------------

TARGET=ChirunoMod
BUILD		    :=	chirunomod-build
SOURCES		    :=	soos
DATA		    :=	data
INCLUDES	    :=	inc
APP_TITLE       :=  ChirunoMod
APP_DESCRIPTION :=  Chiruno Sys Module
APP_AUTHOR      :=  Sono, ChainSwordCS
APP_PRODUCT_CODE:=  CTR-N-ACHA
APP_UNIQUE_ID   :=  0xCF009

APP_TITLE       :=  $(shell echo "$(APP_TITLE)" | cut -c1-128)
APP_DESCRIPTION :=  $(shell echo "$(APP_DESCRIPTION)" | cut -c1-256)
APP_AUTHOR      :=  $(shell echo "$(APP_AUTHOR)" | cut -c1-128)
APP_PRODUCT_CODE:=  $(shell echo $(APP_PRODUCT_CODE) | cut -c1-16)
APP_UNIQUE_ID   :=  $(shell echo $(APP_UNIQUE_ID) | cut -c1-8)

#---------------------------------------------------------------------------------
# options for code generation
#---------------------------------------------------------------------------------
ARCH	:=	-march=armv6k -mtune=mpcore -mfloat-abi=hard

CFLAGS	:=	-g -Wall -Wno-format -O0 -mword-relocations \
			-fomit-frame-pointer -ffast-math \
			$(ARCH)

CFLAGS	+=	$(INCLUDE) -DARM11 -D_3DS

CXXFLAGS	:= $(CFLAGS) -fno-rtti -std=gnu++11

ASFLAGS	:=	-g $(ARCH)
LDFLAGS	=	-specs=3dsx.specs -g $(ARCH) -Wl,-Map,$(notdir $*.map)

LIBS	:= -lturbojpeg -lz -lctru -lm
#---------------------------------------------------------------------------------
# list of directories containing libraries, this must be the top level containing
# include and lib
#---------------------------------------------------------------------------------
LIBDIRS	:= $(CTRULIB) $(DEVKITPRO)/portlibs/armv6k


#---------------------------------------------------------------------------------
# no real need to edit anything past this point unless you need to add additional
# rules for different file extensions
#---------------------------------------------------------------------------------
ifneq ($(BUILD),$(notdir $(CURDIR)))
#---------------------------------------------------------------------------------

export OUTPUT	:=	$(CURDIR)/$(TARGET)

export TOPDIR	:=	$(CURDIR)

export VPATH	:=	$(foreach dir,$(SOURCES),$(CURDIR)/$(dir)) \
			$(foreach dir,$(DATA),$(CURDIR)/$(dir))

export DEPSDIR	:=	$(CURDIR)/$(BUILD)

CFILES		:=  $(shell find $(SOURCES) -name '*.c' -printf "%P\n")	#$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.c)))
CPPFILES	:=  $(shell find $(SOURCES) -name '*.cpp' -printf "%P\n")	#$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.cpp)))
SFILES		:=  $(shell find $(SOURCES) -name '*.s' -printf "%P\n")	#$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.s)))
BINFILES	:=	$(foreach dir,$(DATA),$(notdir $(wildcard $(dir)/*.*)))

#---------------------------------------------------------------------------------
# use CXX for linking C++ projects, CC for standard C
#---------------------------------------------------------------------------------
ifeq ($(strip $(CPPFILES)),)
#---------------------------------------------------------------------------------
	export LD	:=	$(CC)
#---------------------------------------------------------------------------------
else
#---------------------------------------------------------------------------------
	export LD	:=	$(CXX)
#---------------------------------------------------------------------------------
endif
#---------------------------------------------------------------------------------

export OFILES	:=	$(addsuffix .o,$(BINFILES)) \
			$(CPPFILES:.cpp=.o) $(CFILES:.c=.o) $(SFILES:.s=.o)

export INCLUDE	:=	$(foreach dir,$(INCLUDES),-I$(CURDIR)/$(dir)/include) \
			$(foreach dir,$(LIBDIRS),-I$(dir)/include) \
			-I$(CURDIR)/$(BUILD) \
			-I-$(CURDIR)/$(SOURCES)

export LIBPATHS	:=	$(foreach dir,$(LIBDIRS),-L$(dir)/lib) \
			$(foreach dir,$(INCLUDES),-L$(CURDIR)/$(dir)/lib)

ifeq ($(strip $(ICON)),)
	icons := $(wildcard *.png)
	ifneq (,$(findstring $(TARGET).png,$(icons)))
		export APP_ICON := $(TOPDIR)/$(TARGET).png
	else
		ifneq (,$(findstring icon.png,$(icons)))
			export APP_ICON := $(TOPDIR)/icon.png
		endif
	endif
else
	export APP_ICON := $(TOPDIR)/$(ICON)
endif

.PHONY: $(BUILD) clean all

#---------------------------------------------------------------------------------
all: $(BUILD)

$(BUILD):
	@[ -d $@ ] || mkdir -p $@
	@find $(SOURCES) -type d -printf "%P\0" | xargs -0 -I {} mkdir -p $(BUILD)/{}
	@make --no-print-directory -C $(BUILD) -f $(CURDIR)/Makefile

#---------------------------------------------------------------------------------
clean:
	@echo clean ...
	@rm -rf $(BUILD) $(TARGET).3dsx $(OUTPUT).smdh $(TARGET).elf $(TARGET).3ds $(TARGET).cia

#---------------------------------------------------------------------------------
else

DEPENDS	:=	$(OFILES:.o=.d)

#---------------------------------------------------------------------------------
# main targets
#---------------------------------------------------------------------------------
ifeq ($(strip $(NO_SMDH)),)
.PHONY: all
all	:	$(OUTPUT).cia
endif

$(OUTPUT).elf	:	$(OFILES)

$(OUTPUT)-stripped.elf: $(OUTPUT).elf
	@cp $(OUTPUT).elf $(OUTPUT)-stripped.elf
	@$(PREFIX)strip $(OUTPUT)-stripped.elf

$(OUTPUT).cia: $(OUTPUT)-stripped.elf
	@makerom -f cia -o $(OUTPUT).cia -rsf $(TOPDIR)/assets/cia.rsf -target t -exefslogo -elf $(OUTPUT)-stripped.elf -DAPP_TITLE="$(APP_TITLE)" -DAPP_PRODUCT_CODE="$(APP_PRODUCT_CODE)" -DAPP_UNIQUE_ID="$(APP_UNIQUE_ID)"
	@echo "built ... $(notdir $@)"

#---------------------------------------------------------------------------------
# you need a rule like this for each extension you use as binary data
#---------------------------------------------------------------------------------
%.bin.o	:	%.bin
#---------------------------------------------------------------------------------
	@echo $(notdir $<)
	@$(bin2o)

-include $(DEPENDS)

#---------------------------------------------------------------------------------------
endif
#---------------------------------------------------------------------------------------
