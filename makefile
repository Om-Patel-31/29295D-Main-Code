# VEXcode makefile 2019_03_26_01. The default target produces the V5 binary.

# Set VERBOSE to 1 to request command visibility through vex/mkenv.mk.
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# Collect all C/C++ implementation files under src for compilation.
SRC_C  = $(wildcard src/*.cpp) 
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard src/*/*.cpp) 
SRC_C += $(wildcard src/*/*.c)

OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )

# All project headers are dependencies for each source compilation rule.
SRC_H  = $(wildcard include/*.h)

# Changes to the makefile force source recompilation.
SRC_A  = makefile

# project header file locations
INC_F  = include

# build targets
# Build the linked ELF and convert it to the downloadable V5 binary.
all: $(BUILD)/$(PROJECT).bin
	@echo done

# include build rules
include vex/mkrules.mk
