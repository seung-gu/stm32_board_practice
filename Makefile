#Makefile for building binary image for STM32F411ve

CC = gcc
BUILDPREFIX = arm-none-eabi-
BUILDCMD = $(BUILDPREFIX)$(CC)
#Removing -g temporarily to check restore it after checking
GCCOPTIONS = -g -O0 -mcpu=cortex-m4 -mthumb -nostartfiles -mhard-float -mfpu=vfpv4
OBJCOPY = arm-none-eabi-objcopy
OBJOPTIONS = -O binary
CFLAGS = $(addprefix -I, $(include_dirs))
#CFLAGS = $(addprefix -I, $@)

include_dirs = . include
source_dirs =  src
output_binary = linker_example.bin
#Cube IDE seems to be asking for build_example.out only
output_object = prerequisites.out
linker_script = build_example.ld
#output_binary_debug = build_example.bin
#output_object_debug = build_example.o

sources = \
 startup_gcc.c \
 core_initialisation_STM32F411xe.c \
 struct_padding.c
 
 

headers = core_initialisation_STM32F411xe.h \
          memory_map_stm32f411xe.h

vpath %.c $(source_dirs)
vpath %.h $(include_dirs)
vpath %.S $(source_dirs)


.PHONY: all
all: $(output_binary)

$(output_binary): $(output_object)
	$(OBJCOPY) $(OBJOPTIONS) $^ $@
	

$(output_object): $(sources) $(headers)
	$(BUILDCMD) $(GCCOPTIONS) $(CFLAGS) $^ -T $(linker_script) -Wl,-Map=output.map -o  $@


.PHONY: clean
clean:
	rm -rf *.o *.bin *.out 

