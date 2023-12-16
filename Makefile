CC = gcc
BUILDPREFIX = arm-none-eabi-
BUILDCMD = $(BUILDPREFIX)$(CC)
#Removing -g temporarily to check restore it after checking
GCCOPTIONS = -O0 -mcpu=cortex-m4 -g -mthumb -nostartfiles -mhard-float -mfpu=vfpv4
#GCCOPTIONS = -O0 -mcpu=cortex-m4 -g -mthumb -nostartfiles
OBJCOPY = arm-none-eabi-objcopy
OBJOPTIONS = -O binary
CFLAGS = $(addprefix -I, $(include_dirs))

include_dirs = . include
source_dirs = src
source1 = build_example_main.c
source2 = startup_gcc.c

headers = core_initialisation_STM32F411xe.h \
          memory_map_stm32f411xe.h

sources = build_example_main.c \
          core_initialisation_STM32F411xe.c \
		  startup_gcc.c

linker_script = build_example.ld		  

vpath %.c $(source_dirs)
vpath %.h $(include_dirs)
VPATH = $(source_dirs) $(include_dirs)

.PHONY: all
all: build_example.bin

#The prerequisite must be a file or a target, if make does not find a target as this prerequisite, then 
#it checks to see if a file with the same name as the prerequisite exists, if not make stops with 'no rule found' error  

#Automatic Variables $^ follows vpath, just putting the filename in the rule will break the 
#compile if its not in the same path. only targets and prerequisites can take advantage of vpath 
#other variables will not work. Including the headers in the prerequisites is a simple way of version 
# controlling of the headers, these headers get included in the make command and uses vpath but it 
#is unused and has no adverse effect
#on the compile. The headers actually accessed by the code not through the make headers but through the -I option
#There is a duplication here but all to monitor the timestamp of the headers as well

build_example.bin: build_example.out
	arm-none-eabi-objcopy -O binary build_example.out build_example.bin

build_example.out: $(sources) $(headers)
	$(BUILDCMD) $(GCCOPTIONS) $(CFLAGS) $^ -T $(linker_script) -Wl,-Map=output.map -o  $@



#The .PHONY target means the target name is not a real target, even if a file exists with the same name
#still the target is ignored and this rule executed. As long as a file does not exist for a target it 
#will execute as if it's a fresh file. Such a target behaving as a .PHONY target is called an empty target
#A file will not  exist for a target if the command in the rule 
#does not create it 
.PHONY: clean
clean:
	rm -rf *.o *.bin *.out 
