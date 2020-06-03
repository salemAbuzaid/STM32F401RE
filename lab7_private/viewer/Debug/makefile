################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
viewer.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
viewer.list \

OBJCOPY_BIN += \
viewer.bin \


# All Target
all: viewer.elf secondary-outputs

# Tool invocations
viewer.elf: $(OBJS) $(USER_OBJS) D:\POLITO\3rd\ year\digital\ systema\ electronics\STM32\lab7_private\viewer\STM32F401RETX_FLASH.ld
	arm-none-eabi-gcc -o "viewer.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\POLITO\3rd year\digital systema electronics\STM32\lab7_private\viewer\STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="viewer.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

viewer.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "viewer.list"
	@echo 'Finished building: $@'
	@echo ' '

viewer.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "viewer.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
