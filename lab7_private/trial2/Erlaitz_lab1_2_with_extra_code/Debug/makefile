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
Erlaitz_lab1_2_with_extra_code.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Erlaitz_lab1_2_with_extra_code.list \

OBJCOPY_BIN += \
Erlaitz_lab1_2_with_extra_code.bin \


# All Target
all: Erlaitz_lab1_2_with_extra_code.elf secondary-outputs

# Tool invocations
Erlaitz_lab1_2_with_extra_code.elf: $(OBJS) $(USER_OBJS) D:\POLITO\3rd\ year\digital\ systema\ electronics\STM32\lab7_private\trial2\Erlaitz_lab1_2_with_extra_code\STM32F401RETX_FLASH.ld
	arm-none-eabi-gcc -o "Erlaitz_lab1_2_with_extra_code.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\POLITO\3rd year\digital systema electronics\STM32\lab7_private\trial2\Erlaitz_lab1_2_with_extra_code\STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Erlaitz_lab1_2_with_extra_code.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Erlaitz_lab1_2_with_extra_code.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Erlaitz_lab1_2_with_extra_code.list"
	@echo 'Finished building: $@'
	@echo ' '

Erlaitz_lab1_2_with_extra_code.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Erlaitz_lab1_2_with_extra_code.bin"
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
