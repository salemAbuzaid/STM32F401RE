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
ex_stm32_project.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ex_stm32_project.list \

OBJCOPY_BIN += \
ex_stm32_project.bin \


# All Target
all: ex_stm32_project.elf secondary-outputs

# Tool invocations
ex_stm32_project.elf: $(OBJS) $(USER_OBJS) D:\POLITO\3rd\ year\digital\ systema\ electronics\STM32\lab7_private\lab7_ex2_project\ex_stm32_project\STM32F401RETX_FLASH.ld
	arm-none-eabi-gcc -o "ex_stm32_project.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\POLITO\3rd year\digital systema electronics\STM32\lab7_private\lab7_ex2_project\ex_stm32_project\STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ex_stm32_project.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ex_stm32_project.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ex_stm32_project.list"
	@echo 'Finished building: $@'
	@echo ' '

ex_stm32_project.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "ex_stm32_project.bin"
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
