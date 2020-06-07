################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/lab8_ex4/main.c \
../Core/Src/lab8_ex4/stm32f4xx_it.c 

OBJS += \
./Core/Src/lab8_ex4/main.o \
./Core/Src/lab8_ex4/stm32f4xx_it.o 

C_DEPS += \
./Core/Src/lab8_ex4/main.d \
./Core/Src/lab8_ex4/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/lab8_ex4/main.o: ../Core/Src/lab8_ex4/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DLSI_VALUE=32000' -DSTM32F401xE '-DHSE_STARTUP_TIMEOUT=100' '-DHSI_VALUE=16000000' -DDEBUG '-DLSE_STARTUP_TIMEOUT=5000' '-DLSE_VALUE=32768' '-DDATA_CACHE_ENABLE=1' '-DEXTERNAL_CLOCK_VALUE=12288000' '-DVDD_VALUE=3300' '-DINSTRUCTION_CACHE_ENABLE=1' '-DHSE_VALUE=25000000' -DUSE_FULL_LL_DRIVER '-DPREFETCH_ENABLE=1' -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/lab8_ex4/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/lab8_ex4/stm32f4xx_it.o: ../Core/Src/lab8_ex4/stm32f4xx_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DLSI_VALUE=32000' -DSTM32F401xE '-DHSE_STARTUP_TIMEOUT=100' '-DHSI_VALUE=16000000' -DDEBUG '-DLSE_STARTUP_TIMEOUT=5000' '-DLSE_VALUE=32768' '-DDATA_CACHE_ENABLE=1' '-DEXTERNAL_CLOCK_VALUE=12288000' '-DVDD_VALUE=3300' '-DINSTRUCTION_CACHE_ENABLE=1' '-DHSE_VALUE=25000000' -DUSE_FULL_LL_DRIVER '-DPREFETCH_ENABLE=1' -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/lab8_ex4/stm32f4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

