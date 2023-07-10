################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../boards/bsp_can.c 

OBJS += \
./boards/bsp_can.o 

C_DEPS += \
./boards/bsp_can.d 


# Each subdirectory must supply rules for building sources it contributes
boards/bsp_can.o: D:/GitRepo/balanceCopter-STM32/balanceCopter-STM32/boards/bsp_can.c boards/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/GitRepo/balanceCopter-STM32/balanceCopter-STM32/application" -I"D:/GitRepo/balanceCopter-STM32/balanceCopter-STM32/boards" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-boards

clean-boards:
	-$(RM) ./boards/bsp_can.cyclo ./boards/bsp_can.d ./boards/bsp_can.o ./boards/bsp_can.su

.PHONY: clean-boards

