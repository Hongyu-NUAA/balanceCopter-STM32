################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../application/CAN_receive.c \
../application/pid.c 

OBJS += \
./application/CAN_receive.o \
./application/pid.o 

C_DEPS += \
./application/CAN_receive.d \
./application/pid.d 


# Each subdirectory must supply rules for building sources it contributes
application/CAN_receive.o: D:/GitRepo/balanceCopter-STM32/balanceCopter-STM32/application/CAN_receive.c application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/GitRepo/balanceCopter-STM32/balanceCopter-STM32/application" -I"D:/GitRepo/balanceCopter-STM32/balanceCopter-STM32/boards" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
application/pid.o: D:/GitRepo/balanceCopter-STM32/balanceCopter-STM32/application/pid.c application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/GitRepo/balanceCopter-STM32/balanceCopter-STM32/application" -I"D:/GitRepo/balanceCopter-STM32/balanceCopter-STM32/boards" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-application

clean-application:
	-$(RM) ./application/CAN_receive.cyclo ./application/CAN_receive.d ./application/CAN_receive.o ./application/CAN_receive.su ./application/pid.cyclo ./application/pid.d ./application/pid.o ./application/pid.su

.PHONY: clean-application

