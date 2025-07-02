################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver_Layer/LCD/LCD.c 

OBJS += \
./Driver_Layer/LCD/LCD.o 

C_DEPS += \
./Driver_Layer/LCD/LCD.d 


# Each subdirectory must supply rules for building sources it contributes
Driver_Layer/LCD/%.o Driver_Layer/LCD/%.su Driver_Layer/LCD/%.cyclo: ../Driver_Layer/LCD/%.c Driver_Layer/LCD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/dangm/OneDrive/Documents/STM32/01_Simple_Period/Driver_Layer" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Driver_Layer-2f-LCD

clean-Driver_Layer-2f-LCD:
	-$(RM) ./Driver_Layer/LCD/LCD.cyclo ./Driver_Layer/LCD/LCD.d ./Driver_Layer/LCD/LCD.o ./Driver_Layer/LCD/LCD.su

.PHONY: clean-Driver_Layer-2f-LCD

