################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver_Layer/UART/my_uart.c 

OBJS += \
./Driver_Layer/UART/my_uart.o 

C_DEPS += \
./Driver_Layer/UART/my_uart.d 


# Each subdirectory must supply rules for building sources it contributes
Driver_Layer/UART/%.o Driver_Layer/UART/%.su Driver_Layer/UART/%.cyclo: ../Driver_Layer/UART/%.c Driver_Layer/UART/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/dangm/OneDrive/Documents/STM32/20250702_04_RMS/04_RMS/02_Event-Triggered Scheduling_Done/01_Simple_Period/Driver_Layer" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Driver_Layer-2f-UART

clean-Driver_Layer-2f-UART:
	-$(RM) ./Driver_Layer/UART/my_uart.cyclo ./Driver_Layer/UART/my_uart.d ./Driver_Layer/UART/my_uart.o ./Driver_Layer/UART/my_uart.su

.PHONY: clean-Driver_Layer-2f-UART

