################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LC76G/LC76G.c \
../Drivers/LC76G/gps.c 

OBJS += \
./Drivers/LC76G/LC76G.o \
./Drivers/LC76G/gps.o 

C_DEPS += \
./Drivers/LC76G/LC76G.d \
./Drivers/LC76G/gps.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LC76G/%.o Drivers/LC76G/%.su Drivers/LC76G/%.cyclo: ../Drivers/LC76G/%.c Drivers/LC76G/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../USB_Device/App -I../USB_Device/Target -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -I"/Users/bsibb22/Documents/CanSat2025_FSW-NO_RTOS/CanSat2025_FSW/Drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-LC76G

clean-Drivers-2f-LC76G:
	-$(RM) ./Drivers/LC76G/LC76G.cyclo ./Drivers/LC76G/LC76G.d ./Drivers/LC76G/LC76G.o ./Drivers/LC76G/LC76G.su ./Drivers/LC76G/gps.cyclo ./Drivers/LC76G/gps.d ./Drivers/LC76G/gps.o ./Drivers/LC76G/gps.su

.PHONY: clean-Drivers-2f-LC76G

