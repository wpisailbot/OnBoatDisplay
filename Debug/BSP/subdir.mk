################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/epd2in9b.c \
../BSP/epdif.c \
../BSP/epdpaint.c \
../BSP/imagedata.c 

OBJS += \
./BSP/epd2in9b.o \
./BSP/epdif.o \
./BSP/epdpaint.o \
./BSP/imagedata.o 

C_DEPS += \
./BSP/epd2in9b.d \
./BSP/epdif.d \
./BSP/epdpaint.d \
./BSP/imagedata.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o: ../BSP/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xB -I"/home/hejohnson/Code/OnBoatDisplay/Inc" -I"/home/hejohnson/Code/OnBoatDisplay/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/hejohnson/Code/OnBoatDisplay/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/hejohnson/Code/OnBoatDisplay/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/hejohnson/Code/OnBoatDisplay/Drivers/CMSIS/Include" -I"/home/hejohnson/Code/OnBoatDisplay/BSP" -I"/home/hejohnson/Code/OnBoatDisplay/Fonts"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


