################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Fonts/font12.c \
../Fonts/font16.c \
../Fonts/font20.c \
../Fonts/font24.c \
../Fonts/font8.c 

OBJS += \
./Fonts/font12.o \
./Fonts/font16.o \
./Fonts/font20.o \
./Fonts/font24.o \
./Fonts/font8.o 

C_DEPS += \
./Fonts/font12.d \
./Fonts/font16.d \
./Fonts/font20.d \
./Fonts/font24.d \
./Fonts/font8.d 


# Each subdirectory must supply rules for building sources it contributes
Fonts/%.o: ../Fonts/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xB -I"/home/hejohnson/Code/OnBoatDisplay/Inc" -I"/home/hejohnson/Code/OnBoatDisplay/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/hejohnson/Code/OnBoatDisplay/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/hejohnson/Code/OnBoatDisplay/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/hejohnson/Code/OnBoatDisplay/Drivers/CMSIS/Include" -I"/home/hejohnson/Code/OnBoatDisplay/BSP" -I"/home/hejohnson/Code/OnBoatDisplay/Fonts"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


