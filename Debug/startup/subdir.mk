################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f405xx.s 

OBJS += \
./startup/startup_stm32f405xx.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	arm-none-eabi-gcc -c -mcpu=cortex-m4 -g3 -c -I/home/igor/Documents/CubeMXprojects/STM32F405VGFULL/ros_lib -I/home/igor/Documents/CubeMXprojects/STM32F405VGFULL/usr -Wa,-W -x assembler-with-cpp --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

