################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/startup_efr32bg13p.c 

OBJS += \
./platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/startup_efr32bg13p.o 

C_DEPS += \
./platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/startup_efr32bg13p.d 


# Each subdirectory must supply rules for building sources it contributes
platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/startup_efr32bg13p.o: ../platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/startup_efr32bg13p.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DDEBUG=1' -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/startup_efr32bg13p.d" -MT"platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/startup_efr32bg13p.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


