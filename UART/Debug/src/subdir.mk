################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/UART.c \
../src/cr_startup_lpc43xx.c \
../src/sysinit.c 

OBJS += \
./src/UART.o \
./src/cr_startup_lpc43xx.o \
./src/sysinit.o 

C_DEPS += \
./src/UART.d \
./src/cr_startup_lpc43xx.d \
./src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M4 -D__USE_LPCOPEN -D__LPC43XX__ -D__REDLIB__ -D__MULTICORE_NONE -I"C:\Users\Fabia\Desktop\Workspace\edu_ciaa_nxp\inc" -I"C:\Users\Fabia\Desktop\Workspace\lpc_chip_43xx\inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fsingle-precision-constant -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


