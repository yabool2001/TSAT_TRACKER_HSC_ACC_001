################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../my_drivers/Src/lis2dw12_reg.c \
../my_drivers/Src/my_lis2dw12.c 

OBJS += \
./my_drivers/Src/lis2dw12_reg.o \
./my_drivers/Src/my_lis2dw12.o 

C_DEPS += \
./my_drivers/Src/lis2dw12_reg.d \
./my_drivers/Src/my_lis2dw12.d 


# Each subdirectory must supply rules for building sources it contributes
my_drivers/Src/%.o my_drivers/Src/%.su my_drivers/Src/%.cyclo: ../my_drivers/Src/%.c my_drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/mzeml/embedded/TSAT_TRACKER_HSC_ACC_001/my_drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-my_drivers-2f-Src

clean-my_drivers-2f-Src:
	-$(RM) ./my_drivers/Src/lis2dw12_reg.cyclo ./my_drivers/Src/lis2dw12_reg.d ./my_drivers/Src/lis2dw12_reg.o ./my_drivers/Src/lis2dw12_reg.su ./my_drivers/Src/my_lis2dw12.cyclo ./my_drivers/Src/my_lis2dw12.d ./my_drivers/Src/my_lis2dw12.o ./my_drivers/Src/my_lis2dw12.su

.PHONY: clean-my_drivers-2f-Src

