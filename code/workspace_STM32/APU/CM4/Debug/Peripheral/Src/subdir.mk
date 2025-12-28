################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripheral/Src/BNO08x.c \
../Peripheral/Src/MPL3115A2.c \
../Peripheral/Src/SHT31.c \
../Peripheral/Src/fatfs_sd.c 

OBJS += \
./Peripheral/Src/BNO08x.o \
./Peripheral/Src/MPL3115A2.o \
./Peripheral/Src/SHT31.o \
./Peripheral/Src/fatfs_sd.o 

C_DEPS += \
./Peripheral/Src/BNO08x.d \
./Peripheral/Src/MPL3115A2.d \
./Peripheral/Src/SHT31.d \
./Peripheral/Src/fatfs_sd.d 


# Each subdirectory must supply rules for building sources it contributes
Peripheral/Src/%.o Peripheral/Src/%.su Peripheral/Src/%.cyclo: ../Peripheral/Src/%.c Peripheral/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"E:/Projects STM32/workspace_1.17.0/APU/CM4/Peripheral/Inc" -I../FATFS/Target -I../FATFS/App -I../../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Peripheral-2f-Src

clean-Peripheral-2f-Src:
	-$(RM) ./Peripheral/Src/BNO08x.cyclo ./Peripheral/Src/BNO08x.d ./Peripheral/Src/BNO08x.o ./Peripheral/Src/BNO08x.su ./Peripheral/Src/MPL3115A2.cyclo ./Peripheral/Src/MPL3115A2.d ./Peripheral/Src/MPL3115A2.o ./Peripheral/Src/MPL3115A2.su ./Peripheral/Src/SHT31.cyclo ./Peripheral/Src/SHT31.d ./Peripheral/Src/SHT31.o ./Peripheral/Src/SHT31.su ./Peripheral/Src/fatfs_sd.cyclo ./Peripheral/Src/fatfs_sd.d ./Peripheral/Src/fatfs_sd.o ./Peripheral/Src/fatfs_sd.su

.PHONY: clean-Peripheral-2f-Src

