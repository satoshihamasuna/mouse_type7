################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/CPP/Pheripheral/Src/battery.c \
../Core/CPP/Pheripheral/Src/encoder.c \
../Core/CPP/Pheripheral/Src/imu.c \
../Core/CPP/Pheripheral/Src/indicator.c \
../Core/CPP/Pheripheral/Src/interface.c \
../Core/CPP/Pheripheral/Src/ir_sensor.c \
../Core/CPP/Pheripheral/Src/motor.c 

OBJS += \
./Core/CPP/Pheripheral/Src/battery.o \
./Core/CPP/Pheripheral/Src/encoder.o \
./Core/CPP/Pheripheral/Src/imu.o \
./Core/CPP/Pheripheral/Src/indicator.o \
./Core/CPP/Pheripheral/Src/interface.o \
./Core/CPP/Pheripheral/Src/ir_sensor.o \
./Core/CPP/Pheripheral/Src/motor.o 

C_DEPS += \
./Core/CPP/Pheripheral/Src/battery.d \
./Core/CPP/Pheripheral/Src/encoder.d \
./Core/CPP/Pheripheral/Src/imu.d \
./Core/CPP/Pheripheral/Src/indicator.d \
./Core/CPP/Pheripheral/Src/interface.d \
./Core/CPP/Pheripheral/Src/ir_sensor.d \
./Core/CPP/Pheripheral/Src/motor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/CPP/Pheripheral/Src/battery.o: ../Core/CPP/Pheripheral/Src/battery.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Pheripheral/Src/battery.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/CPP/Pheripheral/Src/encoder.o: ../Core/CPP/Pheripheral/Src/encoder.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Pheripheral/Src/encoder.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/CPP/Pheripheral/Src/imu.o: ../Core/CPP/Pheripheral/Src/imu.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Pheripheral/Src/imu.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/CPP/Pheripheral/Src/indicator.o: ../Core/CPP/Pheripheral/Src/indicator.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Pheripheral/Src/indicator.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/CPP/Pheripheral/Src/interface.o: ../Core/CPP/Pheripheral/Src/interface.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Pheripheral/Src/interface.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/CPP/Pheripheral/Src/ir_sensor.o: ../Core/CPP/Pheripheral/Src/ir_sensor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Pheripheral/Src/ir_sensor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/CPP/Pheripheral/Src/motor.o: ../Core/CPP/Pheripheral/Src/motor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Pheripheral/Src/motor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

