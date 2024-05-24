################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/CPP/Component/Src/controll.cpp \
../Core/CPP/Component/Src/half_float.cpp \
../Core/CPP/Component/Src/kalman_filter.cpp \
../Core/CPP/Component/Src/path_follow.cpp 

OBJS += \
./Core/CPP/Component/Src/controll.o \
./Core/CPP/Component/Src/half_float.o \
./Core/CPP/Component/Src/kalman_filter.o \
./Core/CPP/Component/Src/path_follow.o 

CPP_DEPS += \
./Core/CPP/Component/Src/controll.d \
./Core/CPP/Component/Src/half_float.d \
./Core/CPP/Component/Src/kalman_filter.d \
./Core/CPP/Component/Src/path_follow.d 


# Each subdirectory must supply rules for building sources it contributes
Core/CPP/Component/Src/controll.o: ../Core/CPP/Component/Src/controll.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/sato1/Documents/git/mouse_type7_program/mouse_type7/Drivers/ntshell-v0.3.1/src/lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Component/Src/controll.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/CPP/Component/Src/half_float.o: ../Core/CPP/Component/Src/half_float.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/sato1/Documents/git/mouse_type7_program/mouse_type7/Drivers/ntshell-v0.3.1/src/lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Component/Src/half_float.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/CPP/Component/Src/kalman_filter.o: ../Core/CPP/Component/Src/kalman_filter.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/sato1/Documents/git/mouse_type7_program/mouse_type7/Drivers/ntshell-v0.3.1/src/lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Component/Src/kalman_filter.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/CPP/Component/Src/path_follow.o: ../Core/CPP/Component/Src/path_follow.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/sato1/Documents/git/mouse_type7_program/mouse_type7/Drivers/ntshell-v0.3.1/src/lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Component/Src/path_follow.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

