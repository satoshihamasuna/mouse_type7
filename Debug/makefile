################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/ntshell-v0.3.1/src/lib/util/subdir.mk
-include Drivers/ntshell-v0.3.1/src/lib/core/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/CPP/Task/Src/subdir.mk
-include Core/CPP/System/Src/subdir.mk
-include Core/CPP/Subsys/Src/subdir.mk
-include Core/CPP/Pheripheral/Src/subdir.mk
-include Core/CPP/Module/Src/subdir.mk
-include Core/CPP/Component/codegen/subdir.mk
-include Core/CPP/Component/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
mouse_type7.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
mouse_type7.list \

OBJCOPY_BIN += \
mouse_type7.bin \


# All Target
all: mouse_type7.elf secondary-outputs

# Tool invocations
mouse_type7.elf: $(OBJS) $(USER_OBJS) C:\Users\sato1\Documents\git\mouse_type7_program\mouse_type7\STM32F411CEUX_FLASH.ld
	arm-none-eabi-g++ -o "mouse_type7.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\sato1\Documents\git\mouse_type7_program\mouse_type7\STM32F411CEUX_FLASH.ld" -Wl,-Map="mouse_type7.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

mouse_type7.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "mouse_type7.list"
	@echo 'Finished building: $@'
	@echo ' '

mouse_type7.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "mouse_type7.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
