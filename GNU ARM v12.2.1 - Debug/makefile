################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include gecko_sdk_4.4.4/platform/service/system/src/subdir.mk
-include gecko_sdk_4.4.4/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_4.4.4/platform/service/iostream/src/subdir.mk
-include gecko_sdk_4.4.4/platform/service/device_init/src/subdir.mk
-include gecko_sdk_4.4.4/platform/emlib/src/subdir.mk
-include gecko_sdk_4.4.4/platform/emdrv/dmadrv/src/subdir.mk
-include gecko_sdk_4.4.4/platform/driver/debug/src/subdir.mk
-include gecko_sdk_4.4.4/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_4.4.4/platform/common/src/subdir.mk
-include gecko_sdk_4.4.4/platform/Device/SiliconLabs/BGM22/Source/subdir.mk
-include gecko_sdk_4.4.4/hardware/driver/configuration_over_swo/src/subdir.mk
-include gecko_sdk_4.4.4/hardware/board/src/subdir.mk
-include gecko_sdk_4.4.4/app/common/util/app_log/subdir.mk
-include autogen/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := hydro_proj
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: main-build

# Main-build Target
main-build: hydro_proj.axf

# Tool invocations
hydro_proj.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "C:\Users\Balazs\SimplicityStudio\v5_workspace\hydro_proj\autogen\linkerfile.ld" -Wl,--no-warn-rwx-segments -Wl,-u,_printf_float -Xlinker --gc-sections -Xlinker -Map="hydro_proj.map" -mfpu=fpv5-sp-d16 -mfloat-abi=hard --specs=nano.specs -o hydro_proj.axf -Wl,--start-group "./app.o" "./control.o" "./main.o" "./autogen/sl_board_default_init.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_event_handler.o" "./autogen/sl_iostream_handles.o" "./autogen/sl_iostream_init_eusart_instances.o" "./gecko_sdk_4.4.4/app/common/util/app_log/app_log.o" "./gecko_sdk_4.4.4/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_4.4.4/hardware/board/src/sl_board_init.o" "./gecko_sdk_4.4.4/hardware/driver/configuration_over_swo/src/sl_cos.o" "./gecko_sdk_4.4.4/platform/Device/SiliconLabs/BGM22/Source/startup_bgm22.o" "./gecko_sdk_4.4.4/platform/Device/SiliconLabs/BGM22/Source/system_bgm22.o" "./gecko_sdk_4.4.4/platform/common/src/sl_assert.o" "./gecko_sdk_4.4.4/platform/common/src/sl_slist.o" "./gecko_sdk_4.4.4/platform/common/src/sl_string.o" "./gecko_sdk_4.4.4/platform/common/src/sl_syscalls.o" "./gecko_sdk_4.4.4/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_4.4.4/platform/driver/debug/src/sl_debug_swo.o" "./gecko_sdk_4.4.4/platform/emdrv/dmadrv/src/dmadrv.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_burtc.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_cmu.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_core.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_emu.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_eusart.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_gpio.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_ldma.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_msc.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_rtcc.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_system.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_timer.o" "./gecko_sdk_4.4.4/platform/service/device_init/src/sl_device_init_dcdc_s2.o" "./gecko_sdk_4.4.4/platform/service/device_init/src/sl_device_init_emu_s2.o" "./gecko_sdk_4.4.4/platform/service/device_init/src/sl_device_init_hfxo_s2.o" "./gecko_sdk_4.4.4/platform/service/device_init/src/sl_device_init_lfxo_s2.o" "./gecko_sdk_4.4.4/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_4.4.4/platform/service/iostream/src/sl_iostream.o" "./gecko_sdk_4.4.4/platform/service/iostream/src/sl_iostream_eusart.o" "./gecko_sdk_4.4.4/platform/service/iostream/src/sl_iostream_retarget_stdio.o" "./gecko_sdk_4.4.4/platform/service/iostream/src/sl_iostream_stdlib_config.o" "./gecko_sdk_4.4.4/platform/service/iostream/src/sl_iostream_uart.o" "./gecko_sdk_4.4.4/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_4.4.4/platform/service/sleeptimer/src/sl_sleeptimer_hal_burtc.o" "./gecko_sdk_4.4.4/platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.o" "./gecko_sdk_4.4.4/platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "./gecko_sdk_4.4.4/platform/service/sleeptimer/src/sl_sleeptimer_hal_timer.o" "./gecko_sdk_4.4.4/platform/service/system/src/sl_system_init.o" "./gecko_sdk_4.4.4/platform/service/system/src/sl_system_process_action.o" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building out file: hydro_proj.out'
	arm-none-eabi-objcopy "hydro_proj.axf" "hydro_proj.out"
	@echo ' '

	@echo 'Building hex file: hydro_proj.hex'
	arm-none-eabi-objcopy -O ihex "hydro_proj.axf" "hydro_proj.hex"
	@echo ' '

	@echo 'Building bin file: hydro_proj.bin'
	arm-none-eabi-objcopy -O binary "hydro_proj.axf" "hydro_proj.bin"
	@echo ' '

	@echo 'Building s37 file: hydro_proj.s37'
	arm-none-eabi-objcopy -O srec "hydro_proj.axf" "hydro_proj.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "hydro_proj.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) hydro_proj.axf
	-@echo ' '

.PHONY: all clean dependents main-build

-include ../makefile.targets
