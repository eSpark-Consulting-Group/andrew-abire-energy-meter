################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
-lrts2800_ml.lib \
"../2802xRevB_Fast_IQ_ROMSymbols.lib" \
"../2802xRevB_Fast_ONLY_ROMSymbols.lib" \
"../2802xRevB_Fast_RTS_ROMSymbols.lib" \
"../IQmath.lib" \
"../fast_public.lib" \
"./CodeStartBranch.obj" \
"./adc.obj" \
"./clarke.obj" \
"./clk.obj" \
"./comp.obj" \
"./cpu.obj" \
"./ctrl.obj" \
"./filter_fo.obj" \
"./flash.obj" \
"./fw.obj" \
"./gpio.obj" \
"./hal.obj" \
"./ipark.obj" \
"./main.obj" \
"./memCopy.obj" \
"./offset.obj" \
"./osc.obj" \
"./park.obj" \
"./pid.obj" \
"./pie.obj" \
"./pll.obj" \
"./pwm.obj" \
"./pwr.obj" \
"./sci.obj" \
"./spi.obj" \
"./svgen.obj" \
"./svgen_current.obj" \
"./timer.obj" \
"./traj.obj" \
"./usDelay.obj" \
"./user.obj" \
"./wdog.obj" \
"./modbus/source/modbus_slave.obj" \
"./modbus/source/modbus_user.obj" \
"./modbus/source/scidrv.obj" \
"./modbus/source/userio.obj" \
"../F28027F.cmd" \
$(GEN_CMDS__FLAG) \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include modbus/source/subdir_vars.mk
-include subdir_rules.mk
-include modbus/source/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
TRIDO_COMPRESSOR_GMI.out \

EXE_OUTPUTS__QUOTED += \
"TRIDO_COMPRESSOR_GMI.out" \

BIN_OUTPUTS += \
TRIDO_COMPRESSOR_GMI.hex \

BIN_OUTPUTS__QUOTED += \
"TRIDO_COMPRESSOR_GMI.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "TRIDO_COMPRESSOR_GMI.out"

# Tool invocations
TRIDO_COMPRESSOR_GMI.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt -O3 --advice:performance=all --define=_INLINE --define=F2802xF --define=FLASH --define=FAST_ROM_V1p7 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi -z -m"TRIDO_COMPRESSOR_GMI.map" --stack_size=0x180 --warn_sections -i"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/lib" -i"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --reread_libs --diag_suppress=16002 --diag_suppress=10063-D --diag_wrap=off --display_error_number --xml_link_info="TRIDO_COMPRESSOR_GMI_linkInfo.xml" --entry_point=code_start --rom_model -o "TRIDO_COMPRESSOR_GMI.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

TRIDO_COMPRESSOR_GMI.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: C2000 Hex Utility'
	"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/hex2000" --diag_wrap=off -o "TRIDO_COMPRESSOR_GMI.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "CodeStartBranch.obj" "adc.obj" "clarke.obj" "clk.obj" "comp.obj" "cpu.obj" "ctrl.obj" "filter_fo.obj" "flash.obj" "fw.obj" "gpio.obj" "hal.obj" "ipark.obj" "main.obj" "memCopy.obj" "offset.obj" "osc.obj" "park.obj" "pid.obj" "pie.obj" "pll.obj" "pwm.obj" "pwr.obj" "sci.obj" "spi.obj" "svgen.obj" "svgen_current.obj" "timer.obj" "traj.obj" "usDelay.obj" "user.obj" "wdog.obj" "modbus\source\modbus_slave.obj" "modbus\source\modbus_user.obj" "modbus\source\scidrv.obj" "modbus\source\userio.obj" 
	-$(RM) "adc.d" "clarke.d" "clk.d" "comp.d" "cpu.d" "ctrl.d" "filter_fo.d" "flash.d" "fw.d" "gpio.d" "hal.d" "ipark.d" "main.d" "memCopy.d" "offset.d" "osc.d" "park.d" "pid.d" "pie.d" "pll.d" "pwm.d" "pwr.d" "sci.d" "spi.d" "svgen.d" "svgen_current.d" "timer.d" "traj.d" "user.d" "wdog.d" "modbus\source\modbus_slave.d" "modbus\source\modbus_user.d" "modbus\source\scidrv.d" "modbus\source\userio.d" 
	-$(RM) "CodeStartBranch.d" "usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

