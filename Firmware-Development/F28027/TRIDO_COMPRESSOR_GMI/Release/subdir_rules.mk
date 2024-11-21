################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt -O3 --include_path="C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --include_path="F:/ESP32 Project/workspace_trido_v12/TRIDO_COMPRESSOR_GMI/modbus/include" --include_path="F:/ESP32 Project/workspace_trido_v12/TRIDO_COMPRESSOR_GMI/include" --advice:performance=all --define=_INLINE --define=F2802xF --define=FLASH --define=FAST_ROM_V1p7 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt -O3 --include_path="C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --include_path="F:/ESP32 Project/workspace_trido_v12/TRIDO_COMPRESSOR_GMI/modbus/include" --include_path="F:/ESP32 Project/workspace_trido_v12/TRIDO_COMPRESSOR_GMI/include" --advice:performance=all --define=_INLINE --define=F2802xF --define=FLASH --define=FAST_ROM_V1p7 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


