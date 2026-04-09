################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_activate.c \
../Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_control_request.c \
../Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_deactivate.c \
../Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_entry.c \
../Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_initialize.c \
../Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_ioctl.c \
../Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_read_run.c \
../Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_tasks_run.c \
../Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_unitialize.c \
../Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_run.c \
../Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_with_callback.c 

OBJS += \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_activate.o \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_control_request.o \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_deactivate.o \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_entry.o \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_initialize.o \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_ioctl.o \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_read_run.o \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_tasks_run.o \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_unitialize.o \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_run.o \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_with_callback.o 

C_DEPS += \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_activate.d \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_control_request.d \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_deactivate.d \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_entry.d \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_initialize.d \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_ioctl.d \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_read_run.d \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_tasks_run.d \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_unitialize.d \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_run.d \
./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_with_callback.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/usbx/common/usbx_device_classes/src/%.o Middlewares/ST/usbx/common/usbx_device_classes/src/%.su Middlewares/ST/usbx/common/usbx_device_classes/src/%.cyclo: ../Middlewares/ST/usbx/common/usbx_device_classes/src/%.c Middlewares/ST/usbx/common/usbx_device_classes/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -DUX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../USBX/App -I../USBX/Target -I../Middlewares/ST/usbx/common/core/inc -I../Middlewares/ST/usbx/ports/generic/inc -I../Middlewares/ST/usbx/common/usbx_stm32_device_controllers -I../Middlewares/ST/usbx/common/usbx_device_classes/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-usbx-2f-common-2f-usbx_device_classes-2f-src

clean-Middlewares-2f-ST-2f-usbx-2f-common-2f-usbx_device_classes-2f-src:
	-$(RM) ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_activate.cyclo ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_activate.d ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_activate.o ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_activate.su ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_control_request.cyclo ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_control_request.d ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_control_request.o ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_control_request.su ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_deactivate.cyclo ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_deactivate.d ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_deactivate.o ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_deactivate.su ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_entry.cyclo ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_entry.d ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_entry.o ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_entry.su ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_initialize.cyclo ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_initialize.d ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_initialize.o ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_initialize.su ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_ioctl.cyclo ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_ioctl.d ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_ioctl.o ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_ioctl.su ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_read_run.cyclo ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_read_run.d ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_read_run.o ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_read_run.su ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_tasks_run.cyclo ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_tasks_run.d ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_tasks_run.o ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_tasks_run.su ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_unitialize.cyclo ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_unitialize.d ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_unitialize.o ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_unitialize.su ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_run.cyclo ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_run.d ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_run.o ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_run.su ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_with_callback.cyclo ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_with_callback.d ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_with_callback.o ./Middlewares/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_acm_write_with_callback.su

.PHONY: clean-Middlewares-2f-ST-2f-usbx-2f-common-2f-usbx_device_classes-2f-src

