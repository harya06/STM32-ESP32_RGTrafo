################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MLX90640/Src/mlx90640_core.c \
../MLX90640/Src/mlx90640_driver.c \
../MLX90640/Src/mlx90640_i2c.c 

OBJS += \
./MLX90640/Src/mlx90640_core.o \
./MLX90640/Src/mlx90640_driver.o \
./MLX90640/Src/mlx90640_i2c.o 

C_DEPS += \
./MLX90640/Src/mlx90640_core.d \
./MLX90640/Src/mlx90640_driver.d \
./MLX90640/Src/mlx90640_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
MLX90640/Src/%.o MLX90640/Src/%.su MLX90640/Src/%.cyclo: ../MLX90640/Src/%.c MLX90640/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/Users/harya/STM32CubeIDE/workspace_1.19.0/STM-ESP-TRAFO/MIDWARE/FATFS_SD" -I"C:/Users/harya/STM32CubeIDE/workspace_1.19.0/STM-ESP-TRAFO/MLX90640/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MLX90640-2f-Src

clean-MLX90640-2f-Src:
	-$(RM) ./MLX90640/Src/mlx90640_core.cyclo ./MLX90640/Src/mlx90640_core.d ./MLX90640/Src/mlx90640_core.o ./MLX90640/Src/mlx90640_core.su ./MLX90640/Src/mlx90640_driver.cyclo ./MLX90640/Src/mlx90640_driver.d ./MLX90640/Src/mlx90640_driver.o ./MLX90640/Src/mlx90640_driver.su ./MLX90640/Src/mlx90640_i2c.cyclo ./MLX90640/Src/mlx90640_i2c.d ./MLX90640/Src/mlx90640_i2c.o ./MLX90640/Src/mlx90640_i2c.su

.PHONY: clean-MLX90640-2f-Src

