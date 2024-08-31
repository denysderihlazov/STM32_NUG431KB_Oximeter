################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/libs/DEV_Config.c \
../Core/libs/GUI_Paint.c \
../Core/libs/LCD_1in47.c \
../Core/libs/LCD_1inch47_test.c \
../Core/libs/LCD_2inch4.c \
../Core/libs/LCD_2inch4_test.c \
../Core/libs/LCD_2inch_test.c \
../Core/libs/font12.c \
../Core/libs/font12CN.c \
../Core/libs/font16.c \
../Core/libs/font20.c \
../Core/libs/font24.c \
../Core/libs/font24CN.c \
../Core/libs/font8.c \
../Core/libs/image.c 

OBJS += \
./Core/libs/DEV_Config.o \
./Core/libs/GUI_Paint.o \
./Core/libs/LCD_1in47.o \
./Core/libs/LCD_1inch47_test.o \
./Core/libs/LCD_2inch4.o \
./Core/libs/LCD_2inch4_test.o \
./Core/libs/LCD_2inch_test.o \
./Core/libs/font12.o \
./Core/libs/font12CN.o \
./Core/libs/font16.o \
./Core/libs/font20.o \
./Core/libs/font24.o \
./Core/libs/font24CN.o \
./Core/libs/font8.o \
./Core/libs/image.o 

C_DEPS += \
./Core/libs/DEV_Config.d \
./Core/libs/GUI_Paint.d \
./Core/libs/LCD_1in47.d \
./Core/libs/LCD_1inch47_test.d \
./Core/libs/LCD_2inch4.d \
./Core/libs/LCD_2inch4_test.d \
./Core/libs/LCD_2inch_test.d \
./Core/libs/font12.d \
./Core/libs/font12CN.d \
./Core/libs/font16.d \
./Core/libs/font20.d \
./Core/libs/font24.d \
./Core/libs/font24CN.d \
./Core/libs/font8.d \
./Core/libs/image.d 


# Each subdirectory must supply rules for building sources it contributes
Core/libs/%.o Core/libs/%.su Core/libs/%.cyclo: ../Core/libs/%.c Core/libs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-libs

clean-Core-2f-libs:
	-$(RM) ./Core/libs/DEV_Config.cyclo ./Core/libs/DEV_Config.d ./Core/libs/DEV_Config.o ./Core/libs/DEV_Config.su ./Core/libs/GUI_Paint.cyclo ./Core/libs/GUI_Paint.d ./Core/libs/GUI_Paint.o ./Core/libs/GUI_Paint.su ./Core/libs/LCD_1in47.cyclo ./Core/libs/LCD_1in47.d ./Core/libs/LCD_1in47.o ./Core/libs/LCD_1in47.su ./Core/libs/LCD_1inch47_test.cyclo ./Core/libs/LCD_1inch47_test.d ./Core/libs/LCD_1inch47_test.o ./Core/libs/LCD_1inch47_test.su ./Core/libs/LCD_2inch4.cyclo ./Core/libs/LCD_2inch4.d ./Core/libs/LCD_2inch4.o ./Core/libs/LCD_2inch4.su ./Core/libs/LCD_2inch4_test.cyclo ./Core/libs/LCD_2inch4_test.d ./Core/libs/LCD_2inch4_test.o ./Core/libs/LCD_2inch4_test.su ./Core/libs/LCD_2inch_test.cyclo ./Core/libs/LCD_2inch_test.d ./Core/libs/LCD_2inch_test.o ./Core/libs/LCD_2inch_test.su ./Core/libs/font12.cyclo ./Core/libs/font12.d ./Core/libs/font12.o ./Core/libs/font12.su ./Core/libs/font12CN.cyclo ./Core/libs/font12CN.d ./Core/libs/font12CN.o ./Core/libs/font12CN.su ./Core/libs/font16.cyclo ./Core/libs/font16.d ./Core/libs/font16.o ./Core/libs/font16.su ./Core/libs/font20.cyclo ./Core/libs/font20.d ./Core/libs/font20.o ./Core/libs/font20.su ./Core/libs/font24.cyclo ./Core/libs/font24.d ./Core/libs/font24.o ./Core/libs/font24.su ./Core/libs/font24CN.cyclo ./Core/libs/font24CN.d ./Core/libs/font24CN.o ./Core/libs/font24CN.su ./Core/libs/font8.cyclo ./Core/libs/font8.d ./Core/libs/font8.o ./Core/libs/font8.su ./Core/libs/image.cyclo ./Core/libs/image.d ./Core/libs/image.o ./Core/libs/image.su

.PHONY: clean-Core-2f-libs

