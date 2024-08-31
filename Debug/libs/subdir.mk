################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libs/DEV_Config.c \
../libs/GUI_Paint.c \
../libs/LCD_1in47.c \
../libs/LCD_1inch47_test.c \
../libs/LCD_2inch4.c \
../libs/LCD_2inch4_test.c \
../libs/LCD_2inch_test.c \
../libs/font12.c \
../libs/font12CN.c \
../libs/font16.c \
../libs/font20.c \
../libs/font24.c \
../libs/font24CN.c \
../libs/font8.c \
../libs/image.c 

OBJS += \
./libs/DEV_Config.o \
./libs/GUI_Paint.o \
./libs/LCD_1in47.o \
./libs/LCD_1inch47_test.o \
./libs/LCD_2inch4.o \
./libs/LCD_2inch4_test.o \
./libs/LCD_2inch_test.o \
./libs/font12.o \
./libs/font12CN.o \
./libs/font16.o \
./libs/font20.o \
./libs/font24.o \
./libs/font24CN.o \
./libs/font8.o \
./libs/image.o 

C_DEPS += \
./libs/DEV_Config.d \
./libs/GUI_Paint.d \
./libs/LCD_1in47.d \
./libs/LCD_1inch47_test.d \
./libs/LCD_2inch4.d \
./libs/LCD_2inch4_test.d \
./libs/LCD_2inch_test.d \
./libs/font12.d \
./libs/font12CN.d \
./libs/font16.d \
./libs/font20.d \
./libs/font24.d \
./libs/font24CN.d \
./libs/font8.d \
./libs/image.d 


# Each subdirectory must supply rules for building sources it contributes
libs/%.o libs/%.su libs/%.cyclo: ../libs/%.c libs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/denys/STM32CubeIDE/MyPrivateWorkspace/STM32_NUG431KB_Oximeter/libs" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-libs

clean-libs:
	-$(RM) ./libs/DEV_Config.cyclo ./libs/DEV_Config.d ./libs/DEV_Config.o ./libs/DEV_Config.su ./libs/GUI_Paint.cyclo ./libs/GUI_Paint.d ./libs/GUI_Paint.o ./libs/GUI_Paint.su ./libs/LCD_1in47.cyclo ./libs/LCD_1in47.d ./libs/LCD_1in47.o ./libs/LCD_1in47.su ./libs/LCD_1inch47_test.cyclo ./libs/LCD_1inch47_test.d ./libs/LCD_1inch47_test.o ./libs/LCD_1inch47_test.su ./libs/LCD_2inch4.cyclo ./libs/LCD_2inch4.d ./libs/LCD_2inch4.o ./libs/LCD_2inch4.su ./libs/LCD_2inch4_test.cyclo ./libs/LCD_2inch4_test.d ./libs/LCD_2inch4_test.o ./libs/LCD_2inch4_test.su ./libs/LCD_2inch_test.cyclo ./libs/LCD_2inch_test.d ./libs/LCD_2inch_test.o ./libs/LCD_2inch_test.su ./libs/font12.cyclo ./libs/font12.d ./libs/font12.o ./libs/font12.su ./libs/font12CN.cyclo ./libs/font12CN.d ./libs/font12CN.o ./libs/font12CN.su ./libs/font16.cyclo ./libs/font16.d ./libs/font16.o ./libs/font16.su ./libs/font20.cyclo ./libs/font20.d ./libs/font20.o ./libs/font20.su ./libs/font24.cyclo ./libs/font24.d ./libs/font24.o ./libs/font24.su ./libs/font24CN.cyclo ./libs/font24CN.d ./libs/font24CN.o ./libs/font24CN.su ./libs/font8.cyclo ./libs/font8.d ./libs/font8.o ./libs/font8.su ./libs/image.cyclo ./libs/image.d ./libs/image.o ./libs/image.su

.PHONY: clean-libs

