################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Based_System_Communication.c \
../Core/Src/Encoder.c \
../Core/Src/Kalman_Filter.c \
../Core/Src/ModBusRTU.c \
../Core/Src/Prismatic.c \
../Core/Src/Revolute.c \
../Core/Src/Trajectory.c \
../Core/Src/main.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

OBJS += \
./Core/Src/Based_System_Communication.o \
./Core/Src/Encoder.o \
./Core/Src/Kalman_Filter.o \
./Core/Src/ModBusRTU.o \
./Core/Src/Prismatic.o \
./Core/Src/Revolute.o \
./Core/Src/Trajectory.o \
./Core/Src/main.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

C_DEPS += \
./Core/Src/Based_System_Communication.d \
./Core/Src/Encoder.d \
./Core/Src/Kalman_Filter.d \
./Core/Src/ModBusRTU.d \
./Core/Src/Prismatic.d \
./Core/Src/Revolute.d \
./Core/Src/Trajectory.d \
./Core/Src/main.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/BasicMathFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/BayesFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/CommonTables" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/ComplexMathFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/ControllerFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/DistanceFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/FastMathFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/FilteringFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/InterpolationFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/MatrixFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/QuaternionMathFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/StatisticsFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/SupportFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/SVMFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/TransformFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Based_System_Communication.cyclo ./Core/Src/Based_System_Communication.d ./Core/Src/Based_System_Communication.o ./Core/Src/Based_System_Communication.su ./Core/Src/Encoder.cyclo ./Core/Src/Encoder.d ./Core/Src/Encoder.o ./Core/Src/Encoder.su ./Core/Src/Kalman_Filter.cyclo ./Core/Src/Kalman_Filter.d ./Core/Src/Kalman_Filter.o ./Core/Src/Kalman_Filter.su ./Core/Src/ModBusRTU.cyclo ./Core/Src/ModBusRTU.d ./Core/Src/ModBusRTU.o ./Core/Src/ModBusRTU.su ./Core/Src/Prismatic.cyclo ./Core/Src/Prismatic.d ./Core/Src/Prismatic.o ./Core/Src/Prismatic.su ./Core/Src/Revolute.cyclo ./Core/Src/Revolute.d ./Core/Src/Revolute.o ./Core/Src/Revolute.su ./Core/Src/Trajectory.cyclo ./Core/Src/Trajectory.d ./Core/Src/Trajectory.o ./Core/Src/Trajectory.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

