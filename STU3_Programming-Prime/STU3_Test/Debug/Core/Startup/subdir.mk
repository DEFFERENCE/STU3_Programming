################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/BasicMathFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/BayesFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/CommonTables" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/ComplexMathFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/ControllerFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/DistanceFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/FastMathFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/FilteringFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/InterpolationFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/MatrixFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/QuaternionMathFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/StatisticsFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/SupportFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/SVMFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/TransformFunctions" -I"D:/Fibo/Year2/Term2/STU/Programming/STU3_Programming/STU3_Programming-Prime/STU3_Test/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

