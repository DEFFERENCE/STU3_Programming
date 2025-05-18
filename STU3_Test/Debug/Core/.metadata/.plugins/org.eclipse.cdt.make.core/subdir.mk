################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/.metadata/.plugins/org.eclipse.cdt.make.core/specs.c 

OBJS += \
./Core/.metadata/.plugins/org.eclipse.cdt.make.core/specs.o 

C_DEPS += \
./Core/.metadata/.plugins/org.eclipse.cdt.make.core/specs.d 


# Each subdirectory must supply rules for building sources it contributes
Core/.metadata/.plugins/org.eclipse.cdt.make.core/%.o Core/.metadata/.plugins/org.eclipse.cdt.make.core/%.su Core/.metadata/.plugins/org.eclipse.cdt.make.core/%.cyclo: ../Core/.metadata/.plugins/org.eclipse.cdt.make.core/%.c Core/.metadata/.plugins/org.eclipse.cdt.make.core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/BasicMathFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/BayesFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/CommonTables" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/ComplexMathFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/ControllerFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/DistanceFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/FastMathFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/FilteringFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/InterpolationFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/MatrixFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/QuaternionMathFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/StatisticsFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/SupportFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/SVMFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/TransformFunctions" -I"D:/Fibo/Year2/Term2/STU/STU_3_Programming/STU3_Programming/STU3_Test/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f--2e-metadata-2f--2e-plugins-2f-org-2e-eclipse-2e-cdt-2e-make-2e-core

clean-Core-2f--2e-metadata-2f--2e-plugins-2f-org-2e-eclipse-2e-cdt-2e-make-2e-core:
	-$(RM) ./Core/.metadata/.plugins/org.eclipse.cdt.make.core/specs.cyclo ./Core/.metadata/.plugins/org.eclipse.cdt.make.core/specs.d ./Core/.metadata/.plugins/org.eclipse.cdt.make.core/specs.o ./Core/.metadata/.plugins/org.eclipse.cdt.make.core/specs.su

.PHONY: clean-Core-2f--2e-metadata-2f--2e-plugins-2f-org-2e-eclipse-2e-cdt-2e-make-2e-core

