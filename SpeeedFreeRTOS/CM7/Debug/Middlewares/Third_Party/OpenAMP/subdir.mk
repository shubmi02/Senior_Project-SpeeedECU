################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/condition.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/device.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/generic_device.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/generic_init.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/generic_io.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/generic_shmem.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/init.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/io.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/log.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/open-amp/lib/remoteproc/remoteproc_virtio.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg/rpmsg.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg/rpmsg_virtio.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/shmem.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/cortexm/sys.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/time.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/open-amp/lib/virtio/virtio.c \
C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/open-amp/lib/virtio/virtqueue.c 

OBJS += \
./Middlewares/Third_Party/OpenAMP/condition.o \
./Middlewares/Third_Party/OpenAMP/device.o \
./Middlewares/Third_Party/OpenAMP/generic_device.o \
./Middlewares/Third_Party/OpenAMP/generic_init.o \
./Middlewares/Third_Party/OpenAMP/generic_io.o \
./Middlewares/Third_Party/OpenAMP/generic_shmem.o \
./Middlewares/Third_Party/OpenAMP/init.o \
./Middlewares/Third_Party/OpenAMP/io.o \
./Middlewares/Third_Party/OpenAMP/log.o \
./Middlewares/Third_Party/OpenAMP/remoteproc_virtio.o \
./Middlewares/Third_Party/OpenAMP/rpmsg.o \
./Middlewares/Third_Party/OpenAMP/rpmsg_virtio.o \
./Middlewares/Third_Party/OpenAMP/shmem.o \
./Middlewares/Third_Party/OpenAMP/sys.o \
./Middlewares/Third_Party/OpenAMP/time.o \
./Middlewares/Third_Party/OpenAMP/virtio.o \
./Middlewares/Third_Party/OpenAMP/virtqueue.o 

C_DEPS += \
./Middlewares/Third_Party/OpenAMP/condition.d \
./Middlewares/Third_Party/OpenAMP/device.d \
./Middlewares/Third_Party/OpenAMP/generic_device.d \
./Middlewares/Third_Party/OpenAMP/generic_init.d \
./Middlewares/Third_Party/OpenAMP/generic_io.d \
./Middlewares/Third_Party/OpenAMP/generic_shmem.d \
./Middlewares/Third_Party/OpenAMP/init.d \
./Middlewares/Third_Party/OpenAMP/io.d \
./Middlewares/Third_Party/OpenAMP/log.d \
./Middlewares/Third_Party/OpenAMP/remoteproc_virtio.d \
./Middlewares/Third_Party/OpenAMP/rpmsg.d \
./Middlewares/Third_Party/OpenAMP/rpmsg_virtio.d \
./Middlewares/Third_Party/OpenAMP/shmem.d \
./Middlewares/Third_Party/OpenAMP/sys.d \
./Middlewares/Third_Party/OpenAMP/time.d \
./Middlewares/Third_Party/OpenAMP/virtio.d \
./Middlewares/Third_Party/OpenAMP/virtqueue.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/OpenAMP/condition.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/condition.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/condition.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/device.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/device.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/device.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/generic_device.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/generic_device.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/generic_device.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/generic_init.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/generic_init.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/generic_init.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/generic_io.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/generic_io.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/generic_io.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/generic_shmem.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/generic_shmem.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/generic_shmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/init.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/init.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/init.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/io.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/io.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/io.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/log.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/log.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/log.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/remoteproc_virtio.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/open-amp/lib/remoteproc/remoteproc_virtio.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/remoteproc_virtio.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/rpmsg.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg/rpmsg.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/rpmsg.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/rpmsg_virtio.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg/rpmsg_virtio.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/rpmsg_virtio.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/shmem.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/shmem.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/shmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/sys.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/cortexm/sys.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/sys.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/time.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/time.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/time.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/virtio.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/open-amp/lib/virtio/virtio.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/virtio.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/OpenAMP/virtqueue.o: C:/Users/Spartan\ Racing/Downloads/SpeeedFreeRTOS/Middlewares/Third_Party/OpenAMP/open-amp/lib/virtio/virtqueue.c Middlewares/Third_Party/OpenAMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DMETAL_INTERNAL -DMETAL_MAX_DEVICE_REGIONS=2 -DRPMSG_BUFFER_SIZE=512 -DVIRTIO_MASTER_ONLY -DNO_ATOMIC_64_SUPPORT -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../OPENAMP -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include/metal/compiler/gcc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Third_Party/OpenAMP/virtqueue.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-OpenAMP

clean-Middlewares-2f-Third_Party-2f-OpenAMP:
	-$(RM) ./Middlewares/Third_Party/OpenAMP/condition.cyclo ./Middlewares/Third_Party/OpenAMP/condition.d ./Middlewares/Third_Party/OpenAMP/condition.o ./Middlewares/Third_Party/OpenAMP/condition.su ./Middlewares/Third_Party/OpenAMP/device.cyclo ./Middlewares/Third_Party/OpenAMP/device.d ./Middlewares/Third_Party/OpenAMP/device.o ./Middlewares/Third_Party/OpenAMP/device.su ./Middlewares/Third_Party/OpenAMP/generic_device.cyclo ./Middlewares/Third_Party/OpenAMP/generic_device.d ./Middlewares/Third_Party/OpenAMP/generic_device.o ./Middlewares/Third_Party/OpenAMP/generic_device.su ./Middlewares/Third_Party/OpenAMP/generic_init.cyclo ./Middlewares/Third_Party/OpenAMP/generic_init.d ./Middlewares/Third_Party/OpenAMP/generic_init.o ./Middlewares/Third_Party/OpenAMP/generic_init.su ./Middlewares/Third_Party/OpenAMP/generic_io.cyclo ./Middlewares/Third_Party/OpenAMP/generic_io.d ./Middlewares/Third_Party/OpenAMP/generic_io.o ./Middlewares/Third_Party/OpenAMP/generic_io.su ./Middlewares/Third_Party/OpenAMP/generic_shmem.cyclo ./Middlewares/Third_Party/OpenAMP/generic_shmem.d ./Middlewares/Third_Party/OpenAMP/generic_shmem.o ./Middlewares/Third_Party/OpenAMP/generic_shmem.su ./Middlewares/Third_Party/OpenAMP/init.cyclo ./Middlewares/Third_Party/OpenAMP/init.d ./Middlewares/Third_Party/OpenAMP/init.o ./Middlewares/Third_Party/OpenAMP/init.su ./Middlewares/Third_Party/OpenAMP/io.cyclo ./Middlewares/Third_Party/OpenAMP/io.d ./Middlewares/Third_Party/OpenAMP/io.o ./Middlewares/Third_Party/OpenAMP/io.su ./Middlewares/Third_Party/OpenAMP/log.cyclo ./Middlewares/Third_Party/OpenAMP/log.d ./Middlewares/Third_Party/OpenAMP/log.o ./Middlewares/Third_Party/OpenAMP/log.su ./Middlewares/Third_Party/OpenAMP/remoteproc_virtio.cyclo ./Middlewares/Third_Party/OpenAMP/remoteproc_virtio.d ./Middlewares/Third_Party/OpenAMP/remoteproc_virtio.o ./Middlewares/Third_Party/OpenAMP/remoteproc_virtio.su ./Middlewares/Third_Party/OpenAMP/rpmsg.cyclo ./Middlewares/Third_Party/OpenAMP/rpmsg.d ./Middlewares/Third_Party/OpenAMP/rpmsg.o ./Middlewares/Third_Party/OpenAMP/rpmsg.su ./Middlewares/Third_Party/OpenAMP/rpmsg_virtio.cyclo ./Middlewares/Third_Party/OpenAMP/rpmsg_virtio.d ./Middlewares/Third_Party/OpenAMP/rpmsg_virtio.o ./Middlewares/Third_Party/OpenAMP/rpmsg_virtio.su ./Middlewares/Third_Party/OpenAMP/shmem.cyclo ./Middlewares/Third_Party/OpenAMP/shmem.d ./Middlewares/Third_Party/OpenAMP/shmem.o ./Middlewares/Third_Party/OpenAMP/shmem.su ./Middlewares/Third_Party/OpenAMP/sys.cyclo ./Middlewares/Third_Party/OpenAMP/sys.d ./Middlewares/Third_Party/OpenAMP/sys.o ./Middlewares/Third_Party/OpenAMP/sys.su ./Middlewares/Third_Party/OpenAMP/time.cyclo ./Middlewares/Third_Party/OpenAMP/time.d ./Middlewares/Third_Party/OpenAMP/time.o ./Middlewares/Third_Party/OpenAMP/time.su ./Middlewares/Third_Party/OpenAMP/virtio.cyclo ./Middlewares/Third_Party/OpenAMP/virtio.d ./Middlewares/Third_Party/OpenAMP/virtio.o ./Middlewares/Third_Party/OpenAMP/virtio.su ./Middlewares/Third_Party/OpenAMP/virtqueue.cyclo ./Middlewares/Third_Party/OpenAMP/virtqueue.d ./Middlewares/Third_Party/OpenAMP/virtqueue.o ./Middlewares/Third_Party/OpenAMP/virtqueue.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-OpenAMP

