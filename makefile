#Nombre del proyecto
TARGET = temp

#Archivos a compilar
OBJS = main.o app_ints.o app_msps.o startup_stm32f072xb.o system_stm32f0xx.o 
OBJS += stm32f0xx_hal.o stm32f0xx_hal_cortex.o stm32f0xx_hal_rcc.o stm32f0xx_hal_flash.o
OBJS += stm32f0xx_hal_gpio.o
#archivo linker a usar
LINKER = linker.ld
#Simbolos gloobales del programa (#defines globales)
SYMBOLS = -DSTM32F072xB -DUSE_HAL_DRIVER
#directorios con archivos a compilar (.c y .s)
VPATH = app
VPATH += cmsisf0/startups
VPATH += half0/Src
#direcotrios con archivos .h
INCLUDES = -I app
INCLUDES += -I cmsisf0/core
INCLUDES += -I cmsisf0/registers
INCLUDES += -I half0/Inc

#compilador y opciones de compilacion
TOOLCHAIN = arm-none-eabi
CPU = -mcpu=cortex-m0 -mthumb -mfloat-abi=soft
CFLAGS = $(CPU) -Wall -g3 -O0 -std=c99
AFLAGS = $(CPU)
LFLAGS = $(CPU) -Wl,--gc-sections --specs=rdimon.specs --specs=nano.specs -Wl,-Map=Output/$(TARGET).map

#Instrucciones de compilacion
all : $(TARGET)

$(TARGET) : $(addprefix Output/, $(TARGET).elf)
	$(TOOLCHAIN)-objcopy -Oihex $< Output/$(TARGET).hex
	$(TOOLCHAIN)-objdump -S $< > Output/$(TARGET).lst
	$(TOOLCHAIN)-size --format=berkeley $<

Output/$(TARGET).elf : $(addprefix Output/obj/, $(OBJS))
	$(TOOLCHAIN)-gcc $(LFLAGS) -T $(LINKER) -o $@ $^

Output/obj/%.o : %.c
	mkdir -p Output/obj
	$(TOOLCHAIN)-gcc $(CFLAGS) $(INCLUDES) $(SYMBOLS) -o $@ -c $<

Output/obj/%.o : %.s
	$(TOOLCHAIN)-as $(AFLAGS) -o $@ -c $<

#borrar archivos generados
clean :
	rm -r Output

#Programar al tarjeta
flash :
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f0x.cfg -c "program Output/$(TARGET).hex verify reset" -c shutdown
#Conectar OpenOCD con al Tarjeta
open :
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f0x.cfg -c "reset_config srst_only srst_nogate"
#Lanzar sesion de debug (es necesario primero Conectar la tarjeta con OpenOCD)
debug :
	$(TOOLCHAIN)-gdb Output/$(TARGET).elf -iex "set auto-load safe-path /"

