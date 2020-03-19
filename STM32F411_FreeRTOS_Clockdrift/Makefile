TARGET:=FreeRTOS
# TODO change to your ARM gcc toolchain path
TOOLCHAIN_ROOT:=/.
TOOLCHAIN_PATH:=$(TOOLCHAIN_ROOT)/bin
TOOLCHAIN_PREFIX:=arm-none-eabi

# Optimization level, can be [0, 1, 2, 3, s].
OPTLVL:=0
DBG:=-g

FREERTOS:=$(CURDIR)/FreeRTOS
STARTUP:=$(CURDIR)/hardware
LINKER_SCRIPT:=$(CURDIR)/Utilities/stm32_flash.ld

INCLUDE=-I$(CURDIR)/hardware
INCLUDE+=-I$(FREERTOS)/include
INCLUDE+=-I$(FREERTOS)/portable/GCC/ARM_CM4F
INCLUDE+=-I$(CURDIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc
INCLUDE+=-I$(CURDIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc/Legacy
INCLUDE+=-I$(CURDIR)/Libraries/CMSIS/Device/ST/STM32F4xx/Include
INCLUDE+=-I$(CURDIR)/Libraries/CMSIS/Include
INCLUDE+=-I$(CURDIR)/config
INCLUDE+= -I$(CURDIR)/FreeRTOS/include

BUILD_DIR = $(CURDIR)/build
BIN_DIR = $(CURDIR)/binary

# vpath is used so object files are written to the current directory instead
# of the same directory as their source files
vpath %.c $(CURDIR)/Libraries/STM32F4xx_StdPeriph_Driver/src \
          $(CURDIR)/Libraries/syscall $(CURDIR)/hardware $(FREERTOS) \
          $(FREERTOS)/portable/MemMang $(FREERTOS)/portable/GCC/ARM_CM4F

vpath %.s $(STARTUP)
ASRC=startup_stm32f411xe.s

# Project Source Files
SRC+=stm32f4xx_it.c
SRC+=system_stm32f4xx.c
SRC+=main.c
SRC+=syscalls.c

# FreeRTOS Source Files
SRC+=port.c
SRC+=list.c
SRC+=queue.c
SRC+=tasks.c
SRC+=event_groups.c
SRC+=timers.c
SRC+=heap_3.c

# Standard Peripheral Source Files
SRC+=stm32f4xx_hal.c
SRC+=stm32f4xx_hal_adc.c
SRC+=stm32f4xx_hal_adc_ex.c
SRC+=stm32f4xx_hal_can.c
SRC+=stm32f4xx_hal_cec.c
SRC+=stm32f4xx_hal_cortex.c
SRC+=stm32f4xx_hal_crc.c
SRC+=stm32f4xx_hal_cryp.c
SRC+=stm32f4xx_hal_cryp_ex.c
SRC+=stm32f4xx_hal_dac.c
SRC+=stm32f4xx_hal_dac_ex.c
SRC+=stm32f4xx_hal_dcmi.c
SRC+=stm32f4xx_hal_dcmi_ex.c
SRC+=stm32f4xx_hal_dfsdm.c
SRC+=stm32f4xx_hal_dma.c
SRC+=stm32f4xx_hal_dma2d.c
SRC+=stm32f4xx_hal_dma_ex.c
SRC+=stm32f4xx_hal_dsi.c
SRC+=stm32f4xx_hal_eth.c
SRC+=stm32f4xx_hal_exti.c
SRC+=stm32f4xx_hal_flash.c
SRC+=stm32f4xx_hal_flash_ex.c
SRC+=stm32f4xx_hal_flash_ramfunc.c
SRC+=stm32f4xx_hal_fmpi2c.c
SRC+=stm32f4xx_hal_gpio.c
SRC+=stm32f4xx_hal_hash.c
SRC+=stm32f4xx_hal_hash_ex.c
SRC+=stm32f4xx_hal_hcd.c
SRC+=stm32f4xx_hal_i2c.c
SRC+=stm32f4xx_hal_i2c_ex.c
SRC+=stm32f4xx_hal_i2s.c
SRC+=stm32f4xx_hal_i2s_ex.c
SRC+=stm32f4xx_hal_irda.c
SRC+=stm32f4xx_hal_iwdg.c
SRC+=stm32f4xx_hal_lptim.c
SRC+=stm32f4xx_hal_ltdc.c
SRC+=stm32f4xx_hal_ltdc_ex.c
SRC+=stm32f4xx_hal_mmc.c
SRC+=stm32f4xx_hal_nand.c
SRC+=stm32f4xx_hal_nor.c
SRC+=stm32f4xx_hal_pccard.c
SRC+=stm32f4xx_hal_pcd.c
SRC+=stm32f4xx_hal_pcd_ex.c
SRC+=stm32f4xx_hal_pwr.c
SRC+=stm32f4xx_hal_pwr_ex.c
SRC+=stm32f4xx_hal_qspi.c
SRC+=stm32f4xx_hal_rcc.c
SRC+=stm32f4xx_hal_rcc_ex.c
SRC+=stm32f4xx_hal_rng.c
SRC+=stm32f4xx_hal_rtc.c
SRC+=stm32f4xx_hal_rtc_ex.c
SRC+=stm32f4xx_hal_sai.c
SRC+=stm32f4xx_hal_sai_ex.c
SRC+=stm32f4xx_hal_sd.c
SRC+=stm32f4xx_hal_sdram.c
SRC+=stm32f4xx_hal_smartcard.c
SRC+=stm32f4xx_hal_smbus.c
SRC+=stm32f4xx_hal_spdifrx.c
SRC+=stm32f4xx_hal_spi.c
SRC+=stm32f4xx_hal_sram.c
SRC+=stm32f4xx_hal_tim.c
SRC+=stm32f4xx_hal_tim_ex.c
SRC+=stm32f4xx_hal_uart.c
SRC+=stm32f4xx_hal_usart.c
SRC+=stm32f4xx_hal_wwdg.c
SRC+=stm32f4xx_ll_adc.c
SRC+=stm32f4xx_ll_crc.c
SRC+=stm32f4xx_ll_dac.c
SRC+=stm32f4xx_ll_dma.c
SRC+=stm32f4xx_ll_dma2d.c
SRC+=stm32f4xx_ll_exti.c
SRC+=stm32f4xx_ll_fmc.c
SRC+=stm32f4xx_ll_fsmc.c
SRC+=stm32f4xx_ll_gpio.c
SRC+=stm32f4xx_ll_i2c.c
SRC+=stm32f4xx_ll_lptim.c
SRC+=stm32f4xx_ll_pwr.c
SRC+=stm32f4xx_ll_rcc.c
SRC+=stm32f4xx_ll_rng.c
SRC+=stm32f4xx_ll_rtc.c
SRC+=stm32f4xx_ll_sdmmc.c
SRC+=stm32f4xx_ll_spi.c
SRC+=stm32f4xx_ll_tim.c
SRC+=stm32f4xx_ll_usart.c
SRC+=stm32f4xx_ll_usb.c
SRC+=stm32f4xx_ll_utils.c


CDEFS=-DUSE_STDPERIPH_DRIVER
CDEFS+=-DSTM32F4XX
CDEFS+=-DSTM32F40_41xxx
CDEFS+=-DHSE_VALUE=8000000
CDEFS+=-D__FPU_PRESENT=1
CDEFS+=-D__FPU_USED=1
CDEFS+=-DARM_MATH_CM4

MCUFLAGS=-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -finline-functions -Wdouble-promotion -std=gnu99
COMMONFLAGS=-O$(OPTLVL) $(DBG) -Wall -ffunction-sections -fdata-sections
CFLAGS=$(COMMONFLAGS) $(MCUFLAGS) $(INCLUDE) $(CDEFS)

LDLIBS=-lm -lc -lgcc
LDFLAGS=$(MCUFLAGS) -u _scanf_float -u _printf_float -fno-exceptions -Wl,--gc-sections,-T$(LINKER_SCRIPT),-Map,$(BIN_DIR)/$(TARGET).map

CC=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
LD=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
OBJCOPY=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-objcopy
AS=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-as
AR=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-ar
GDB=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gdb

OBJ = $(SRC:%.c=$(BUILD_DIR)/%.o)

$(BUILD_DIR)/%.o: %.c
	@echo [CC] $(notdir $<)
	@$(CC) $(CFLAGS) $< -c -o $@

all: $(OBJ)
	@echo [AS] $(ASRC)
	@$(AS) -o $(ASRC:%.s=$(BUILD_DIR)/%.o) $(STARTUP)/$(ASRC)
	@echo [LD] $(TARGET).elf
	@$(CC) -o $(BIN_DIR)/$(TARGET).elf $(LDFLAGS) $(OBJ) $(ASRC:%.s=$(BUILD_DIR)/%.o) $(LDLIBS)
	@echo [HEX] $(TARGET).hex
	@$(OBJCOPY) -O ihex $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).hex
	@echo [BIN] $(TARGET).bin
	@$(OBJCOPY) -O binary $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).bin

.PHONY: clean

clean:
	@echo [RM] OBJ
	@rm -f $(OBJ)
	@rm -f $(ASRC:%.s=$(BUILD_DIR)/%.o)
	@echo [RM] BIN
	@rm -f $(BIN_DIR)/$(TARGET).elf
	@rm -f $(BIN_DIR)/$(TARGET).hex
	@rm -f $(BIN_DIR)/$(TARGET).bin

flash:
	@st-flash write $(BIN_DIR)/$(TARGET).bin 0x8000000