BOARD_TYPE          := 0x04
BOARD_REVISION      := 0x02
# Previous version was 0x080, 0x081 introduces forced boot from bkp registers,
# 0x082 fixes halt (by not double-initing board) 
BOOTLOADER_VERSION  := 0x83
HW_TYPE             := 0x01

MCU                 := cortex-m3
CHIP                := STM32F103CBT
BOARD               := STM32103CB_CC_Rev1
MODEL               := MD
MODEL_SUFFIX        := _CC

OPENOCD_JTAG_CONFIG ?= stlink-v2.cfg
OPENOCD_CONFIG      := stm32f1x.cfg

# Note: These must match the values in link_$(BOARD)_memory.ld
BL_BANK_BASE        := 0x08000000  # Start of bootloader flash
BL_BANK_SIZE        := 0x00003000  # Should include BD_INFO region
FW_BANK_BASE        := 0x08003000  # Start of firmware flash
FW_BANK_SIZE        := 0x0001D000  # Should include FW_DESC_SIZE (116kb)

FW_DESC_SIZE        := 0x00000064

EE_BANK_BASE        := 0x00000000
EE_BANK_SIZE        := 0x00000000

EF_BANK_BASE        := 0x08000000  # Start of entire flash image (usually start of bootloader as well)
EF_BANK_SIZE        := 0x00020000  # Size of the entire flash image (from bootloader until end of firmware)

OSCILLATOR_FREQ     := 8000000
SYSCLK_FREQ         := 72000000
FW_DESC_BASE        := 0
