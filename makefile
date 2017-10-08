
MCU          = atmega2560
F_CPU        = 16000000
TARGET       = arduino_mega2560__m2560
LWCLONE_SRC  = main.c i2cmaster.c MMA8451.c uart.c

include default.mk
