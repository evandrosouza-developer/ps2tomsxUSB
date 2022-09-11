##
## This file is part of the PS/2 to MSX Keyboard converter enviroment:
## PS/2 to MSX keyboard Converter and MSX Keyboard Subsystem Emulator
## designs, based on libopencm3 project.
##
## Copyright (C) 2022 Evandro Souza <evandro.r.souza@gmail.com>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

BINARY = ps2-msxF4F1
OBJS = msxmap.o ps2handl.o dbasemgt.o get_intelhex.o sys_timer.o serial_no.o cdcacm.o serial.o hr_timer.o SpecialFaultHandlers.o

#######=== First step: Identify target inside the Design config file ===########
DSN_CONF_FILE = system.h
WHAT_FIND     = "define MCU"
ifneq ("$(wildcard $(DSN_CONF_FILE))","") #Check if CONF_FILE is available
  cat := $(if $(filter $(OS),Windows_NT),type,cat) #Check OS (Only Windows use type. The others use cat)
  MCU := $(shell $(cat) $(DSN_CONF_FILE) | grep $(WHAT_FIND)) #Find MCU definition

  ifeq ($(word 2,$(MCU)),MCU) #Check if the second word is "MCU" (for example MCU1 is not valid)
    MCU := $(strip $(word $(words $(MCU)),$(MCU))) #Get the last word

    doM3 =
    doM4 =
    ifeq ($(MCU), STM32F103 ) #with ending space
      doM3 = true
    endif #ifeq ($(MCU),STM32F103 )
    ifeq ($(MCU), STM32F103)
      doM3 = true
    endif #ifeq ($(MCU),STM32F103) #without ending space
    ifeq ($(MCU), STM32F401 )
      doM4 = true
    endif #ifeq ($(MCU),STM32F401 )
    ifeq ($(MCU), STM32F401)
      doM4 = true
    endif #ifeq ($(MCU),STM32F401)

    ######=== Second step: Initialize common OCD/BMP/STLink variables ===#######
    ######### OpenOCD specific variables - not related to target ###############
    OOCD           ?= openocd
    OOCD_INTERFACE ?= stlink-v2
    ######### Black Magic Probe specific variables #############################
    # Set the BMP_PORT to a serial port and then BMP is used for flashing
    BMP_PORT       ?= 
    ######### texane/stlink specific variables #################################
    STLINK_PORT    ?= :4242
    ############################################################################

    ##=== Third step: Initialize specific make variables related to target ===##
    ifeq ($(doM3), true)
      $(info Making actions for Core M3 family (Blue Pill stm32f103c6t6 & up))
      LDSCRIPT        = stm32f103x6msx.ld
      ##########################################################################
      LIBNAME         = opencm3_stm32f1
      DEFS           += -DSTM32F1
      FP_FLAGS       ?= -msoft-float
      ARCH_FLAGS      = -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd
      ######### OpenOCD variables specific to target ###########################
      OOCD_TARGET	   ?= stm32f1x
      ##########################################################################
    endif #ifeq ($(MCU),STM32F103 )

    ifeq ($(doM4), true)
      $(info Making actions for Core M4 family (Black Pill stm32f401xC & up))
      LDSCRIPT        = stm32f401xCmsx.ld
      #### You should use linker script generation! Specify device! ############
      ifeq ($(DEVICE),)
        LIBNAME		    = opencm3_stm32f4
        DEFS         += -DSTM32F4
        FP_FLAGS	   ?= -mfloat-abi=hard -mfpu=fpv4-sp-d16
        ARCH_FLAGS    = -mthumb -mcpu=cortex-m4 $(FP_FLAGS)
      endif
      ######### OpenOCD variables specific to target ###########################
      OOCD_TARGET    ?= stm32f4x
      ##########################################################################
    endif #ifeq ($(MCU),STM32F401 )

    ######=== Common fourth step: execute standard libopencm3 rules.mk ===######
		include rules.mk
    ############################################################################

  else #ifeq ($(word 2,$(MCU)),MCU)
    $(error String $(WHAT_FIND) not found on $(CONF_FILE))
  endif #ifeq ($(word 2,$(MCU)),MCU)

else #ifneq ("$(wildcard $(CONF_FILE))","")
  $(error $(CONF_FILE) NOT FOUND)
endif #ifneq ("$(wildcard $(CONF_FILE))","")

