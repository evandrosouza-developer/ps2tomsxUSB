##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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

BINARY=	ps2-msxUSB
OBJS =	msxmap.o \
				ps2handl.o\
				dbasemgtF1.o\
				dbasemgtF4.o\
				sys_timer.o\
				serial.o\
				hr_timer.o\
				SpecialFaultHandlers.o\
				cdcacm.o\
				winusb.o\
				usb21_standard.o\
				webusb.o\
				logger.o

LDSCRIPT = stm32f401xC.ld

include Makefile.include

## To run openocd, use:
## sudo openocd -s ../tcl -f stlink-swd.ocd
## or st-util

## To run gdb, use:
## 1) If the server is openocd:
## arm-none-eabi-gdb target extended-remote localhost:3333
##
## 2) If the server is st-util:
## arm-none-eabi-gdb target extended-remote localhost:4242
