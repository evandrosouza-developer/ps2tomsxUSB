/*
 * This file was part of the libopencm3 project.
 * Copyright (C) 2015 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Linker script for STM32F401xC, 256k flash, 64k RAM. */
/* Linker script for STM32F401xD, 384k flash, 96k RAM. */

/* Define memory regions. */
MEMORY
{
	ram (rwx)				: ORIGIN = 0x20000000, LENGTH = 64K
/*ram (rwx)				: ORIGIN = 0x20000000, LENGTH = 96K	STM32F401CDU6*/
	rom (rx)				: ORIGIN = 0x08000000, LENGTH = 48K
	fixed_data (rx)	: ORIGIN = 0x0800C000, LENGTH = 16K
	rom2 (rx)				: ORIGIN = 0x08010000, LENGTH = 192K
/*rom2 (rx)				: ORIGIN = 0x08010000, LENGTH = 320K	STM32F401CDU6*/
}

SECTIONS
{
	.text2	 : {
		*(.text*)	/* Program code */
		. = ALIGN(4);
	} >rom2
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld

