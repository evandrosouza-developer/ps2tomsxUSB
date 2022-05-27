# PS/2 to MSX Keyboard Converter Interface based on STM32

This interface has the function of adapt and use a PS/2 keyboard as source to zeroed matrix based computers, like MSX and ZX-Spectrum as destination. It is meant to connect a keyboard, which only provides a PS/2 connector, to a MSX (or any one that have a up to 15 active columns and reads zeroes througth 8 bits - up to 15 x 8 matrix) computer.
The aim of this design is to allow user to customaze and update the database layout of this PS/2 Keyboard, as like as the MSX one, through an USB (or serial) interface.
To edit the Database file, both source and target keyboard layouts, I boldly recommend you to use the excel file `PS2toMSX-BR Mapping220204.xlsm` available at github page.
The default database mappings for the keyboard layouts are:
- Source (PS/2 keyboard): 275 (ABNT2 layout - Brazilian Technical Norms Association);
- Target: Brazilian Sharp/Epcom Hotbit HB8000 MSX.

This interface powers and has been tested as OK on the following Windows PS/2 keyboards:
- Compaq RT235BTWBR;
- Clone KE11090749;
- Clone #09100.

The original (PS/2 to USB) code, now restricted to ps2handl.c and ps2handl.h, was originally developed based on:
- Cherry G80-3000LSMDE;

## Dependencies

- `arm-none-eabi-gcc`
- `arm-none-eabi-gdb`
- `arm-none-eabi-binutils`
- `arm-none-eabi-newlib`
- `stlink`
- `openocd (if you wnat to debug)`
- `mpfr`

## Preparations

After cloning the repository you need to make the following preparations:

```
git submodule init
git submodule update
cd libopencm3
make
cd ..
make
```

## Hardware and Setup

You will obviously need a STM32F401CCU6 or the newer one STM32F401CEU6 chip. I have used a chinese WeAct STM32F4x1 MiniF4 Black Pill V3.0. The software was made considering 25.000Mhz oscillator crystal, to clock the STM32 microcontroller chip at 84MHz. The connections are:

1) PS/2 Keyboard (J3- PS/2 Port) :
- PS/2 power - Pin 1 - Connect to PS/2 mini-din 45322 pin 4;
- Clock      - Pin 2 - Connect to PS/2 mini-din 45322 pin 5;
- Data       - Pin 3 - Connect to PS/2 mini-din 45322 pin 1;
- GNDD       - Pin 4 - Connect to PS/2 mini-din 45322 pin 3.

2) MSX computer:
Obs.: You have to access PPI Ports B0 to B7 (Lines X0 to X7), Port C0 to C3 (Y0 to Y3), Port C6 (Caps Lock, pin 11 of DIP package), and for Russian and Japanese Computers (Cyrillic and Kana alphabets), you have to get access to YM2149 IOB7, pin 6 of DIP package.
- Y3 - Connect to MSX PPI 8255 Signal PC3;
- Y2 - Connect to MSX PPI 8255 Signal PC2;
- Y1 - Connect to MSX PPI 8255 Signal PC1;
- Y0 - Connect to MSX PPI 8255 Signal PC0;
- X7 - Connect to MSX PPI 8255 Signal PB7 (Hotbit HB-8000 CI-15 Pin 25);
- X6 - Connect to MSX PPI 8255 Signal PB6 (Hotbit HB-8000 CI-15 Pin 24);
- X5 - Connect to MSX PPI 8255 Signal PB5 (Hotbit HB-8000 CI-15 Pin 23);
- X4 - Connect to MSX PPI 8255 Signal PB4 (Hotbit HB-8000 CI-15 Pin 22);
- X3 - Connect to MSX PPI 8255 Signal PB3 (Hotbit HB-8000 CI-15 Pin 21);
- X2 - Connect to MSX PPI 8255 Signal PB2 (Hotbit HB-8000 CI-15 Pin 20);
- X1 - Connect to MSX PPI 8255 Signal PB1 (Hotbit HB-8000 CI-15 Pin 19);
- X0 - Connect to MSX PPI 8255 Signal PB0 (Hotbit HB-8000 CI-15 Pin 18);
- Caps LED - Connect to MSX PPI 8255 Signal PC6 (Hotbit HB-8000 CI-15 Pin 11 / Expert XP-800 CI-4);
- Kana LED - Connect to MSX YM2149 IOB7, pin 6 of DIP package. If the MSX doesn't have this, you can leave it open, as it already has an internal pull-up connection.

3) USB or Serial connection: Needed only to update internal PS/2 to MSX key mapping Database. To create this Intel Hex file, better to use the Macro based Excel file, so you have to trust and enable macro excecution in excel file `PS2toMSX-BR Mapping220204.xlsm` available at github page. Fill the desired mapping settings on the `Your MSX Definitions` and click on the MSX picture at upper left corner to run the automation process. This automation is also compatible with LibreOffice/StarOffice/OpenOffice, as tested with LibreOffice up to version 7.3.
The structure of the Database is:
	The  three first columns of each line are the mapped scan codes;
	The 4th column is The Control Byte, detailed bellow:
	CONTROL BYTE:
		High nibble is Reserved;
		(bit 3) Combined Shift;
		(bit 2) Reserved-Not used;
		(bits 1-0) Modifyer Type:
		.0 - Default mapping
		.1 - NumLock Status+Shift changes
		.2 - PS/2 Shift
		.3 - Reserved-Not used
	
	This table has 3 modifyers: Up two MSX keys are considered to each mapping behavior modifying:
	
	5th and 6th columns have the mapping ".0 - Default mapping";
	7th and 8th columns have the mapping ".1 - NumLock Status+Shift changes";
	7th and 8th columns also have the mapping ".2 - PS/2 Shift", where I need to
	release the sinalized Shift in PS/2 to the MSX and put the coded key, and so,
	release them, reapplying the Shift key, deppending on the initial state;
	
	
	Each column has a MSX coded key, with the follwing structure:
	(Bit 7:4) MSX Y PPI 8255 PC3:0 is send to an OC outputs BCD decoder, for example:
					 In the case of Hotbit HB8000, the keyboard scan is done as a 9 columns scan, CI-16 7445N 08 to 00;
					 If equals to 1111 (Y=15), there is no MSX key mapped.
	(Bit 3)	 		 0: keypress
					 1: key release;
	(Bit 2:0) MSX X, ie, which bit will carry the key, to be read by PPI 8255 PB7:0.
	

## Hardware and Setup
STM32F4x1 MiniF4 already comes with DFU (Device Firmware Upgrade) available in the system ROM to download code via USB port, so you don't neither need ST-Link, J-link or Black Magic Probe nor serial adapters to download your code. So, on linux, just follow these steps:

1. Install dfu-util. This example is for Debian family Linux (Debian, Ubuntu, Mint, etc): `sudo apt install dfu-util`
2. Make sure the chip is at least 25°C (you may let it working for a while with the help of your finger), because its internal RC oscillator was factory trimmed to 25°C;
3. Plug the USB cable to your computer and the STM32F4x1 MiniF4 (Black Pill) board while holding both NRST and BOOT0;
4. Then release BOOT0 AFTER 0.5 second you released NRST;
5. Run the command to flash the code itself: `dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D ps2-msxUSB.bin`

On windows you can download STM32CubeProg on ST site, replacing step 1. You have to adjust step 5 to this tool. Please follow ST instructions.

I recomend to use a ST-Link v2 Programmer, Black Magic Probe (or similar) to flash the program using make flash onto the STM32, as so, you will be able to do debugging.

Blue Pill (STM32F103C6T6 and STM32F103C8T6)
Usage of Blue Pill is an option, but you will have to compile the code, as the default is the black pill, because the default is cheaper, faster, more modern and powerful, and it has enough resources to have USB implemented.
Use a ST-Link v2 Programmer (or similar) to flash the program using `make flash` onto the STM32.

