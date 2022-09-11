# STM32: PS/2 to USB Converter

This fully functional design uses MiniSTM32F4xC board to convert between PS/2 and MSX computer. It is meant to connect a keyboard, which only provides a PS/2 connection, to a MSX (or any one that have a up to 15 active columns and reads zeroes througth 8 bits - up to 15 x 8 matrix) retro computer.
This code powers and has been tested on the following Windows keyboards, with brazilian 275 layout:
- Compaq RT235BTWBR;
- Clone KE11090749;
- Clone #09100.

The original code was originally developed based on:
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

You will obviously need a STM32F401CCU6 or the newer one STM32F401CEU6 chip. I have designed a PCB considering the WeAct V3.0 black pill. The software was made considering 25.000Mhz oscillator crystal, to clock the STM32 microcontroller chip at 84MHz. The connections are:

1) PS/2 Keyboard (J3- PS/2 Port) :
- PS/2 power - Pin 1 - Connect to PS/2 mini-din 45322 pin 4;
- Clock      - Pin 2 - Connect to PS/2 mini-din 45322 pin 5;
- Data       - Pin 3 - Connect to PS/2 mini-din 45322 pin 1;
- GNDD       - Pin 4 - Connect to PS/2 mini-din 45322 pin 3.

2) MSX computer:
Obs.: You have to access PPI Ports B0 to B7 (Lines X0 to X7), Port C0 to C3 (Y0 to Y3), Port C6 (Caps Lock, pin 11 of DIP package), and for Russian and Japanese Computers (Cyrillic and Kana alphabets), you have to get access to YM2149 IOB7, pin 6 of DIP package.
- Y3 - Connect to MSX PPI 8255 Signal PC3 (HB-8000 CI-15 Pin 17 / XP-800 CI-4 Pin 17);
- Y2 - Connect to MSX PPI 8255 Signal PC2 (HB-8000 CI-15 Pin 16 / XP-800 CI-4 Pin 16);
- Y1 - Connect to MSX PPI 8255 Signal PC1 (HB-8000 CI-15 Pin 15 / XP-800 CI-4 Pin 15);
- Y0 - Connect to MSX PPI 8255 Signal PC0 (HB-8000 CI-15 Pin 14 / XP-800 CI-4 Pin 14);
- X7 - Connect to MSX PPI 8255 Signal PB7 (HB-8000 CI-15 Pin 25 / XP-800 CI-4 Pin 25);
- X6 - Connect to MSX PPI 8255 Signal PB6 (HB-8000 CI-15 Pin 24 / XP-800 CI-4 Pin 24);
- X5 - Connect to MSX PPI 8255 Signal PB5 (HB-8000 CI-15 Pin 23 / XP-800 CI-4 Pin 23);
- X4 - Connect to MSX PPI 8255 Signal PB4 (HB-8000 CI-15 Pin 22 / XP-800 CI-4 Pin 22);
- X3 - Connect to MSX PPI 8255 Signal PB3 (HB-8000 CI-15 Pin 21 / XP-800 CI-4 Pin 21);
- X2 - Connect to MSX PPI 8255 Signal PB2 (HB-8000 CI-15 Pin 20 / XP-800 CI-4 Pin 20);
- X1 - Connect to MSX PPI 8255 Signal PB1 (HB-8000 CI-15 Pin 19 / XP-800 CI-4 Pin 19);
- X0 - Connect to MSX PPI 8255 Signal PB0 (HB-8000 CI-15 Pin 18 / XP-800 CI-4 Pin 18);
- Caps LED - Connect to MSX PPI 8255 Signal PC6 (HB-8000 CI-15 Pin 11 / XP-800 CI-4 Pin 11);
- Kana LED - Connect to MSX YM2149 IOB7, pin 6 of DIP package. If the MSX doesn't have this, you can leave it open, as it already has an internal pull-up connection.

3) Serial connection: Needed only to update internal PS/2 to MSX key mapping Database. To create this Intel Hex file, better to use the Macro based Excel file, so you have to trust and enable macro excecution in excel app.
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
	7th e 8th columns have the mapping ".1 - NumLock Status+Shift changes";
	9th and 10th columns have the mapping ".2 - PS/2 Shift", where I need to
	release the sinalized Shift in PS/2 to the MSX and put the coded key, and so,
	release them, reapplying the Shift key, deppending on the initial state;
	
	
	Each column has a MSX coded key, with the sollwing structure:
	(Bit 7:4) MSX Y PPI 8255 PC3:0 is send to an OC outputs BCD decoder, for example:
					 In the case of Hotbit HB8000, the keyboard scan is done as a 9 columns scan, CI-16 7445N 08 to 00;
					 If equals to 1111 (Y=15), there is no MSX key mapped.
	(Bit 3)	 		 0: keypress
					 1: key release;
	(Bit 2:0) MSX X, ie, which bit will carry the key, to be read by PPI 8255 PB7:0.
	
## Download the code to the board

STM32F4x1 MiniF4 already comes with DFU (Device Firmware Upgrade) available in the system ROM to download code, so you don't need ST-Link, J-link or Black Magic Probe to download your code, so, on linux, just follow these steps:
1) Install dfu-util. This example is for Debian like Linux (Debian, Ubuntu, Mint, etc):
`sudo apt install dfu-util`
2) Make the .bin file, as dfu-util is not compatible with .elf: 
`arm-none-eabi-objcopy -Obinary ps2-msxF4.elf ps2-msxF4.bin`
3) Make sure the chip is at least 25°C (you may let it working for a while and help with your funger), because it uses internal oscillator, factory trimmed to 25°C;
4) Plug the USB cable to your computer and the STM32F4 board while holding both NRST and BOOT0;
5) Then release BOOT0 AFTER 0.5 second you released NRST;
6) Run the command to flash the code itself:
`dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D ps2-msxF4.bin`

On windows you can download STM32CubeProg on ST site, replacing step 1. You have to adjust step 6 to this toool. Please follow ST instructions.

I recomend to use a ST-Link v2 Programmer, Black Magic Probe (or similar) to flash the program using `make flash` onto the STM32.
