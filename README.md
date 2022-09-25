# STM32: PS/2 to MSX Keyboard Converter

This interface has the function of connect a PS/2 keyboard as source to zero active matrix based computers, like MSX and ZX-Spectrum as destination. It is meant to connect a keyboard, which only provides a PS/2 connector, to a MSX (or any one that have a up to 15 active columns and reads zeroes througth 8 bits - up to 15 x 8 matrix) computer.

The differencial of this design is to allow user to customize and update the database layout of this PS/2 Keyboard, as like as the MSX one, through an USB (or serial) interface and a tty terminal on host side.

To edit the Database file, both source and target keyboard layouts, I prepaired a dedicated "key assembler in excel", so I can boldly recommend you to use the excel file `PS2toMSX-BR Mapping220901.xlsm` available at github page.

The default database mappings for the keyboard layouts are:

- Source (PS/2 keyboard): 275 (ABNT2 layout - Brazilian Technical Norms Association);

- Target: Brazilian Sharp/Epcom Hotbit HB8000 MSX.

This firmware was made to support both Blue Pill and Black Pill and it is part of the PS/2 to MSX Converter Enviroment, all of them in my github page, that consists:
```
1)The PS/2 to MSX Keyboard Converter itself, which contains:
1.1) The firmware with source files;
1.2) Schematics and PCB design:
1.2.1) Electronics part Schematics with Kicad files;
1.2.2) Single sided PCB layout with Kicad files and complete set of Gerber files;

2)The PS/2 to MSX Converter Tester, which contains:
2.1) The firmware with source files;
2.2) Schematics design with Kicad files;

3) Tool to create/modify the Database (Translation tables to map from PS/2 Scan Codes to MSX Matrix Codes) in excel, but it has compatible macros to be executed by Libre Office, Open Office, Star Office and so on.

4) Tini TTY I/O - tio (Linux app with source files) to communicate with console and easily allowing the user to use it with different keyboard layouts and languages.
```
In case of STM32F103C8T6 (Flash 64K RAM 20K) Blue Pill, you have to aplly STM32F103C6T6 (Flash 32K RAM 10K) without change anything else.

In case of Black Pill, the base system is STM32F401CCU6 so, to use a more memory one, just follow the same instructions here.

This firmware powers and has been tested on the following Windows keyboards, with brazilian 275 layout:

- Compaq RT235BTWBR;

- Clone KE11090749;

- Clone #09100.


The original code was originally developed based on:

- Cherry G80-3000LSMDE.


## Why not STM32F103C8T6 Blue Pill?

Because Blue Pill has no 5V tolerant pins available to connect PS/2 + MSX + Serial + USB, unlike Black Pill. Even serial (UART) was feasibled using a 3.3V only port.

Although, this limitation came with a bonus: The need of a cheaper and greater avaiability 32KB flash MCU: STM32F103C6T6


## Dependencies

- `arm-none-eabi-gcc`
- `arm-none-eabi-gdb`
- `arm-none-eabi-binutils`
- `stlink + openocd (if you want to debug)`
- `libopenmcm3`

Obs.: If you plan to keep only one copy of LibopenCM3 in your computer, I really suggest you to create the variable OPENCM3_DIR in our system enviroment.

## Preparations

After cloning the repository you need to make the following preparations:

Go to libopencm3 you cloned (eg: cd libopencm3) and prepares it to use by typing:

```
make TARGETS='stm32/f1 stm32/f4'
```

Go to your PS/2 to MSX Converter Tester project folder and assure that you choose the right target MCU in the system.h file line 69, and make, as follows:
```
#define MCU                       STM32F103
or
#define MCU                       STM32F401

make
```

```
arm-none-eabi-size ps2-msx-kb-convF1.elf
   text	   data	    bss	    dec	    hex	filename
  27016	   1096	   2196	  30308	   7664	ps2-msx-kb-convF1.elf


arm-none-eabi-size ps2-msx-kb-convF4.elf
   text	   data	    bss	    dec	    hex	filename
  35624	   1128	   8132	  44884	   af54	ps2-msx-kb-convF4.elf
```


## Hardware and Setup for Blue Pill:

You will obviously need a STM32F103C6T6 or a STM32F103C8T6 (F1 chip). I have used chinese blue pills. The software was made aiming in use of compatible processors, like GD32 for example. 
  
The software was made considering 8.000Mhz oscillator crystal, to clock the STM32 microcontroller chip at 72MHz. I tested with both STM32F103C6T6 and STM32F103C8T6.

The connections are:

1) PS/2 Keyboard:

- Clock Pin PA12 - Connect to PS/2 mini-din 45322 pin 5 throught R16;

- Data Pin PA9 - Connect to PS/2 mini-din pin 1 throught R18;

- +5V PS/2 power - Connect collector of Q2 to mini-din pin 4;

- GND - Connect GNDD (Digital Ground) to mini-din pin 3.



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


3) Serial console:
 It is the only option for console available if you are using Blue Pill. Vide ## Why not STM32F103C8T6 Blue Pill?

 The connection is needed only to update internal PS/2 to MSX key mapping Database. To create this Intel Hex file, better to use the Macro based Excel file, so you have to trust and enable macro excecution in excel app. 

  Config: 115200, 8, n, 1 (115200 bps, 8 bits, no parity, 1 stop bit);

  Tx: A2

  Rx: A3

  *******************************************************************************************************

  Obs.: It is a only 3.3V port, compatible to TTL levels. Do not use it with "1" level higher than 3.3V!!

  *******************************************************************************************************

 About physical connection: The connection has to be done with TX of your host system connected with RX of the PS/2 to MSX Keyboard Converter (device), and the device's TX has to be connected to host's RX.


## Hardware and Setup for Black Pill:

You will obviously need a a F4 chip (STM32F401CCU6 or up, if using WeAct Mini F4 compatible black pill).

The software was made considering 25.000Mhz oscillator crystal, to clock the STM32 microcontroller chip at 84MHz. I tested with both STM32F401CCU6 and STM32F401CDU6.

The connections are:


2) PS/2 Keyboard (J3- PS/2 Port) :

- PS/2 power - Pin 1 - Connect to PS/2 mini-din 45322 pin 4;

- Clock      - Pin 2 - Connect to PS/2 mini-din 45322 pin 5;

- Data       - Pin 3 - Connect to PS/2 mini-din 45322 pin 1;

- GNDD       - Pin 4 - Connect to PS/2 mini-din 45322 pin 3.


3) MSX computer:

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


3) USB Type C: Needed only to update internal PS/2 to MSX key mapping Database. To create this Intel Hex file, better to use the Macro based Excel file, so you have to trust and enable macro excecution in excel app. The USB cable is the same as you use with yor mobile phone (USB Type-A Male x USB Type C Male).

4) Serial console: Same observations of USB are applicable here, with exception of the cable itself. Here the connection has to be done with TX of your host system connected with RX of the PS/2 to MSX Keyboard Converter (device), and the device's TX has to be connected to host's RX.

  Config: 115200, 8, n, 1 (115200 bps, 8 bits, no parity, 1 stop bit);

  Tx: A9

  Rx: A10

  *******************************************************************************************************

  Obs.: It uses a 5V tolerant port, which is compatible to TTL levels. Do not use it with "1" level higher than 5V!!

  *******************************************************************************************************


## Technical detail of the Database structure

The structure of the Database is:

	There are 320 lines, so this structure is capable of manage up to 159 PS/2 keys with their respective make and break codes. The first and last lines are reserved for control (Database version, Database unavailable: seek next, double consistensy check, among others);

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
  
	7th and 8th columns share mappings   ".1" and ".2":

	                                     ".1 - NumLock Status+Shift changes";
  
	                                     ".2 - PS/2 Shift", where I need to release the sinalized Shift in PS/2 to the MSX and put the coded key, and so, release them, reapplying the Shift key, deppending on the initial state;
  
		
	Each column has a MSX coded key, with the follwing structure:
  
	(Bit 7:4) MSX Y PPI 8255 PC3:0 is send to an OC outputs BCD decoder, for example:
  
					 In the case of Hotbit HB8000, the keyboard scan is done as a 9 columns scan, CI-16 7445N 08 to 00;
           
					 If equals to 1111 (Y=15), there is no MSX key mapped.
           
	(Bit 3)	 0: keypress
  
					 1: key release;
           
	(Bit 2:0) MSX X, ie, which bit will carry the key, to be read by PPI 8255 PB7:0.
	

## Download your code to hardware

Use a ST-Link v2 Programmer (or similar), Black Magic Probe or another Serial Wire supported tool to flash the program using `make flash` onto the STM32.

If you choose a Black Pill, even you don't have plans to develop, you earn a bonus: you will not have the need of a dedicated programmer like ST-Link, J-link or Black Magic Probe to download your code, as STM32F4x1 MiniF4 already comes with DFU (Device Firmware Upgrade) available in the system ROM to do so through USB. So, on linux, just follow these steps:

1) Install dfu-util. This example is for Debian derivated Linux (Debian, Ubuntu, Mint, etc):
`sudo apt install dfu-util`

2) Make the .bin file, as dfu-util is not compatible with .elf: 
`arm-none-eabi-objcopy -Obinary tester-ps2-msxF4.elf tester-ps2-msxF4.bin`

3) Make sure the chip is at least 25°C (you may let it working for a while and help with the heat of your finger), because it uses internal oscillator to clock USB, factory trimmed to 25°C;

4) Plug the USB cable to your computer and the STM32F4 board while holding both NRST and BOOT0;

5) Then release BOOT0 AFTER 0.5 second you released NRST;

6) Now you can see a new USB device in your linux enviroment: 0483:df11. Run the command to flash the code itself:
`dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D tester-ps2-msxF4.bin`

7) Unplug the USB cable and power on the device to run the code.

On windows you can download STM32CubeProg on ST site, replacing step 1. You have to adjust step 6 to this tool. Please follow ST instructions.

