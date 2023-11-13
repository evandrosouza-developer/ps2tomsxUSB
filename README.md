# STM32: PS/2 to MSX Keyboard Converter

# Introduction

First of all, to people who use or had used MSX, welcome, bem-vindos, welkom, bienvenidos, добро пожаловать, 환영하다 and ようこそ, to visit my repository.

Instead of an application to this old japanese home computer standards is not so widely useful in nowadays, I made this to learn STM32 hardware and software technics, and I'd like it to be useful and let us back in time and make our MSX computer jewlers work with joyful to our pleasure.

The MSX Keyboard enviroment components (PS/2 to MSX Keyboard Converter, MSX Keyboard subsystem Emilator and the modifyed TIO - A simple terminal) are fully functional and debugged and here I used a lot of concepts that, IMHO could be used as idea sources to your applications.  

# Lets do it!

This interface has the function of connect a PS/2 keyboard (or a USB one in compatibility mode) as source to TTL zero (low state) active matrix based computers, like MSX and ZX-Spectrum as destination. It is meant to connect a keyboard, which only provides a PS/2 connector, to a MSX (or any one that have a up to 15 active columns and reads zeroes througth 8 bits - up to 15 x 8 matrix) computer.  

The differencial of this design is to allow user to customize and do a hot update of the database layout of this PS/2 Keyboard, as like as the MSX one, through an USB (or async serial) interface and a tty terminal on host side.  

To edit the Database file, both source and target keyboard layouts, I prepaired a dedicated "key assembler in excel", so I can boldly recommend you to use the excel file `PS2toMSX_Database_Compiler.xlsm` available at github page. The excel file exports a IHD.hex (Intel Hex Database) to be hot applyed to PS/2 to MSX Keyboard Converter when it boots without PS/2 Keyboard.  

************************** IMPORTANT ******************************  
If you are using Blackpill board and are uploading Database through USB, 
please do a FULL DISCONNECT (ALL lines INCUDING POWER) from MSX,
to avoid short circuit through power supply lines!  
It would be better to do this with Black Pill pulled out from PS/2 to MSX Keyboard Converter board.  
*******************************************************************  

The default database mappings for the keyboard layouts are:

- Source (PS/2 keyboard): 275 (ABNT2 layout - Brazilian Technical Norms Association);

- Target: Brazilian Sharp/Epcom Hotbit HB8000 MSX.  

This firmware was made to support both Blue Pill and Black Pill and it is part of the PS/2 to MSX Converter Enviroment, all of them in my github page, that consists:
```
1)The PS/2 to MSX Keyboard Converter itself, which contains:
1.1) The firmware with source files;
1.2) Doxygen full firmware documentation;
1.3) Schematics and PCB design:
1.3.1) Electronics part Schematics with Kicad files;
1.3.2) Single sided PCB layout with Kicad files and complete set of Gerber files;
1.3.3) The performance tests of this one are inside the PS/2 to MSX Converter Tester folder.

2)The PS/2 to MSX Converter Tester, which contains:
2.1) The firmware with source files;
2.2) Doxygen full firmware documentation;
2.3) Schematics design with Kicad files;
2.4) The performance tests of the PS/2 to MSX Keyboard Converter are located here.

3) Tool to create/modify the Database (Translation tables to map from PS/2 Scan Codes to MSX Matrix Codes)
in excel, but it has compatible macros to be executed by Libre Office, Open Office and so on.

4) Tini TTY I/O - tio (Linux app with source files) to communicate with console and easily allowing the
user to use it with different keyboard layouts and languages.
```
In case of STM32F103C8T6 (Flash 64K RAM 20K) Blue Pill, you have to aplly STM32F103C6T6 (Flash 32K RAM 10K) without change anything else.

In case of Black Pill, the base system is STM32F401CCU6 so, to use a more memory one, just follow the same instructions here.

This firmware powers and has been tested on the following Windows keyboards, with brazilian 275 layout:  
- Compaq RT235BTWBR;  
- Clone KE11090749;  
- Clone #09100.


The original code was originally developed based on:  
- Cherry G80-3000LSMDE.


# Why not STM32F103C8T6 Blue Pill?

Because Blue Pill has no 5V tolerant pins available to connect PS/2 + MSX + Serial + USB, unlike Black Pill. Even serial (UART) was feasibled using a 3.3V only port.  
Although, this limitation came with a bonus: The need of a cheaper and greater availability 32KB flash MCU: STM32F103C6T6  


# Dependencies

- `libopencm3`
- `arm-none-eabi-gcc`
- `arm-none-eabi-binutils`

If you plan to debug:
- `arm-none-eabi-gdb`
- `stlink + openocd`

Obs.: If you plan to keep only one copy of LibopenCM3 in your computer, I strongly suggest you to setup the variable OPENCM3_DIR in our system enviroment.

# Preparations

After cloning the repository you need to make the following procedure:

Go to libopencm3 you cloned (eg: cd libopencm3) and make it to be useful by typing:

```
cd libopencm3
make TARGETS='stm32/f1 stm32/f4'
```

Go to your PS/2 to MSX Converter Tester project folder and assure that you choose the right target MCU in the system.h file line 69, and make, as follows:

With a plain text editor:
```
#define MCU                       STM32F401
or
#define MCU                       STM32F103
```

Save the file, and,  
Create the image file to be sent to the MCU (Micro Controller Unit):

```
make
```
The created image compiled with the Arm GNU Toolchain 12.3.1 20230626 (Arm GNU Toolchain 12.3.Rel1 (Build arm-12.35)) on an aarch64 debian bullseye linux computer is created with the following characteristics, according to the choosen MCU:
```
arm-none-eabi-size ps2-msx-kb-convF4.elf
   text	   data	    bss	    dec	    hex	filename
  35592	     60	   8120	  43772	   aafc	ps2-msx-kb-convF4.elf

arm-none-eabi-size ps2-msx-kb-convF1.elf
   text	   data	    bss	    dec	    hex	filename
  26900	     24	   2176	  29100	   71ac	ps2-msx-kb-convF1.elf

```


# Hardware and Setup

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
- Kana LED - Connect to MSX YM2149 or AY3-8910 IOB7, pin 6 of DIP package. If the MSX doesn't have this indicator LED, you can leave it open, as it already has an internal pull-up connection. In brazilian, argentinian and most european MSX, for example, this is a non-connect pin.   

3) Serial console:
 It is the only option for console available if you are using Blue Pill. See ## Why not STM32F103C8T6 Blue Pill?

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
- Kana LED - Connect to MSX YM2149 or AY3-8910 IOB7, pin 6 of DIP package. If the MSX doesn't have this indicator LED, you can leave it open, as it already has an internal pull-up connection. In brazilian, argentinian and most european MSX, for example, this is a non-connect pin.   
  
3) USB Type C: Needed only to update internal PS/2 to MSX key mapping Database. To create this Intel Hex file, better to use the Macro based Excel file, so you have to trust and enable macro excecution in excel app. The USB cable is the same as you use with yor mobile phone (USB Type-A Male x USB Type C Male).  
  
4) Serial console: Same observations of USB are applicable here, with exception of the cable itself. Here the connection has to be done with TX of your host system connected with RX of the PS/2 to MSX Keyboard Converter (device), and the device's TX has to be connected to host's RX.  
  
  Config: 115200, 8, n, 1 (115200 bps, 8 bits, no parity, 1 stop bit);

  Tx: A9  
  Rx: A10  

  *******************************************************************************************************

  Obs.: It uses a 5V tolerant port, which is compatible to TTL levels. Do not use it with "1" level higher than 5V!!

  *******************************************************************************************************  


# The Database structure: Keyboard from/to mapping

This design was developed to connect, as default, an ABNT-2 brazilian PC keyboard to the brazilian MSX Sharp/Epcom HB-8000, but it is very easy to apply a hot change of its mapping:

- You have to compile and upload this new database to the Converter. This task is easily managed with the help of a Database compiler: The excel file named `PS2toMSX_Database_Compiler.xlsm`. This file has 3 sheets: One for HB-8000, the second as XP-800 and the third one as an International MSX. Unfortunately, the last one was not tested.  


## Technical details about the Database structure

	There are 320 lines, so this structure is capable of manage up to 159 PS/2 keys with their respective make and break codes.

	The first and last lines are reserved for control (Database version, Database unavailable: seek next, double consistensy check, among others);
	
	On the first line there are information related to this Database version:
	- Bytes 0, 1 and 2: Database version ("0x01", "0x00", "0xFF")
	- Byte 3: Mapped function. Description:  
	  |  (bit 7-6): Reserved - keep it at high state;  
	  |  (bit 5): default value of enable_xon_xoff;  
	  |  (bit 4): default value of ps2numlockstate at power up;  
	  |  (bits 3-0): y_dummy (non valid colunm);
	- Bytes 4-7: Reserved - Keep it as 0xFFFFFFFF;
        When this copy of the Database is let unavailable, the first line becomes ("0x00", "0x00", "0x00", "0x00", "0x01", "0x02", 
	"0x04", "0x08"), meaning to seek the next one.

	On the line 319 (the last one) there are information related to this Database integrity:  
	- Bytes 0-5: Reserved - Keep each byte as 0xFF;  
	- Byte 6: CheckSum (Integrity);  
	- Byte 7: bcc (a type of vertical parity used for integrity);  

	Starting on line 1, the raw of the Database:  
  
	The three first columns of each line are the mapped scan codes;  
  
	The 4th column is The Control Byte, detailed bellow:   
	CONTROL BYTE:  
	- High nibble is Reserved;  
	- (bit 3) Combined Shift;  
	- (bit 2) Reserved-Not used;  
	- (bits 1-0) Modifyer Type:  
	  |		0 - Default mapping  
	  |		1 - NumLock Status+Shift changes  
	  |		2 - PS/2 Shift  
	  |		3 - Reserved-Not used  

	This table has 3 modifyers: Up two MSX keys are considered to each mapping behavior modifying:
	- 5th and 6th columns have the mapping 0 - "Default mapping";  
	- 7th and 8th columns share mappings:  1 and 2:  
	  |                                    1 - "NumLock Status+Shift changes";  
	  |                                    2 - "PS/2 Shift", where I need to release the sinalized Shift in PS/2 to the MSX and put the code key, and so, release them, reapplying the Shift key, deppending on the initial state;  

	Each column has a MSX encoded key, with the following structure:
	- (bit 7:4) MSX Y PPI 8255 PC3:0 is sent to a BCD decoder with OC outputs, for example:  
	  |	In the case of Hotbit HB8000, the keyboard scan is done as a 9 columns scan, CI-16 7445N 08 to 00;  
	  |	If equals to 1111 (Y=15), there is no MSX key mapped.
	- (bit 3) => 0: keypress / 1: key release;             
	- (bit 2:0) MSX X, ie, which bit of the PPI 8255 PB7:0 will carry information of bit 3 (keypress/key release).  
	

# Download your code to hardware

Use a ST-Link v2 Programmer (or similar), Black Magic Probe or another Serial Wire supported tool to flash the program using `make flash` onto the STM32.

If you choose a Black Pill, even you don't have plans to develop, you earn a bonus: you will not have the need of a dedicated programmer like ST-Link, J-link or Black Magic Probe to download your code, as STM32F4x1 MiniF4 already comes with DFU (Device Firmware Upgrade) available in the system ROM to do so through USB. So, on linux, just follow these steps:

1) Unplug the MSX connection or take off the Black Pill from the PS/2 to MSX Converter Main Board;  
2) Install dfu-util. This example is for Debian derivated Linux (Debian, Ubuntu, Mint, etc):
`sudo apt install dfu-util`  
3) Make sure the chip is at least 25°C (you may let it working for a while and help with the heat of your finger), because it uses internal oscillator to clock USB, factory trimmed to 25°C;  
4) Plug the USB cable to your computer and the STM32F4 board while holding both NRST and BOOT0;  
5) Then release BOOT0 AFTER 0.5 second you released NRST;  
6) Now you can see a new USB device in your linux enviroment, through `lsusb` command: 0483:df11. Run the command to flash the code itself:
`dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D ps2-msx-kb-convF4.bin`  
7) Unplug the USB cable, undo the step 1 and power on the device to run the code.  
  
On windows you can download STM32CubeProg on ST site, replacing step 2. You have to adjust step 6 to this tool. Please follow ST instructions.

