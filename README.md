# STM32: PS/2 to MSX Converter Tester

This code was made to support both Blue Pill and Black Pill and it is part of the PS/2 to MSX Converter Enviroment, all of them in my github page, that consists:
```
1)The PS/2 to MSX Converter itself, which contains:
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

So this firmware was developed to facilitate a Blue or Black Pill to act as a MSX keyboard sub system emulator, to test (and develop) the PS/2 to MSX keyboard Converter.

This firmware updated version is USB compatible and UART is running over DMA, so it uses almost 100% of the STM32F103C6T6 processing power and resources, and its html documentation is accessible by html/index.html file.

When there is no USB host, it works at legacy (using serial 2 in Blue Pill and Serial 1 in Black Pill as console port), but when its USB is enumerated, the console port is the first logical cdcacm port on host, and the second logical port is a serial<=>USB converter.

This code is common to the two adapters I made, both based in STM32 and fits to this chip. The flash used by this implementation fits with available amount:

# Preparations

After cloning the repository you need to make the following preparations:

Go to libopencm3 you cloned (eg: cd libopencm3) and make it to be useful by typing:

```
cd libopencm3
make TARGETS='stm32/f1 stm32/f4'
```

Go to your PS/2 to MSX Converter Tester project folder and assure that you choose the right target MCU in the system.h file line 62, and make, as follows:

With a text editor:
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
The created image compiled with an Arm GNU Toolchain 12.2.MPACBTI-Bet1 (Build arm-12-mpacbti.16)) 12.2.0 version on an aarch64 debian bullseye linux computer is created with the following characteristics, according to the choosen MCU:
```
arm-none-eabi-size ps2-msx-kb-convF4.elf
   text	   data	    bss	    dec	    hex	filename
  35676      60    8120   43856    ab50 ps2-msx-kb-convF4.elf

arm-none-eabi-size ps2-msx-kb-convF1.elf
   text	   data	    bss	    dec	    hex	filename
  26956      24    2176   29156    71e4 ps2-msx-kb-convF1.elf
```
 

***************  IMPORTANT NOTES **********************************************

1) The system is configurable through a 3.3V serial console, so DO NOT use RS-232 voltage levels or even 5V in STM32F103 here, as they are going to instantly damage your board!

2) 3.3V is compatible with standard TTL Levels.

*******************************************************************************


## Boot screen:
```
MSX keyboard subsystem emulator based on STM32F401CCU6 miniF4 (Black Pill v2.0+)
Serial number is XXXXXXXX
Firmware built on MM dd yyyy hh:mm:ss

This boot was requested from NRST_pin and PowerOn. Booting...

Configuring:
. Auxiliary message. Refer to following details.
- 5V compatible pin ports and interrupts to interface like a real MSX;
- High resolution timer2;
- SysTick;
- Ports config locking.

Boot complete! Press ? to show available options.

> 
```


Details about first configuring message:

=>If usb is enumerated, the message will be:

- USB has been enumerated => Console and UART are over USB.

=>If usb was not enumerated, the message will be:

- USB host not found => Using Console over UART. Now USB is disabled

=>If the device does not support usb, the message will be:

- Non USB version. Console is over USART.


## Options menu:
```
(?) Available options
1) General:
   r (Show Running config);
   c (Caps Lock line <- On/Off/Blink);
   k (Kana line      <- On/Off/Blink);
2) Scan related:
   s (Scan submenu - Set first [Y Begin] and last [Y End] colunms to scan);
   + (Increase scan rate);
   - (Decrease scan rate);
   p (Toggle pause scan);
   n (Next step colunm scan)                        <= when scan is paused;
   Space (One shot scan, from [Y Begin] to [Y End]) <= when scan is paused;
3) Times / Delays / Duties:`   a) Time to read X_Scan (after Y_Scan) update:
   < (decrease by 0.25μs);
   > (increase by 0.25μs);
   b) Read duty cycle: 1 work N idle. N may be 0 to maximum for speed:
   i (After one sweep active read cycle, configure number of idle cycles).
> 
```

## Dependencies

- `arm-none-eabi-gcc`
- `arm-none-eabi-gdb`
- `arm-none-eabi-binutils`
- `stlink` + `openocd (if you want to debug)`
- `libopenmcm3`

Obs.: If you plan to keep only one copy of LibopenCM3 in your computer, I really suggest you to create the variable OPENCM3_DIR in our system enviroment.

## Preparations

After cloning the repository you need to make the following preparations:

Go to libopencm3 you cloned (eg: cd libopencm3) and prepares it to use by typing:

```
make TARGETS='stm32/f1 stm32/f4'
```

Go to your PS/2 to MSX Converter Tester project folder and assure that you choose the right target MCU in the system.h file line 62.
```
#define MCU                       STM32F103
or
#define MCU                       STM32F401

make
```

## Hardware and Setup for Blue Pill:

You will obviously need a STM32F103C6T6 or a STM32F103C8T6 (F1 chip). I have used chinese blue pills. The software was made aiming in use of compatible processors, like GD32 for example. 
  
The software was made considering 8.000Mhz oscillator crystal, to clock the STM32 microcontroller chip at 72MHz. I tested with both STM32F103C6T6 and STM32F103C8T6.

The connections are:

1) Serial console:

  Config: 115200, 8, n, 1 (115200 bps, 8 bits, no parity, 1 stop bit);

  Tx: A2

  Rx: A3

  *******************************************************************************************************

  Obs.: It is a only 3.3V port, compatible to TTL levels. Do not use it with "1" level higher than 3.3V!!

  *******************************************************************************************************


2) To PS/2 to MSX Adapter: (Attention: Mappings are unique to the Blue Pill target plattform)

  - PB8  (X0) - Connect to /X0 pin of the adapter;

  - PB9  (X1) - Connect to /X1 pin of the adapter;

  - PB10 (X2) - Connect to /X2 pin of the adapter;

  - PB11 (X3) - Connect to /X3 pin of the adapter;

  - PB12 (X4) - Connect to /X4 pin of the adapter;

  - PB13 (X5) - Connect to /X5 pin of the adapter;

  - PB14 (X6) - Connect to /X6 pin of the adapter;

  - PB15 (X7) - Connect to /X7 pin of the adapter;

  - PA4  (Y0) - Connect to Y0 pin of the adapter;

  - PA5  (Y1) - Connect to Y1 pin of the adapter;

  - PA6  (Y2) - Connect to Y2 pin of the adapter;

  - PA7  (Y3) - Connect to Y3 pin of the adapter;

  - PB5 (CAPS)- Connect to /Caps pin of the adapter;

  - PB6 (KANA)- Connect to /Kana pin of the adapter; pull-up connection.



## Hardware and Setup for Black Pill:

You will obviously need a a F4 chip (STM32F401CCU6 or up, if using WeAct Mini F4 compatible black pill).

The software was made considering 25.000Mhz oscillator crystal, to clock the STM32 microcontroller chip at 84MHz. I tested with both STM32F401CCU6 and STM32F401CDU6.

The connections are:
  
1) Serial console:

  Config: 115200, 8, n, 1 (115200 bps, 8 bits, no parity, 1 stop bit);

  Tx: A9

  Rx: A10

  *******************************************************************************************************

  Obs.: It uses a 5V tolerant port, which is compatible to TTL levels. Do not use it with "1" level higher than 5V!!

  *******************************************************************************************************



2) To PS/2 to MSX Adapter: (Attention: Mappings are unique to the Black Pill target plattform)
  
  - PB12 (X0) - Connect to /X0 pin of the adapter;

  - PB10 (X1) - Connect to /X1 pin of the adapter;

  - PB10 (X2) - Connect to /X2 pin of the adapter;

  - PB13 (X3) - Connect to /X3 pin of the adapter;

  - PB14 (X4) - Connect to /X4 pin of the adapter;

  - PB1  (X5) - Connect to /X5 pin of the adapter;

  - PB15 (X6) - Connect to /X6 pin of the adapter;

  - PB0  (X7) - Connect to /X7 pin of the adapter;

  - PA8  (Y0) - Connect to Y0 pin of the adapter;

  - PA7  (Y1) - Connect to Y1 pin of the adapter;

  - PA6  (Y2) - Connect to Y2 pin of the adapter;

  - PA5  (Y3) - Connect to Y3 pin of the adapter;

  - PB4 (CAPS)- Connect to /Caps pin of the adapter;

  - PB6 (KANA)- Connect to /Kana pin of the adapter; pull-up connection.


## Hardware observations

As no PCB will be developed for this tester, I recommend the aquisition of black pill for this function.

If you are going to develop to ARM, I strongly suggest the use of Black Magic Probe. It is a wonderful tool that, if you can not spend USD 75,00 in buying the orginal to support the project, its openness brings trust, as you can debug the code by yourself, and allow you to use a lot of different targets to do the function. Today I should recommend to use a Black Pill to do the Black Magic Probe functionality, if you are not going to buy an original one, due to 128K flash limitations of STM32F103, as Black Magic Probe Project is evolving fast recently.


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

