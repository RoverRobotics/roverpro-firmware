Firmware
========

[TOC]

## Release files

The latest release may be acquired from https://github.com/RoverRobotics/roverpro-firmware/releases/latest

The following files are most important

* `bootypic.hex` - Bootloader. This is responsible for bringing up the device and installing new firmware
* `PowerBoard.hex` - Power Board firmware. This is responsible for driving the motors, providing hardware feedback, and more. It is expected to run on top of the bootloader, but can handle the device on its own.
* `docs/` - Documentation generated from the source code.
  * `index.html` is the main entry point for the documentation

A [HEX file](https://en.wikipedia.org/wiki/Intel_HEX) is the basic binary file format for compiled programs on a PIC. Note that a HEX file does not necessarily cover the entire program space - the bootloader and power board firmware have minimal overlap.

The released files also contain [ELF files](https://en.wikipedia.org/wiki/Executable_and_Linkable_Format). These have program data and potentially useful debugging info, but the tools for programming a rover don't directly understand this format, requiring the file first be converted to a HEX.

## Installing the bootloader

Part of the firmware is a bootloader, based on the [booty project](https://booty.readthedocs.io/en/latest/index.html). This is responsible for minimal robot operation (bringing up the power bus to power an onboard computer) and rewriting the firmware on the robot.

The bootloader image is called `bootypic.hex`. This must be installed with a developer tool like the [PICKit 3 In-Circuit Debugger](https://www.microchip.com/DevelopmentTools/ProductDetails/PG164130), as the bootloader cannot overwrite itself.

To quick install with the `PK3CMD` command line utility (Windows only):

```cmd
set pk3cmd="C:\opt\Microchip\MPLAB IDE\Programmer Utilities\PICkit3\PK3CMD.exe"
set part=24fj256gb106
set oldhex="%USERPROFILE%\Downloads\backup.hex"
set newhex="%USERPROFILE%\Documents\roverpro-firmware\build\clion\bootypic\bootypic.hex"
%pk3cmd% -P%part% -GF%oldhex% -R%newhex%
```

## Using the bootloader to install firmware

With the bootloader installed, you don't need the PICKit to install the firmware. With a header board attached to the computer via USB:

```cmd
set %newhex%="%USERPROFILE%\Downloads\install\PowerBoard.hex"
python3 -m pip install --upgrade roverpro
pitstop flash %newhex%
```

### Bootloader expected behavior

The Power Board firmware is designed to run either alone or on top of the bootloader.

Depending on the reason the board was reset, the rover will run a different startup procedure.

 * The bootloader energizes the power bus.
 * If we restarted due to either a power on or error, and the rover has any firmware[^1], start the firmware.
 * Otherwise, the bootloader starts listening for commands.
    * If the bootloader receives a command[^2], it reads/writes/erases the requested data.
    * If no command is received for a period (1 second after a software reset, 10 seconds after a hardware reset) and the rover has any firmware[^5] the bootloader ends and firmware launches.

If the firmware is working, a flash request from `pitstop` will issue a software reset request over UART.

If the firmware is corrupt or nonresponsive, perform a hardware reset by shorting the RESET pins (P6) on the interface board. The bootloader will then remain active for an extended period of time (10s), during which you can flash it with `pitstop`.

[^1]: Note that the bootloader uses a different communication protocol from the main powerboard firmware.
[^2]: The check for firmware  is very cursory. If a rover has a partial firmware image due to being interrupted during bootload, it may still attempt to boot into that firmware.

### Troubleshooting bootloader

 * If booty says "device not responding", it means that either the rover has no booted or the rover is no longer in bootloader mode:
   * If the green serial board LED is **off**, the rover has not yet run its boot sequence. Make sure the battery has charge.
   * If the green serial board LED is **on**, the has run its boot sequence. The rover only remains in bootloader mode for 10 seconds after being powered on - if that time is elapsed, you need to reboot the robot. Hold down the red side power button for 5 seconds to restart the rover into bootloader mode then retry (note the light may or may not turn off immediately upon reboot).
 * The location of your hexfile may vary. If not found, booty will report "no such file or directory"

## Creating a combo image

The bootloader and the PowerBoard firmware are distributed as separate hex files but you can create a combined image so that both can be installed at once via the PicKit.

Download hex files for both the bootloader and the PowerBoard. From here on in, I assume you have both files local.

Note that the bootloader and the powerboard contain some conflicting data. Usually the bootloader protects its own data and ignores conflicting values when flashing new firmware, but since we're not running the bootloader, we must take care that the values from the bootloader are used:

* The reset vector in the beginning (so if installed without a bootloader, it starts the firmware directly)
* The configuration bits at the end (which affect stuff like code protection and processor speed; these are actually the same between the bootloader and the application).

You can create a combo image in several ways. Below are instructions via MPLAB 8.

### Creating a combo image with MPLAB 8

1. configure -> Settings -> Program Loading
2. Uncheck all the checkboxes (Clear program memory upon loading a program, Clear configuration bits upon loading a program, ...)
3. File -> Import ...; Choose the application hex file, e.g. `PowerBoard-1.11.2.hex`
4. File -> Import ...; Choose the bootloader hex file, e.g. `bootypic-1.11.2.hex`
5. File -> Export (The defaults should be correct: Program memory = 0 : 0x2abf6, Configuration bits checked, File Format Intel 32-bit Hex)
6. Name your new combo image, e.g. `combo-1.11.2.hex`

### Creating a combo image with HexMerge

1. Install the `intelhex` Python package, which contains `hexmerge.py`, a utility for combining hex files:
   
   ```shell
   python3 -m pip install intelhex --force --no-binary :all:
   ```
   
2. Now create a hex image of both the bootloader and the powerboard.[^3]
   
   ```shell
   app=PowerBoard-1.11.1.hex
   boot=bootypic-1.11.1.hex
   out=combined-1.11.1.hex
   hexmerge.py -o $out $boot::7 $app:8:3ff $boot:800:3fff $app:4000:557F3 $boot:557F4:
   ```
   
[^3]:You probably noticed the addresses after the file name. These are necessary because the bootloader and the application have conflicting opinions about what should go where in parts of memory. These values may change with different versions of the firmware. [See below][addresses] for more.

### Installing a combo image

Install this image just as you would the bootloader or the pre-bootloader firmware.

## Development

### IDE and build tools

#### XC16

The [Microchip XC16 Toolsuite](https://www.microchip.com/mplab/compilers) is the compiler I use for all firmware builds. This toolsuite contains a compiler/linker/assembler and also standard libraries for the PIC24F MCU's. I recommend installing this to the path "C:/opt/Microchip/xc16", since the default (in "Program Files (x86)") contains spaces, which can cause `make` and other build tools to complain.

Prior to 1.60, the compiler had some issues with the `--verbose` flag, which caused problems with CMake, so I recommend using v1.60.


#### MPLab 8.92

This IDE is Windows-only but is pretty good for debugging the PIC. Download from here:

http://ww1.microchip.com/downloads/en/DeviceDoc/MPLAB_IDE_8_92.zip


* To allow loading multiple projects in the same IDE window: Configure -> Settings -> Projects -> Use one-to-one project-workspace model  (uncheck).

* In the Watch window, expand SFR bitfields: (right click on Watch window) -> Properties -> Preferences -> Expand SFR Bitfields

* Note that the output type should be `elf`: (right click project) -> Build Options -> XC16 ASM/C Suite -> Output File Format -> ELF/DWARF.
  ELF has the benefit that you can isolate each function in a section and remove unused sections to get a smaller build size.

* You may need to tell MPLab where to find the compiler: Project -> Select Language Toolchain -> Microchip XC16 Toolsuite


Code tips for debuggability as of MPLAB v8.92:

* Turn off `-ffunction-sections`: (right click project) -> XC16 C -> Isolate each function in a section (uncheck).
  This prevents MPLab from properly showing the call stack and locals.

* If you have an array with the length as a `const`, MPLAB will be unable to infer the size of the array. The debugger will only show the first element of the array by default.
  Instead use a `#define` or `typedef`.

  ```c
  const CSIZE = 3;
  int bad_ar1[CSIZE]; // < Debugger will have trouble
  typedef int ArType[CSIZE];
  ArType bad_ar2;     // < Debugger will have trouble with this too.

  int ok_ar1[3];     // < This will work
  #define DSIZE 3
  int ok_ar2[DSIZE]; // < This will work too
  typedef int ArType[DSIZE];
  ArType ok_ar3;     // < So will this
  ```

#### ~~MPLAB X~~

USE AT YOUR OWN RISK.

MPLAB X (5.30) has numerous bugs. Even its bugs have bugs.

Also, using MPLAB with PICKit 3 installs a firmware onto the PICKit that has trouble flashing a PIC and is difficult to uninstall.
If you have done this, use the [PICKit 3 tools](http://ww1.microchip.com/downloads/en/DeviceDoc/PICkit3%20Programmer%20Application%20v3.10.zip) to reset the PICKit's firmware.

#### CLion

I recommend using a CMake-aware IDE like CLion for development.

For debugging, use MPLAB 8. 

#### CMake



### Building with CMake

1. Ensure you have XC16
2. Generate build files with CMake with the toolchain file `cmake/PowerBoard_toolchain.cmake`
3. Build it!

```shell
cd roverpro-firmware
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="cmake/PowerBoard_toolchain.cmake"
cmake --build build
```

### IDE iteration

On Windows, you can attach a debugger to the powerboard in order to debug.

MPLAB Project files can be found in the `MPLAB` directory.

You can build in the MPLAB IDE using the project files (\*.mcp) in the MPLAB subfolder, though there may be some gotchas. Due to difficulties with XC16 and how MPLAB handles command line arguments, it is likely that the projects will build with the legacy libc and pic32 libraries. (in the `*.map` file this looks like `lega-pic30-elf` and `lega-c-elf` instead of `pic30-elf` and `c-elf`).

1. Install MPLAB IDE 8.92 and XC16 toolsuite, and plug in your PICKit
2. Open MPLAB/PowerBoard.mcp
3. Debugger -> Select Tool -> PicKit3
4. F10 (or Project -> Make). This will build and run the project.

Note either the Bootloader or PowerBoard can be run in this way (though the bootloader will crash when trying to run the app and the PowerBoard will skip the bootloader)

### Addresses <span id='addresses'></span>

The bootloader and application must live on the same chip, so they coexist in the same address space. Note that the below addresses may change in different versions of the firmware.

Note that the below addresses are expressed in 2 ways. The **PC address** counts double-byte, and is half-open (end address is not *in* the range), as used by XC16 and Microchip. The **HEX address** is a byte-oriented range and is closed (end address is part of the range), as used by intelhex files.

| PC Address  | HEX Address | Meaning                             | Notes                                                        |
| ----------- | ----------- | ----------------------------------- | ------------------------------------------------------------ |
| 0:4         | 0:7         | Reset vector; goto start of program | Both application and bootloader define this. **When both exist, the Bootloader should win** |
| 4:200       | 8:3FF       | Interrupt table                     | Reserved for application - by design the bootloader uses no interrupts |
| 200:400     | 400:7FF     | Program memory unused               | This is normal program memory, but due to how RTSP works on the PIC, erasing the interrupt table also erases this memory. So it's not used by the bootloader. |
| 400:2000    | 800:3FFF    | Program memory - bootloader         |                                                              |
| 2000:2ABFA  | 4000:557F3  | Program memory - application        |                                                              |
| 2ABFA:2ABFE | 557F4:557FB | PIC config words                    | Both application and bootloader define this. **They should be the same for application and bootloader.** |

### Addressing gotchas

To disassemble, you can use `xc16-objdump`. The beginning of the bootloader might look like:

```shell
xc16=/opt/microchip/xc16/v1.60/
objdump=$xc16/bin/xc16-objdump
$objdump build/clion/bootypic/bootypic.elf -d | less
```

These are two different representations of the same program section:

```
C:\Users\dan\Documents\RoverPro-firmware\build\bootypic\bootypic.elf:     file format elf32-pic30

Disassembly of section .reset:

00000000 <.reset>:
   0:	00 04 04    	goto      0x400 <__reset>
   2:	00 00 00 
Disassembly of section .text:

00000400 <__reset>:
 400:	6f 26 21    	mov.w     #0x1266, w15
 402:	0e 7f 24    	mov.w     #0x47f0, w14
 404:	0e 01 88    	mov.w     w14, 0x20
 406:	00 00 00    	nop       
 408:	00 00 20    	mov.w     #0x0, w0
 40a:	00 00 e0    	cp0.w     w0
 40c:	02 00 32    	bra       Z, 0x412 <CORCON_RESET>
 40e:	00 01 20    	mov.w     #0x10, w0
 410:	20 02 88    	mov.w     w0, 0x44
 
 00000412 <CORCON_RESET>:
 412:	14 00 07    	rcall     0x43c <__psv_init>
 414:	59 00 07    	rcall     0x4c8 <__crt_start_mode> <__crt_start_mode_normal>
 416:	00 00 e0    	cp0.w     w0
```

```ihex
:020000040000fa
:080000000004040000000000f0
:020000040000fa
:100800006f2621000e7f24000e01880000000000ea
:10081000000020000000e000020032000001200083
:100820002002880014000700590007000000e000c3
```

Reading hex can be a little weird. Numbers are *little-endian*, so on the first line, the bytes `00 04` means 0x0400; i.e. 1024.

In PIC24F, the memory addressing scheme is a little wonky since data is organized into 24-bit words but the architecture likes dealing in 16-bit ints.

Typically, Microchip tools will show 2-word pairs starting at even addresses to adjust for this. Things get a little hairy when accounting for PSV data in memory (where even addresses refer to the low byte, odd refer to the middle byte, and the high byte is left unused).

<table>
<thead>
<tr><th>Data</th><th>byte</th><th>byte</th><th>byte</th><th>0</th><th>byte</th><th>byte</th><th>byte</th><th>0</th></tr></thead>
<tbody><tr><td>PC address</td><td colspan=3>n</td><td>(no address)</td><td colspan=3>n+2</td><td>(no address)</td></tr><tr><td>PSV address</td><td>n</td><td>n+1</td><td colspan=2>(no address)</td><td>n+2</td><td>n+3</td><td colspan=2>(no address)</td></tr><tr><td>Intel Hex address</td><td>2n</td><td>2n+1</td><td>2n+2</td><td>2n+3</td><td>2n+4</td><td>2n+5</td><td>2n+6</td><td>2n+7</td></tr></tbody>
</table>

Note that PSV and Intel hex addressing take opposite approaches; PSV leaves 1/3 of the physical memory unusable in return for having a fully usable address space, whereas Intel Hex leaves 1/4 of the address space unusable in return for having a fully addressable physical memory.

Also take care that addresses in Intel Hex format are *twice* those in, e.g. the linker script and source code and the HEX file contains extra bytes (set to zero) in these addresses.


### Code style tools

To tidy up code, I like using **[clang-format](https://clang.llvm.org/docs/ClangFormat.html)**, and have provided a .clang-format file.

#### Ubuntu installation of clang-format

```bash
sudo apt install clang-format
```
#### Windows installation of clang-format and git

```batch
choco install git llvm
```

Before committing, please run the following to clang-format all uncommitted C files:

```bash
git diff HEAD --name-only --relative --diff-filter=d -- **.h **.c | xargs clang-format -style=file -i
```
```batch
set PATH=C:\Program Files\Git\usr\bin;%PATH%
git diff HEAD --name-only --relative --diff-filter=d -- **.h **.c | xargs clang-format -style=file -i
```

This will tidy up all locally modified files. You can make this a precommit hook if you like.


### Deployment instructions with MPLAB

Given a released hex file, you can deploy to the robot power board with MPLAB instead of the PICKit3 standalone tool. Note this will erase the bootloader, so make sure to flash the "bootypic.hex" hex file after development is done.

1. File -> Import... -> (choose hex file)
2. Programmer -> Select Programmer -> PICKit 3
3. (Only needed if the target rover has no battery. If it has a power source, this step will fail) Programmer -> Settings -> Power tab -> Power Target circuit from PICKit3 -> OK
4. Programmer -> Program
