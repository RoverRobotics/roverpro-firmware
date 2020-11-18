Firmware
========

## Release files

The latest release may be acquired from https://github.com/RoverRobotics/roverpro-firmware/releases/latest

The following files are included.

* `bootypic.hex` - Bootloader. This is responsible for bringing up the device and installing new firmware
* `PowerBoard.hex` - Power Board firmware. This is responsible for driving the motors, providing hardware feedback, and more. It is expected to run on top of the bootloader, but can handle the device on its own.
* `docs` - Documentation generated from the source code. This is mostly useful only for developers.

## Installing the bootloader

The bootloader image is called `bootypic.hex`. This must be installed with a developer tool like the PICKit 3. To quick install with the `PK3CMD` command line utility:

```cmd
set pk3cmd="C:\opt\Microchip\MPLAB IDE\Programmer Utilities\PICkit3\PK3CMD.exe"
set part=24fj256gb106
set oldhex="%USERPROFILE%\Downloads\backup.hex"
set newhex="%USERPROFILE%\Documents\roverpro-firmware\build\clion\bootypic\bootypic.hex"
%pk3cmd% -P%part% -GF%oldhex% -R%newhex%
```

## Installing firmware on top of the bootloader

Assuming the bootloader is installed, you don't need the PICKit to install the firmware. With a header board attached to the computer via USB:

```cmd
set %newhex%="%USERPROFILE%\Documents\roverpro-firmware\build\clion\PowerBoard\PowerBoard.hex"
python3 -m pip install --upgrade roverpro
pitstop flash %newhex%
```

### Bootloader expected behavior

The Power Board firmware is designed to run either alone or on top of the bootloader.

Depending on the reason the board was reset, the rover will run a different startup procedure.

 * The bootloader energizes the power bus.
 * If we restarted due to either a power on or error, and the rover has any firmware[^2], start the firmware.
 * Otherwise, the bootloader starts listening for commands.
    * If the bootloader receives a command[^1], it reads/writes/erases the requested data.
    * If no command is received for a period (1 second after a software reset, 10 seconds after a hardware reset) and the rover has any firmware[^2] the bootloader ends and firmware launches.

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
3. File -> Import ...; Choose the application hex file, e.g. `PowerBoard-1.11.1.hex`
4. File -> Import ...; Choose the bootloader hex file, e.g. `bootypic-1.11.1.hex`
5. File -> Export (The defaults should be correct: Program memory = 0 : 0x2abf6, Configuration bits checked, File Format Intel 32-bit Hex)
6. Name your new combo image, e.g. `combo-1.11.1.hex`

### Creating a combo image with HexMerge

1. Install the `intelhex` Python package, which contains `hexmerge.py`, a utility for combining hex files:
   
   ```shell
   python3 -m pip install intelhex --force --no-binary :all:
   ```
   
2. Now create a hex image of both the bootloader and the powerboard.[^1]
   
   ```shell
   app=PowerBoard-1.11.1.hex
   boot=bootypic-1.11.1.hex
   out=combined-1.11.1.hex
   hexmerge.py -o $out $boot::03 $app:04:01ff $boot:0400:1fff $app:2000:02ABF9 $boot:02ABFA:
   ```
   
[^1]:You probably noticed the addresses after the file name. These are necessary because the bootloader and the application have conflicting opinions about what should go where in parts of memory. These values may change with different versions of the firmware. [See below][addresses] for more.

### Installing a combo image

Install this image just as you would the Bootloader or the pre-bootloader firmware.

## Development

### IDE and build tools

I recommend using a CMake-aware IDE like CLion for development.

For debugging, use MPLAB 8. MPLAB X (5.30) has numerous bugs. Even its bugs have bugs.

### Building with CMake

1. Install XC16 the [Microchip XC16 Toolsuite](https://www.microchip.com/mplab/compilers). This toolsuite contains a compiler/linker/assembler and also standard libraries for the PIC24F MCU's. I recommend installing this to the path "C:/opt/Microchip/xc16", since the default (in "Program Files (x86)" contains spaces, which can cause some build tools to complain)
2. Generate build files with CMake
3. Build it!

```shell
cd roverpro-firmware
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="cmake/PowerBoard_toolchain.cmake"
cmake --build build
```

### IDE iteration

On Windows, you can attach a debugger to the powerboard in order to debug.

You can build in the MPLAB IDE using the project files in the MPLAB subfolder, though there may be some gotchas. Due to difficulties with XC16 and how MPLAB handles command line arguments, it is likely that the projects will build with the legacy libc and pic32 libraries. (in the `*.map` file this looks like `lega-pic30-elf` and `lega-c-elf` instead of `pic30-elf` and `c-elf`).

1. Install MPLAB IDE 8.92 and XC16 toolsuite, and plug in your PICKit
2. Open MPLAB/PowerBoard.mcp
3. Debugger -> Select Tool -> PicKit3
4. F10 (or Project -> Make). This will build and run the project.

Note either the Bootloader or PowerBoard can be run in this way (though the bootloader will crash when trying to run the app and the PowerBoard will skip the bootloader)

### Bootloader and Application addresses <span id='addresses'></span>

The bootloader and application must live on the same chip, so they coexist in the same address space. Note that the below addresses may change in different versions of the firmware.

| Addresses       | Meaning                              | Notes                                                        |
| --------------- | ------------------------------------ | ------------------------------------------------------------ |
| 000000:0x000003 | Reset vector; go to start of program | Both application and bootloader define this. **When both exist, the Bootloader should win** |
| 000004:0001ff   | Interrupt table                      | Reserved for application - by design the bootloader uses no interrupts |
| 000200:0003ff   | Program memory unused                | This is normal program memory, but due to how RTSP works on the PIC, erasing the interrupt table also erases this memory. So it's not used by the bootloader. |
| 000400:001fff   | Program memory - bootloader          |                                                              |
| 002000:02ABF9   | Program memory - application         |                                                              |
| 02ABFA:02ABFF   | PIC config words                     | Both application and bootloader define this. **They should be the same for application and bootloader.** |

To disassemble, you can use `xc16-objdump`. The beginning of the bootloader might look like:

```
xc16=/opt/microchip/xc16/v1.60/
objdump=$xc16/bin/xc16-objdump
$objdump build/clion/bootypic/bootypic.elf -d | head -n 15

build/clion/bootypic/bootypic.elf:     file format elf32-pic30

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
```

A couple things to note:

1. Instructions are aligned at addresses divisible by 2 and consist of 24 bytes.
2. Every even address has 2 bytes, and every odd address has 1 byte.
3. Reading hex can be a little weird. Numbers are little-endian, so the bytes `00 04` means 0x0400; i.e. 1024.

In the above code, we `goto __reset`. The `__reset` subroutine, defined in `libpic30`, initializes the heap, the stack, and runtime constants, then calls `main()`.

### Code style tools

To tidy up code, I like using **[clang-format](https://clang.llvm.org/docs/ClangFormat.html)**, and have provided a .clang-format file.

### Debugging

Load the .elf file into Microchip MPLAB X. (using the elf file instead of the hex will allow you to see debugging info)

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

Code tips for debuggability as of MPLAB v8.92:

* You can build the executable as either an ELF or COF (Project -> Build Options ... -> Project -> XC16 ASM/C Suite -> Output-File Format). ELF has the benefit that you can isolate each function in a section and remove unused sections to get a smaller build size. COF has the benefit that the debugger works more reliably and you can use the "Locate Headers" tool to find headers you've included not mentioned in the mcp file. If you cannot view local variables, try switching to COF.

* Typedef'd types should have a name anyway. It looks redundant, but the debugger will display the value as an enum instead of an int without that first "my_enum".

  ```C
  typedef my_enum1 {\*...*\} my_enum1; //< Don't do this
  typedef enum my_enum2 {\*...*\} my_enum2; //< Do this instead
  ```

* Similarly, if you have a magic size array, declare the size as a defined value, not a constant. Otherwise, the debugger will show the array as an opaque pointer instead of as an array.

  ```c
  static const SIZE = 10; //< Don't do this
  #define SIZE 10; //< Do this instead
  unsigned int my_array[SIZE];
  ```

### Deployment instructions with MPLAB

Given a released hex file, you can deploy to the robot power board with MPLAB instead of the PICKit3 standalone tool. Note this will erase the bootloader, so make sure to flash the "bootypic.hex" hex file after development is done.

1. File -> Import... -> (choose hex file)
2. Programmer -> Select Programmer -> PICKit 3
3. (Only needed if the target rover has no battery. If it has a power source, this step will fail) Programmer -> Settings -> Power tab -> Power Target circuit from PICKit3 -> OK
4. Programmer -> Program

## Power Board Firmware Development

The main robot firmware code is the Power Board. This is responsible for communicating with the motors / batteries / fans / serial port.

firmware.mcp = main project file. Open this with MPLab IDE v8, not MPLAB X
