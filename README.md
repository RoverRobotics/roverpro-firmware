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

### Troubleshooting the bootloader

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
5. File -> Export (The defaults should be correct: Program memory = `0 : 0x2abf6`, Configuration bits checked, File Format Intel 32-bit Hex)
6. Name your new combo image, e.g. `combo-1.11.1.hex`

### Creating a combo image with HexMerge

1. Install the `intelhex` Python package, which contains `hexmerge.py`, a utility for combining hex files:
   
   ```
   python3 -m pip install intelhex --force --no-binary :all:
   ```
   
2. Now create a hex image of both the bootloader and the powerboard:
   
   ```
   hexmerge.py PowerBoard-1.11.1.hex bootypic-1.11.1.hex -o combo-1.11.1.hex --overlap=replace
   ```

### Installing a combo image

Install this image just as you would the bootloader or the pre-bootloader firmware.

## Development

### IDE and build tools

I recommend using a CMake-aware IDE like [CLion](https://www.jetbrains.com/clion/download) for development.

Get [MPLAB XC16 Compiler v1.60](https://www.microchip.com/en-us/development-tools-tools-and-software/mplab-xc-compilers). Other versions may work, but v1.60 fixes major interoperability issues with CMake. On Windows, I recommend installing this to the path "C:/opt/Microchip/xc16", since the default (in "Program Files (x86)" contains spaces, which can cause `make` (a build tool used by MPLab) to complain.

While not the best for active development, for interactive debugging, use MPLAB 8 (Windows only). Don't use MPLAB X. MPLAB X (5.30) has numerous bugs. Even its bugs have bugs.

### Building with CMake

1. Ensure you have XC16
2. Generate build files with CMake with the toolchain file `cmake/PowerBoard_toolchain.cmake`
3. Build it!

```shell
cd roverpro-firmware
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="cmake/PowerBoard_toolchain.cmake" 
cmake --build build
```

### Interactive debugging

On Windows, you can attach a debugger to the powerboard in order to debug.

MPLAB Project files can be found in the `MPLAB` directory.

You can build in the MPLAB IDE using the project files (\*.mcp) in the MPLAB subfolder, though there may be some gotchas. Due to difficulties with XC16 and how MPLAB handles command line arguments, it is likely that the projects will build with the legacy libc and pic32 libraries. (in the `*.map` file this looks like `lega-pic30-elf` and `lega-c-elf` instead of `pic30-elf` and `c-elf`).

1. Install MPLAB IDE 8.92 and XC16 toolsuite, and plug in your PICKit
2. Open MPLAB/PowerBoard.mcp
3. Debugger -> Select Tool -> PicKit3
4. F10 (or Project -> Make). This will build and run the project.

Note either the Bootloader or PowerBoard can be run in this way (though the bootloader will crash when trying to run the app and the PowerBoard will skip the bootloader)

### Continuous Integration

This [project uses Github Actions](https://github.com/RoverRobotics/roverpro-firmware/actions) to build every commit. Whenever you push code, it attempts to build it and makes sure `pitstop` can load the PowerBoard firmware. Note that this uses the bootloader already on the robot --- changes to the bootloader aren't immediately reflected in CI.

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

Code tips for debuggability as of MPLAB v8.92:

* You can build the executable as either an ELF or COF (Project -> Build Options ... -> Project -> XC16 ASM/C Suite -> Output-File Format). ELF has the benefit that you can isolate each function in a section and remove unused sections to get a smaller build size. COF has the benefit that the debugger works more reliably and you can use the "Locate Headers" tool to find headers you've included not mentioned in the mcp file. If you cannot view local variables, try switching to COF.

* The xc16 elf option `--gc-sections`  may interfere with debugging.

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
