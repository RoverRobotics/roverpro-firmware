Firmware
========

## Installation with bootloader

The rover includes an onboard bootloader to allow you to update the firmware.

1. Ensure you have  [booty bootloader client](https://pypi.org/project/booty/) installed on your computer, which can be installed with:

   ```bash
   pip3 install "booty>=0.3"
   ```

2. Download a PowerBoard hexfile from https://github.com/RoverRobotics/OpenRoverFirmware-dan/releases/latest and use booty to flash it onto the rover:

3. Connect the rover on serial port 1 to your computer using a [FTDI cable and Payload USB/Serial Breakout Board](https://roverrobotics.com/products/payload-usb-serial-breakout-board/).

4. Power on the robot.

5. Flash the firmware with booty

   ```bash
   booty --port COM3 --baudrate 57600 --hexfile "Downloads/PowerBoard-1.7.4.hex" --erase --load --verify
   ```

### Troubleshooting bootloader

 * Your port may vary. On my Windows computer it is `COM3`, on a Linux computer it may be `ttyUSB0` or something similar. Make sure you have the correct port selected, and the baud rate is 57600.
 * If booty says "device not responding", it means that either the rover has no booted or the rover is no longer in bootloader mode:
   * If the green serial board LED is **off**, the rover has not yet run its boot sequence. Make sure the battery has charge.
   * If the green serial board LED is **on**, the has run its boot sequence. The rover only remains in bootloader mode for 10 seconds after being powered on - if that time is elapsed, you need to reboot the robot. Hold down the red side power button for 5 seconds to restart the rover into bootloader mode then retry (note the light may or may not turn off immediately upon reboot).
 * The location of your hexfile may vary. If not found, booty will report "no such file or directory"

## Development

### IDE and build tools

I recommend using a CMake-aware IDE like CLion for development.

For debugging, use MPLAB 8. MPLAB X (5.30) has numerous bugs. Even its bugs have bugs.

### Building with cmake

1. Install XC16 the [Microchip XC16 Toolsuite](https://www.microchip.com/mplab/compilers). This toolsuite contains a compiler/linker/assembler and also standard libraries for the PIC24F MCU's. I recommend installing this to the path "C:/opt/Microchip/xc16", since the default (in "Program Files (x86)" contains spaces, which can cause some build tools to complain)
2. Install Ninja (CMake on Windows w/ Visual Studio generator does not like using other C compilers)
3. Generate build files with CMake

```
cd OpenRoverFirmware
mkdir build; cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE="..\cmake\PowerBoard_toolchain.cmake"
cmake --build .
```

### Building the docs

Everything should be ready for doxygen. (on Windows, `choco install doxygen.install graphviz`)

To build the docs, switch to the PowerBoard subfolder and run `doxygen`.

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
