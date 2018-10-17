firmware
========

##Installation

Released hex files can be found at https://github.com/RoverRobotics/OpenRoverFirmware-dan/releases

Given a hex file, deploy to the robot power board with [MPLAB IDE 8.9.2](https://www.microchip.com/development-tools/pic-and-dspic-downloads-archive) using a PICKit 3.

1. File -> Import... -> (choose hex file)
2. Programmer -> Select Programmer -> PICKit 3
3. (Only needed if the target rover has no battery) Programmer -> Settings -> Power tab -> Power Target circuit from PICKit3 -> OK
4. Programmer -> Program

## Development

To build, use the Debug mode (if you're attaching a PICKit) or Release mode (if you're using this with other things). Note that if you build in Debug mode and you hit a breakpoint (`BREAKPOINT()` macro), execution will halt and wait for the debugger. If no debugger is attached, the [Watchdog Timer](http://ww1.microchip.com/downloads/en/devicedoc/39697a.pdf) will restart the device.

To tidy up code, I like using **[clang-format 6](http://releases.llvm.org/6.0.1/tools/clang/docs/ClangFormat.html)**, and have provided a .clang-format file. Clang 6 is currently the latest release for Ubuntu, but feel free to use newer. Ubuntu installation:

```bash
sudo apt install clang-format-6.0
sudo ln -s `which clang-format-6.0` /usr/local/bin/clang-format
```
Before committing, please run:

```bash
git diff --name-only --cached --relative --diff-filter=d -- '*.h' '*.c' | xargs clang-format -style=file -i
```
This will tidy up all locally modified files. You can make this a precommit hook if you like.

The MCP files can be opened in [MPLAB IDE](https://www.microchip.com/development-tools/pic-and-dspic-downloads-archive) (not MPLAB X). Code tips for debuggability as of MPLAB v8.92:

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

