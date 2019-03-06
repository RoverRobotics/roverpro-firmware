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
   booty --port COM3 --baudrate 57600 --hexfile "Downloads/PowerBoard-1.4.0.hex" --erase --load --verify
   ```

### Troubleshooting bootloader

 * Your port may vary. On my Windows computer it is `COM3`, on a Linux computer it may be `ttyUSB0` or something similar. Make sure you have the correct port selected, and the baud rate is 57600.
 * If booty says "device not responding", it means that either the rover has no booted or the rover is no longer in bootloader mode:
    * If the green serial board LED is **off**, the rover has not yet run its boot sequence. Make sure the battery has charge.
   * If the green serial board LED is **on**, the has run its boot sequence. The rover only remains in bootloader mode for 10 seconds after being powered on - if that time is elapsed, you need to reboot the robot. Hold down the red side power button for 5 seconds to restart the rover into bootloader mode then retry (note the light may or may not turn off immediately upon reboot).
* The location of your hexfile may vary. If not found, booty will report "no such file or directory"


## UART Protocol


## Development

### IDE and build tools

The MCP files can be opened in [MPLAB IDE v8.92](http://ww1.microchip.com/downloads/en/DeviceDoc/MPLAB_IDE_8_92.zip) (not MPLAB X) and should be built with the [Microchip XC16 Toolsuite](https://www.microchip.com/mplab/compilers). This toolsuite contains a compiler/linker/assembler and also standard libraries for the PIC24F MCU's.

To build, use the Debug mode (if you're attaching a PICKit) or Release mode (if you're not using a PICKit). Note that if you build in Debug mode and you hit a breakpoint (`BREAKPOINT()` macro), execution will halt for the debugger. If no debugger is attached, the device will immediately restart.

### Building the docs

Everything should be ready for doxygen. (on Windows, `choco install doxygen.install graphviz`)

To build the docs, switch to the Power_Board subfolder and run `doxygen`.

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

* You can build the executable as either an ELF or COF (Project -> Build Options ... -> Project -> XC16 ASM/C Suite -> Output-File Format). ELF has the benefit that you can isolate each function in a section and remove unused sections to get a smaller build size. COF has the benefit that the debugger works more reliably and you can use the "Locate Headers" tool to find headers you've `#include`d not mentioned in the mcp file. If you cannot view local variables, try switching to COF.

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

firmware.mcp = main project file. Open this with MPLab IDE v8.89




```flow
# render with flowchart.js
# Typora will render this inline
st=>start: Start
BatVolChecking Timer enabled
BatRecovery Timer disabled
SpeedUpdate Timer depends on motor commands
op=>operation: Wait for 1ms timer tick
Increment enabled timers
batvolchecking=>condition: Is BATVolChecking Timer elapsed?
c3a=>operation: Increment Overcurrent Counter
hicurrent=>condition: Is either battery current high (>512)?
hicurrent_yes=>operation: Increment Overcurrent Counter
locurrent=>condition: Is either battery current low (<341)?
locurrent_yes=>operation: Reset Overcurrent Counter to 0
long_overcurrent=>condition: Is Overcurrent Counter > 10?
long_overcurrent_yes=>operation: Set duty to all motors to 0
Set Overcurrent Flag
Start BATRecoveryTimer
bat_recovery=>condition: is BatRecovery Timer expired?
bat_recovery_yes=>operation: Clear OverCurrent flag
Disable the battery recovery timer
motor_speeds=>condition: Is SpeedUpdate Timer expired? (for each motor)
update_speed_oc=>condition: Is Overcurrent Flag set?
update_speed_oc_yes=>operation: set duty cycle to 0
update_speed_oc_no=>operation: set duty cycle to commanded speed
cond=>condition: Yes or No?
etc=>subroutine: more main loop stuff
todo=>end: Todo...

st->op->batvolchecking
hicurrent(yes)->hicurrent_yes
hicurrent_yes->long_overcurrent
long_overcurrent(yes)->long_overcurrent_yes->bat_recovery
long_overcurrent(no)->bat_recovery
batvolchecking(yes)->hicurrent
batvolchecking(no)->bat_recovery
hicurrent(no)->locurrent
locurrent(yes)->locurrent_yes->bat_recovery
locurrent(no)->bat_recovery
bat_recovery(yes)->bat_recovery_yes->motor_speeds
bat_recovery(no)->motor_speeds
motor_speeds(yes)->update_speed_oc
update_speed_oc(yes)->update_speed_oc_yes->etc
update_speed_oc(no)->update_speed_oc_no->etc
motor_speeds(no)->etc
etc->op
```

#### Motor_ADC1Interrupt

Periodically, the hardware [A/D Converter module](http://ww1.microchip.com/downloads/en/DeviceDoc/39705b.pdf)  calls the interrupt function `Motor_ADC1Interrupt` to get incoming analog signals:

- ADC1BUF4 = AN4 = Flipper Motor position 1 (?)
- ADC1BUF5 = AN5 = Flipper Motor position 2 (?)
- ADC1BUF3 = Left Motor Current
- ADC1BUF1 = Right Motor Current
- ADC1BUFB = Flipper Motor Current
- ADC1BUF6 = Battery Voltage A
- ADC1BUF7 = Battery Voltage B

These values are all stored in rolling buffers of 4 values, whose values are not directly used but their averages are.

### device_robot_motor_i2c.c

I2C2Update and I2C3Update functions query devices on two separate I2C buses and are both asynchronous, doing work and waiting for the device to respond. Communication with I2C is very similar to SMBus, so to start understanding what's going on, see http://smbus.org/specs/SMBus_3_0_20141220.pdf.

I2C2Update:

- Fan controller:
  - Get left fan temperature
  - Get right fan temperature
  - Set fan speed
- Battery A:
  - Get state of charge (% full)
  - Get battery status
  - Get battery mode
  - Get temperature

I2C3Update:

- Battery Charger:
  - read charger state (is charging or not)
- Battery B:
  - Get state of charge (% full)
  - Get battery status
  - Get battery mode
  - Get temperature

<script type='application/javascript'>
// NB: Typora does not like blank lines within HTML code blocks.
//
function gravizo(script_element) {
    // gravizo is a great tool that renders graphviz, plantuml, and umlgraph
    let img = document.createElement('img')
    img.src = 'https://g.gravizo.com/svg?' + encodeURIComponent(script_element.innerText) 
    script_element.parentNode.insertBefore(img, script_element)
}
//
document.addEventListener('DOMContentLoaded', function(){
    // turn all inline script tags into images
    [...document.querySelectorAll('script[type="text/vnd.graphviz"]'),
     ...document.querySelectorAll('script[type="text/x-plantuml"]'),
     ...document.querySelectorAll('script[type="text/x-umlgraph"]'),
    ].forEach(gravizo)
})
</script>



