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

 * Your port may vary. On my Windows computer it is `COM3`, on a Linux computer it may be `ttyUSB0` or something similar.
 * If booty says "device not responding" and the green serial board LED is **off**, the rover may be unpowered or not properly connected to the computer. Make sure the battery has charge, the cable is connected properly, you have the correct port selected, and the baud rate is 57600.
* If booty says "device not responding" and the green serial board LED is **on**, the rover has booted into regular operation. The rover only remains in bootloader mode for 10 seconds after being powered on. Hold down the red side power button for 5 seconds to restart the rover into bootloader mode then retry (note the light may or may not turn off immediately upon reboot).
* The location of your hexfile may vary. If not found, booty will report "no such file or directory"


## UART Protocol


## Development

### IDE and build tools

The MCP files can be opened in [MPLAB IDE v8.92](http://ww1.microchip.com/downloads/en/DeviceDoc/MPLAB_IDE_8_92.zip) (not MPLAB X) and should be built with the [Microchip XC16 Toolsuite](https://www.microchip.com/mplab/compilers). This contains not only a compiler/linker/assembler but also standard libraries for the PIC24F MCU's.

To build, use the Debug mode (if you're attaching a PICKit) or Release mode (if you're using this with other things). Note that if you build in Debug mode and you hit a breakpoint (`BREAKPOINT()` macro), execution will halt and wait for the debugger. If no debugger is attached, the device will immediately restart.

### Building the docs

Everything should be ready for doxygen. (on Windows, `choco install doxygen.install graphviz`)

To build the docs, switch to the Power_Board subfolder and run `doxygen`.



### Code style tools

To tidy up code, I like using **[clang-format](https://clang.llvm.org/docs/ClangFormat.html)**, and have provided a .clang-format file. Clang 6 is currently the latest release for Ubuntu, but feel free to use newer.

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

### Call Diagram of Important functions

<script type='text/vnd.graphviz'>
  digraph g {
  rankdir=LR;
  subgraph cluster_1 {
    label = "main.c";
    main -> InitializeSystem;
    InitializeSystem -> USBDeviceInit;
    InitializeSystem -> USBDeviceAttach;
  }
  subgraph cluster_2 {
    label = "Pic24F hardware interrupts"
    _U1RXInterrupt;
    _U1TXInterrupt;
    _ADCInterrupt;
    _T3Interrupt;
    _IC1Interrupt;
_IC2Interrupt;
_IC3Interrupt;
  }
  subgraph cluster_3 {
    label = "device_robot_motor.c";
    IniAD;
    Device_MotorController_Process;
    closed_loop_control_init;
    Motor_T3Interrupt;
    Motor_ADC1Interrupt;
    DeviceRobotMotorInit;
    FANCtrlIni;
    handle_closed_loop_control;
  }
  subgraph cluster_4 {
    label = "pid.c";
    PID_Init;
    PID_ComputeEffort;
    PID_Reset;
    PID_Reset_Integral;
  }
  subgraph cluster_6 {
    label = "uart_control.c";
    uart_init;
    uart_tick;
    uart_tx_isf;
    uart_rx_isf;
  }
  subgraph cluster_7 {
    label = "device_power_bus.c";
    power_bus_init;
    power_bus_tick;
  }
  subgraph cluster_8 {
    label = "device_robot_motor_i2c.c";
    i2c2_tick -> re_init_i2c2;
    i2c3_tick -> re_init_i2c3;
  }
subgraph cluster_9 {
    label = "motor.c";
    motor_tach_init;
    motor_tach_event_capture;
    motor_tach_get_period;
  }
_IC1Interrupt-> motor_tach_event_capture;
_IC2Interrupt-> motor_tach_event_capture;
_IC3Interrupt-> motor_tach_event_capture;
  _ADCInterrupt -> Motor_ADC1Interrupt;
  _U1TXInterrupt -> uart_tx_isf;
  _U1RXInterrupt -> uart_rx_isf;
  _T3Interrupt -> Motor_T3Interrupt;
  InitializeSystem -> DeviceRobotMotorInit;
  closed_loop_control_init -> PID_Init;
  DeviceRobotMotorInit -> IniAD;
  DeviceRobotMotorInit -> uart_init;
  DeviceRobotMotorInit -> power_bus_init;
  DeviceRobotMotorInit -> FANCtrlIni;
  DeviceRobotMotorInit -> closed_loop_control_init;
  Device_MotorController_Process -> power_bus_tick;
  Device_MotorController_Process -> handle_closed_loop_control;
  Device_MotorController_Process -> i2c2_tick;
  Device_MotorController_Process -> i2c3_tick;
  Device_MotorController_Process -> uart_tick;
  handle_closed_loop_control -> PID_ComputeEffort;
  handle_closed_loop_control -> PID_Reset;
  handle_closed_loop_control -> PID_Reset_Integral;
  }
</script>



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

### registers.h

This file contains metadata about global variables which are used to communicate to and from the robot. Though not truly CPU registers, we call them registers anyway. e.g.:

​	`REGISTER( REG_ROBOT_REL_SOC_A, DEVICE_READ,  DEVICE_MOTOR, SYNC, int16_t )`

- `REG_ROBOT_REL_SOC_A` = name of global variable holding this value
- `DEVICE_READ` = the firmware will read this value from hardware.
- `DEVICE_MOTOR` = [DEFUNCT]
- `SYNC` = [DEFUNCT]
- `uint16_t` = datatype of variable holding this value

Note that the REGISTER macro is *not* defined in this file. It is defined externally and then this file is imported.

usb_config.c declares a global variable per register to hold the physical data. It also defines an array (`struct REGISTER registers[]`) that makes register metadata available at runtime (except for the datatype).

At this time, the only code that uses register metadata is `main.c::ProcessIO`, which reads/writes registers based on their index in `registers[]`, i.e. their order in `registers.h`

```c
// usb_config.h
struct REGISTER
{
   SIZE     size;
   SYNC_BIT sync;
   DEVICE   device;
   RW       rw;
   DATA_PTR ptr;
};

extern struct REGISTER registers[];
#define REGISTER( a, b, c, d, e)       extern e a __attribute__((VAR_ATTRIBS));
#include "registers.h"

// usb_config.c

#define REGISTER( a, b, c, d, e )    e a;
//...
#include "registers.h"
//...
#undef  REGISTER
//...

#define REGISTER_START()            struct REGISTER registers[] = { 
#define REGISTER( a, b, c, d, e )                                   {sizeof(e),d,c,b,&a},
#define REGISTER_END()                                              {0} };
//...
#include "registers.h"
```

### device_robot_motor.c

This file has the main robot logic. It is structured around a synchronous 1 millisecond timer, and just about everything in it runs on a multiple of that timer.

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



