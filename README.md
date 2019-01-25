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

Communicate with the firmware over UART at baud rate 57600.

Each message to the rover is 7 bytes:

1. Start byte = 253
2. Left motor speed - (0-124 = backwards, 125 = hard brake, 126-250 = forward)
3. Right motor speed
4. Flipper motor speed
5. Command Verb
6. Command Argument
7. Checksum = 255 - (sum of all bytes except start byte) % 255

The rover only responds if command verb is 10. All values are 16-bit integers, unsigned unless noted below.

The response is 5 bytes:
1. Start byte = 253
2. Data Element #
3. Value (hi byte)
4. Value (lo byte)
5. Checksum = 255 - (sum of all bytes except start byte) % 255

Upon receiving a message, the rover will set the motor speeds and may also take an additional action specified by the command verb. If no valid message is received is received for a while (333ms), the motors will come to a halt.

### UART Command Verbs

|      | Name                 | Description                                                  |
| ---- | -------------------- | ------------------------------------------------------------ |
| 0    | ---                  | No additional action                                         |
| 10   | get data             | Rover will respond with the data element specified by arg    |
| 20   | set fan target speed | Rover will set the cooling fan speed to the arg (0-240) for a while (333ms) |
| 230  | restart              | Rover will restart. If arg=0, then the bootloader will be skipped. If arg=0, then the rover will skip the bootloader. If arg=1, then the rover will enter the bootloader upon restart. |
| 240  | set drive mode       | If arg = 0, rover will be driven in open loop mode (commanded speeds will be the direction and effort of the motor)<br />If arg=1, rover will be driven in closed loop mode (commanded speeds will be the intended speed of the motor) |
| 230  | flipper calibrate    | If arg = 230, calibrate the flipper. Note the robot must be manually cycled before it will accept additional commands. |

### UART Data Elements

| #    | Name                           | Allowable data                                               | Comments                                                     |
| ---- | ------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 0    | battery (A+B) current (external)| 34 = 1A                                                     | Total current from batteries                                 |
|~~2~~ | ~~left motor speed~~           |                                                              |                                                              |
|~~4~~ | ~~right motor speed~~          |                                                              |                                                              |
| 6    | flipper position 1             | relative position of 15 - 330 degrees of one sensor          | Flipper position sensor 1                                    |
| 8    | flipper position 2             | relative position of 15 - 330 degrees of one sensor          | Flipper position sensor 2                                    |
| 10   | left motor current             | 34 = 1A                                                      |                                                              |
| 12   | right motor current            | 34 = 1A                                                      |                                                              |
| 14   | left motor encoder count       | 0 - 65535                                                    | May overflow or underflow. Increments when motor driven forward, decrements backward |
| 16   | right motor encoder count      | 0 - 65535                                                    | May overflow or underflow. Increments when motor driven forward, decrements backward |
| 18   | motors fault flag              | Bit flags                                                    | (value & 0x0100) = left motor (value & 0x0001) = right motor |
| 20   | left motor temperature         | degrees celsius                                              |                                                              |
|~~22~~| ~~right motor temperature~~    | degrees celsius                                              |                                                              |
| 24   | battery A voltage (external)   | 58 = 1V                                                      |                                                              |
| 26   | battery B voltage (external)   | 58 = 1V                                                      |                                                              |
| 28   | left motor encoder interval    |                                                              | 0 when motor stopped. Else proportional to motor period (inverse motor speed) |
| 30   | right motor encoder interval   |                                                              |                                                              |
|~~32~~| flipper motor encoder interval |                                                              |                                                              |
| 34   | battery A state of charge      | 0-100 %                                                      | 0 = battery empty; 100 = battery fully charged               |
| 36   | battery B state of charge      | 0-100 %                                                      |                                                              |
| 38   | battery charging state         | 0 or 0xdada=56026                                            | 0 = not charging; 0xdada = charging                          |
| 40   | release version                | Structured decimal                                           | XYYZZ, where X=major version, Y=minor version, Z = patch version.<br />e.g. 10502 = version 1.05.02<br />The value 16421 will be reported for pre-1.3 versions|
| 42   | battery A current (external)   | 0-1023, 34 = 1A                                              |                                                              |
| 44   | battery B current (external)   | 0-1023, 34 = 1A                                              |                                                              |
| 46   | motor flipper angle            | 0-360, degrees (actual data range needs to be tested)        | Flipper angle                                                |
| 48   | fan speed                      | 0-240,                                                       | Actual fan speed, reported by fan controller                 |
| 50   | drive mode                     | 0 (open loop) or 1 (closed loop)                             |                                                              |
| 52   | battery A status               | Bit flags                                                    | Alarm bits:<br />* 0x8000 OVER_CHARGED_ALARM<br />* 0x4000 TERMINATE_CHARGE_ALARM<br />* 0x1000 OVER_TEMP_ALARM<br />* 0x0800 TERMINATE_DISCHARGE_ALARM<br />* 0x0200 REMAINING_CAPACITY_ALARM<br />* 0x0100 REMAINING_TIME_ALARM<br />Status bits:<br />* 0x0080 INITIALIZED<br />* 0x0040 DISCHARGING<br />* 0x0020 FULLY_CHARGED<br />* 0x0010 FULLY_DISCHARGED |
| 54   | battery B status               | Bit flags                                                    |                                                              |
| 56   | battery A mode                 | Bit flags                                                    | Bit 7 (value & 0x80) gives whether battery needs a condition cycle. Other values probably useless |
| 58   | battery B mode                 | Bit flags                                                    | Bit 7 (value & 0x80) gives whether battery needs a condition cycle. Other values probably useless |
| 60   | battery A temperature          | Temperature of battery above absolute 0 in deciKelvins       |                                                              |
| 62   | battery B temperature          | Temperature of battery above absolute 0 in deciKelvins       |                                                              |
| 64   | battery A voltage (internal)   | Voltage of battery in mV                                     |                                                              |
| 66   | battery B voltage (internal)   | Voltage of battery in mV                                     |                                                              |
| 68   | battery A current (internal)   | (signed) Current of battery in mA                            | negative values for discharge, positive for charging         |
| 70   | battery B current (internal)   | (signed) Current of battery in mA                            | negative values for discharge, positive for charging         |

note: for battery reporting, "internal" means the value comes from the SmartBattery's internal sensor. "external" means the value is reported by circuitry outside the SmartBattery

## Development

### IDE and build tools

The MCP files can be opened in [MPLAB IDE v8.92](http://ww1.microchip.com/downloads/en/DeviceDoc/MPLAB_IDE_8_92.zip) (not MPLAB X) and should be built with the [Microchip XC16 Toolsuite](https://www.microchip.com/mplab/compilers). This contains not only a compiler/linker/assembler but also standard libraries for the PIC24F MCU's.

To build, use the Debug mode (if you're attaching a PICKit) or Release mode (if you're using this with other things). Note that if you build in Debug mode and you hit a breakpoint (`BREAKPOINT()` macro), execution will halt and wait for the debugger. If no debugger is attached, the device will immediately restart.

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

* You can build the executable as either an ELF or COF (Project -> Build Options ... -> Project -> XC16 ASM/C Suite -> Output-File Format). ELF has the benefit that you can isolate each function in a section and remove unused sections to get a smaller build size. COF has the benefit that the debugger works more reliably. If you cannot view local variables, try switching to GOF.

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

```
$ cd /mnt/c/Users/dan/Documents/OpenRoverFirmware-dan/Power_Board
$ tree -h
.
├── [3.1K]  CMakeLists.txt
├── [ 512]  doc
│   ├── [106K]  2011Arm_Base_Datasheet.doc
│   ├── [107K]  2011Arm_Link1_Datasheet.doc
│   ├── [ 17K]  2011Robot_PowerBoard_AssistantCalculationSheet.xlsx
│   ├── [ 31K]  2011_Robot_PowerBoard200_Evaluation_Datasheet.docx
│   ├── [ 63K]  2011_Robot_PowerBoardDatasheet.docx
│   └── [  62]  XbeeModuleConfiguration.txt
├── [5.7K]  firmware.mcp
├── [ 512]  include
│   ├── [1.5K]  Filters.h
│   ├── [1.1K]  HardwareProfile.h
│   ├── [3.5K]  PID.h
│   ├── [ 550]  counter.h
│   ├── [ 887]  device_power_bus.h
│   ├── [3.0K]  device_robot_motor.h
│   ├── [ 182]  device_robot_motor_i2c.h
│   ├── [ 215]  device_robot_motor_loop.h
│   ├── [4.2K]  hardware_definitions.h
│   ├── [6.7K]  i2clib.h
│   ├── [ 740]  interrupt_switch.h
│   ├── [1.7K]  motor.h
│   ├── [ 16K]  registers.h
│   ├── [1.7K]  stdhdr.h
│   ├── [ 813]  uart_control.h
│   └── [3.3K]  usb_config.h
├── [ 512]  microchip
│   ├── [5.7K]  Compiler.h
│   ├── [ 25K]  DEE Emulation 16-bit.c
│   ├── [5.3K]  DEE Emulation 16-bit.h
│   ├── [2.6K]  Flash Operations.s
│   ├── [ 512]  USB
│   │   ├── [5.9K]  usb.h
│   │   ├── [ 28K]  usb_ch9.h
│   │   ├── [ 23K]  usb_common.h
│   │   ├── [ 47K]  usb_device.h
│   │   ├── [8.3K]  usb_function_audio.h
│   │   ├── [6.1K]  usb_function_ccid.h
│   │   ├── [ 22K]  usb_function_cdc.h
│   │   ├── [8.9K]  usb_function_generic.h
│   │   ├── [ 12K]  usb_function_hid.h
│   │   ├── [4.6K]  usb_function_midi.h
│   │   ├── [ 20K]  usb_function_msd.h
│   │   ├── [ 22K]  usb_hal.h
│   │   ├── [ 17K]  usb_hal_pic18.h
│   │   ├── [ 17K]  usb_hal_pic24.h
│   │   ├── [ 15K]  usb_hal_pic32.h
│   │   ├── [ 56K]  usb_host.h
│   │   ├── [ 24K]  usb_host_audio_v1.h
│   │   ├── [ 31K]  usb_host_cdc.h
│   │   ├── [8.7K]  usb_host_cdc_interface.h
│   │   ├── [ 11K]  usb_host_charger.h
│   │   ├── [ 23K]  usb_host_generic.h
│   │   ├── [ 32K]  usb_host_hid.h
│   │   ├── [ 24K]  usb_host_hid_parser.h
│   │   ├── [ 21K]  usb_host_msd.h
│   │   ├── [ 14K]  usb_host_msd_scsi.h
│   │   ├── [105K]  usb_host_printer.h
│   │   ├── [ 16K]  usb_host_printer_esc_pos.h
│   │   ├── [ 11K]  usb_host_printer_pcl_5.h
│   │   ├── [8.7K]  usb_host_printer_postscript.h
│   │   ├── [7.3K]  usb_host_printer_primitives.h
│   │   ├── [ 22K]  usb_otg.h
│   │   ├── [6.0K]  usb_printer_pos_bixolon_srp_270.h
│   │   ├── [5.8K]  usb_printer_pos_epson_tm_t88iv.h
│   │   ├── [5.9K]  usb_printer_pos_seiko_dpu_v445.h
│   │   └── [5.8K]  usb_printer_pos_seiko_mpu_l465.h
│   └── [7.4K]  uart2.h
└── [ 512]  src
    ├── [ 831]  Filters.c
    ├── [3.7K]  PID.c
    ├── [ 649]  counter.c
    ├── [5.5K]  device_power_bus.c
    ├── [ 30K]  device_robot_motor.c
    ├── [6.7K]  device_robot_motor_i2c.c
    ├── [6.3K]  device_robot_motor_loop.c
    ├── [ 18K]  i2clib.c
    ├── [1.6K]  interrupt_switch.c
    ├── [8.5K]  main.c
    ├── [ 10K]  motor.c
    ├── [1.2K]  stdfunctions.c
    ├── [9.6K]  uart_control.c
    ├── [1.2K]  usb_config.c
    ├── [3.8K]  usb_descriptors.c
    └── [ 97K]  usb_device.c

5 directories, 80 files
```

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

This function also populates the data buffers for UART communication.

#### Motor_U1TXInterrupt / Motor_U1RXInterrupt

These functions are called by the hardware [UART module](http://ww1.microchip.com/downloads/en/DeviceDoc/en026583.pdf) as transmit (TX) and receive (RX) interrupts. The logic to actually populate the data to be transmitted is in `Motor_ADC1Interrupt`, but it should not be.

This function takes the incoming two bytes, and treats the first one as a command instruction, and the second as a command argument.

Command Instructions:

```c
#define P1_Read_Register 10
#define P1_Fan_Command 20
#define P1_Low_Speed_Set 240
#define P1_Calibration_Flipper 250
```

The command argument for P1_Read_Register is which register to read. This is not the index in `registers[]`, but is hardcoded into a long switch statement.

P1_Fan_Command: Set the side fan speed to the value of the command argument.

P1_Low_Speed_Set: Set low speed mode variable. Expected value 0 or 1. This has the same effect as setting `REG_MOTOR_SLOW_SPEED=1`

P1_Calibration_Flipper: command argument is ignored. Sets `Xbee_Calibration` flag, which causes `calibrate_flipper_angle_sensor` to be called.

UpdateSpeed is where this commands the robot motors to change speeds.

The main loop for gathering data from hardware is in `Device_MotorController_Process`,  where it calls `I2C2Update` and `I2C3Update`.

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



