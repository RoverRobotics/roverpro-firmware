firmware
========

## Repository Structure

```
roverpro-firmware
├── bootypic                        # Bootloader (not implemented currently)
├── Common                          # Common libraries
└── Rover_Pro                       # Rover Pro firware files                
    ├──Internal Charger             # Firmware for controlling battery charging
    ├──Motor Controller             # Deprecated
    └──Power Board                  # primary firware that runs the robot
        ├──closed_loop_control      # PID loop for controlling motor RPM instead of current
        ├──doc                      # Useful documents
        ├──empia_eeprom             # Deprecated
        ├──microchip                # Microchip libraries
        └──src                      # Source code
  
```


## Flashing
Needed: 
- MPLab Software (not MPLabX)
- PicKit3
- Rover Pro Interface Board V3

### Step One:
Remove the battery from your Rover Pro and wire the PicKit3 using the interface boards 5pin programming header

### Step Two:
Open MPLab, and set the programmer to PIckKit3, configure it to power the chip using the PicKit

### Step Three:
Program the firmware

## Compiling from Source
If you have made a custom modification to the firware and want to compile is from source, follow these instructions

Use MPLab (not MPLabX) to compile firmware into a .hex file
