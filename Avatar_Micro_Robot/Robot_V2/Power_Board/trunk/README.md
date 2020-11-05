The following files are required to be in the include path by the Microchip library:

HardwareProfile.h (found in ./)
usb_config.h (found in ../inc)

Microchip needs certain files in the include path. The project include path is the following:
./
../inc
../inc/microchip

## 1.0.7 Changes

1.0.7 addresses several critical issues that have been present since the 1.0.3 firmware.

# Issue 1 - tachometer signal and direction signal oscillation during stall conditions. 

This issue isn't a firmware fault per se, but actually is a suboptimal condition from the allegro
chip that must be handled by the microcontroller. So in that sense, it is a system issue that the firmware must compensate for.

During motor stall events, the tachometer and direction
signal oscillate rapidly indicating highly unrealistic velocity and direction changes for a rover robot
in the physical world. When the robot
is stationary, the tachometer period (time between edges) is often less than 100 timer counts meaning rapid 
speed. The direction bit is changing at a similar rate. If this condition is not handled, the firmware simply
interprets the stall as rapid speed and highly variable direction, which is obviously suboptimal.

The fix is very, very simple. In the capture routine, also check to see if the motor has changed direction from 
the previous capture. If the direction signal has changed state, simply output low speed for a fixed number of capture periods.
If the direction bit is oscillating, the capture routine will keep outputting low speeds until it is no longer oscillating.
This is remarkably effective at ensuring that 0 speed is output during stall conditions (as opposed to some highly
outrageous speed).

This drastic improvement allows a fundamentally different approach to the openrover stack. The openrover stack used to be filled with
all kinds of logic, filtering, slope-checking, and other compensation methods for dealing with this highly suboptimal
device behavior. Now the openrover stack can rely more on the data from the device, and potentially be more responsive/accurate 
in odometry data, closed loop control, etc etc.

# Issue 2 - unsafe timer rollover conditions were randomly zeroing period measurements at low robot speed.





