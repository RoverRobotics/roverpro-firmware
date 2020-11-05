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

Also, for this fix to work at maximal effectiveness, every capture edge needs to be interrupted upon instead of only rising edges. This is effectively saying that using 100% of the tachometer data is better than using 50% of the tachometer data, which makes intuitive sense.

This drastic improvement allows a fundamentally different approach to the openrover stack. The openrover stack used to be filled with
all kinds of logic, filtering, slope-checking, and other compensation methods for dealing with this highly suboptimal
device behavior. Now the openrover stack can rely more on the data from the device, and potentially be more responsive/accurate 
in odometry data, closed loop control, etc etc.

# Issue 2 - unsafe timer rollover conditions were randomly zeroing period measurements at low robot speed.

In the firmware, TMR4 is utilized in conjunction with a basic routine to ensure that stale tachometer 
measurements are purged from the system. If the robot deccelerates rapidly for example and then sits for a few
seconds, there is a decent likelihood that some residual non-zero speed is sitting in global variables from
the capture interrupts. The code is intended to clear that non-zero speed (zero it out) after some period of time
when it becomes stale.

While the intent was good here, the implementation lacked logic to deal with the rollover. A 32-bit meta-counter
variable was overflowing unsafely, and as a result, the elapsed_time tracking had unpredictable behavior. Because elapsed_time
on the capture data was not reliable, this code would randomly zero out period data (making the odometry data unreliable).
This issue would
manifest itself as intermittently clearing all period data, sometimes while the robot was moving! This issue was most prevalent at low-speeds (~0.3 m/s) where the elapsed time between capture events was long, and could be seen as 'blips to 
zero' in the odometry data.

The fix for this issue was also simple. Handle the rollover condition on the meta-timer variable and all is well.

# Other cleanup areas.

The encoder count feature was not supported in 1.0.3. The variables piped into this serial message were not being updated by any ISR. Rather than allowing those variables to be a random value (whatever was in RAM at startup), those values were explicitly set to 0. This should avoid any confusion there.

The GetRPM() function is now supported. This data is returned as a SIGNED 16-bit int, meaning the direction of the motors is ACTUALLY RETURNED FROM THE ROBOT. Having the robot report both it's motor speed **and** motor direction instead of just tachomoter period without direction means we can **stop assuming** the direction of the robot in the driver and start actually allowing the robot to report its current direction.








