This equatorial platform is set at a latitude of 49 degrees, and is driven by an STM32 microcontroller, which drives a Nema17 stepper motor. This motor is connected to a t8 lead screw, and is then attached to (2x) threaded rod ends,  and the top plate of the EQ platform. The EQ platform is aligned to, and pivots parallel to earth’s polar axis. It rides on 2 elliptic segments specifically made to match my latitude  (this is to accommodate the angle of polar axis in the sky from the southern pivot point of the platform).


  The platform is driven at a rate of 15 degrees per hour, which is the same rate as earth's rotation. This is done by calculating the number of steps the stepper motor will need per millimeter along the threaded rod to push the drive bracket, then translating that number into milliseconds delay between each coil activation on the stepper motor.

   A hand switch is used to start and stop the stepper motor from driving the mount, and at the end of each side of the z-axis slide bracket, there are limit switches so that when the platform is at it’s maximum angle, it will return or ‘slew’ quickly to the starting position.
