# LSRM Control Algorithm

This was used as a learning project to determine the best methods to effectively and efficiently translate a motor carriage using distance sensing, external input signals and rapid firing relays.

## C

The control code was implemented using C and a STM32 microcontroller. The STM32 provided an interface between the software and hardware aspects of this project.

## General Idea of the Code

The purpose behind the control code provided was to excite each motor segment individually by sending current through the control coil of the solid state relays - controlled by a STM32 microcontroller. The excitation sequence for forward motion was as follows: A-C-B and the reverse sequence: C-B-A. Note that A, B, and C refer to the phases or motor segments.

While the motor is moving, the microcontroller was programmed to convert the PPR (Pulses Per Revolution) count from an optical encoder into millimeters travelled and provide the stop command once a user defined distance has been met. The motor was programmed to perform 3 of these distance measurements. In addition, the microcontroller accepted input from two breakout limit switches and an onboard push button to dictate motor directionality and control. Once all three motor movements were complete, the motor would always return home and engage the stop command until turned off.

## Detailed Explanation

Refer to the thorough comments placed throughout the code for reference to what role each class/function performs.

## Future Improvements to the Code

If the time constraint on the project had been removed, wireless interfacing between the microcontroller and any android device was going to be provided. A simple mobile application was in the plans to be completed that would communicate via Serial Communication. This was going to be possible through the use of a HC-06 bluetooth module connected to the microcontroller paired to the mobile device.

Within the mobile application, the intended design was to provide the user the optionality to enter an unrestricted amount of distance movements, as well as to allow the user to manually control the distance travelled by the motor through the use of "Forward" and "Reverse" UI buttons. Furthermore, if the motor reached either end of the rail, the corresponding UI button would be disabled and greyed out.

Aesthetic inclusions on the mobile application was to add animations signifying which direction the motor is moving. This could be done through developing a sprite sheet of the actual motor in various small incremental distance movements and stiching them together.
