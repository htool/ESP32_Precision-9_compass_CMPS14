# ESP32_Precision-9_compass_CMPS14
Digital compass acting as B&amp;G Precision 9 based on ESP32 and CMPS14

## ESP32 board
To have easy access to N2k [this board is recommended](https://hatlabs.github.io/sh-esp32/).
There is a part on the board for prototyping. You can use this to add the CMPS14 directly on the board

## Goal
- A highly accurate compass which also reports roll, pitch, rate of turn and possibly heave.
- Be able to use standard config screen in the B&G Zeus/Vulcan/Triton(2) to configure offsets and do calibration.

## CMPS14
After trying the MPU9250 first, Iwasn't happy with the noise level. The CMPS14 seems more stable and makes the code way easier.

## Status

### What's working
 - Recognised as B&G Precision-9.
 - Heading send out as Vessel Heading PGN 127250 as Magnetic at 10Hz.
 - Magnetic variation PGN is picked up and then Vessel Heading PGN 127250 is send out as True as well.
 - Heel and Trim are send out as Attitude PGN 127257 at 20Hz.
 - Rate of turn is send out as PGN 127251 at 10Hz.
 - MFD configuration of heading, trim and heel offset.
 - Offsets are stored in EEPROM when set.
 - Auto calibration mode is recognised,
   - Off, means.... Off, no autocalibration
   - On, means Autocalibration on for Gyro, Accelerometer and Magnetometer. Deletes stored calibration profile
   - Locked, means Off, no autocalibration. Save calibration profile
   - Auto, means Autocalibration off for Gyro and Accelerometer. On for Magnetometer

### calibration

Normaly the CMPS14 is calibrated in the factory. But it can be usefull to calibrate the device on (or near) the spot you want to use it.
The procedure is:

 - Connect your device to your computer and watch the serial port
 - Set Autocalibration mode on 'On' with the 'c' key
 - Type 'o' to enable output
 - Watch: [Calibration] 11001111 . In the ideal situation it wil look like written. But before calibration there could be more 0 and less 1'Save
 - Put your device on different sides, each for about 1 second. Keep your device steady for that second.
 - Set the device on minimal 4 sides.
 - Watch the Calibration and continue untill the fifth and sixth position shows a 1 (Accelerometer)
 - Then slowly rotate the device in all three axis. 180 degrees and back in about 2 to 3 seconds.
 - Watch the Calibration and continue untill the seventh and eigth position shows a 1 (Magnetometer)
 - When done calibrating, press 'o' to stop the output
 - Then press 'c' to change calibration to 'Locked' to save the calibration profile.
 - Then press 'c' again to change calibration to 'auto'
 - You can now place your device on it's final spot.
 
### What's half working
 - Deviation calibration routine. The stop/finsihed signal works, but processing the outcome is still missing. This routine process should build a boat specific deviation table.

### What's not working
 - Actual creation of the deviation table, which from what I understand can be done using Fourier transformation on the data from the 390 deg circle.
 - Heave isn't measured yet and thus not sent. Also here I think Fourier transformation on Z accellaration data can be used to calculate amplitude.
