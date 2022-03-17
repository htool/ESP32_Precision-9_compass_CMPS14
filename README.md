# ESP32_Precision-9_compass_CMPS14
Digital compass acting as B&amp;G Precision 9 based on ESP32 and CMPS14

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

### What's half working
 - Deviation calibration stop/cancel from MFD. The stop/cancel signal works, but the 'finished' signal is still missing as well as the actual routing. This routine is the 390 deg circle that allows building a boat specific deviation table.
 - Auto calibration mode is recognised, but not used yet.

### What's not working
 - Actual creation of the deviation table, which from what I understand can be done using Fourier transformation on the data from the 390 deg circle.
 - Heave isn't measured yet and thus not sent. Also here I think Fourier transformation on Z accellaration data can be used to calculate amplitude.
