# ito-mpu6050-kalman-raspberry
Sample Kalman filtered data from gyro sensor MPU6050 (GY-521) via I2C on a Raspberry Pi

Demonstration of the Sensor Extension Board from https://www.itoffice.eu for Raspberry Pi.

## What the software does
This software prints demo gyro data out in the terminal (stdio/stdout) that in one  
column shows the effect of the Kalman filter and in another column a complementary  
filter, compared to no filter in a third column (in reverse order), for as well the  
roll as the pitch angle.

## Notes on hardware
This C/C++ code is intended to compile and run on a Raspberry Pi. Other hardware than 
Raspberry Pi might use something different than wiringPiI2C and wiringPi to 
communicate with the sensor. 'stdio' is a typical Linux library, and 
microcontrollers might use something entirely different to return visible data.

## Copyright and Clean Code
Copyright (C) 2019 Andreas Chr. Dyhrberg. All rights reserved.

This code is based on the work in Python from Kristian Lauszus, but I have ported 
it to C/C++, added code, reordered and not at least rewritten it heavily to let it fit 
to the principles of Clean Code by Robert C. Martin. Credit and thanks to Kristian 
Lauszus for his initial work in Python.

If you have any suggestions to the code regarding Clean Code, or any other relevant 
comment, please let me know. (Exceptions to Clean Code are: No lower-camel case, 
equal signs etc. in columns accross variable lists)

## License
This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

## Credit
The Kalman filter used is from Kristian Lauszus, TKJ Electronics:
https://github.com/TKJElectronics/KalmanFilter

## Mathematical background
Source for equations and mathematical stuff used: 
http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

## Compiling
Look at the sample output
https://github.com/itofficeeu/ito-mpu6050-kalman-raspberry/blob/master/ito-mpu6050-kalman_terminal_out_sample.txt