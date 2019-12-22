 /*
 Demonstration of the Sensor Extension Board from itoffice.eu for Raspberry Pi.

 This software prints demo gyro data out in the terminal (stdio/stdout) that in one  
 column shows the effect of the Kalman filter and in another column a complementary  
 filter, compared to no filter in a third column (in reverse order), for as well the  
 roll as the pitch angle.

 This C/C++ code is intended to compile and run on a Raspberry Pi. Other hardware than 
 Raspberry Pi might use something different than wiringPiI2C and wiringPi to 
 communicate with the sensor. 'stdio' is a typical Linux library, and 
 microcontrollers might use something entirely different to return visible data.

 Copyright (C) 2019 Andreas Chr. Dyhrberg. All rights reserved.

 This code is based on the work in Python from Kristian Lauszus, but I have ported 
 it to C/C++, added code, reordered and not at least rewritten it heavily to let it fit 
 to the principles of Clean Code by Robert C. Martin. Credit and thanks to Kristian 
 Lauszus for his initial work in Python.

 If you have any suggestions to the code regarding Clean Code, or any other relevant 
 comment, please let me know. (Exceptions to Clean Code are: No lower-camel case, 
 equal signs etc. in columns accross variable lists)

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Source for equations and mathematical stuff used: 
 http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

 The Kalman filter used is from Kristian Lauszus, TKJ Electronics:
 https://github.com/TKJElectronics/KalmanFilter
 */

#include "Kalman.h" /* Source: https://github.com/TKJElectronics/KalmanFilter */
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <math.h>

/* MPU6050 */
#define MPU6050_I2C_DEVICE_ADDRESS     0x68
#define REGISTER_FOR_POWER_MANAGEMENT  0x6B  /* PWR_MGMT_1 */
#define REGISTER_FOR_SAMPLE_RATE       0x19  /* SMPLRT_DIV */
#define REGISTER_FOR_ACCEL_XOUT_H      0x3B
#define REGISTER_FOR_ACCEL_YOUT_H      0x3D
#define REGISTER_FOR_ACCEL_ZOUT_H      0x3F
#define REGISTER_FOR_GYRO_XOUT_H       0x43
#define REGISTER_FOR_GYRO_YOUT_H       0x45
#define REGISTER_FOR_GYRO_ZOUT_H       0x47
#define REGISTER_FOR_TEMP_OUT_H        0x41
#define SLEEP_MODE_DISABLED            0x00

/* Different math and print constants */
#define RAD_TO_DEG                     (180.0 / M_PI)
#define DRIFT_MAX_DEGREES              180
#define LABEL_REPEAT_RATE              30

/* To restrict roll instead of pitch to ±90 degrees, comment out the following line */
#define PITCH_RESTRICT_90_DEG

/* MPU6050 variables */
int gyro_device_handler;
double accX;
double accY;
double accZ;
double gyroX;
double gyroY;
double gyroZ;
double temp_raw;

/* Variables used for printing */
int counter = 0;
double temp_degrees_c;
double roll_gyro;
double roll;
double roll_kalman;         /* Angle exposed to a Kalman filter */
double roll_complementary;  /* Angle exposed to a Complementary filter */
double pitch;
double pitch_gyro;
double pitch_kalman;        /* Angle exposed to a Kalman filter */
double pitch_complementary; /* Angle exposed to a Complementary filter */

int read_word_2c(int register_h)
{
    int val;
    val = wiringPiI2CReadReg8(gyro_device_handler, register_h);
    val = val << 8;
    val += wiringPiI2CReadReg8(gyro_device_handler, register_h+1);
    if (val >= 0x8000)
        val = -(65536 - val);
    return val;
}

void read_sensor_data()
{
    accX     = read_word_2c(REGISTER_FOR_ACCEL_XOUT_H);
    accY     = read_word_2c(REGISTER_FOR_ACCEL_YOUT_H);
    accZ     = read_word_2c(REGISTER_FOR_ACCEL_ZOUT_H);
    gyroX    = read_word_2c(REGISTER_FOR_GYRO_XOUT_H);
    gyroY    = read_word_2c(REGISTER_FOR_GYRO_YOUT_H);
    gyroZ    = read_word_2c(REGISTER_FOR_GYRO_ZOUT_H);
    temp_raw = read_word_2c(REGISTER_FOR_TEMP_OUT_H);
}

double convert_to_deg_per_sec(double a)
{
    return a / 131.0;
}

double distance(double a, double b)
{
    return sqrt((a*a) + (b*b));
}

double atan2_deg(double a, double b)
{
    return atan2(a,b) * RAD_TO_DEG;
}

double atan_deg(double a, double b, double c)
{
    return atan(a / distance(b, c)) * RAD_TO_DEG;
}

double max_drift_correction(double gyro, double kalman)
{
    if (gyro < -DRIFT_MAX_DEGREES || gyro > DRIFT_MAX_DEGREES)
        return kalman;
    else
        return gyro;
}

double max_90_deg_correction(double rate, double kalman)
{
    if (abs(kalman) > 90)
        return -rate;
    else
        return rate;
}

void print_columns()
{
    if (counter % LABEL_REPEAT_RATE == 0)
        printf("roll \t roll_gyro \t roll_complementary \t roll_kalman \t \t \t pitch \t pitch_gyro \t pitch_complementary \t pitch_kalman \t \t \t temp/*C \r\n");

    printf("%.1f", roll); printf("\t\t");
    printf("%.1f", roll_gyro); printf("\t\t\t");
    printf("%.1f", roll_complementary); printf("\t\t");
    printf("%.1f", roll_kalman); printf("\t");

    printf("\t\t");
    printf("%.1f", pitch); printf("\t\t");
    printf("%.1f", pitch_gyro); printf("\t\t\t");
    printf("%.1f", pitch_complementary); printf("\t\t");
    printf("%.1f", pitch_kalman); printf("\t");

    printf("\t\t");
    printf("%.1f", temp_degrees_c); printf("\t");

    printf("\r\n");
    delay(5);
}

int main()
{
    int timer;
    double seconds_passed;
    double roll_gyro_rate_deg_per_sec;
    double pitch_gyro_rate_deg_per_sec;

    Kalman kalman_roll;
    Kalman kalman_pitch;

    gyro_device_handler = wiringPiI2CSetup(MPU6050_I2C_DEVICE_ADDRESS);
    wiringPiI2CWriteReg8(gyro_device_handler,REGISTER_FOR_POWER_MANAGEMENT,SLEEP_MODE_DISABLED);

    /* Wait for sensor to stabilize */
    delay(150);

    /* Set the gyro starting angles */
    read_sensor_data();

#ifdef PITCH_RESTRICT_90_DEG
    roll  = atan2_deg(accY, accZ);
    pitch = atan_deg(-accX, accY, accZ);
#else
    roll  = atan_deg(accY, accX, accZ);
    pitch = atan2_deg(-accX, accZ);
#endif

    /* Set some more initial values */
    kalman_roll.setAngle(roll);
    roll_gyro           = roll;
    roll_complementary  = roll;  /* Angle exposed to a complementary filter */
    kalman_pitch.setAngle(pitch);
    pitch_gyro          = pitch;
    pitch_complementary = pitch; /* Angle exposed to a complementary filter */
    timer               = micros();

    while(1)
    {
        read_sensor_data();

        temp_degrees_c              = ((double)temp_raw / 340.0) + 36.53;
        seconds_passed              = (double)(micros() - timer) / 1000000;
        timer                       = micros();
        roll_gyro_rate_deg_per_sec  = convert_to_deg_per_sec(gyroX);
        pitch_gyro_rate_deg_per_sec = convert_to_deg_per_sec(gyroY);

    #ifdef PITCH_RESTRICT_90_DEG
        
        /* Eq. 25 and 26 from source for equations */
        roll  = atan2_deg(accY,accZ);
        pitch = atan_deg(-accX, accY, accZ);

        /* Let pitch have -90 and 90 degrees to be the continuous (and roll ±180) */
        if ( abs(roll)<= 90 || abs(roll_kalman)<= 90 )
        {
            /* Calculate roll_kalman */
            roll_kalman = kalman_roll.getAngle(roll, roll_gyro_rate_deg_per_sec, seconds_passed);
        }
        else
        {
            kalman_roll.setAngle(roll);
            roll_complementary = roll;
            roll_kalman        = roll;
            roll_gyro          = roll;
        }
        pitch_gyro_rate_deg_per_sec = max_90_deg_correction(pitch_gyro_rate_deg_per_sec, roll_kalman);
        pitch_kalman                = kalman_pitch.getAngle(pitch, pitch_gyro_rate_deg_per_sec, seconds_passed);

    #else
        /* Eq. 28 and 29 from source for equations */
        roll  = atan_deg(accY, accX, accZ);
        pitch = atan2_deg(-accX,accZ);

        /* Let roll have -90 and 90 degrees to be the continuous (and pitch ±180) */
        if ( abs(pitch)<= 90 || abs(pitch_kalman)<= 90 )
        {
            /* Calculate pitch_kalman */
            pitch_kalman = kalman_pitch.getAngle(pitch, pitch_gyro_rate_deg_per_sec, seconds_passed);
        }
        else
        {
            kalman_pitch.setAngle(pitch);
            pitch_complementary = pitch;
            pitch_kalman        = pitch;
            pitch_gyro          = pitch;
        }
        roll_gyro_rate_deg_per_sec = max_90_deg_correction(roll_gyro_rate_deg_per_sec, pitch_kalman);
        roll_kalman                = kalman_roll.getAngle(roll, roll_gyro_rate_deg_per_sec, seconds_passed);
    #endif

        /* Calculate gyro angles without any filter */
        roll_gyro  += roll_gyro_rate_deg_per_sec * seconds_passed;
        pitch_gyro += pitch_gyro_rate_deg_per_sec * seconds_passed;

        /* Calculate gyro angle using the unbiased rate */
        //roll_gyro += kalman_roll.getRate() * seconds_passed;
        //pitch_gyro += kalman_pitch.getRate() * seconds_passed;

        roll_gyro  = max_drift_correction(roll_gyro, roll_kalman);
        pitch_gyro = max_drift_correction(pitch_gyro, pitch_kalman);

        /* Calculate the angle using a Complimentary filter */
        roll_complementary  = 0.93 * (roll_complementary + roll_gyro_rate_deg_per_sec * seconds_passed) + 0.07 * roll;
        pitch_complementary = 0.93 * (pitch_complementary + pitch_gyro_rate_deg_per_sec * seconds_passed) + 0.07 * pitch;

        print_columns();
        counter++;
    }
}
