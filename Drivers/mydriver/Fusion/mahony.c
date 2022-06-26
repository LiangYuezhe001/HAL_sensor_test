#include "Fusion.h"
#include "main.h"

#define SAMPLE_RATE 200

u8 mahony_init()
{
// Define calibration (replace with actual calibration data)
/* 
Inertial calibration
The FusionCalibrationInertial function applies gyroscope and accelerometer calibration parameters using the calibration model:

ic = Ms(iu - b)

ic is the calibrated inertial measurement and return value
iu is the uncalibrated inertial measurement and uncalibrated argument
M is the misalignment matrix and misalignment argument
s is the sensitivity diagonal matrix and sensitivity argument
b is the offset vector and offset argument

Magnetic calibration
The FusionCalibrationMagnetic function applies magnetometer calibration parameters using the calibration model:

mc = Smu - h

mc is the calibrated magnetometer measurement and return value
mu is the uncalibrated magnetometer measurement and uncalibrated argument
S is the soft iron matrix and softIronMatrix argument
h is the hard iron offset vector and hardIronOffset argument*/
    const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
        .gain = 0.5f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f,
        .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);
}