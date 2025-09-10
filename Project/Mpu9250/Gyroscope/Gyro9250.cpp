///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INCLUDES /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include "LoggerHw.hpp"
#include "Gyro9250.hpp"
#include "Settings.hpp"

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// MACROS/DEFINITIONS ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define SELF_TEST_X_GYRO     0x00
#define SELF_TEST_Y_GYRO     0x01
#define SELF_TEST_Z_GYRO     0x02
#define XG_OFFSET_H          0x13
#define XG_OFFSET_L          0x14
#define YG_OFFSET_H          0x15
#define YG_OFFSET_L          0x16
#define ZG_OFFSET_H          0x17
#define ZG_OFFSET_L          0x18
#define CONFIG               0x1B
#define CONFIG_X_CTEN        0x80
#define CONFIG_Y_CTEN        0x40
#define CONFIG_Z_CTEN        0x20
#define CONFIG_FS_SEL1       0x10
#define CONFIG_FS_SEL0       0x08
#define CONFIG_FCHOICE_B1    0x02
#define CONFIG_FCHOICE_B0    0x01

#define XOUT_H               0x43
#define XOUT_L               0x44
#define YOUT_H               0x45
#define YOUT_L               0x46
#define ZOUT_H               0x47
#define ZOUT_L               0x48

#define RESOLUTION           32768
#define SAMPLES_NUM          50
#define SENSITIVITY_FOR_250  131.0f   // [deg/s]
#define SENSITIVITY_FOR_500  65.5f    // [deg/s]
#define SENSITIVITY_FOR_1000 32.8f    // [deg/s]
#define SENSITIVITY_FOR_2000 16.4f    // [deg/s]

#define CALIBRATION_TIME     100

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// FUNCTIONS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

Gyro9250::Gyro9250 (I2cHw & vI2cHw) : Imu <Gyro9250> (CALIBRATION_TIME), i2cHw (vI2cHw) {}

void Gyro9250::Init (void)
{
    uint8_t data = CONFIG_FS_SEL1 |
                   CONFIG_FS_SEL0 |
                   CONFIG_FCHOICE_B1;
    i2cHw.Send (I2cHw::EI2c::Zero, CONFIG, &data, 0);

    uint8_t sensitivity = 0;
    i2cHw.Receive  (I2cHw::EI2c::Zero, CONFIG, &sensitivity, 1);
    setSensitivity (sensitivity);
}

ImuSettings::Axes Gyro9250::GetAxes (void)
{
    std::array <uint8_t, 6> data;
    i2cHw.Receive (I2cHw::EI2c::Zero, XOUT_H, data.data (), data.size ());

    const int16_t x = static_cast<int16_t>((data [0] << 8) | data [1]);
    const int16_t y = static_cast<int16_t>((data [2] << 8) | data [3]);
    const int16_t z = static_cast<int16_t>((data [4] << 8) | data [5]);

    return { static_cast<float>((x - Settings::GetInst ()->Imu.Gyro.Offset.X) / Sensitivity),
             static_cast<float>((y - Settings::GetInst ()->Imu.Gyro.Offset.Y) / Sensitivity),
             static_cast<float>((z - Settings::GetInst ()->Imu.Gyro.Offset.Z) / Sensitivity) };
}

void Gyro9250::Calibrate (void)
{
    LOGI (Module, "Stand still. Calibrate for 1s");

    Settings::GetInst ()->Imu.Gyro.Offset.X = 0;
    Settings::GetInst ()->Imu.Gyro.Offset.Y = 0;
    Settings::GetInst ()->Imu.Gyro.Offset.Z = 0;

    uint16_t numOfSamples = 0;
    float    xSum         = 0,
             ySum         = 0,
             zSum         = 0;

    Settings::GetInst ()->Timer.Imu.IsOn = true;
    while (IsCalibInProgress () == true)
    {
        ImuSettings::Axes axes = GetAxes ();
        xSum += axes.X;
        ySum += axes.Y;
        zSum += axes.Z;
        numOfSamples++;
    }

    Settings::GetInst ()->Timer.Imu.IsOn    = false;
    Settings::GetInst ()->Timer.Imu.Counter = 0;

    Settings::GetInst ()->Imu.Gyro.Offset.X = xSum * Sensitivity / numOfSamples;
    Settings::GetInst ()->Imu.Gyro.Offset.Y = ySum * Sensitivity / numOfSamples;
    Settings::GetInst ()->Imu.Gyro.Offset.Z = zSum * Sensitivity / numOfSamples;

    LOGI (Module, "Offsets: X: " + std::to_string (Settings::GetInst ()->Imu.Gyro.Offset.X) +
                  "[LSB], Y: "   + std::to_string (Settings::GetInst ()->Imu.Gyro.Offset.Y) +
                  "[LSB], Z: "   + std::to_string (Settings::GetInst ()->Imu.Gyro.Offset.Z) + "[LSB]");
}

void Gyro9250::setSensitivity (const uint8_t vData)
{
    const bool fsSel1 = (vData & CONFIG_FS_SEL1);
    const bool fsSel0 = (vData & CONFIG_FS_SEL0);

    if      (fsSel1 == false && fsSel0 == false) { Sensitivity = SENSITIVITY_FOR_250;  }
    else if (fsSel1 == false && fsSel0 == true ) { Sensitivity = SENSITIVITY_FOR_500;  }
    else if (fsSel1 == true  && fsSel0 == false) { Sensitivity = SENSITIVITY_FOR_1000; }
    else if (fsSel1 == true  && fsSel0 == true ) { Sensitivity = SENSITIVITY_FOR_2000; }
    else                                         { Sensitivity = SENSITIVITY_FOR_250;  }
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// END OF FILE ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
