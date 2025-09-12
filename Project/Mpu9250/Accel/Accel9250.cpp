///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INCLUDES /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include "Settings.hpp"
#include "LoggerHw.hpp"
#include "Accel9250.hpp"

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// MACROS/DEFINITIONS ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define SELF_TEST_X_ACCEL                 0x0D
#define SELF_TEST_Y_ACCEL                 0x0E
#define SELF_TEST_Z_ACCEL                 0x0F

#define CONFIG                            0x1C
#define CONFIG_AX_ST_EN2                  0x80
#define CONFIG_AY_ST_EN1                  0x40
#define CONFIG_AZ_ST_EN0                  0x20
#define CONFIG_FS_SEL1                    0x10
#define CONFIG_FS_SEL0                    0x08

#define CONFIG2                           0x1D
#define CONFIG2_FCHOICE_B1                0x08
#define CONFIG2_FCHOICE_B0                0x04
#define CONFIG2_A_DLPF_CFG1               0x02
#define CONFIG2_A_DLPF_CFG0               0x01

#define LP_ODR                            0x1E
#define LP_ODR_LPOSC_CLKSEL3              0x08
#define LP_ODR_LPOSC_CLKSEL2              0x04
#define LP_ODR_LPOSC_CLKSEL1              0x02
#define LP_ODR_LPOSC_CLKSEL0              0x01

#define XOUT_H                            0x3B
#define XOUT_L                            0x3C
#define YOUT_H                            0x3D
#define YOUT_L                            0x3E
#define ZOUT_H                            0x3F
#define ZOUT_L                            0x40

#define MOT_DETECT_CTRL                   0x69
#define MOT_DETECT_CTRL_ACCEL_INT_EL_EN   0x80
#define MOT_DETECT_CTRL_ACCEL_INT_EL_MODE 0x40

#define XA_OFFSET_H                       0x77
#define XA_OFFSET_L                       0x78
#define YA_OFFSET_H                       0x7A
#define YA_OFFSET_L                       0x7B
#define ZA_OFFSET_H                       0x7D
#define ZA_OFFSET_L                       0x7E

#define SENSITIVITY_FOR_2G                16384.0f    // [LSB/g]
#define SENSITIVITY_FOR_4G                8192.0f     // [LSB/g]
#define SENSITIVITY_FOR_8G                4096.0f     // [LSB/g]
#define SENSITIVITY_FOR_16G               2048.0f     // [LSB/g]

#define CALIBRATION_TIME                  100

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// FUNCTIONS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

Accel9250::Accel9250 (I2cHw & vI2cHw) : Imu <Accel9250> (CALIBRATION_TIME), i2cHw (vI2cHw) {}

void Accel9250::Init (void)
{
    uint8_t data = CONFIG_FS_SEL1 | CONFIG_FS_SEL0;
    i2cHw.Send (I2cHw::EI2c::Zero, CONFIG, &data, 1);

    data = CONFIG2_FCHOICE_B0 | CONFIG2_A_DLPF_CFG0;
    i2cHw.Send (I2cHw::EI2c::Zero, CONFIG2, &data, 1);

    uint8_t sensitivity = 0;
    i2cHw.Receive  (I2cHw::EI2c::Zero, CONFIG, &sensitivity, 1);
    LOGI (module, "Sensitivity read: %d", sensitivity);
    setSensitivity (sensitivity);
}

ImuSettings::Axes Accel9250::GetAxes (void)
{
    std::array <uint8_t, 6> data;
    i2cHw.Receive (I2cHw::EI2c::Zero, XOUT_H, data.data (), data.size ());

    const int16_t x = static_cast<int16_t>((data [0] << 8) | data [1]);
    const int16_t y = static_cast<int16_t>((data [2] << 8) | data [3]);
    const int16_t z = static_cast<int16_t>((data [4] << 8) | data [5]);

    return { static_cast<float>((x - Settings::GetInst ()->Imu.Accel.Offset.X) / Sensitivity),
             static_cast<float>((y - Settings::GetInst ()->Imu.Accel.Offset.Y) / Sensitivity),
             static_cast<float>((z - Settings::GetInst ()->Imu.Accel.Offset.Z) / Sensitivity) };
}

void Accel9250::Calibrate (void)
{
    LOGI (module, "Stand still. Calibrate for 1s");

    Settings::GetInst ()->Imu.Accel.Offset.X = 0;
    Settings::GetInst ()->Imu.Accel.Offset.Y = 0;
    Settings::GetInst ()->Imu.Accel.Offset.Z = 0;

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

    Settings::GetInst ()->Imu.Accel.Offset.X = xSum * Sensitivity / numOfSamples;
    Settings::GetInst ()->Imu.Accel.Offset.Y = ySum * Sensitivity / numOfSamples;
    Settings::GetInst ()->Imu.Accel.Offset.Z = Sensitivity - (zSum * Sensitivity / numOfSamples);

    LOGI (module, "Offsets: X: " + std::to_string (Settings::GetInst ()->Imu.Accel.Offset.X) +
                  "[LSB], Y: "   + std::to_string (Settings::GetInst ()->Imu.Accel.Offset.Y) +
                  "[LSB], Z: "   + std::to_string (Settings::GetInst ()->Imu.Accel.Offset.Z) + "[LSB]");
}

void Accel9250::setSensitivity (const uint8_t vData)
{
    const bool fsSel1 = (vData & CONFIG_FS_SEL1);
    const bool fsSel0 = (vData & CONFIG_FS_SEL0);

    if      (fsSel1 == false && fsSel0 == false) { Sensitivity = SENSITIVITY_FOR_2G;  }
    else if (fsSel1 == false && fsSel0 == true ) { Sensitivity = SENSITIVITY_FOR_4G;  }
    else if (fsSel1 == true  && fsSel0 == false) { Sensitivity = SENSITIVITY_FOR_8G;  }
    else if (fsSel1 == true  && fsSel0 == true ) { Sensitivity = SENSITIVITY_FOR_16G; }
    else                                         { Sensitivity = SENSITIVITY_FOR_2G;  }
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// END OF FILE ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
