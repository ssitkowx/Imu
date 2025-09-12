///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INCLUDES /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <tuple>
#include <math.h>
#include "Utils.hpp"
#include "RtosHw.hpp"
#include "Mag9250.hpp"
#include "LoggerHw.hpp"

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// MACROS/DEFINITIONS ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define WIA                      0x00
#define WIA_DATA                 0x48

#define INFO                     0x01

#define ST1                      0x02
#define ST1_DOR                  0x02
#define ST1_DRDY                 0x01

#define HXL                      0x03
#define HYL                      0x05
#define HYH                      0x06
#define HZL                      0x07
#define HZH                      0x08

#define ST2                      0x09
#define ST2_BITM                 0x10
#define ST2_HOFL                 0x08

#define CNTL1                    0x0A
#define CNTL1_BIT                0x10
#define CNTL1_MOD3               0x08
#define CNTL1_MOD2               0x04
#define CNTL1_MOD1               0x02
#define CNTL1_MOD0               0x01

#define CNTL2                    0x0B
#define CNTL2_SRST               0x01

#define ASTC                     0x0C
#define ASTC_SELF                0x40

#define TS1                      0x0D
#define TS2                      0x0E
#define I2CDIS                   0x0F
#define ASAX                     0x10
#define ASAY                     0x11
#define ASAZ                     0x12

#define OFFSETS_ZERO_SAMPLES_NUM 50

#define SENSITIVITY_FOR_14_BIT   (10.0f * 4912.0f / 8190.0f)     // [mili Gaus]
#define SENSITIVITY_FOR_16_BIT   (10.0f * 4912.0f / 32760.0f)    // [mili Gaus]

#define CALIBRATION_TIME         600
//#define CALIBRATION_TIME         40

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// FUNCTIONS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

Mag9250::Mag9250 (I2cHw & vI2cHw) : Imu <Mag9250> (CALIBRATION_TIME), i2cHw (vI2cHw) {}

void Mag9250::Init (void)
{
    std::array <uint8_t, 3> data;
    i2cHw.Receive (I2cHw::EI2c::One, WIA, (uint8_t *)&data, data.size ());

    LOGI (module, "Data from Mag: %d", data [0]);

    if (data [0] != WIA_DATA) { LOGE (module, "Device not found"); return; }

    i2cHw.Send (I2cHw::EI2c::One, CNTL1, 0x00, 0);
    RtosHw::GetInst ()->DelayInMs (10);

    data [0] = CNTL1_MOD3 |
               CNTL1_MOD2 |
               CNTL1_MOD1 |
               CNTL1_MOD0;
    i2cHw.Send (I2cHw::EI2c::One, CNTL1, &data [0], 1);
    RtosHw::GetInst ()->DelayInMs (10);

    i2cHw.Receive (I2cHw::EI2c::One, ASAX, (uint8_t *)&data, data.size ());
    Calibration.Factory.X = (float)(data [0] - 128) / 256.0f + 1.0f;
    Calibration.Factory.Y = (float)(data [1] - 128) / 256.0f + 1.0f;
    Calibration.Factory.Z = (float)(data [2] - 128) / 256.0f + 1.0f;

    data [0] = 0x00;
    i2cHw.Send (I2cHw::EI2c::One, CNTL1, &data [0], 1);
    RtosHw::GetInst ()->DelayInMs (10);

    data [0] = CNTL1_BIT  |
               CNTL1_MOD2 |
               CNTL1_MOD1;
    i2cHw.Send (I2cHw::EI2c::One, CNTL1, &data [0], 1);
    RtosHw::GetInst ()->DelayInMs (10);

    data [0] =  ST2;
    i2cHw.Send (I2cHw::EI2c::One, CNTL1_BIT, &data [0], 1);

    setSensitivity (data [0]);
}

ImuSettings::Axes Mag9250::GetAxes (void)
{
    static int16_t x, y, z;

    uint8_t data;
    i2cHw.Receive (I2cHw::EI2c::One, ST1, &data, 1);

    if ((data & ST1_DRDY) == true)
    {
        std::array <uint8_t, 7> data = {};
        i2cHw.Receive (I2cHw::EI2c::One, HXL, data.data (), data.size ());
        if ((data [6] & ST2_HOFL) == 0)
        {
            x = static_cast<int16_t>((data [1] << 8) | data [0]);
            y = static_cast<int16_t>((data [3] << 8) | data [2]);
            z = static_cast<int16_t>((data [5] << 8) | data [4]);
            RtosHw::GetInst ()->DelayInMs (10);
        }
    }

    return { static_cast<float>((x - Settings::GetInst ()->Imu.Mag.Offset.X) * Sensitivity * Calibration.Factory.X * Settings::GetInst ()->Imu.Mag.Scale.X),
             static_cast<float>((y - Settings::GetInst ()->Imu.Mag.Offset.Y) * Sensitivity * Calibration.Factory.Y * Settings::GetInst ()->Imu.Mag.Scale.Y),
             static_cast<float>((z - Settings::GetInst ()->Imu.Mag.Offset.Z) * Sensitivity * Calibration.Factory.Z * Settings::GetInst ()->Imu.Mag.Scale.Z) };
}

void Mag9250::Calibrate (void)
{
    if (CALIBRATION_TIME == 0) { return; }

    LOGD (module, "Rotate device. Calibrate for 1min");

    Settings::GetInst ()->Imu.Mag.Scale.X  = 1;
    Settings::GetInst ()->Imu.Mag.Scale.Y  = 1;
    Settings::GetInst ()->Imu.Mag.Scale.Z  = 1;

    Settings::GetInst ()->Imu.Mag.Offset.X = 0;
    Settings::GetInst ()->Imu.Mag.Offset.Y = 0;
    Settings::GetInst ()->Imu.Mag.Offset.Z = 0;

    float xMin =  32767, yMin =  32767, zMin =  32767;
    float xMax = -xMin , yMax = -yMin , zMax = -zMin;

    Settings::GetInst ()->Timer.Imu.IsOn = true;
    while (IsCalibInProgress () == true)
    {
        const ImuSettings::Axes axes = GetAxes ();

        LOGI (module, "Mag: X: %f, Y: %f, Z: %f", axes.X, axes.Y, axes.Z);

        if (axes.X < xMin)
        {
            xMin = axes.X;
            LOGI (module, "xMin: " + std::to_string (xMin));
        }
        if (axes.Y < yMin)
        {
            yMin = axes.Y;
            LOGI (module, "yMin: " + std::to_string (yMin));
        }
        if (axes.Z < zMin)
        {
            zMin = axes.Z;
            LOGI (module, "zMin: " + std::to_string (zMin));
        }
        if (axes.X > xMax)
        {
            xMax = axes.X;
            LOGD (module, "xMax: " + std::to_string (xMax));
        }
        if (axes.Y > yMax)
        {
            yMax = axes.Y;
            LOGI (module, "yMax: " + std::to_string (yMax));
        }
        if (axes.Z > zMax)
        {
            zMax = axes.Z;
            LOGI (module, "zMax: " + std::to_string (zMax));
        }
    }

    Settings::GetInst ()->Timer.Imu.IsOn    = false;
    Settings::GetInst ()->Timer.Imu.Counter = 0;

    calcHardIronOffset ({ xMin, yMin, zMin }, { xMax, yMax, zMax });
    calcSoftIronCoeff  ({ xMin, yMin, zMin }, { xMax, yMax, zMax });

    LOGI (module, "xMin: " + std::to_string (xMin) + " yMin: " + std::to_string (yMin) + " zMin: " + std::to_string (zMin));
    LOGI (module, "xMax: " + std::to_string (xMax) + " yMax: " + std::to_string (yMax) + " zMax: " + std::to_string (zMax));
    LOGI (module, "Hard iron. Offsets: X: " + std::to_string (Settings::GetInst ()->Imu.Mag.Offset.X) + "[LSB], Y: " + std::to_string (Settings::GetInst ()->Imu.Mag.Offset.Y) + "[LSB], Z: " + std::to_string (Settings::GetInst ()->Imu.Mag.Offset.Z) + "[LSB]");
    LOGI (module, "Soft iron. Scale  : X: " + std::to_string (Settings::GetInst ()->Imu.Mag.Scale.X) + " Y: "        + std::to_string (Settings::GetInst ()->Imu.Mag.Scale.Y)  + " Z: "       + std::to_string (Settings::GetInst ()->Imu.Mag.Scale.Z));
}

void Mag9250::setSensitivity (const uint8_t vData)
{
    const bool bitm = vData & CNTL1_BIT;

    if (bitm == false) { Sensitivity = SENSITIVITY_FOR_14_BIT; }
    else               { Sensitivity = SENSITIVITY_FOR_16_BIT; }
}

void Mag9250::calcSoftIronCoeff (const ImuSettings::Axes vMin, const ImuSettings::Axes vMax)
{
    const float deltaX  = static_cast <float>(0.5 * (vMax.X - vMin.X));
    const float deltaY  = static_cast <float>(0.5 * (vMax.Y - vMin.Y));
    const float deltaZ  = static_cast <float>(0.5 * (vMax.Z - vMin.Z));
    const float delta   = static_cast <float>((deltaX + deltaY + deltaZ) / 3);

    if (deltaX == 0 ||
        deltaY == 0 ||
        deltaZ == 0 )
    {
        LOGE (module, "Delta can't be zero. Calibration incorrect");
        return;
    }

    Settings::GetInst ()->Imu.Mag.Scale.X = static_cast <float>(delta / deltaX);
    Settings::GetInst ()->Imu.Mag.Scale.Y = static_cast <float>(delta / deltaY);
    Settings::GetInst ()->Imu.Mag.Scale.Z = static_cast <float>(delta / deltaZ);
}

void Mag9250::calcHardIronOffset (const ImuSettings::Axes vMin, const ImuSettings::Axes vMax)
{
    Settings::GetInst ()->Imu.Mag.Offset.X = static_cast <float>(0.5 * (vMax.X + vMin.X) / (Sensitivity * Calibration.Factory.X * Settings::GetInst ()->Imu.Mag.Scale.X));
    Settings::GetInst ()->Imu.Mag.Offset.Y = static_cast <float>(0.5 * (vMax.Y + vMin.Y) / (Sensitivity * Calibration.Factory.Y * Settings::GetInst ()->Imu.Mag.Scale.Y));
    Settings::GetInst ()->Imu.Mag.Offset.Z = static_cast <float>(0.5 * (vMax.Z + vMin.Z) / (Sensitivity * Calibration.Factory.Z * Settings::GetInst ()->Imu.Mag.Scale.Z));
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// END OF FILE ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
