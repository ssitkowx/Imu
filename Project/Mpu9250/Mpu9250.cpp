///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INCLUDES /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include "RtosHw.hpp"
#include "Mpu9250.hpp"
#include "Settings.hpp"
#include "LoggerHw.hpp"

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// MACROS/DEFINITIONS ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define SMPLRT_DIV                        0x19

#define CONFIG                            0x1A
#define CONFIG_FIFO_MODE                  0x40
#define CONFIG_EXT_SYNC_SET2              0x20
#define CONFIG_EXT_SYNC_SET1              0x10
#define CONFIG_EXT_SYNC_SET0              0x08
#define CONFIG_EXT_DLPF_CFG2              0x04
#define CONFIG_EXT_DLPF_CFG1              0x02
#define CONFIG_EXT_DLPF_CFG0              0x01

#define WOM_THR                           0x1F

#define FIFO_EN                           0x23
#define FIFO_EN_TEMP_FIFO_EN              0x80
#define FIFO_EN_GYRO_XOUT                 0x40
#define FIFO_EN_GYRO_YOUT                 0x20
#define FIFO_EN_GYRO_ZOUT                 0x10
#define FIFO_EN_ACCEL                     0x08
#define FIFO_EN_SLV2                      0x04
#define FIFO_EN_SLV1                      0x02
#define FIFO_EN_SLV0                      0x01

#define I2C_MST_CTRL                      0x24
#define I2C_MST_CTRL_MULT_MST_EN          0x80
#define I2C_MST_CTRL_WAIT_FOR_ES          0x40
#define I2C_MST_CTRL_SLV_3_FIFO_EN        0x20
#define I2C_MST_CTRL_I2C_MST_P_NSR        0x10
#define I2C_MST_CTRL_I2C_MST_CLK3         0x08
#define I2C_MST_CTRL_I2C_MST_CLK2         0x04
#define I2C_MST_CTRL_I2C_MST_CLK1         0x02
#define I2C_MST_CTRL_I2C_MST_CLK0         0x01

#define I2C_SLV0_ADDR                     0x25
#define I2C_SLV0_REG                      0x26
#define I2C_SLV0_CTRL                     0x27
#define I2C_SLV1_ADDR                     0x28
#define I2C_SLV1_REG                      0x29
#define I2C_SLV1_CTRL                     0x2A
#define I2C_SLV2_ADDR                     0x2B
#define I2C_SLV2_REG                      0x2C
#define I2C_SLV2_CTRL                     0x2D
#define I2C_SLV3_ADDR                     0x2E
#define I2C_SLV3_REG                      0x2F
#define I2C_SLV3_CTRL                     0x30
#define I2C_SLV4_ADDR                     0x31
#define I2C_SLV4_REG                      0x32
#define I2C_SLV4_DO                       0x33
#define I2C_SLV4_CTRL                     0x34
#define I2C_SLV4_DI                       0x35
#define I2C_MST_STATUS                    0x36

#define INT_PIN_CFG                       0x37
#define INT_PIN_CFG_ACTL                  0x80
#define INT_PIN_CFG_OPEN                  0x40
#define INT_PIN_CFG_LATCH_INT_EN          0x20
#define INT_PIN_CFG_INT_ANYRD_2CLEAR      0x10
#define INT_PIN_CFG_ACTL_FSY_NC           0x08
#define INT_PIN_CFG_FSYNC_INT_MOD_E_EN    0x04
#define INT_PIN_CFG_BYPASS_EN             0x02

#define INT_ENABLE                        0x38
#define INT_ENABLE_WOM_EN                 0x40
#define INT_ENABLE_FIFO_OFLOW_EN          0x10
#define INT_ENABLE_FSYNC_INT_EN           0x08
#define INT_ENABLE_RAW_RDY_EN             0x01

#define INT_STATUS                        0x3A
#define INT_STATUS_WOM_INT                0x40
#define INT_STATUS_FIFO_OFLOW_INT         0x10
#define INT_STATUS_FSYNC_INT              0x08
#define INT_STATUS_RAW_DATA_RDY_INT       0x01

#define TEMP_OUT_H                        0x41
#define TEMP_OUT_L                        0x42
#define EXT_SENS_DATA_00                  0x49
#define EXT_SENS_DATA_01                  0x4A
#define EXT_SENS_DATA_02                  0x4B
#define EXT_SENS_DATA_03                  0x4C
#define EXT_SENS_DATA_04                  0x4D
#define EXT_SENS_DATA_05                  0x4E
#define EXT_SENS_DATA_06                  0x4F
#define EXT_SENS_DATA_07                  0x50
#define EXT_SENS_DATA_08                  0x51
#define EXT_SENS_DATA_09                  0x52
#define EXT_SENS_DATA_10                  0x53
#define EXT_SENS_DATA_11                  0x54
#define EXT_SENS_DATA_12                  0x55
#define EXT_SENS_DATA_13                  0x56
#define EXT_SENS_DATA_14                  0x57
#define EXT_SENS_DATA_15                  0x58
#define EXT_SENS_DATA_16                  0x59
#define EXT_SENS_DATA_17                  0x5A
#define EXT_SENS_DATA_18                  0x5B
#define EXT_SENS_DATA_19                  0x5C
#define EXT_SENS_DATA_20                  0x5D
#define EXT_SENS_DATA_21                  0x5E
#define EXT_SENS_DATA_22                  0x5F
#define EXT_SENS_DATA_23                  0x60
#define I2C_SLV0_DO                       0x63
#define I2C_SLV1_DO                       0x64
#define I2C_SLV2_DO                       0x65
#define I2C_SLV3_DO                       0x66
#define I2C_MST_DELAY_CTRL                0x67
#define SIGNAL_PATH_RESET                 0x68

#define SIGNAL_PATH_RESET_GYRO_RST        0x04
#define SIGNAL_PATH_RESET_ACCEL_RST       0x02
#define SIGNAL_PATH_RESET_TEMP_RST        0x01

#define USER_CTRL                         0x6A
#define USER_CTRL_FIFO_EN                 0x40
#define USER_CTRL_I2C_MST_EN              0x20
#define USER_CTRL_I2C_IF_DIS              0x10
#define USER_CTRL_I2C_FIFO_RST            0x04
#define USER_CTRL_I2C_I2C_MST_RST         0x02
#define USER_CTRL_I2C_SIG_COND_RST        0x01

#define PWR_MGMT_1                        0x6B
#define PWR_MGMT_1_H_RESET                0x80
#define PWR_MGMT_1_SLEEP                  0x40
#define PWR_MGMT_1_CYCLE                  0x20
#define PWR_MGMT_1_GYRO_STANDBY           0x10
#define PWR_MGMT_1_PD_PTAT                0x08
#define PWR_MGMT_1_CLKSEL2                0x04
#define PWR_MGMT_1_CLKSEL1                0x02
#define PWR_MGMT_1_CLKSEL0                0x01

#define PWR_MGMT_2                        0x6C
#define PWR_MGMT_2_DIS_XA                 0x20
#define PWR_MGMT_2_DIS_YW                 0x10
#define PWR_MGMT_2_DIS_ZA                 0x08
#define PWR_MGMT_2_DIS_XG                 0x04
#define PWR_MGMT_2_DIS_YG                 0x02
#define PWR_MGMT_2_DIS_ZG                 0x01

#define PFIFO_COUNTH                      0x72
#define PFIFO_COUNTL                      0x73
#define FIFO_R_W                          0x74

#define WHO_AM_I                          0x75
#define WHO_AM_I_DATA                     0x71

#define ANGLE_COEF                        0.05f
#define LOW_PASS_FILTER_COEFF             0.4f
#define HIGH_PASS_FILTER_COEFF           (ONE - LOW_PASS_FILTER_COEFF)

#define MICRO_TO_SECONDS                  0.000001f

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// FUNCTIONS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void Mpu9250::Init (void)
{
    uint8_t data = 0x00;
    i2cHw.Receive (I2cHw::EI2c::Zero, PWR_MGMT_1, &data, 1);
    RtosHw::GetInst ()->DelayInMs (100);

    data = PWR_MGMT_1_CLKSEL0;
    i2cHw.Send (I2cHw::EI2c::Zero, PWR_MGMT_1, &data, 0);
    RtosHw::GetInst ()->DelayInMs (100);

    data = CONFIG_EXT_DLPF_CFG2 |
           CONFIG_EXT_DLPF_CFG1 |
           CONFIG_EXT_DLPF_CFG0;
    i2cHw.Send (I2cHw::EI2c::Zero, CONFIG, &data, 1);

    data = 0x04;
    i2cHw.Send (I2cHw::EI2c::Zero, SMPLRT_DIV, &data, 1);

    data = 0x00;
    i2cHw.Send (I2cHw::EI2c::Zero, FIFO_EN, &data, 1);

    data = I2C_MST_CTRL_MULT_MST_EN  |
           I2C_MST_CTRL_I2C_MST_CLK3 |
           I2C_MST_CTRL_I2C_MST_CLK2 |
           I2C_MST_CTRL_I2C_MST_CLK0;
    i2cHw.Send (I2cHw::EI2c::Zero, I2C_MST_CTRL, &data, 1);

    data = 0;
    i2cHw.Receive (I2cHw::EI2c::Zero, WHO_AM_I, &data, 1);

    if      (data == WHO_AM_I_DATA) { LOGI (module, "Detected Mpu9250"); }
    else if (data == 0x70)
    {
        LOGI (module, "Detected Mpu6500. Mag disabled");
        IsMagEnabled = false;
    }
    else
    {
        LOGE (module, "Unknown device: 0x%02X", data);
        return;
    }

    if (IsMagEnabled == true)
    {
        data = INT_PIN_CFG_LATCH_INT_EN | INT_PIN_CFG_BYPASS_EN;
        i2cHw.Send (I2cHw::EI2c::Zero, INT_PIN_CFG, &data, 1);

        data = INT_ENABLE_RAW_RDY_EN;
        i2cHw.Send (I2cHw::EI2c::Zero, INT_ENABLE, &data, 1);
        RtosHw::GetInst ()->DelayInMs (100);

        i2cHw.InitMag ();
        Mag  .Init    ();
    }

    Gyro .Init ();
    Accel.Init ();
}

void Mpu9250::Process (void)
{
    Settings::GetInst ()->Imu.Angle = getAngles ();
}

void Mpu9250::Calibrate (void)
{
    Gyro .Calibrate ();
    Accel.Calibrate ();

    if (IsMagEnabled == true) { Mag.Calibrate (); }
}

float Mpu9250::getTemp (void)
{
    uint8_t data [2] = {};
    i2cHw.Receive (I2cHw::EI2c::Zero, TEMP_OUT_H, data, 2);
    return (((data [0] << 8) | data [1]) - 21) / 333.87f - 21;
}

struct ImuSettings::Angles Mpu9250::getAngles (void)
{
    Settings::GetInst ()->Imu.EFilter = ImuSettings::EFilter::Quaternion;
    const ImuSettings::Axes acclAxes = Accel.GetAxes ();
    const ImuSettings::Axes gyroAxes = Gyro .GetAxes ();
    ImuSettings::Axes       magAxes  = { 1, 1, 1 };

    if (IsMagEnabled == true) { magAxes = Mag.GetAxes (); }

    ImuSettings::Angles angles   = {};
    const double        diffTime = TimeoutHw.GetDiffTime () * MICRO_TO_SECONDS;

    switch (Settings::GetInst ()->Imu.EFilter)
    {
        case ImuSettings::EFilter::MagYaw:
        {
            angles = filter.GetMagYawProcess ({ acclAxes.X,
                                                acclAxes.Y,
                                                acclAxes.Z
                                              },
                                              { magAxes.X,
                                                magAxes.Y,
                                                magAxes.Y
                                              });
            break;
        }
        case ImuSettings::EFilter::Kalman1:
        {
            angles = filter.Kalman1Process (acclAxes, gyroAxes, diffTime);
            break;
        }
        case ImuSettings::EFilter::Kalman2:
        {
            angles = filter.Kalman2Process (acclAxes, gyroAxes, diffTime);
            break;
        }
        case ImuSettings::EFilter::Quaternion:
        {
            angles = filter.QuaternionProcess ({ acclAxes.X,
                                                 acclAxes.Y,
                                                 acclAxes.Z
                                               },
                                               { static_cast <float>(gyroAxes.X / RAD_TO_DEG),
                                                 static_cast <float>(gyroAxes.Y / RAD_TO_DEG),
                                                 static_cast <float>(gyroAxes.Z / RAD_TO_DEG)
                                               },
                                               { magAxes.X,
                                                 magAxes.Y,
                                                 magAxes.Y
                                               },
                                                 diffTime);
            break;
        }
        default:
        {
            LOGE (module, "Unsupported filter");
        }
    };

    return angles;
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// END OF FILE ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
