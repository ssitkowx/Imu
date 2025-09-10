///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INCLUDES /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include "RtosHw.hpp"
#include <string_view>
#include "LoggerHw.hpp"
#include "Mpu9250Fixture.hxx"

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// VARIABLES ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

using ::testing::Return;
using ::testing::Invoke;
using ::testing::InSequence;
using ::testing::SetArgPointee;

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// FUNCTIONS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST_F (Mpu9250Fixture, Calibrate)
{
    LOGW (Module, "Calibrate");

    EXPECT_CALL (I2cHw, Receive (I2cHw::EI2c::One, 0, _, 3))    .Times    (1)
                                                                .WillOnce (SetArgPointee<2>(0x48));
    EXPECT_CALL (I2cHw, Receive (I2cHw::EI2c::Zero, 0x75, _, 1)).Times    (1)
                                                                .WillOnce (SetArgPointee<2>(0x71));

    std::array<uint8_t, 6> gyroData  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    std::array<uint8_t, 6> accelData = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    std::array<uint8_t, 7> magData   = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    ::testing::InSequence inSequence;
    for (uint8_t iter = 0; iter < 10; iter++)
    {
        EXPECT_CALL (GyroMock, IsCalibInProgress ())                   .Times          (1)
                                                                       .WillRepeatedly (Return (true));
        EXPECT_CALL (I2cHw   , Receive (I2cHw::EI2c::Zero, 0x43, _, 6)).Times          (1)
                                                                       .WillRepeatedly (Invoke ([&](const I2cHw::EI2c, const uint16_t, uint8_t * const vData, const uint16_t vLen)
        {
            const uint16_t len = std::min<uint16_t>(vLen, 6);
            gyroData [1] += 10;
            gyroData [3] += 10;
            gyroData [5] += 10;
            memcpy (vData, gyroData.data (), len);
        }));
    }

    EXPECT_CALL (GyroMock, IsCalibInProgress ()).WillRepeatedly (Return (false));

    for (uint8_t iter = 0; iter < 10; iter++)
    {
        EXPECT_CALL (AccelMock, IsCalibInProgress ())                   .Times          (1)
                                                                        .WillRepeatedly (Return (true));
        EXPECT_CALL (I2cHw    , Receive (I2cHw::EI2c::Zero, 0x3B, _, 6)).Times          (1)
                                                                        .WillRepeatedly (Invoke ([&](const I2cHw::EI2c, const uint16_t, uint8_t * const vData, const uint16_t vLen)
        {
            const uint16_t len = std::min<uint16_t>(vLen, 6);
            accelData [1] += 20;
            accelData [3] += 20;
            accelData [5] += 20;
            memcpy (vData, accelData.data (), len);
        }));
    }

    EXPECT_CALL (AccelMock, IsCalibInProgress ()).WillRepeatedly (Return (false));

    for (uint8_t iter = 0; iter < 10; iter++)
    {
        EXPECT_CALL (MagMock, IsCalibInProgress ())                   .Times          (1)
                                                                      .WillRepeatedly (Return (true));
        EXPECT_CALL (I2cHw  , Receive (I2cHw::EI2c::One , 0x02, _, 1)).Times          (1)
                                                                      .WillOnce       (SetArgPointee<2>(0x01));
        
        EXPECT_CALL (I2cHw  , Receive (I2cHw::EI2c::One, 0x03, _, 7)) .Times          (1)
                                                                      .WillRepeatedly (Invoke ([&](const I2cHw::EI2c, const uint16_t, uint8_t * const vData, const uint16_t vLen)
        {
            const uint16_t len = std::min<uint16_t>(vLen, 7);
            magData [0] += 15;
            magData [2] += 15;
            magData [4] += 15;
            memcpy (vData, magData.data (), len);
        }));
    }

    EXPECT_CALL (MagMock, IsCalibInProgress ()).WillRepeatedly (Return (false));

    Mpu9250.Init      ();
    Mpu9250.Calibrate ();

    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Gyro.Offset.X, 54);
    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Gyro.Offset.Y, 54);
    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Gyro.Offset.Z, 54);
    
    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Accel.Offset.X, 110);
    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Accel.Offset.Y, 110);
    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Accel.Offset.Z, 16274);

    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Mag.Scale.X, 0);
    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Mag.Scale.Y, 1);
    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Mag.Scale.Z, 1);

    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Mag.Offset.X, 82);
    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Mag.Offset.Y, 82);
    EXPECT_EQ ((uint16_t)Settings::GetInst ()->Imu.Mag.Offset.Z, 82);
}

TEST_F (Mpu9250Fixture, CheckQuaternionAngles)
{
    LOGW (Module, "Quaternion");

    EXPECT_CALL (I2cHw, Receive (I2cHw::EI2c::One, 0, _, 3))    .Times    (1)
                                                                .WillOnce (SetArgPointee<2>(0x48));
    EXPECT_CALL (I2cHw, Receive (I2cHw::EI2c::Zero, 0x75, _, 1)).Times    (1)
                                                                .WillOnce (SetArgPointee<2>(0x71));

    std::array<uint8_t, 6> gyroData  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    std::array<uint8_t, 6> accelData = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    std::array<uint8_t, 7> magData   = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    ::testing::InSequence inSequence;
    EXPECT_CALL (I2cHw, Receive (I2cHw::EI2c::Zero, 0x3B, _, 6)).Times          (1)
                                                                .WillRepeatedly (Invoke ([&](const I2cHw::EI2c, const uint16_t, uint8_t * const vData, const uint16_t vLen)
    {
        const uint16_t len = std::min<uint16_t>(vLen, 6);
        accelData [1] += 2;
        accelData [3] += 2;
        accelData [5] += 2;
        memcpy (vData, accelData.data (), len);
    }));

    EXPECT_CALL (I2cHw, Receive (I2cHw::EI2c::Zero, 0x43, _, 6)).Times          (1)
                                                                .WillRepeatedly (Invoke ([&](const I2cHw::EI2c, const uint16_t, uint8_t * const vData, const uint16_t vLen)
    {
        const uint16_t len = std::min<uint16_t>(vLen, 6);
        gyroData [1] += 1;
        gyroData [3] += 1;
        gyroData [5] += 1;
        memcpy (vData, gyroData.data (), len);
    }));

    EXPECT_CALL (I2cHw, Receive (I2cHw::EI2c::One , 0x02, _, 1)).Times          (1)
                                                                .WillOnce       (SetArgPointee<2>(0x01));
    
    EXPECT_CALL (I2cHw, Receive (I2cHw::EI2c::One, 0x03, _, 7)) .Times          (1)
                                                                .WillRepeatedly (Invoke ([&](const I2cHw::EI2c, const uint16_t, uint8_t * const vData, const uint16_t vLen)
    {
        const uint16_t len = std::min<uint16_t>(vLen, 7);
        magData [0] += 2;
        magData [2] += 1;
        magData [4] += 2;
        memcpy (vData, magData.data (), len);
    }));

    EXPECT_CALL (Mpu9250.TimeoutHw, GetDiffTime ()).WillRepeatedly (Return (1000000));

    Mpu9250.Init    ();
    Mpu9250.Process ();

    EXPECT_EQ ((int16_t)Settings::GetInst ()->Imu.Angle.Roll , 56);
    EXPECT_EQ ((int16_t)Settings::GetInst ()->Imu.Angle.Pitch,-29);
    EXPECT_EQ ((int16_t)Settings::GetInst ()->Imu.Angle.Yaw  , 336);
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// END OF FILE ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
