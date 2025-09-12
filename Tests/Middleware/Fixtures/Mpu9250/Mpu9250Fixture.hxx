#pragma once

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INCLUDES /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include "I2cHw.hpp"
#include "RtosHw.hpp"
#include "Mpu9250.hpp"
#include "LoggerHw.hpp"
#include <gtest/gtest.h>
#include "Mag9250Mock.hpp"
#include "Gyro9250Mock.hpp"
#include "Accel9250Mock.hpp"

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// VARIABLES ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

using ::testing::_;
using ::testing::AnyNumber;

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// CLASSES/STRUCTURES ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

class Mpu9250Fixture : public ::testing::Test
{
    public:
        static constexpr char * Module = (char *)"Mpu9250Fixture";
        
        class I2cHw         I2cHw;
        class Mag9250Mock   MagMock;
        class Gyro9250Mock  GyroMock;
        class Accel9250Mock AccelMock;
        class Mpu9250       Mpu9250;

        Mpu9250Fixture () : MagMock   (I2cHw),
                            GyroMock  (I2cHw),
                            AccelMock (I2cHw),
                            Mpu9250   (I2cHw, MagMock, GyroMock, AccelMock)
        {
            SET_RTOS_INST   (&rtosHw);
            SET_LOGGER_INST (&loggerHw);
        }
        ~Mpu9250Fixture () = default;

        void TestBody () override { }

    protected:
        static inline class Settings settings;
        class               RtosHw   rtosHw;
        class               LoggerHw loggerHw;

        static void SetUpTestSuite ()
        {
            SET_SETTINGS_INST (&settings);
        }
        
        void SetUp () override
        {
            //Mpu9250.IsMagEnabled = true;
            
            Settings::GetInst ()->Imu.Accel.Offset.X = 0;
            Settings::GetInst ()->Imu.Accel.Offset.Y = 0;
            Settings::GetInst ()->Imu.Accel.Offset.Z = 0;

            Settings::GetInst ()->Imu.Gyro.Offset.X = 0;
            Settings::GetInst ()->Imu.Gyro.Offset.Y = 0;
            Settings::GetInst ()->Imu.Gyro.Offset.Z = 0;

            Settings::GetInst ()->Imu.Mag.Scale.X = 1;
            Settings::GetInst ()->Imu.Mag.Scale.Y = 1;
            Settings::GetInst ()->Imu.Mag.Scale.Z = 1;

            Settings::GetInst ()->Imu.Mag.Offset.X = 0;
            Settings::GetInst ()->Imu.Mag.Offset.Y = 0;
            Settings::GetInst ()->Imu.Mag.Offset.Z = 0;

            EXPECT_CALL (I2cHw , InitMag         ())          .Times (AnyNumber ());
            EXPECT_CALL (I2cHw , InitAcclAndGyro ())          .Times (AnyNumber ());
            EXPECT_CALL (I2cHw , Send            (_, _, _, _)).Times (AnyNumber ());
            EXPECT_CALL (I2cHw , Receive         (_, _, _, _)).Times (AnyNumber ());
            EXPECT_CALL (rtosHw, DelayInMs       (_))         .Times (AnyNumber ());
        }
        
        void TearDown () override { }
};

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// END OF FILE ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
