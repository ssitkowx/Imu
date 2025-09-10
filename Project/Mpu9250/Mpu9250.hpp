#pragma once

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INCLUDES /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <array>
#include "Imu.hpp"
#include "Filters.hpp"
#include "Mag9250.hpp"
#include "Gyro9250.hpp"
#include "Accel9250.hpp"
#include "TimeoutHw.hpp"

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// CLASSES/STRUCTURES ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

class Mpu9250 final
{
    static constexpr char * module = (char *)"Mpu9250";

    public:
        Mpu9250 (I2cHw    & vI2cHw, Mag9250   & vMag,
                 Gyro9250 & vGyro , Accel9250 & vAccel) : i2cHw (vI2cHw),
                                                          Mag   (vMag),
                                                          Gyro  (vGyro),
                                                          Accel (vAccel) {}
        ~Mpu9250 () = default;

        bool IsMagEnabled = false;
        
        void Init      (void);
        void Process   (void);
        void Calibrate (void);

        Mag9250   & Mag;
        Gyro9250  & Gyro;
        Accel9250 & Accel;
        class TimeoutHw   TimeoutHw;

    private:
        Filters filter;
        I2cHw & i2cHw;

        float                      getTemp   (void);
        struct ImuSettings::Angles getAngles (void);
};

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// END OF FILE ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
