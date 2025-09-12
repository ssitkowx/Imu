#pragma once

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INCLUDES /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include "Imu.hpp"
#include "I2cHw.hpp"
#include "Settings.hpp"

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// CLASSES/STRUCTURES ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

class Mag9250 : public Imu<Mag9250>
{
    static constexpr char * module = (char *)"Mag9250";
    friend Imu<Mag9250>;

    public:
        struct
        {
            ImuSettings::Axes Factory = { 1, 1, 1 };
        } Calibration;

        explicit Mag9250 (I2cHw & vI2cHw);
        ~Mag9250 () = default;

        void              Init              (void);
        ImuSettings::Axes GetAxes           (void);
        void              Calibrate         (void);
        bool              IsCalibInProgress (void) { return Settings::GetInst ()->Timer.Imu.Counter < CalibTime; }

    private:
        I2cHw & i2cHw;

        void setSensitivity     (const uint8_t vData);
        void calcSoftIronCoeff  (const ImuSettings::Axes vMin, const ImuSettings::Axes vMax);
        void calcHardIronOffset (const ImuSettings::Axes vMin, const ImuSettings::Axes vMax);
};

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// END OF FILE ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
