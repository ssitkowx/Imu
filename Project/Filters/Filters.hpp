#pragma once

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INCLUDES /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include "Settings.hpp"
#include "TimeoutHw.hpp"

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// CLASSES/STRUCTURES ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

class Filters final
{
    public:
        Filters () = default;
        ~Filters () = default;

        ImuSettings::Angles Kalman1Process    (const ImuSettings::Axes vAccel, const ImuSettings::Axes vGyro, const double vDiffTime);
        ImuSettings::Angles Kalman2Process    (const ImuSettings::Axes vAccel, const ImuSettings::Axes vGyro, const double vDiffTime);

        ImuSettings::Angles GetMagYawProcess  (const ImuSettings::Axes vAccel, const ImuSettings::Axes vMag);
        ImuSettings::Angles QuaternionProcess (const ImuSettings::Axes vAccel, const ImuSettings::Axes vGyro, const ImuSettings::Axes vMag, const double vDiffTime);

    private:
        struct Kalmans
        {
            struct Kalman1
            {
                struct Coefs
                {
                    double QBias    = 0.003f;
                    double QAngle   = 0.001f;
                    double RMeasure = 0.03f;
                };

                const Coefs Coef;
                double      Bias;
                double      Angle;
                double      Data [2][2];
            };

            struct
            {
                Kalman1 X;
                Kalman1 Y;
            } One;

            struct
            {
                const float Coef = 0.1f;
            } Two;

            struct
            {
                double A          [2][2] = { 1   , 0, 0, 1    };
                double B          [2][2] = { 0   , 0, 0, 1    };
                double H          [2][2] = { 1   , 0, 0, 1    };
                double Q          [2][2] = { 0.01, 0, 0, 0.01 };
                double R          [2][2] = { 0.01, 0, 0, 0.01 };
                double P          [2][2] = { 1   , 0, 0, 1    };
                double Velocities [2]    = { 0   , 0          };
            } Three;
        } kalman;

        static constexpr float beta        = 0.6045998f;
        static constexpr float declination = 6.3f;

        float                  q [4]       = { 1.0f, 0.0f, 0.0f, 0.0f };
        TimeoutHw              timeoutHw;

        void   kalman3PredictGyro  (const double vGyro);
        void   kalman3PredictAccel (const double vAccel);
        double kalman1GetAngle     (Kalmans::Kalman1 & vKalman1, double vNewAngle, double vNewRate, double vDiffTime);
};

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// END OF FILE ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
