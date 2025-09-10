///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INCLUDES /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include "Filters.hpp"

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// MACROS/DEFINITIONS ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define PI 3.14159f

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// FUNCTIONS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

ImuSettings::Angles Filters::Kalman1Process (const ImuSettings::Axes vAccel, const ImuSettings::Axes vGyro, const double vDiffTime)
{
    double angleX, angleY;

    double rollA;
    const double normalA = sqrt (vAccel.X * vAccel.X + vAccel.Z * vAccel.Z);
    if (normalA != 0.0) { rollA = atan (vAccel.Y / normalA) * RAD_TO_DEG; }
    else                { rollA = 0.0; }

    const double pitchA = atan2 (-vAccel.X, vAccel.Z) * RAD_TO_DEG;
    if ((pitchA < -90 && vAccel.Y > 90) || (pitchA > 90 && vAccel.Y < -90))
    {
        angleY             = pitchA;
        kalman.One.Y.Angle = pitchA;
    }
    else
    {
        angleY = kalman1GetAngle (kalman.One.Y, pitchA, vGyro.Y, vDiffTime);
    }

    float gyroX = vGyro.X;
    if (fabs (angleY) > 90) { gyroX= -gyroX; }

    angleX = kalman1GetAngle (kalman.One.X, rollA, vGyro.Y, vDiffTime);

    return { (int16_t)angleX, (int16_t)angleY, 0 };
};

ImuSettings::Angles Filters::Kalman2Process (const ImuSettings::Axes vAccel, const ImuSettings::Axes vGyro, const double vDiffTime)
{
    static constexpr float     coef = 0.1f;
    static ImuSettings::Axes   gyroAxes;
    static ImuSettings::Angles angles;

    const float rollA    = atan2 (vAccel.Y , sqrt (pow (vAccel.X, 2) + pow (vAccel.Z, 2))) * RAD_TO_DEG;
    const float pitchA   = atan2 (-vAccel.X, sqrt (pow (vAccel.Y, 2) + pow (vAccel.Z, 2))) * RAD_TO_DEG;

    gyroAxes.X += vGyro.X * vDiffTime;
    gyroAxes.Y += vGyro.Y * vDiffTime;
    gyroAxes.Z += vGyro.Z * vDiffTime;

    angles.Roll  = coef * gyroAxes.X + (1 - coef) * rollA;
    angles.Pitch = coef * gyroAxes.Y + (1 - coef) * pitchA;

    return angles;
}

ImuSettings::Angles Filters::GetMagYawProcess (const ImuSettings::Axes vAccel, const ImuSettings::Axes vMag)
{
    const float roll  = atan2 (vAccel.Y , sqrt (pow (vAccel.X, 2) + pow (vAccel.Z, 2)));
    const float pitch = atan2 (-vAccel.X, sqrt (pow (vAccel.Y, 2) + pow (vAccel.Z, 2)));
    const float mx    = vMag.X * cos (pitch) + vMag.Y * sin (roll) * sin (pitch) - vMag.Z * cos (roll) * sin (pitch);
    const float my    = vMag.Y * cos (roll)  + vMag.Z * sin (roll);
    float yaw         = atan2 (my, mx) * RAD_TO_DEG + declination;

    if (yaw < 0) { yaw += 360.0f; }

    return { (int16_t)(roll * RAD_TO_DEG), (int16_t)(pitch * RAD_TO_DEG) , (int16_t)yaw };
}

ImuSettings::Angles Filters::QuaternionProcess (const ImuSettings::Axes vAccel, const ImuSettings::Axes vGyro, const ImuSettings::Axes vMag, const double vDiffTime)
{
    static ImuSettings::Angles angles;
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _4bx, _4bz;
    float _2q1mx, _2q1my, _2q1mz, _2q2mx;
    float _2q1   = 2.0f * q1;
    float _2q2   = 2.0f * q2;
    float _2q3   = 2.0f * q3;
    float _2q4   = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1   = q1 * q1;
    float q1q2   = q1 * q2;
    float q1q3   = q1 * q3;
    float q1q4   = q1 * q4;
    float q2q2   = q2 * q2;
    float q2q3   = q2 * q3;
    float q2q4   = q2 * q4;
    float q3q3   = q3 * q3;
    float q3q4   = q3 * q4;
    float q4q4   = q4 * q4;

    float norm = sqrtf (vAccel.X * vAccel.X + vAccel.Y * vAccel.Y + vAccel.Z * vAccel.Z);
    if (norm == 0.0F) { return angles; }

    norm = 1.0f / norm;
    const float ax = vAccel.X * norm;
    const float ay = vAccel.Y * norm;
    const float az = vAccel.Z * norm;

    const float gx = vGyro.X;
    const float gy = vGyro.Y;
    const float gz = vGyro.Z;

    // Normalise magnetometer measurement
    norm = sqrtf (vMag.X * vMag.X + vMag.Y * vMag.Y + vMag.Z * vMag.Z);
    if (norm == 0.0f) { return angles; }

    norm = 1.0f / norm;
    const float mx = norm * vMag.X;
    const float my = norm * vMag.Y;
    const float mz = norm * vMag.Z;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz   * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 =  _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) -  4.0f  * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) -  4.0f  * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 =  _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

    norm = sqrtf (s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
    norm = 1.0f / norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Quaternions transformation
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    const float deltat = vDiffTime;
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;

    norm = sqrtf (q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

    // Converts Quaternions to Euler angles
    float a12, a22, a31, a32, a33;
    a12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 = q[0] *  q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 = q[0] *  q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    float pitch = -asinf (a32)      * RAD_TO_DEG;
    float roll  = atan2f (a31, a33) * RAD_TO_DEG;
    float yaw   = atan2f (a12, a22) * RAD_TO_DEG + declination;

    if(yaw < 0) { yaw += 360.0f; }

    angles.Pitch = pitch;
    angles.Roll  = roll;
    angles.Yaw   = yaw;

    return angles;
}

void Filters::kalman3PredictGyro (const double vGyro)
{
    // Measurement update
    double z[2] = { kalman.Three.Velocities [0], vGyro };  // Measurement vector

    // Kalman gain
    double S[2][2];
    S[0][0] = kalman.Three.H[0][0] * kalman.Three.P[0][0] * kalman.Three.H[0][0] + kalman.Three.R[0][0];
    S[0][1] = 0;
    S[1][0] = 0;
    S[1][1] = kalman.Three.H[1][1] * kalman.Three.P[1][1] * kalman.Three.H[1][1] + kalman.Three.R[1][1];

    double K[2][2];
    K[0][0] = kalman.Three.P[0][0] * kalman.Three.H[0][0] / S[0][0];
    K[0][1] = 0;
    K[1][0] = 0;
    K[1][1] = kalman.Three.P[1][1] * kalman.Three.H[1][1] / S[1][1];

    // Update estimate
    kalman.Three.Velocities [0] += K[0][0] * (z[0] - kalman.Three.H[0][0] * kalman.Three.Velocities [0]);    // [m/s]
    kalman.Three.Velocities [1] += K[1][1] * (z[1] - kalman.Three.H[1][1] * kalman.Three.Velocities [1]);    // [rad/s]

    // Update covariance
    double I[2][2] = {{1, 0}, {0, 1}};
    double IKH[2][2];
    IKH[0][0] = I[0][0] - K[0][0] * kalman.Three.H[0][0];
    IKH[0][1] = 0;
    IKH[1][0] = 0;
    IKH[1][1] = I[1][1] - K[1][1] * kalman.Three.H[1][1];

    double P_updated[2][2];
    P_updated[0][0] = IKH[0][0] * kalman.Three.P[0][0];
    P_updated[0][1] = 0;
    P_updated[1][0] = 0;
    P_updated[1][1] = IKH[1][1] * kalman.Three.P[1][1];

    kalman.Three.P[0][0] = P_updated[0][0];
    kalman.Three.P[0][1] = P_updated[0][1];
    kalman.Three.P[1][0] = P_updated[1][0];
    kalman.Three.P[1][1] = P_updated[1][1];
}

void Filters::kalman3PredictAccel (const double vAccel)
{
    // State prediction
    double x_pred[2];
    x_pred[0] = kalman.Three.A[0][0] * kalman.Three.Velocities [0] + kalman.Three.B[0][0] * vAccel;
    x_pred[1] = kalman.Three.A[1][1] * kalman.Three.Velocities [1];

    // Covariance prediction
    double P_pred[2][2];
    P_pred[0][0] = kalman.Three.A[0][0] * kalman.Three.P[0][0] * kalman.Three.A[0][0] + kalman.Three.Q[0][0];
    P_pred[0][1] = 0;
    P_pred[1][0] = 0;
    P_pred[1][1] = kalman.Three.A[1][1] * kalman.Three.P[1][1] * kalman.Three.A[1][1] + kalman.Three.Q[1][1];

    // Update state and covariance
    kalman.Three.Velocities[0]    = x_pred[0];
    kalman.Three.Velocities[1]    = x_pred[1];
    kalman.Three.P         [0][0] = P_pred[0][0];
    kalman.Three.P         [0][1] = P_pred[0][1];
    kalman.Three.P         [1][0] = P_pred[1][0];
    kalman.Three.P         [1][1] = P_pred[1][1];
}

double Filters::kalman1GetAngle (Kalmans::Kalman1 & vKalman1, double vNewAngle, double vNewRate, double vDiffTime)
{
    double rate = vNewRate - vKalman1.Bias;
    vKalman1.Angle += vDiffTime * rate;

    vKalman1.Data[0][0] += vDiffTime * (vDiffTime * vKalman1.Data[1][1] - vKalman1.Data[0][1] - vKalman1.Data[1][0] + vKalman1.Coef.QAngle);
    vKalman1.Data[0][1]  -= vDiffTime * vKalman1.Data[1][1];
    vKalman1.Data[1 ][0] -= vDiffTime * vKalman1.Data[1][1];
    vKalman1.Data[1 ][1]  += vKalman1.Coef.QBias * vDiffTime;

    double S = vKalman1.Data[0][0] + vKalman1.Coef.RMeasure;

    double K[2];
    K[0] = vKalman1.Data[0][0] / S;
    K[1]  = vKalman1.Data[1 ][0] / S;

    double y = vNewAngle - vKalman1.Angle;
    vKalman1.Angle += K[0] * y;
    vKalman1.Bias  += K[1 ] * y;

    double P00_temp = vKalman1.Data[0][0];
    double P01_temp = vKalman1.Data[0][1 ];

    vKalman1.Data[0][0] -= K[0] * P00_temp;
    vKalman1.Data[0][1 ] -= K[0] * P01_temp;
    vKalman1.Data[1][0 ] -= K[1 ] * P00_temp;
    vKalman1.Data[1][1  ] -= K[1 ] * P01_temp;

    return vKalman1.Angle;
};

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// END OF FILE ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
