#pragma once

#include "ck_utilities/CKMath.hpp"
#include <cmath> 

struct DriveMotorValues
{
    double left;
    double right;
};

class DriveHelper
{
private:
    // These factor determine how fast the wheel traverses the "non linear" sine curve.
    static constexpr double kHighWheelNonLinearity = 0.65;
    static constexpr double kLowWheelNonLinearity = 0.5;

    static constexpr double kHighNegInertiaScalar = 4.0;

    static constexpr double kLowNegInertiaThreshold = 0.65;
    static constexpr double kLowNegInertiaTurnScalar = 3.5;
    static constexpr double kLowNegInertiaCloseScalar = 4.0;
    static constexpr double kLowNegInertiaFarScalar = 5.0;

    static constexpr double kHighSensitivity = 0.95;
    static constexpr double kLowSensitiity = 1.3;

    static constexpr double kQuickStopDeadband = 0.2;
    static constexpr double kQuickStopWeight = 0.1;
    static constexpr double kQuickStopScalar = 5.0;

    double mOldWheel = 0.0;
    double mQuickStopAccumlator = 0.0;
    double mNegInertiaAccumlator = 0.0;
public:

    DriveMotorValues calculateOutput(double throttle, double wheel, bool isQuickTurn, bool isHighGear);
};