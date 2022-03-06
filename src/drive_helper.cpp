#include "drive_helper.hpp"

DriveMotorValues DriveHelper::calculateOutput(double throttle, double wheel, bool isQuickTurn, bool isHighGear)
{
    double negInertia = wheel - mOldWheel;
    mOldWheel = wheel;

    double wheelNonLinearity;
    if (isHighGear) {
        wheelNonLinearity = kHighWheelNonLinearity;
        double denominator = sin(M_PI / 2.0 * wheelNonLinearity);
        // Apply a sin function that's scaled to make it feel better.
        wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
    } else {
        wheelNonLinearity = kLowWheelNonLinearity;
        double denominator = sin(M_PI / 2.0 * wheelNonLinearity);
        // Apply a sin function that's scaled to make it feel better.
        wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
    }

    double leftPwm, rightPwm, overPower;
    double sensitivity;

    double angularPower;
    double linearPower;

    // Negative inertia!
    double negInertiaScalar;
    if (isHighGear) {
        negInertiaScalar = kHighNegInertiaScalar;
        sensitivity = kHighSensitivity;
    } else {
        if (wheel * negInertia > 0) {
            // If we are moving away from 0.0, aka, trying to get more wheel.
            negInertiaScalar = kLowNegInertiaTurnScalar;
        } else {
            // Otherwise, we are attempting to go back to 0.0.
            if (std::fabs(wheel) > kLowNegInertiaThreshold) {
                negInertiaScalar = kLowNegInertiaFarScalar;
            } else {
                negInertiaScalar = kLowNegInertiaCloseScalar;
            }
        }
        sensitivity = kLowSensitiity;
    }
    double negInertiaPower = negInertia * negInertiaScalar;
    mNegInertiaAccumlator += negInertiaPower;

    wheel = wheel + mNegInertiaAccumlator;
    if (mNegInertiaAccumlator > 1) {
        mNegInertiaAccumlator -= 1;
    } else if (mNegInertiaAccumlator < -1) {
        mNegInertiaAccumlator += 1;
    } else {
        mNegInertiaAccumlator = 0;
    }
    linearPower = throttle;

    // Quickturn!
    if (isQuickTurn) {
        if (std::fabs(linearPower) < kQuickStopDeadband) {
            double alpha = kQuickStopWeight;
            mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator
                    + alpha * ck::math::limit(wheel, 1.0) * kQuickStopScalar;
        }
        overPower = 1.0;
        angularPower = wheel;
    } else {
        overPower = 0.0;
        angularPower = std::fabs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
        if (mQuickStopAccumlator > 1) {
            mQuickStopAccumlator -= 1;
        } else if (mQuickStopAccumlator < -1) {
            mQuickStopAccumlator += 1;
        } else {
            mQuickStopAccumlator = 0.0;
        }
    }

    rightPwm = leftPwm = linearPower;
    leftPwm += angularPower;
    rightPwm -= angularPower;

    if (leftPwm > 1.0) {
        rightPwm -= overPower * (leftPwm - 1.0);
        leftPwm = 1.0;
    } else if (rightPwm > 1.0) {
        leftPwm -= overPower * (rightPwm - 1.0);
        rightPwm = 1.0;
    } else if (leftPwm < -1.0) {
        rightPwm += overPower * (-1.0 - leftPwm);
        leftPwm = -1.0;
    } else if (rightPwm < -1.0) {
        leftPwm += overPower * (-1.0 - rightPwm);
        rightPwm = -1.0;
    }

    return DriveMotorValues{std::max(std::min(leftPwm, 1.0), -1.0) , std::max(std::min(rightPwm, 1.0), -1.0)};
}