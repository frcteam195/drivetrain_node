#include "drive_helper.hpp"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <vector>

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

geometry_msgs::Pose null_pose()
{
    geometry_msgs::Pose result;
    result.position.x = 0;
    result.position.y = 0;
    result.position.z = 0;
    tf2::Quaternion t;
    t.setRPY(0,0,0);
    result.orientation.w = t.getW();
    result.orientation.x = t.getX();
    result.orientation.y = t.getY();
    result.orientation.z = t.getZ();
    return result;
}

geometry_msgs::Pose apply_twist(geometry_msgs::Pose initial_pose, geometry_msgs::Twist twist, double dt)
{
    geometry_msgs::Pose result = initial_pose;
    result.position.x += twist.linear.x * dt;
    result.position.y += twist.linear.y * dt;
    result.position.z += twist.linear.z * dt;
    tf2::Quaternion applied_quat;
    applied_quat.setRPY(twist.angular.x * dt, twist.angular.y * dt, twist.angular.z * dt);
    tf2::Quaternion initial_quat;
    tf2::fromMsg(initial_pose.orientation, initial_quat);
    tf2::Quaternion overall_quat = initial_quat * applied_quat;
    result.orientation = tf2::toMsg(overall_quat);
    return result;
}

geometry_msgs::Pose apply_transform(geometry_msgs::Pose initial_pose, geometry_msgs::Transform transform)
{
    geometry_msgs::Pose result;
    result.position.x = initial_pose.position.x + transform.translation.x;
    result.position.y = initial_pose.position.y + transform.translation.y;
    result.position.z = initial_pose.position.z + transform.translation.z;

    tf2::Quaternion initial_quat;
    tf2::Quaternion rotation_quat;
    initial_quat.setW(initial_pose.orientation.w);
    initial_quat.setX(initial_pose.orientation.x);
    initial_quat.setY(initial_pose.orientation.y);
    initial_quat.setZ(initial_pose.orientation.z);
    rotation_quat.setW(transform.rotation.w);
    rotation_quat.setX(transform.rotation.x);
    rotation_quat.setY(transform.rotation.y);
    rotation_quat.setZ(transform.rotation.z);

    tf2::Quaternion result_rotation = initial_quat * rotation_quat;
    result.orientation.w = result_rotation.getW();
    result.orientation.x = result_rotation.getX();
    result.orientation.y = result_rotation.getY();
    result.orientation.z = result_rotation.getZ();

    return result;
}

double normalize_to_2_pi(double value)
{
    value = fmod(value, 360.0);
    if (value < 0.0)
        value += 360.0;
    return value;
}

double smallest_traversal(double angle, double target_angle)
{
    double left = -normalize_to_2_pi(angle - target_angle);
    double right = normalize_to_2_pi(target_angle - angle);
    if(fabs(left) < fabs(right))
    {
        return left;
    }
    return right;
}

std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> calculate_swerve_outputs
    (geometry_msgs::Twist desired_twist,
    std::vector<geometry_msgs::Transform> wheel_transforms,
    double projection_time_s)
{
    std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> results;

    geometry_msgs::Pose robot_initial_pose = null_pose();
    geometry_msgs::Pose robot_projected_pose = apply_twist(robot_initial_pose, desired_twist, projection_time_s);

    double largest_hypot = 0;

    for(std::vector<geometry_msgs::Transform>::iterator i = wheel_transforms.begin();
        i != wheel_transforms.end();
        i++)
    {
        geometry_msgs::Pose wheel_initial_pose = apply_transform(robot_initial_pose, (*i));
        geometry_msgs::Pose wheel_projected_pose = apply_transform(robot_projected_pose, (*i));

        double hypot = sqrt((wheel_projected_pose.position.x * wheel_projected_pose.position.x) +
                            (wheel_projected_pose.position.y * wheel_projected_pose.position.y));

        largest_hypot = std::fmax(hypot, largest_hypot);

        double wheel_end_yaw = std::asin(hypot / wheel_projected_pose.position.x);

        tf2::Quaternion wheel_initial_quaternion;
        tf2::fromMsg(wheel_initial_pose.orientation, wheel_initial_quaternion);
        double initial_roll, initial_pitch, initial_yaw;
        (void) initial_roll;
        (void) initial_pitch;
        tf2::Matrix3x3(wheel_initial_quaternion).getRPY(initial_roll, initial_pitch, initial_yaw);

        initial_yaw = normalize_to_2_pi(initial_yaw);
        double mirrored_initial_yaw = normalize_to_2_pi(initial_yaw + M_PI);

        double normal_smallest_traversal = smallest_traversal(initial_yaw, wheel_end_yaw);
        double mirror_smallest_traversal = smallest_traversal(mirrored_initial_yaw, wheel_end_yaw);


        double smallest_overall_traversal = std::fabs(normal_smallest_traversal) < std::fabs(mirror_smallest_traversal) ? normal_smallest_traversal : mirror_smallest_traversal;
        
        tf2::Quaternion smallest_traversal_pose;
        smallest_traversal_pose.setRPY(0, 0, smallest_overall_traversal);

        std::pair<geometry_msgs::Pose, geometry_msgs::Twist> wheel_result;
        wheel_result.first.position = wheel_projected_pose.position;
        wheel_result.first.orientation = toMsg(smallest_traversal_pose);
        wheel_result.second.linear.x = wheel_projected_pose.position.x / projection_time_s;
        wheel_result.second.linear.y = wheel_projected_pose.position.y / projection_time_s;
        wheel_result.second.linear.z = wheel_projected_pose.position.z / projection_time_s;
        wheel_result.second.angular.x = 0;
        wheel_result.second.angular.y = 0;
        wheel_result.second.angular.z = smallest_overall_traversal / projection_time_s;

        results.push_back(wheel_result);
    }

    for(std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>>::iterator i = results.begin();
        i != results.end();
        i++)
    {
        double hypot = std::sqrt((*i).first.position.x * (*i).first.position.x +
                                 (*i).first.position.y * (*i).first.position.y);

        double ratio = hypot / largest_hypot;
        (*i).first.position.x *= ratio;
        (*i).first.position.y *= ratio;
        (*i).second.linear.x *= ratio;
        (*i).second.linear.y *= ratio;
    }

    return results;

}