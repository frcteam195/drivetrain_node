#include "drivetrain_node.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <map>
#include <mutex>
#include <iostream>
#include <iomanip>

#include <nav_msgs/Odometry.h>
#include <rio_control_node/Joystick_Status.h>
#include <rio_control_node/Robot_Status.h>
#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Motor_Configuration.h>
#include <turret_node/turret_status.h>
#include <local_planner_node/TrajectoryFollowCue.h>
#include <rio_control_node/Motor_Status.h>
#include <ck_utilities/Motor.hpp>
#include <ck_utilities/CKMath.hpp>
#include <ck_utilities/ParameterHelper.hpp>

#include <hmi_agent_node/HMI_Signals.h>

#include "drive_helper.hpp"

#define INCHES_TO_METERS 0.0254

ros::NodeHandle* node;
static constexpr double ENCODER_TICKS_TO_M_S = 1.0;

int mRobotStatus;
float mJoystick1x;
float mJoystick1y;
std::mutex mThreadCtrlLock;
uint32_t mConfigUpdateCounter;
static local_planner_node::TrajectoryFollowCue traj_follow_cue;
static bool about_to_shoot = false;

Motor* leftMasterMotor;
Motor* rightMasterMotor;
std::vector<Motor*> leftFollowersMotor;
std::vector<Motor*> rightFollowersMotor;
DriveHelper driveHelper;

void robotStatusCallback(const rio_control_node::Robot_Status& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);
	mRobotStatus = msg.robot_state;
}

void turret_status_callback(const turret_node::turret_status& msg)
{
	about_to_shoot = msg.about_to_shoot;
}

void trajectoryCueCallback(const local_planner_node::TrajectoryFollowCue& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);
    traj_follow_cue = msg;
}

void publishOdometryData(const rio_control_node::Motor_Status& msg)
{
	double left_velocity = 0;
	double right_velocity = 0;
	for(std::vector<rio_control_node::Motor_Info>::const_iterator i = msg.motors.begin();
	    i != msg.motors.end();
		i++)
	{
		if ( (*i).id == left_master_id)
		{
			left_velocity = ((*i).sensor_velocity * wheel_diameter_inches * M_PI * INCHES_TO_METERS) / 60.0;
		}
		if ( (*i).id == right_master_id)
		{
			right_velocity = ((*i).sensor_velocity * wheel_diameter_inches * M_PI * INCHES_TO_METERS) / 60.0;
		}
	}

	double robot_velocity = (left_velocity + right_velocity) / 2.0;
	double angular_velocity = (right_velocity - left_velocity) / (robot_track_width_inches * INCHES_TO_METERS);

	nav_msgs::Odometry odometry_data;
    odometry_data.header.stamp = ros::Time::now();
	odometry_data.header.frame_id = "odom";
	odometry_data.child_frame_id = "base_link";

	odometry_data.pose.pose.orientation.w = 0;
	odometry_data.pose.pose.orientation.x = 0;
	odometry_data.pose.pose.orientation.y = 0;
	odometry_data.pose.pose.orientation.z = 0;
	odometry_data.pose.pose.position.x = left_velocity;
	odometry_data.pose.pose.position.y = right_velocity;
	odometry_data.pose.pose.position.z = 0;

	odometry_data.twist.twist.linear.x = robot_velocity;
	odometry_data.twist.twist.linear.y = 0;
	odometry_data.twist.twist.linear.z = 0;

	odometry_data.twist.twist.angular.x = 0;
	odometry_data.twist.twist.angular.y = 0;
	odometry_data.twist.twist.angular.z = angular_velocity;

	odometry_data.pose.covariance =
	   { 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001,};

	odometry_data.twist.covariance =
	   { 0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.0, 0.001,};

	static ros::Publisher odometry_publisher = node->advertise<nav_msgs::Odometry>("/RobotOdometry", 1);
	odometry_publisher.publish(odometry_data);
}

void motorStatusCallback(const rio_control_node::Motor_Status& msg)
{
	publishOdometryData(msg);

	std::lock_guard<std::mutex> lock(mThreadCtrlLock);
}

void hmiSignalsCallback(const hmi_agent_node::HMI_Signals& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);

	switch (mRobotStatus)
	{
	// case rio_control_node::Robot_Status::AUTONOMOUS:
	// {
    //     if(traj_follow_cue.traj_follow_active)
    //     {
    //         double angular_velocity = traj_follow_cue.velocity.angular.z;
    //         double temp = angular_velocity * robot_track_width_inches * INCHES_TO_METERS;

    //         double average_velocity = traj_follow_cue.velocity.linear.x;
    //         double left_velocity = average_velocity - (temp / 2.0);
    //         double right_velocity = average_velocity + (temp / 2.0);

    //         double left_rpm = left_velocity / (wheel_diameter_inches * M_PI * INCHES_TO_METERS) * 60.0;
    //         double right_rpm = right_velocity / (wheel_diameter_inches * M_PI * INCHES_TO_METERS) * 60.0;

    //         leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT,
    //                               drive_Kv * left_rpm,
    //                               0 );

    //         rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT,
    //                                drive_Kv * right_rpm,
    //                                0 );
    //     }
    //     else
    //     {

    //         leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
    //         rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
    //     }
	// }
    // break;
	case rio_control_node::Robot_Status::TELEOP:
	case rio_control_node::Robot_Status::AUTONOMOUS:
	{
		float shoot_multiplier = 1.0;
		if(about_to_shoot)
		{
			shoot_multiplier = 0.0;
		}
		DriveMotorValues dv = driveHelper.calculateOutput( msg.drivetrain_fwd_back * shoot_multiplier,
														   msg.drivetrain_left_right * shoot_multiplier,
														   msg.drivetrain_quickturn,
														   true );

        leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, dv.left, 0 );
		rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, dv.right, 0 );
	}
	break;
	default:
	{
		leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
		rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
	}
	break;
	}
}


void initMotors()
{
    leftMasterMotor = new Motor( left_master_id, (Motor::Motor_Type)motor_type );
    rightMasterMotor = new Motor( right_master_id, (Motor::Motor_Type)motor_type );

    leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
    leftMasterMotor->config().set_fast_master(true);
    leftMasterMotor->config().set_inverted( left_master_inverted );
    leftMasterMotor->config().set_neutral_mode( brake_mode_default ? MotorConfig::NeutralMode::BRAKE : MotorConfig::NeutralMode::COAST);
    leftMasterMotor->config().set_voltage_compensation_saturation( voltage_comp_saturation );
    leftMasterMotor->config().set_voltage_compensation_enabled( voltage_comp_enabled );
	leftMasterMotor->config().set_open_loop_ramp(0.2);
	leftMasterMotor->config().set_supply_current_limit(true, 40, 0, 0);
    leftMasterMotor->config().apply();

    rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
    rightMasterMotor->config().set_fast_master(true);
    rightMasterMotor->config().set_inverted( right_master_inverted );
    rightMasterMotor->config().set_neutral_mode( brake_mode_default ? MotorConfig::NeutralMode::BRAKE : MotorConfig::NeutralMode::COAST);
    rightMasterMotor->config().set_voltage_compensation_saturation( voltage_comp_saturation );
    rightMasterMotor->config().set_voltage_compensation_enabled( voltage_comp_enabled );
	rightMasterMotor->config().set_open_loop_ramp(0.2);
	rightMasterMotor->config().set_supply_current_limit(true, 40, 0, 0);
    rightMasterMotor->config().apply();

    // followers
    //  left
    for (size_t i = 0; i < left_follower_ids.size(); i++)
	{
        Motor* follower_motor = new Motor( left_follower_ids[i], (Motor::Motor_Type)motor_type );
        follower_motor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
        follower_motor->config().set_follower(true, left_master_id);
        follower_motor->config().set_inverted( left_follower_inverted[i] );
        follower_motor->config().set_neutral_mode( brake_mode_default ? MotorConfig::NeutralMode::BRAKE : MotorConfig::NeutralMode::COAST);
        follower_motor->config().set_voltage_compensation_saturation( voltage_comp_saturation );
    	follower_motor->config().set_voltage_compensation_enabled( voltage_comp_enabled );
		follower_motor->config().set_open_loop_ramp(0.2);
		follower_motor->config().set_supply_current_limit(true, 40, 0, 0);
        follower_motor->config().apply();
        leftFollowersMotor.push_back(follower_motor);
    }

    //  right
    for (size_t i = 0; i < right_follower_ids.size(); i++)
	{
        Motor* follower_motor = new Motor( right_follower_ids[i], (Motor::Motor_Type)motor_type );
        follower_motor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
        follower_motor->config().set_follower(true, right_master_id);
        follower_motor->config().set_inverted( right_follower_inverted[i] );
        follower_motor->config().set_neutral_mode( brake_mode_default ? MotorConfig::NeutralMode::BRAKE : MotorConfig::NeutralMode::COAST);
        follower_motor->config().set_voltage_compensation_saturation( voltage_comp_saturation );
    	follower_motor->config().set_voltage_compensation_enabled( voltage_comp_enabled );
		follower_motor->config().set_open_loop_ramp(0.2);
		follower_motor->config().set_supply_current_limit(true, 40, 0, 0);
        follower_motor->config().apply();
        rightFollowersMotor.push_back(follower_motor);
    }

}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "drivetrain");

	ros::NodeHandle n;

	node = &n;

	bool required_params_found = true;

	required_params_found &= n.getParam(CKSP(left_master_id), left_master_id);
	required_params_found &= n.getParam(CKSP(left_follower_ids), left_follower_ids);
	required_params_found &= n.getParam(CKSP(left_sensor_inverted), left_sensor_inverted);
	required_params_found &= n.getParam(CKSP(left_master_inverted), left_master_inverted);
	required_params_found &= n.getParam(CKSP(left_follower_inverted), left_follower_inverted);
	required_params_found &= left_follower_ids.size() == left_follower_inverted.size();
	required_params_found &= n.getParam(CKSP(right_master_id), right_master_id);
	required_params_found &= n.getParam(CKSP(right_follower_ids), right_follower_ids);
	required_params_found &= n.getParam(CKSP(right_sensor_inverted), right_sensor_inverted);
	required_params_found &= n.getParam(CKSP(right_master_inverted), right_master_inverted);
	required_params_found &= n.getParam(CKSP(right_follower_inverted), right_follower_inverted);
	required_params_found &= right_follower_ids.size() == right_follower_inverted.size();
	required_params_found &= n.getParam(CKSP(motor_type), motor_type);
	required_params_found &= n.getParam(CKSP(voltage_comp_saturation), voltage_comp_saturation);
	required_params_found &= n.getParam(CKSP(voltage_comp_enabled), voltage_comp_enabled);
	required_params_found &= n.getParam(CKSP(brake_mode_default), brake_mode_default);
	required_params_found &= n.getParam(CKSP(gear_ratio_motor_to_output_shaft), gear_ratio_motor_to_output_shaft);
	required_params_found &= n.getParam(CKSP(wheel_diameter_inches), wheel_diameter_inches);
	required_params_found &= n.getParam(CKSP(robot_track_width_inches), robot_track_width_inches);
	required_params_found &= n.getParam(CKSP(robot_linear_inertia), robot_linear_inertia);
	required_params_found &= n.getParam(CKSP(robot_angular_inertia), robot_angular_inertia);
	required_params_found &= n.getParam(CKSP(robot_angular_drag), robot_angular_drag);
	required_params_found &= n.getParam(CKSP(drive_Ks_v_intercept), drive_Ks_v_intercept);
	required_params_found &= n.getParam(CKSP(drive_Kv), drive_Kv);
	required_params_found &= n.getParam(CKSP(drive_Ka), drive_Ka);
	required_params_found &= n.getParam(CKSP(velocity_kP), velocity_kP);
	required_params_found &= n.getParam(CKSP(velocity_kI), velocity_kI);
	required_params_found &= n.getParam(CKSP(velocity_kD), velocity_kD);
	required_params_found &= n.getParam(CKSP(velocity_kF), velocity_kF);
	required_params_found &= n.getParam(CKSP(velocity_iZone), velocity_iZone);
	required_params_found &= n.getParam(CKSP(velocity_maxIAccum), velocity_maxIAccum);
	required_params_found &= n.getParam(CKSP(motion_cruise_velocity), motion_cruise_velocity);
	required_params_found &= n.getParam(CKSP(motion_accel), motion_accel);
	required_params_found &= n.getParam(CKSP(motion_s_curve_strength), motion_s_curve_strength);
	required_params_found &= n.getParam(CKSP(supply_current_limit), supply_current_limit);
	required_params_found &= n.getParam(CKSP(supply_current_limit_threshold), supply_current_limit_threshold);
	required_params_found &= n.getParam(CKSP(supply_current_limit_threshold_exceeded_time), supply_current_limit_threshold_exceeded_time);

	if (!required_params_found)
	{
		ROS_ERROR("Missing required parameters. Please check the list and make sure all required parameters are included");
		return 1;
	}

	ros::Subscriber joystickStatus = node->subscribe("/HMISignals", 1, hmiSignalsCallback);
	ros::Subscriber motorStatus = node->subscribe("MotorStatus", 1, motorStatusCallback);
	ros::Subscriber robotStatus = node->subscribe("RobotStatus", 1, robotStatusCallback);
	ros::Subscriber trajectoryCue = node->subscribe("/active_trajectory", 1, trajectoryCueCallback);
	ros::Subscriber turret_status_subscriber = node->subscribe("/TurretStatus", 1, turret_status_callback);

    initMotors();

	ros::spin();
	return 0;
}
