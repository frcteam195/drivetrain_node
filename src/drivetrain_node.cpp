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
#include <local_planner_node/TrajectoryFollowCue.h>
#include <rio_control_node/Motor_Status.h>

#define STR_PARAM(s) #s
#define CKSP(s) ckgp( STR_PARAM(s) )
#define INCHES_TO_METERS 0.0254

ros::NodeHandle* node;
ros::Publisher mMotorControlPublisher;
ros::Publisher mMotorConfigurationPublisher;
static constexpr float kJoystickDeadband = 0.05f;
static constexpr int DRIVE_JOYSTICK = 0;
static constexpr double ENCODER_TICKS_TO_M_S = 1.0;
rio_control_node::Motor_Control mMotorControlMsg;
rio_control_node::Motor_Configuration mMotorConfigurationMsg;
rio_control_node::Motor* mLeftMaster;
rio_control_node::Motor* mRightMaster;
int mRobotStatus;
float mJoystick1x;
float mJoystick1y;
std::mutex mThreadCtrlLock;
uint32_t mConfigUpdateCounter;
static local_planner_node::TrajectoryFollowCue traj_follow_cue;

template <typename T>
inline int signum(T val)
{
	return (T(0) < val) - (val < T(0));
}

static const std::string nodeParamPrefix = "/drivetrain_node/";
std::string ckgp(std::string instr)
{
	std::string retVal = nodeParamPrefix;
	retVal += instr;
	return retVal;
}

float normalizeJoystickWithDeadband(float val, float deadband) {
	val = (std::abs(val) > std::abs(deadband)) ? val : 0.0;

	if (val != 0)
	{
		val = signum(val) * ((std::abs(val) - deadband) / (1.0 - deadband));
	}

	return (std::abs(val) > std::abs(deadband)) ? val : 0.0;
}

void robotStatusCallback(const rio_control_node::Robot_Status& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);
	mRobotStatus = msg.robot_state;
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

	switch (mRobotStatus)
	{
	case rio_control_node::Robot_Status::AUTONOMOUS:
	{
        if(traj_follow_cue.traj_follow_active)
        {
            double angular_velocity = traj_follow_cue.velocity.angular.z;
            double temp = angular_velocity * robot_track_width_inches * INCHES_TO_METERS;

            double average_velocity = traj_follow_cue.velocity.linear.x;
            double left_velocity = average_velocity - (temp / 2.0);
            double right_velocity = average_velocity + (temp / 2.0);

            double left_rpm = left_velocity / (wheel_diameter_inches * M_PI * INCHES_TO_METERS) * 60.0;
            double right_rpm = right_velocity / (wheel_diameter_inches * M_PI * INCHES_TO_METERS) * 60.0;

            mLeftMaster->control_mode = mLeftMaster->PERCENT_OUTPUT;
            mLeftMaster->output_value = drive_Kv * left_rpm;
            mRightMaster->control_mode = mRightMaster->PERCENT_OUTPUT;
            mRightMaster->output_value = drive_Kv * right_rpm;

            // mLeftMaster->control_mode = mLeftMaster->VELOCITY;
            // mLeftMaster->arbitrary_feedforward = KV * left_velocity;
            // mLeftMaster->output_value = left_velocity;
            // mRightMaster->control_mode = mRightMaster->VELOCITY;
            // mRightMaster->arbitrary_feedforward = KV * right_velocity;
            // mRightMaster->output_value = right_velocity;
        }
        else
        {
            mLeftMaster->control_mode = mLeftMaster->PERCENT_OUTPUT;
            mLeftMaster->output_value = 0;
            mRightMaster->control_mode = mRightMaster->PERCENT_OUTPUT;
            mRightMaster->output_value = 0;
        }
	}
    break;
	case rio_control_node::Robot_Status::TELEOP:
	{
        mLeftMaster->control_mode = mLeftMaster->PERCENT_OUTPUT;
		mLeftMaster->output_value = std::max(std::min(mJoystick1y + mJoystick1x, 1.0f), -1.0f);
        mRightMaster->control_mode = mRightMaster->PERCENT_OUTPUT;
		mRightMaster->output_value = std::max(std::min(mJoystick1y - mJoystick1x, 1.0f), -1.0f);
	}
		break;
	default:
		break;
	}

	mMotorControlPublisher.publish(mMotorControlMsg);

	if (mConfigUpdateCounter++ % 100 == 0)
	{
		//Send a config update roughly every 500ms
		mMotorConfigurationPublisher.publish(mMotorConfigurationMsg);
	}
}

void joystickStatusCallback(const rio_control_node::Joystick_Status& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);
	if (msg.joysticks.size() > 0)
	{
		if (msg.joysticks[DRIVE_JOYSTICK].axes.size() > 1)
		{
			mJoystick1x = normalizeJoystickWithDeadband(msg.joysticks[DRIVE_JOYSTICK].axes[0], kJoystickDeadband);
			mJoystick1y = -normalizeJoystickWithDeadband(msg.joysticks[DRIVE_JOYSTICK].axes[1], kJoystickDeadband);
		}
	}
}

void configureFollowers(rio_control_node::Motor& master, std::vector<int>& followerIds, std::vector<bool>& followersInverted)
{
	for (size_t i = 0; i < followerIds.size(); i++)
	{
		rio_control_node::Motor follower;
		follower.id = followerIds[i];
		follower.output_value = master.id;
		follower.controller_type = (int8_t)motor_type;
		follower.control_mode = rio_control_node::Motor::FOLLOWER;
		mMotorControlMsg.motors.push_back(follower);

		rio_control_node::Motor_Config followerConfig;
		followerConfig.id = followerIds[i];
		followerConfig.controller_type = (int8_t)motor_type;
		followerConfig.controller_mode = rio_control_node::Motor_Config::FOLLOWER;
		followerConfig.invert_type = followersInverted[i] ? rio_control_node::Motor_Config::OPPOSE_MASTER : rio_control_node::Motor_Config::FOLLOW_MASTER;
		followerConfig.neutral_mode = brake_mode_default ? rio_control_node::Motor_Config::BRAKE : rio_control_node::Motor_Config::COAST;
		mMotorConfigurationMsg.motors.push_back(followerConfig);
	}
}

void initMotorConfig()
{
	rio_control_node::Motor leftMaster;
	leftMaster.id = left_master_id;
	leftMaster.controller_type = (int8_t)motor_type;
	leftMaster.output_value = 0;
	leftMaster.control_mode = rio_control_node::Motor::PERCENT_OUTPUT;
	mMotorControlMsg.motors.push_back(leftMaster);

	rio_control_node::Motor rightMaster;
	rightMaster.id = right_master_id;
	rightMaster.controller_type = (int8_t)motor_type;
	rightMaster.output_value = 0;
	rightMaster.control_mode = rio_control_node::Motor::PERCENT_OUTPUT;
	mMotorControlMsg.motors.push_back(rightMaster);

	mLeftMaster = &mMotorControlMsg.motors[0];
	mRightMaster = &mMotorControlMsg.motors[1];

	rio_control_node::Motor_Config leftMasterMotorConfig;
	leftMasterMotorConfig.id = left_master_id;
	leftMasterMotorConfig.controller_type = (int8_t)motor_type;
	leftMasterMotorConfig.controller_mode = rio_control_node::Motor_Config::FAST_MASTER;
	leftMasterMotorConfig.invert_type = left_master_inverted ? rio_control_node::Motor_Config::INVERT_MOTOR_OUTPUT : rio_control_node::Motor_Config::NONE;
	leftMasterMotorConfig.neutral_mode = brake_mode_default ? rio_control_node::Motor_Config::BRAKE : rio_control_node::Motor_Config::COAST;
	mMotorConfigurationMsg.motors.push_back(leftMasterMotorConfig);

	rio_control_node::Motor_Config rightMasterMotorConfig;
	rightMasterMotorConfig.id = right_master_id;
	rightMasterMotorConfig.controller_type = (int8_t)motor_type;
	rightMasterMotorConfig.controller_mode = rio_control_node::Motor_Config::FAST_MASTER;
	rightMasterMotorConfig.invert_type = right_master_inverted ? rio_control_node::Motor_Config::INVERT_MOTOR_OUTPUT : rio_control_node::Motor_Config::NONE;
	rightMasterMotorConfig.neutral_mode = brake_mode_default ? rio_control_node::Motor_Config::BRAKE : rio_control_node::Motor_Config::COAST;
	mMotorConfigurationMsg.motors.push_back(rightMasterMotorConfig);

	configureFollowers(leftMaster, left_follower_ids, left_follower_inverted);
	configureFollowers(rightMaster, right_follower_ids, right_follower_inverted);

	mMotorControlPublisher.publish(mMotorControlMsg);
	mMotorConfigurationPublisher.publish(mMotorConfigurationMsg);
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

	mMotorControlPublisher = node->advertise<rio_control_node::Motor_Control>("MotorControl", 1);
	mMotorConfigurationPublisher = node->advertise<rio_control_node::Motor_Configuration>("MotorConfiguration", 1);


	ros::Subscriber joystickStatus = node->subscribe("JoystickStatus", 10, joystickStatusCallback);
	ros::Subscriber motorStatus = node->subscribe("MotorStatus", 10, motorStatusCallback);
	ros::Subscriber robotStatus = node->subscribe("RobotStatus", 10, robotStatusCallback);
	ros::Subscriber trajectoryCue = node->subscribe("/active_trajectory", 10, trajectoryCueCallback);

	initMotorConfig();



	ros::spin();
	return 0;
}