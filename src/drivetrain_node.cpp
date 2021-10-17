#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <map>
#include <mutex>

#include <rio_control_node/Joystick_Status.h>
#include <rio_control_node/Robot_Status.h>
#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Motor_Configuration.h>
#include <rio_control_node/Motor_Status.h>

ros::NodeHandle* node;
ros::Publisher mMotorControlPublisher;
ros::Publisher mMotorConfigurationPublisher;
static constexpr float kJoystickDeadband = 0.05f;
static constexpr int DRIVE_JOYSTICK = 0;
static constexpr int LEFT_MASTER_ID = 1;
static constexpr int LEFT_FOLLOWER1_ID = 2;
static constexpr int LEFT_FOLLOWER2_ID = 3;
static constexpr int RIGHT_MASTER_ID = 4;
static constexpr int RIGHT_FOLLOWER1_ID = 5;
static constexpr int RIGHT_FOLLOWER2_ID = 6;
rio_control_node::Motor_Control mMotorControlMsg;
rio_control_node::Motor_Configuration mMotorConfigurationMsg;
rio_control_node::Motor* mLeftMaster;
rio_control_node::Motor* mRightMaster;
int mRobotStatus;
float mJoystick1x;
float mJoystick1y;
std::mutex mThreadCtrlLock;
uint32_t mConfigUpdateCounter;

template <typename T>
inline int signum(T val)
{
	return (T(0) < val) - (val < T(0));
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

void motorStatusCallback(const rio_control_node::Motor_Status& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);

	switch (mRobotStatus)
	{
	case rio_control_node::Robot_Status::AUTONOMOUS:
	{
		
	}
		break;
	case rio_control_node::Robot_Status::TELEOP:
	{
		mLeftMaster->output_value = std::max(std::min(mJoystick1y + mJoystick1x, 1.0f), -1.0f);
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

void initMotorConfig()
{
	rio_control_node::Motor leftMaster;
	leftMaster.id = LEFT_MASTER_ID;
	leftMaster.controller_type = rio_control_node::Motor::TALON_SRX;
	leftMaster.output_value = 0;
	leftMaster.control_mode = rio_control_node::Motor::PERCENT_OUTPUT;
	mMotorControlMsg.motors.push_back(leftMaster);

	rio_control_node::Motor rightMaster;
	rightMaster.id = RIGHT_MASTER_ID;
	rightMaster.controller_type = rio_control_node::Motor::TALON_SRX;
	rightMaster.output_value = 0;
	rightMaster.control_mode = rio_control_node::Motor::PERCENT_OUTPUT;
	mMotorControlMsg.motors.push_back(rightMaster);

	rio_control_node::Motor mLeftFollower1;
	mLeftFollower1.id = LEFT_FOLLOWER1_ID;
	mLeftFollower1.output_value = leftMaster.id;
	mLeftFollower1.controller_type = rio_control_node::Motor::TALON_SRX;
	mLeftFollower1.control_mode = rio_control_node::Motor::FOLLOWER;
	mMotorControlMsg.motors.push_back(mLeftFollower1);

	rio_control_node::Motor mLeftFollower2;
	mLeftFollower2.id = LEFT_FOLLOWER2_ID;
	mLeftFollower2.output_value = leftMaster.id;
	mLeftFollower2.controller_type = rio_control_node::Motor::TALON_SRX;
	mLeftFollower2.control_mode = rio_control_node::Motor::FOLLOWER;
	mMotorControlMsg.motors.push_back(mLeftFollower2);

	rio_control_node::Motor mRightFollower1;
	mRightFollower1.id = RIGHT_FOLLOWER1_ID;
	mRightFollower1.output_value = rightMaster.id;
	mRightFollower1.controller_type = rio_control_node::Motor::TALON_SRX;
	mRightFollower1.control_mode = rio_control_node::Motor::FOLLOWER;
	mMotorControlMsg.motors.push_back(mRightFollower1);

	rio_control_node::Motor mRightFollower2;
	mRightFollower2.id = RIGHT_FOLLOWER2_ID;
	mRightFollower2.output_value = rightMaster.id;
	mRightFollower2.controller_type = rio_control_node::Motor::TALON_SRX;
	mRightFollower2.control_mode = rio_control_node::Motor::FOLLOWER;
	mMotorControlMsg.motors.push_back(mRightFollower2);

	mLeftMaster = &mMotorControlMsg.motors[0];
	mRightMaster = &mMotorControlMsg.motors[1];

	mMotorControlPublisher.publish(mMotorControlMsg);

	rio_control_node::Motor_Config leftMasterMotorConfig;
	leftMasterMotorConfig.id = LEFT_MASTER_ID;
	leftMasterMotorConfig.controller_type = rio_control_node::Motor_Config::TALON_SRX;
	leftMasterMotorConfig.controller_mode = rio_control_node::Motor_Config::FAST_MASTER;
	leftMasterMotorConfig.invert_type = rio_control_node::Motor_Config::NONE;
	leftMasterMotorConfig.neutral_mode = rio_control_node::Motor_Config::COAST;
	mMotorConfigurationMsg.motors.push_back(leftMasterMotorConfig);

	rio_control_node::Motor_Config rightMasterMotorConfig;
	rightMasterMotorConfig.id = RIGHT_MASTER_ID;
	rightMasterMotorConfig.controller_type = rio_control_node::Motor_Config::TALON_SRX;
	rightMasterMotorConfig.controller_mode = rio_control_node::Motor_Config::FAST_MASTER;
	rightMasterMotorConfig.invert_type = rio_control_node::Motor_Config::INVERT_MOTOR_OUTPUT;
	rightMasterMotorConfig.neutral_mode = rio_control_node::Motor_Config::COAST;
	mMotorConfigurationMsg.motors.push_back(rightMasterMotorConfig);

	rio_control_node::Motor_Config leftFollower1MotorConfig;
	leftFollower1MotorConfig.id = LEFT_FOLLOWER1_ID;
	leftFollower1MotorConfig.controller_type = rio_control_node::Motor_Config::TALON_SRX;
	leftFollower1MotorConfig.controller_mode = rio_control_node::Motor_Config::SLAVE;
	leftFollower1MotorConfig.invert_type = rio_control_node::Motor_Config::FOLLOW_MASTER;
	leftFollower1MotorConfig.neutral_mode = rio_control_node::Motor_Config::COAST;
	mMotorConfigurationMsg.motors.push_back(leftFollower1MotorConfig);

	rio_control_node::Motor_Config leftFollower2MotorConfig;
	leftFollower2MotorConfig.id = LEFT_FOLLOWER2_ID;
	leftFollower2MotorConfig.controller_type = rio_control_node::Motor_Config::TALON_SRX;
	leftFollower2MotorConfig.controller_mode = rio_control_node::Motor_Config::SLAVE;
	leftFollower2MotorConfig.invert_type = rio_control_node::Motor_Config::FOLLOW_MASTER;
	leftFollower2MotorConfig.neutral_mode = rio_control_node::Motor_Config::COAST;
	mMotorConfigurationMsg.motors.push_back(leftFollower2MotorConfig);

	rio_control_node::Motor_Config rightFollower1MotorConfig;
	rightFollower1MotorConfig.id = RIGHT_FOLLOWER1_ID;
	rightFollower1MotorConfig.controller_type = rio_control_node::Motor_Config::TALON_SRX;
	rightFollower1MotorConfig.controller_mode = rio_control_node::Motor_Config::SLAVE;
	rightFollower1MotorConfig.invert_type = rio_control_node::Motor_Config::FOLLOW_MASTER;
	rightFollower1MotorConfig.neutral_mode = rio_control_node::Motor_Config::COAST;
	mMotorConfigurationMsg.motors.push_back(rightFollower1MotorConfig);

	rio_control_node::Motor_Config rightFollower2MotorConfig;
	rightFollower2MotorConfig.id = RIGHT_FOLLOWER2_ID;
	rightFollower2MotorConfig.controller_type = rio_control_node::Motor_Config::TALON_SRX;
	rightFollower2MotorConfig.controller_mode = rio_control_node::Motor_Config::SLAVE;
	rightFollower2MotorConfig.invert_type = rio_control_node::Motor_Config::FOLLOW_MASTER;
	rightFollower2MotorConfig.neutral_mode = rio_control_node::Motor_Config::COAST;
	mMotorConfigurationMsg.motors.push_back(rightFollower2MotorConfig);

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

	mMotorControlPublisher = node->advertise<rio_control_node::Motor_Control>("MotorControl", 1);
	mMotorConfigurationPublisher = node->advertise<rio_control_node::Motor_Configuration>("MotorConfiguration", 1);


	ros::Subscriber joystickStatus = node->subscribe("JoystickStatus", 10, joystickStatusCallback);
	ros::Subscriber motorStatus = node->subscribe("MotorStatus", 10, motorStatusCallback);
	ros::Subscriber robotStatus = node->subscribe("RobotStatus", 10, robotStatusCallback);

	initMotorConfig();



	ros::spin();
	return 0;
}