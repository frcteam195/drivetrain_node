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
#include <turret_node/Turret_Status.h>
#include <rio_control_node/Motor_Status.h>
#include <ck_utilities/Motor.hpp>
#include <ck_utilities/CKMath.hpp>
#include <ck_utilities/ParameterHelper.hpp>
#include <ck_utilities/ValueRamper.hpp>
#include <ck_utilities/MovingAverage.hpp>

#include <hmi_agent_node/HMI_Signals.h>

#include "drive_helper.hpp"
#include "drivetrain_node/Drivetrain_Diagnostics.h"
#include <quesadilla_auto_node/Planner_Output.h>


//#define CHARACTERIZE_DRIVE
#ifdef CHARACTERIZE_DRIVE
#include "drive_physics_characterizer_node/Drive_Characterization_Output.h"
#endif

#define INCHES_TO_METERS 0.0254

//#define DYNAMIC_RECONFIGURE_TUNING
#ifdef DYNAMIC_RECONFIGURE_TUNING
#include <dynamic_reconfigure/server.h>
#include <drivetrain_node/DriveTuningConfig.h>
#endif

ros::NodeHandle* node;
static constexpr double ENCODER_TICKS_TO_M_S = 1.0;

int mRobotStatus;
float mJoystick1x;
float mJoystick1y;
std::mutex mThreadCtrlLock;
uint32_t mConfigUpdateCounter;
static bool about_to_shoot = false;

Motor* leftMasterMotor;
Motor* rightMasterMotor;
std::vector<Motor*> leftFollowersMotor;
std::vector<Motor*> rightFollowersMotor;
DriveHelper driveHelper;
ValueRamper* mLeftValueRamper;
ValueRamper* mRightValueRamper;
ck::MovingAverage mTargetLinearVelocityFilter(1);
ck::MovingAverage mTargetAngularVelocityFilter(1);

drivetrain_node::Drivetrain_Diagnostics drivetrain_diagnostics;
quesadilla_auto_node::Planner_Output drive_planner_output_msg;

static constexpr double kDriveGearReduction = (50.0 / 11.0) * (44.0 / 30.0);
static constexpr double kDriveRotationsPerTick = 1.0 / 2048.0 * 1.0 / kDriveGearReduction;

enum class DriveControlMode : int
{
	OPEN_LOOP_ARCADE = 0,
	OPEN_LOOP_CHEESY_DRIVE = 1,
	OPEN_LOOP_TODD_CURVATURE_DRIVE = 2,
	VELOCITY_ARCADE_DRIVE = 3,
	VELOCITY_CURVATURE_DRIVE = 4
};

static DriveControlMode drive_control_config {DriveControlMode::OPEN_LOOP_ARCADE};

#ifdef DYNAMIC_RECONFIGURE_TUNING
enum class DriveTuningMode : int
{
	Normal = 0,
	TuningVelocityPID = 1,
	TuningkVkA = 2
};

static DriveTuningMode tuning_control_mode = DriveTuningMode::Normal;
static double tuning_velocity_target = 0;
void tuning_config_callback(drivetrain_node::DriveTuningConfig &config, uint32_t level)
{
	(void) level;
	leftMasterMotor->config().set_kP(config.drive_kP);
	leftMasterMotor->config().set_kI(config.drive_kI);
	leftMasterMotor->config().set_kD(config.drive_kD);
	leftMasterMotor->config().set_kF(config.drive_kF);
	leftMasterMotor->config().set_i_zone(config.drive_iZone);
	leftMasterMotor->config().set_max_i_accum(config.drive_maxIAccum);
	leftMasterMotor->config().set_closed_loop_ramp(config.drive_closed_loop_ramp);

	rightMasterMotor->config().set_kP(config.drive_kP);
	rightMasterMotor->config().set_kI(config.drive_kI);
	rightMasterMotor->config().set_kD(config.drive_kD);
	rightMasterMotor->config().set_kF(config.drive_kF);
	rightMasterMotor->config().set_i_zone(config.drive_iZone);
	rightMasterMotor->config().set_max_i_accum(config.drive_maxIAccum);
	rightMasterMotor->config().set_closed_loop_ramp(config.drive_closed_loop_ramp);

	tuning_control_mode = (DriveTuningMode)config.drive_control_mode;
	tuning_velocity_target = config.drive_tuning_velocity_target;
	drive_Kv = config.drive_kV;
	drive_Ka = config.drive_kA;
}
#endif


void robotStatusCallback(const rio_control_node::Robot_Status& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);
	mRobotStatus = msg.robot_state;
}


#ifdef CHARACTERIZE_DRIVE
static bool characterizing_drive = false;
static double characterizing_drive_left_output = true;
static double characterizing_drive_right_output = true;
void drive_characterization_callback(const drive_physics_characterizer_node::Drive_Characterization_Output& msg)
{
	characterizing_drive = msg.characterizing_drive;
	characterizing_drive_left_output = msg.left_drive_output;
	characterizing_drive_right_output = msg.right_drive_output;
}
#endif

void turret_status_callback(const turret_node::Turret_Status& msg)
{
	about_to_shoot = msg.about_to_shoot;
}

void planner_callback(const quesadilla_auto_node::Planner_Output& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);
	drive_planner_output_msg = msg;
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
	static ros::Time prev_time(0);
	static double prevLeftVel = 0;
	static double prevRightVel = 0;
	static std::map<uint16_t, rio_control_node::Motor_Info> motor_map;
	publishOdometryData(msg);
	for (const rio_control_node::Motor_Info& m : msg.motors)
	{
		motor_map[m.id] = m;
	}

	ros::Time curr_time = ros::Time::now();
	double dt = (curr_time - prev_time).toSec();

	// std::lock_guard<std::mutex> lock(mThreadCtrlLock);
	if (motor_map.count(left_master_id))
	{
		drivetrain_diagnostics.actualVelocityLeft = motor_map[left_master_id].sensor_velocity * (wheel_diameter_inches * M_PI * INCHES_TO_METERS) / 60.0;
		drivetrain_diagnostics.leftRPMActual = motor_map[left_master_id].sensor_velocity;
	}
	if (motor_map.count(right_master_id))
	{
		drivetrain_diagnostics.actualVelocityRight = motor_map[right_master_id].sensor_velocity * (wheel_diameter_inches * M_PI * INCHES_TO_METERS) / 60.0;
		drivetrain_diagnostics.rightRPMActual = motor_map[right_master_id].sensor_velocity;
	}

	if (prev_time != ros::Time(0) && dt != 0)
	{
		drivetrain_diagnostics.actualAccelLeft = (drivetrain_diagnostics.actualVelocityLeft - prevLeftVel) / dt;
		drivetrain_diagnostics.actualAccelRight = (drivetrain_diagnostics.actualVelocityRight - prevRightVel) / dt;
	}
	else
	{
		drivetrain_diagnostics.actualAccelLeft = 0;
		drivetrain_diagnostics.actualAccelRight = 0;
	}
	drivetrain_diagnostics.dt = dt;
	
	prevLeftVel = drivetrain_diagnostics.actualAccelLeft;
	prevRightVel = drivetrain_diagnostics.actualAccelRight;
	prev_time = curr_time;
}

void hmiSignalsCallback(const hmi_agent_node::HMI_Signals& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);
	
	bool brake_mode = msg.drivetrain_brake;

	switch (mRobotStatus)
	{
	case rio_control_node::Robot_Status::AUTONOMOUS:
	{
		double left_rps = drive_planner_output_msg.left_motor_output_rad_per_sec;
		double left_ff_voltage = drive_planner_output_msg.left_motor_feedforward_voltage;
		double left_accel_rps2 = drive_planner_output_msg.left_motor_accel_rad_per_sec2;

		drivetrain_diagnostics.targetVelocityLeft = left_rps;
		drivetrain_diagnostics.targetAccelLeft = left_accel_rps2;

		double right_rps = drive_planner_output_msg.right_motor_output_rad_per_sec;
		double right_ff_voltage = drive_planner_output_msg.right_motor_feedforward_voltage;
		double right_accel_rps2 = drive_planner_output_msg.right_motor_accel_rad_per_sec2;

		drivetrain_diagnostics.targetVelocityRight = right_rps;
		drivetrain_diagnostics.targetAccelRight = right_accel_rps2;

		double left_accel_out = ck::math::radians_per_second_to_ticks_per_100ms(left_accel_rps2, kDriveRotationsPerTick) / 1000.0;
		double right_accel_out = ck::math::radians_per_second_to_ticks_per_100ms(right_accel_rps2, kDriveRotationsPerTick) / 1000.0;

		// leftMasterMotor->set( Motor::Control_Mode::VELOCITY,
		// 						ck::math::rads_per_sec_to_rpm(left_rps),
		// 						left_ff_voltage / 12.0 + velocity_kD * left_accel_out / 1023.0
		// 						);

		// rightMasterMotor->set( Motor::Control_Mode::VELOCITY,
		// 						ck::math::rads_per_sec_to_rpm(right_rps),
		// 						right_ff_voltage / 12.0 + velocity_kD * right_accel_out / 1023.0
		// 						);

		leftMasterMotor->set( Motor::Control_Mode::VELOCITY,
								ck::math::rads_per_sec_to_rpm(left_rps),
								0
								);

		rightMasterMotor->set( Motor::Control_Mode::VELOCITY,
								ck::math::rads_per_sec_to_rpm(right_rps),
								0
								);
		drivetrain_diagnostics.leftAppliedArbFF = left_ff_voltage / 12.0 + velocity_kD * left_accel_out / 1023.0;
		drivetrain_diagnostics.rightAppliedArbFF = right_ff_voltage / 12.0 + velocity_kD * right_accel_out / 1023.0;
		drivetrain_diagnostics.rawLeftMotorOutput = ck::math::rads_per_sec_to_rpm(left_rps);
		drivetrain_diagnostics.rawRightMotorOutput = ck::math::rads_per_sec_to_rpm(right_rps);
	}
    break;
	case rio_control_node::Robot_Status::TELEOP:
	{
		float shoot_multiplier = 1.0;
		if(about_to_shoot)
		{
			shoot_multiplier = 0.0;
		}

		double leftPre = 0;
		double rightPre = 0;

		switch (drive_control_config)
		{
			case DriveControlMode::OPEN_LOOP_CHEESY_DRIVE:
			{
				DriveMotorValues dv = driveHelper.calculateOutput( msg.drivetrain_fwd_back * shoot_multiplier,
																msg.drivetrain_left_right * shoot_multiplier,
																msg.drivetrain_quickturn,
																true );
				leftPre = dv.left;
				rightPre = dv.right;
				break;
			}
			case DriveControlMode::OPEN_LOOP_ARCADE:
			{
				leftPre = std::max(std::min(msg.drivetrain_fwd_back + msg.drivetrain_left_right, 1.0), -1.0) * shoot_multiplier;
				rightPre = std::max(std::min(msg.drivetrain_fwd_back - msg.drivetrain_left_right, 1.0), -1.0) * shoot_multiplier;
				break;
			}
			case DriveControlMode::OPEN_LOOP_TODD_CURVATURE_DRIVE:
			case DriveControlMode::VELOCITY_ARCADE_DRIVE:
			case DriveControlMode::VELOCITY_CURVATURE_DRIVE:
			{
				//Not Implemented
				leftPre = 0;
				rightPre = 0;
				break;
			}
		}
		
		drivetrain_diagnostics.rawLeftMotorOutput = leftPre;
		drivetrain_diagnostics.rawRightMotorOutput = rightPre;
		double left = mLeftValueRamper->calculateOutput(leftPre);
		double right = mRightValueRamper->calculateOutput(rightPre);

		drivetrain_diagnostics.rampedLeftMotorOutput = left;
		drivetrain_diagnostics.rampedRightMotorOutput = right;

#ifdef CHARACTERIZE_DRIVE
		if (characterizing_drive)
		{
			left = characterizing_drive_left_output;
			right = characterizing_drive_right_output;
		}
#endif

#ifdef DYNAMIC_RECONFIGURE_TUNING
		switch (tuning_control_mode)
		{
			case DriveTuningMode::Normal:
			{
				leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, left, 0 );
				rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, right, 0 );
				break;
			}
			case DriveTuningMode::TuningVelocityPID:
			{
				drivetrain_diagnostics.tuningVelocityRPMTarget = tuning_velocity_target;
				leftMasterMotor->set( Motor::Control_Mode::VELOCITY, tuning_velocity_target, 0 );
				rightMasterMotor->set( Motor::Control_Mode::VELOCITY, tuning_velocity_target, 0 );
				break;
			}
			case DriveTuningMode::TuningkVkA:
			{
				drivetrain_diagnostics.tuningVelocityRPMTarget = tuning_velocity_target;

				//Start?? 0.001852534562
				leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, (drive_Kv * tuning_velocity_target) / 12.0, 0 );
				rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, (drive_Kv * tuning_velocity_target) /12.0, 0 );
				break;
			}
		}
#else

        leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, left, 0 );
		rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, right, 0 );
#endif
	}
	break;
	default:
	{
		leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
		rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
	}
	break;
	}

	if (brake_mode)
	{
		leftMasterMotor->config().set_neutral_mode(MotorConfig::NeutralMode::BRAKE);
		leftMasterMotor->config().apply();
		for (Motor* mF : leftFollowersMotor)
		{
			mF->config().set_neutral_mode(MotorConfig::NeutralMode::BRAKE);
			mF->config().apply();
		}
		rightMasterMotor->config().set_neutral_mode(MotorConfig::NeutralMode::BRAKE);
		rightMasterMotor->config().apply();
		for (Motor* mF : rightFollowersMotor)
		{
			mF->config().set_neutral_mode(MotorConfig::NeutralMode::BRAKE);
			mF->config().apply();
		}
	}
	else
	{
		leftMasterMotor->config().set_neutral_mode(MotorConfig::NeutralMode::COAST);
		leftMasterMotor->config().apply();
		for (Motor* mF : leftFollowersMotor)
		{
			mF->config().set_neutral_mode(MotorConfig::NeutralMode::COAST);
			mF->config().apply();
		}
		rightMasterMotor->config().set_neutral_mode(MotorConfig::NeutralMode::COAST);
		rightMasterMotor->config().apply();
		for (Motor* mF : rightFollowersMotor)
		{
			mF->config().set_neutral_mode(MotorConfig::NeutralMode::COAST);
			mF->config().apply();
		}
	}

	static ros::Publisher drivetrain_diagnostics_publisher = node->advertise<drivetrain_node::Drivetrain_Diagnostics>("/DrivetrainDiagnostics", 1);
	drivetrain_diagnostics_publisher.publish(drivetrain_diagnostics);
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
	leftMasterMotor->config().set_open_loop_ramp(open_loop_ramp);
	leftMasterMotor->config().set_supply_current_limit(true, supply_current_limit, supply_current_limit_threshold, supply_current_limit_threshold_exceeded_time);
	leftMasterMotor->config().set_kP(velocity_kP);
	leftMasterMotor->config().set_kI(velocity_kI);
	leftMasterMotor->config().set_kD(velocity_kD);
	leftMasterMotor->config().set_kF(velocity_kF);
	leftMasterMotor->config().set_i_zone(velocity_iZone);
	leftMasterMotor->config().set_max_i_accum(velocity_maxIAccum);
	leftMasterMotor->config().set_closed_loop_ramp(closed_loop_ramp);
    leftMasterMotor->config().apply();

    rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
    rightMasterMotor->config().set_fast_master(true);
    rightMasterMotor->config().set_inverted( right_master_inverted );
    rightMasterMotor->config().set_neutral_mode( brake_mode_default ? MotorConfig::NeutralMode::BRAKE : MotorConfig::NeutralMode::COAST);
    rightMasterMotor->config().set_voltage_compensation_saturation( voltage_comp_saturation );
    rightMasterMotor->config().set_voltage_compensation_enabled( voltage_comp_enabled );
	rightMasterMotor->config().set_open_loop_ramp(open_loop_ramp);
	rightMasterMotor->config().set_supply_current_limit(true, supply_current_limit, supply_current_limit_threshold, supply_current_limit_threshold_exceeded_time);
	rightMasterMotor->config().set_kP(velocity_kP);
	rightMasterMotor->config().set_kI(velocity_kI);
	rightMasterMotor->config().set_kD(velocity_kD);
	rightMasterMotor->config().set_kF(velocity_kF);
	rightMasterMotor->config().set_i_zone(velocity_iZone);
	rightMasterMotor->config().set_max_i_accum(velocity_maxIAccum);
	rightMasterMotor->config().set_closed_loop_ramp(closed_loop_ramp);
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
		follower_motor->config().set_open_loop_ramp(open_loop_ramp);
		follower_motor->config().set_supply_current_limit(true, supply_current_limit, supply_current_limit_threshold, supply_current_limit_threshold_exceeded_time);
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
		follower_motor->config().set_open_loop_ramp(open_loop_ramp);
		follower_motor->config().set_supply_current_limit(true, supply_current_limit, supply_current_limit_threshold, supply_current_limit_threshold_exceeded_time);
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
	required_params_found &= n.getParam(CKSP(robot_scrub_factor), robot_scrub_factor);
	required_params_found &= n.getParam(CKSP(drive_Ks_v_intercept), drive_Ks_v_intercept);
	required_params_found &= n.getParam(CKSP(drive_Kv), drive_Kv);
	required_params_found &= n.getParam(CKSP(drive_Ka), drive_Ka);
	required_params_found &= n.getParam(CKSP(velocity_kP), velocity_kP);
	required_params_found &= n.getParam(CKSP(velocity_kI), velocity_kI);
	required_params_found &= n.getParam(CKSP(velocity_kD), velocity_kD);
	required_params_found &= n.getParam(CKSP(velocity_kF), velocity_kF);
	required_params_found &= n.getParam(CKSP(velocity_iZone), velocity_iZone);
	required_params_found &= n.getParam(CKSP(velocity_maxIAccum), velocity_maxIAccum);
	required_params_found &= n.getParam(CKSP(closed_loop_ramp), closed_loop_ramp);
	required_params_found &= n.getParam(CKSP(motion_cruise_velocity), motion_cruise_velocity);
	required_params_found &= n.getParam(CKSP(motion_accel), motion_accel);
	required_params_found &= n.getParam(CKSP(motion_s_curve_strength), motion_s_curve_strength);
	required_params_found &= n.getParam(CKSP(open_loop_ramp), open_loop_ramp);
	required_params_found &= n.getParam(CKSP(supply_current_limit), supply_current_limit);
	required_params_found &= n.getParam(CKSP(supply_current_limit_threshold), supply_current_limit_threshold);
	required_params_found &= n.getParam(CKSP(supply_current_limit_threshold_exceeded_time), supply_current_limit_threshold_exceeded_time);
	required_params_found &= n.getParam(CKSP(joystick_input_ramp_accel), joystick_input_ramp_accel);
	required_params_found &= n.getParam(CKSP(joystick_input_ramp_decel), joystick_input_ramp_decel);
	required_params_found &= n.getParam(CKSP(joystick_input_ramp_zero_val), joystick_input_ramp_zero_val);
	required_params_found &= n.getParam(CKSP(joystick_input_ramp_max_val), joystick_input_ramp_max_val);
	required_params_found &= n.getParam(CKSP(drive_control_mode), drive_control_mode);

	if (!required_params_found)
	{
		ROS_ERROR("Missing required parameters for node %s. Please check the list and make sure all required parameters are included", ros::this_node::getName().c_str());
		return 1;
	}

	drive_control_config = (DriveControlMode)drive_control_mode;

	mLeftValueRamper = new ValueRamper(joystick_input_ramp_accel, joystick_input_ramp_decel, joystick_input_ramp_zero_val, joystick_input_ramp_max_val);
	mRightValueRamper = new ValueRamper(joystick_input_ramp_accel, joystick_input_ramp_decel, joystick_input_ramp_zero_val, joystick_input_ramp_max_val);

	ros::Subscriber joystickStatus = node->subscribe("/HMISignals", 1, hmiSignalsCallback);
	ros::Subscriber motorStatus = node->subscribe("MotorStatus", 1, motorStatusCallback);
	ros::Subscriber robotStatus = node->subscribe("RobotStatus", 1, robotStatusCallback);
	ros::Subscriber planner_sub = node->subscribe("/QuesadillaPlannerOutput", 1, planner_callback);
	ros::Subscriber turret_status_subscriber = node->subscribe("/TurretStatus", 1, turret_status_callback);
#ifdef CHARACTERIZE_DRIVE
	ros::Subscriber drive_characterization_subscriber = node->subscribe("/DriveCharacterizationOutput", 1, drive_characterization_callback);
#endif

    initMotors();

#ifdef DYNAMIC_RECONFIGURE_TUNING
	dynamic_reconfigure::Server<drivetrain_node::DriveTuningConfig> server;
	dynamic_reconfigure::Server<drivetrain_node::DriveTuningConfig>::CallbackType f;
	f = boost::bind(&tuning_config_callback, _1, _2);
	server.setCallback(f);
#endif

	ros::spin();
	return 0;
}
