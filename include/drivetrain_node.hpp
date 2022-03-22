#pragma once
#include <vector>
#include <stdint.h>

int left_master_id;
std::vector<int> left_follower_ids;
bool left_sensor_inverted;
bool left_master_inverted;
std::vector<bool>  left_follower_inverted;

int right_master_id;
std::vector<int> right_follower_ids;
bool right_sensor_inverted;
bool right_master_inverted;
std::vector<bool>  right_follower_inverted;

int motor_type;
double voltage_comp_saturation;
bool voltage_comp_enabled;

bool brake_mode_default;

double gear_ratio_motor_to_output_shaft;
double wheel_diameter_inches;
double robot_track_width_inches;
double robot_linear_inertia;
double robot_angular_inertia;
double robot_angular_drag;

double drive_Ks_v_intercept;
double drive_Kv;
double drive_Ka;

double velocity_kP;
double velocity_kI;
double velocity_kD;
double velocity_kF;
double velocity_iZone;
double velocity_maxIAccum;
double motion_cruise_velocity;
double motion_accel;
double motion_s_curve_strength;

double open_loop_ramp;
double supply_current_limit;
double supply_current_limit_threshold;
double supply_current_limit_threshold_exceeded_time;