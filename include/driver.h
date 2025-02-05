#pragma once
#include"vex.h"

extern double desire_heading;

extern float wheel_diameter;
extern float wheel_ratio;
extern float drive_in_to_deg_ratio;

extern float drive_max_voltage;
extern float drive_kp;
extern float drive_ki;
extern float drive_kd;
extern float drive_starti;

extern float drive_settle_error;
extern float drive_settle_time;
extern float drive_timeout;

extern float heading_max_voltage;
extern float heading_kp;
extern float heading_ki;
extern float heading_kd;
extern float heading_starti;

extern float swing_max_voltage;
extern float swing_kp;
extern float swing_ki;
extern float swing_kd;
extern float swing_starti;

extern float swing_settle_error;
extern float swing_settle_time;
extern float swing_timeout;
extern float desired_angle;
extern float desired_heading;

extern float turn_max_voltage;
extern float turn_settle_error;
extern float turn_settle_time;
extern float turn_timeout;
extern float turn_kp;
extern float turn_ki;
extern float turn_kd;
extern float turn_starti;

float gets_absolute_heading();

float get_absolute_heading();

void drive_with_voltage(float leftVoltage, float rightVoltage);

float get_left_position_in();

float get_right_position_in();

void daoche(double du);
void daoche2(double du);
void daoche3(double du);
void daoche4(double du);
void daoche2_1(double du);
void daoche3_1(double du);
void qianzhuan(double du);
void penggan(double du);
void daoche_(double du);
void daoche2_(double du);
void qianjing(double du);
void qianjing2(double du);
void qianjing3(double du);


void firstdistance(double e);
void coast_drive_distance(float distance);
void coast_drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

void drive_direct_distance(float distance);
void drive_direct_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);


void drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

void drive_distance(float distance);


void drive_distance_another(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

void drive_distance_another(float distance);

void drive_firstdistance(float distance);

void drive_firstdistance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

void start_with_voltage(float Arm_L_Voltage);

void left_swing_to_angle(float angle);

void right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti);

void right_swing_to_angle(float angle);

void turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti);

void turn_to_angle(float angle);
void turnlow_to_angle(float angle);

void drivelow_distance(float distance);
void drive1_distance(float distance);
void drivelow_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
void gudingjuli(float dis);

void spin_to_angle(float angle);
void spin_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti);
