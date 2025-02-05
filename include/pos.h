#pragma once
#include "vex.h"
using namespace vex;
typedef struct Point{
    float x = 0;
    float y = 0;
};
class MyTimer{//计时器
private:
    double startTime;
public:
    MyTimer();
    MyTimer(double);
    void reset();
    int getTime() const;
    double getTimeDouble() const;
};


void rotation_init(void);
void updateLMileage(void);
void updateRMileage(void);
double deg2rad(double deg);
void updateLSpeed(void);
void updateRSpeed(void);
void updateSelfXSpeed(void);
void updateSelfYSpeed(void);
void updateGlobalX(void);
void updateGlobalY(void);
void updateGlobalXSpeed(void);
void updateGlobalYSpeed(void);
void updateIMUHeading(void);
void updateInertialHeading(void);
void update_all(void);
float return_globalX(void);
float return_globalY(void);
void update_global_point(void);

/************geometry_ku****************/
void drive_distance_geometry(float aimpos_x,float aimpos_y);
void drive_distance_geometry(float aimpos_x,float aimpos_y, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

/***************************************/