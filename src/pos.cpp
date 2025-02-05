#include "vex.h"
#include <math.h>
#define the_radius_of_encoderwheel 4.0*0.393700787//cm
using namespace vex;
float IMUCoefficient = 3600 / 3594;//参数可改，而且必须要改
float lastLMileage = 0;
float curLMileage = 0;
float lastRMileage = 0;
float curRMileage = 0;
float lastLSpeed = 0;
float curLSpeed = 0;
float lastRSpeed = 0;
float curRSpeed = 0;
float selfXSpeed = 0;
float selfYSpeed = 0;
float globalXSpeed = 0;
float globalYSpeed = 0;
float lastglobalXSpeed = 0;
float lastglobalYSpeed = 0;
float sampleTime = 0;
float lastTime = 0;//用于更新采样时间
float LSpeed[3] = {0, 0, 0};
float RSpeed[3] = {0, 0, 0};
float FiltLSpeed[3] = {0, 0, 0};
float FiltRSpeed[3] = {0, 0, 0};
float LEncoderAngle = 0;
float REncoderAngle = 0;
float fliter_b[3] = {0.0675, 0.1349, 0.0675};
float fliter_a[3] = {1, -1.1429, 0.4128};
float curIMUHeading = 0;
float lastIMUHeading = 0;
float globalX = 0;
float globalY = 0;
Point GlobalPoint;
MyTimer time1;

const float positionRefreshTime = 10; 

float drive_max_voltage_geometry=10;
float drive_kp_geometry=1.5;//1.5//2.5
float drive_ki_geometry=0.12;
float drive_kd_geometry=8;
float drive_starti_geometry=0;
float drive_settle_error_geometry=1.5;
float drive_settle_time_geometry=100;
float drive_timeout_geometry=2000;
float heading_max_voltage_geometry=6;
float heading_kp_geometry=0.62;//0.42
float heading_ki_geometry=0.12;//0
float heading_kd_geometry=0.7;
float heading_starti_geometry=0;
double desire_heading_geometry = 0;
double heading_geometry = 0;
// FiltLSpeed[0] = FiltLSpeed[1] = FiltLSpeed[2] = 0;
//     FiltRSpeed[0] = FiltRSpeed[1] = FiltRSpeed[2] = 0;


//**************计时器相关函数**************************//
MyTimer::MyTimer(){
    startTime = Brain.Timer.value();
}

void MyTimer::reset(){
    startTime = Brain.Timer.value();
}

int MyTimer::getTime() const{
    return floor((Brain.Timer.value() - startTime) * 1000); // return time (msec) from startTime
}

double MyTimer::getTimeDouble() const{
    return Brain.Timer.value() - startTime; // return time (sec) from startTime
}
//*********************************//







// void updateIMUHeading(){
//     lastIMUHeading = curIMUHeading;
//     curIMUHeading = Inertial.heading();
// }

// float reduce_0_to_360(float angle) {
//   while(!(angle >= 0 && angle < 360)) {
//     if( angle < 0 ) { angle += 360; }
//     if(angle >= 360) { angle -= 360; }
//   }
//   return(angle);
// }
// float gets_absolute_heading(){ 
//   return( reduce_0_to_360( Inertial.rotation()) ); 
// }

double IMUHeading(){
    double heading_geometry = Inertial.rotation(rotationUnits::deg);
    heading_geometry = heading_geometry * IMUCoefficient;
    while (heading_geometry < 0) heading_geometry += 360;
    while (heading_geometry > 360) heading_geometry -= 360;
    return heading_geometry;
}

float deg2rad_geometry(float deg){
    return deg * M_PI / 180.0;
}

void updateInertialHeading(){
    lastIMUHeading = curIMUHeading;
    curIMUHeading = deg2rad_geometry(IMUHeading());
}



void rotation_init(){
    leftEncoder.resetPosition();
    rightEncoder.resetPosition();//里程计初始化
}
//updateInertialHeading();
//updateLMileage();
//updateRMileage();
//updateLSpeed();
//updateRSpeed();
//updateSelfXSpeed();
//updateSelfYSpeed();
//updateGlobalXSpeed();
//updateGlobalYSpeed();

void updateLMileage()
{
    lastLMileage = curLMileage;

    curLMileage = deg2rad_geometry(-leftEncoder.position(degrees)) * the_radius_of_encoderwheel;
}
void updateRMileage()
{
    lastRMileage = curRMileage;

    curRMileage = deg2rad_geometry(-rightEncoder.position(degrees)) * the_radius_of_encoderwheel;
}

/**
 * @brief 左定位轮速度 cm/s
 * 
 * @return float 
 */
void updateLSpeed(){
    lastLSpeed = curLSpeed;

    double ret = (curLMileage - lastLMileage) * 1000 / sampleTime;                      //前向差分法
    if (std::abs(ret) > 1000 || std::abs(ret) < 0.001) ret = 0;
    
    // 滤波前的速度
    LSpeed[2] = LSpeed[1];
    LSpeed[1] = LSpeed[0];
    LSpeed[0] = ret;

    // 滤波后的速度
    FiltLSpeed[2] = FiltLSpeed[1];
    FiltLSpeed[1] = FiltLSpeed[0];
    FiltLSpeed[0] = (fliter_b[0] * LSpeed[0] + fliter_b[1] * LSpeed[1] + fliter_b[2] * LSpeed[2] - fliter_a[1] * FiltLSpeed[1] - fliter_a[2] * FiltLSpeed[2]) / fliter_a[0]; 
    if (std::abs(FiltLSpeed[0]) < 0.001) FiltLSpeed[0] = 0;

    curLSpeed = FiltLSpeed[0]; 
    // curLSpeed = ret;
}

/**
 * @brief 右定位轮速度 cm/s
 * 
 * @return float 
 */
void updateRSpeed(){
    lastRSpeed = curRSpeed;

    double ret = (curRMileage - lastRMileage) * 1000 / sampleTime;                      //前向差分法
    if (std::abs(ret) > 1000 || std::abs(ret) < 0.001) ret = 0;                                   //速度过大或过小，认为是噪声
    // 伪kalman滤波
    // 滤波前的速度
    RSpeed[2] = RSpeed[1];
    RSpeed[1] = RSpeed[0];
    RSpeed[0] = ret;

    // 滤波后的速度
    FiltRSpeed[2] = FiltRSpeed[1];
    FiltRSpeed[1] = FiltRSpeed[0];
    FiltRSpeed[0] = (fliter_b[0] * RSpeed[0] + fliter_b[1] * RSpeed[1] + fliter_b[2] * RSpeed[2] - fliter_a[1] * FiltRSpeed[1] - fliter_a[2] * FiltRSpeed[2]) / fliter_a[0];
    if (std::abs(FiltRSpeed[0]) < 0.001) FiltRSpeed[0] = 0;

    curRSpeed = FiltRSpeed[0];
    // curRSpeed = ret;
}

/**
 * @brief 自身坐标系y轴速度 cm/s
 * 
 * @return double 
 */
void updateSelfYSpeed(){
    selfYSpeed = curLSpeed;
    //如果里程计不是按照正交形式摆放，那么就用下面那段代码
    // double D = -cos(REncoderAngle) * sin(LEncoderAngle) - cos(LEncoderAngle) * sin(REncoderAngle);
    // double Dy = curLSpeed * -cos(REncoderAngle) - curRSpeed * cos(LEncoderAngle);
    // selfYSpeed = Dy / D;

}

/**
 * @brief 自身坐标系x轴速度 cm/s
 * 
 * @return double 
 */
void updateSelfXSpeed(){
    selfXSpeed = curRSpeed;
    // double D = -cos(REncoderAngle) * sin(LEncoderAngle) - cos(LEncoderAngle) * sin(REncoderAngle);
    // double Dx = curRSpeed * sin(LEncoderAngle) - curLSpeed * sin(REncoderAngle);
    // selfXSpeed = Dx / D;
}

/**
 * @brief 世界坐标系y轴速度 cm/s
 * 
 * @return float 
 */
void updateGlobalYSpeed(){
    curIMUHeading = get_absolute_heading();
    lastglobalYSpeed = globalYSpeed;
    globalYSpeed = selfYSpeed * cos(curIMUHeading) - selfXSpeed * sin(curIMUHeading);
    if (std::abs(globalYSpeed) < 0.01) globalYSpeed = 0;
    if (std::abs(globalYSpeed) > 250) globalYSpeed = lastglobalYSpeed;
}

/**
 * @brief 世界坐标系x轴速度 cm/s
 * 
 * @return float 
 */
void updateGlobalXSpeed(){
    curIMUHeading = get_absolute_heading();
    lastglobalXSpeed = globalXSpeed;
    globalXSpeed = selfYSpeed * sin(curIMUHeading) + selfXSpeed * cos(curIMUHeading);
    if (std::abs(globalXSpeed) < 0.01) globalXSpeed = 0;
    if (std::abs(globalXSpeed) > 250) globalXSpeed = lastglobalXSpeed;

    // # ifdef debug
    //     if (CollectFlag)
    //     {
    //         cout <<setiosflags(ios::fixed)<<setprecision(2)<< globalXSpeed << " " << globalYSpeed << "\n";
    //     }
    // # endif
}

void updateGlobalX(){
    double d = globalYSpeed * sampleTime / 1000;                            //前向差分法
    // double d = (globalYSpeed + lastglobalYSpeed) * sampleTime / 1000 / 2;   //双线性变换法
    if (std::abs(d) < 0.001)
        return;
    else
        globalY = globalY + d;
}

void updateGlobalY(){
    double d = globalXSpeed * sampleTime / 1000;                            //前向差分法
    // double d = (globalXSpeed + lastglobalXSpeed) * sampleTime / 1000 / 2;   //双线性变换法
    if (std::abs(d) < 0.001)
        return;
    else
        globalX = globalX + d;
}

void update_all(){
    sampleTime = (time1.getTimeDouble() - lastTime) * 1000;
    lastTime = time1.getTimeDouble();//更新采样时间
    updateInertialHeading();
    updateLMileage();
    updateRMileage();
    updateLSpeed();
    updateRSpeed();
    updateSelfXSpeed();
    updateSelfYSpeed();
    updateGlobalXSpeed();
    updateGlobalYSpeed();
    updateGlobalX();
    updateGlobalY();
}

float return_globalX(){
    update_all();
    return globalX;
}
float return_globalY(){
    update_all();
    return globalY;
}
void update_global_point(){
    while(true){
    update_all();
    GlobalPoint.x = globalX;
    GlobalPoint.y = globalY;
    this_thread::sleep_for(positionRefreshTime);}
}


/************geometry_ku****************/
void drive_distance_geometry(float aimpos_x,float aimpos_y){
    drive_distance_geometry(aimpos_x, aimpos_y, get_absolute_heading(), drive_max_voltage_geometry, heading_max_voltage_geometry, drive_settle_error_geometry, drive_settle_time_geometry, drive_timeout_geometry, drive_kp_geometry, drive_ki_geometry, drive_kd_geometry, drive_starti_geometry, heading_kp_geometry, heading_ki_geometry, heading_kd_geometry, heading_starti_geometry);
}

void drive_distance_geometry(float aimpos_x,float aimpos_y, float heading_geometry, float drive_max_voltage_geometry, float heading_max_voltage_geometry, float drive_settle_error_geometry, float drive_settle_time_geometry, float drive_timeout_geometry, float drive_kp_geometry, float drive_ki_geometry, float drive_kd_geometry, float drive_starti_geometry, float heading_kp_geometry, float heading_ki_geometry, float heading_kd_geometry, float heading_starti_geometry){
    float curpos_X = GlobalPoint.x;
    float curpos_Y = GlobalPoint.y;
    float distance = sqrt((aimpos_x-curpos_X)*(aimpos_x-curpos_X)+(aimpos_y-curpos_Y)*(aimpos_y-curpos_Y));
    heading_geometry=get_absolute_heading();
    desire_heading_geometry = heading_geometry;
    PID drivePID(distance, drive_kp_geometry, drive_ki_geometry, drive_kd_geometry, drive_starti_geometry, drive_settle_error_geometry, drive_settle_time_geometry, drive_timeout_geometry);
    PID headingPID(reduce_negative_180_to_180(heading_geometry - gets_absolute_heading()), heading_kp_geometry, heading_ki_geometry, heading_kd_geometry, heading_starti_geometry);
    float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float average_position = start_average_position;
    while(drivePID.is_settled() == false){
        average_position = (get_left_position_in()+get_right_position_in())/2.0;
        float drive_error_geometry = distance+start_average_position-average_position;
        float heading_error_geometry = reduce_negative_180_to_180(heading_geometry - get_absolute_heading());
        float drive_output = drivePID.compute(drive_error_geometry);
        float heading_output = headingPID.compute(heading_error_geometry);
        drive_output = clamp(drive_output, -drive_max_voltage_geometry, drive_max_voltage_geometry);
        heading_output = clamp(heading_output, -heading_max_voltage_geometry, heading_max_voltage_geometry);
        drive_with_voltage(drive_output + heading_output, drive_output - heading_output);
        task::sleep(10);
        }
        L.stop(coast);R.stop(coast);
}

/***************************************/