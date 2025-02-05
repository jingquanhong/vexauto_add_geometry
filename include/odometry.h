#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "vex.h"
#include <cmath>
class Odometry{
private:
   double globalX,globalY;//获取全局位置
   double heading;//通过惯性传感器获取朝向  //后续对惯导进行滤波
   double leftDistance,rightDistance;
    vex::inertial& gyro;//惯导
    vex::rotation& leftEncoder;//左轮编码
    vex::rotation& rightEncoder;//右轮编码
    
    double lastleftDistance,lastrightDistance;
    double degToRad(double degrees) { 
        return degrees * M_PI / 180.0; 
        }
public:
    Odometry(vex::inertial& imuSensor, vex::rotation& leftEnc, vex::rotation& rightEnc) : gyro(imuSensor), leftEncoder(leftEnc), rightEncoder(rightEnc),
          globalX(0), globalY(0), heading(0),
          leftDistance(0), rightDistance(0),
          lastleftDistance(0), lastrightDistance(0) {}

    void reset();                   
    void updatePosition();          
    double getX() const;            
    double getY() const;          
    double getHeading() const;      
};

class PID_EXTERN{
private:
  double kp;
  double ki;
  double kd;
  double error,lasterror,integral,derivative;
  double output;
public:
  PID_EXTERN(double kp,double ki,double kd) : kp(kp),ki(ki),kd(kd),lasterror(0),
  integral(0),derivative(0){}
  double calculate(double target,double current){
    error=target-current;
    integral+=error;
    derivative=error-lasterror;
    lasterror=error;
    output=kp*error+ki*integral+kd*derivative;
    return output;
  }
  void reset(){
    error=0;
    lasterror=0;
    integral=0;
    derivative=0;
  }
};
void driveStraight(Odometry& odom,PID_EXTERN& drivePID,PID_EXTERN& headingPID,double targetDistance,double maxSpeed);

#endif