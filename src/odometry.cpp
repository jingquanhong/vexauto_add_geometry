#include "vex.h"

#include <cmath>
#include <algorithm>
#define the_radius_of_encoderwheel 4.0//按厘米算的
#define M_PI 3.14159265358979323846
#define wide_of_vexrobot 12.0//按厘米算的

void driveStraight(Odometry& odom,PID_EXTERN& drivePID,PID_EXTERN& headingPID,double targetDistance,double maxSpeed){
    odom.reset();
    drivePID.reset();
    headingPID.reset();
    
    double leftSpeed=0;
    double rightSpeed=0;
    double targetHeading=odom.getHeading();
    while(1){
        odom.updatePosition();
        double currentDistance=sqrt(pow(odom.getX(),2)+pow(odom.getY(),2));
        double drivePower=drivePID.calculate(targetDistance,currentDistance);
        double currentHeading=odom.getHeading();
        double headingCorrection=headingPID.calculate(targetHeading,currentHeading);
        // clamp(drive_output, -drive_max_voltage, drive_max_voltage);
        // leftSpeed = (drivePower-headingCorrection < -maxSpeed) ? -maxSpeed
        //           : ((drivePower-headingCorrection > maxSpeed) ? maxSpeed
        //                                                       : drivePower-headingCorrection);
        // rightSpeed = (drivePower+headingCorrection < -maxSpeed) ? -maxSpeed
        //           : ((drivePower+headingCorrection > maxSpeed) ? maxSpeed
        //                                                       : drivePower+headingCorrection);
        //使用c++中的clamp函数替换掉复杂选择类型函数
        leftSpeed = clamp(drivePower - headingCorrection, -maxSpeed, maxSpeed);
        rightSpeed = clamp(drivePower + headingCorrection, -maxSpeed, maxSpeed);
                     
        L.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
        R.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
        if (fabs(targetDistance - currentDistance) < 0.5) { //误差小于0.5  //按厘米算
            break;
        }
        wait(20, msec);
        
        L.stop();
        R.stop();
    }

}


void Odometry::reset(){
    globalX=0;
    globalY=0;
    heading=degToRad(gyro.heading());
    leftEncoder.resetPosition();
}
void Odometry::updatePosition(){
    heading=degToRad(gyro.heading());
    double currentleftDistance=leftEncoder.position(vex::rotationUnits::deg)/360.0*(2*M_PI*the_radius_of_encoderwheel);
    double currentrightDistance=rightEncoder.position(vex::rotationUnits::deg)/360.0*(2*M_PI*the_radius_of_encoderwheel);//有两路编码轮大小不一样的情况，一般用于过坡，但是vex肯定式不会有的
    double deltaleft=currentleftDistance-lastleftDistance;
    double deltaright=currentrightDistance-lastrightDistance;
    
    lastleftDistance=currentleftDistance;
    lastrightDistance=currentrightDistance;

    //计算XY分别的增量，用于后续加入pid运算
    //leftEncoder--->Y    rightEncoder--->X
    double deltaDistance=(deltaleft+deltaright)/2.0;
    double deltaHeading=(deltaright-deltaleft)/wide_of_vexrobot;
    globalX+=deltaDistance*cos(heading);
    globalY+=deltaDistance*sin(heading);
}
double Odometry::getX() const {
    return globalX;

}
double Odometry::getY() const {
    return globalY;

}
double Odometry::getHeading() const {
    return heading;
    
}