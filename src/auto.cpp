#include "vex.h"

vex::mutex mtx;
int flag1=0;
void init(){
    Arm.resetPosition();
    Inertial.calibrate();
    waitUntil(!Inertial.isCalibrating());
    Inertial.setHeading(0,degrees);
    time0.reset();
}

void pid_test(){
    //turn_to_angle(-90);
    drive_distance(-24);
}

void autoplay(){
    init();
    //pid_test();
    //thread t0(visionpro);
    
    thread t1(release);
    thread t3(shijue);
    // thread t2(try1);
    //thread t3(open);
    thread t2(blueplace);
    //thread t3(rings4);
}

void shijue()
{
    while(flag1==0)
    {
        vision_function();
    }
}

void release(){
    Capture.open();
    Lock.open();
    //wait(0.7, sec);
    //Capture.close();
    // Arm.spinToPosition(500, deg, 600, rpm);

}

void try1(){
    drive_firstdistance(47);
}

void redplace(){
    drive_direct_distance(45);
    drivelow_distance(5.4);
    Capture.close();
    wait(1,sec);
    left_swing_to_angle(-75);
    Takein.spin(fwd,68,pct);
    wait(0.5,sec);
    Takein.spin(fwd,68,pct);
    drivelow_distance(-15);
    wait(0.5, sec);
    drive_distance(6);
    wait(0.5, sec);
    drive_distance(-6);



    wait(0.5,sec);

    turn_to_angle(-27);
    

    drive_distance(-26);
    wait(0.8, sec);
    turn_to_angle(-75);
    wait(0.5, sec);
    
    Takein.spin(reverse,68,pct);
    rings4();
    wait(1,sec);
    drive_distance(10);
    Takein.spin(fwd,0,pct);
    turn_to_angle(115);
    drivelow_distance(15);
    Capture.open();
    drive_distance(-8);
    turn_to_angle(240);
    drive_distance(40);
    drivelow_distance(6);
    Capture.close();
    wait(1,sec);
    drive_distance(-30);
    
}

void blueplace(){
    drive_direct_distance(45);
    drivelow_distance(5.4);
    Capture.close();
    wait(1,sec);
    left_swing_to_angle(-75);
    Takein.spin(fwd,68,pct);
    wait(0.5,sec);
    Takein.spin(fwd,68,pct);
    drivelow_distance(-15);
    wait(0.5, sec);
    drive_distance(6);
    wait(0.5, sec);
    drive_distance(-6);



    wait(0.5,sec);

    turn_to_angle(-27);
    

    drive_distance(-26);
    wait(0.8, sec);
    turn_to_angle(-75);
    wait(0.5, sec);
    
    Takein.spin(reverse,68,pct);
    rings4();
    wait(1,sec);
    drive_distance(10);
    Takein.spin(fwd,0,pct);
    turn_to_angle(-115);
    
    wait(2,sec);
    drive_distance(40);
}

void open(){
    Capture.open();
}

void intake(){
    Takein.spin(reverse, 68, pct);
    wait(1, sec);
    Takein.spin(fwd, 68, pct);         
}

void rings2(){
    drivelow_distance(24);
    wait(0.5, sec);
    drive_distance(6);
    wait(0.5, sec);
    drive_distance(-6);
}
void visionadd(){
    while(1){
        vision_function();
    }
}
void visionpro(){
    while(1){
        vision_function();
    }
}

void rings4(){
    drive_distance(-40, desired_heading,10, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, 
    drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
    Takein.spin(fwd,68,pct);
    wait(1,sec);
    // drivelow_distance(10);
    // drivelow_distance(-10);
    // wait(0.5, sec);
    // drivelow_distance(10);
    // drivelow_distance(-10);
    // wait(0.5, sec);
    // drivelow_distance(10);
    // drivelow_distance(-10);
    // wait(0.5, sec);
    drive_distance(10);
    drive_distance(-10);
    wait(0.5, sec);
    drive_distance(10);
    drive_distance(-10);
    wait(0.5, sec);
    drive_distance(10);
    drive_distance(-10);
    wait(0.5, sec);
    drive_distance(10);
    drive_distance(-11);
    wait(0.5, sec);

}
void b(){
    wait(1,sec);
    out.open();
    Takein.spin(fwd,100,pct);
    wait(1,sec);
    out.close();

}
void c(){
    Arm_L.spinToPosition(0, degrees, 100, velocityUnits::pct);
}
void odometry_auto()
{
c();

Capture.close();
drive_distance(-108.0);//80
drivelow_distance(-14.5);//先取庄

Capture.open();//取庄
wait(0.25,sec);


turn_to_angle(-30);//转30度
wait(0.25,sec);
//cave.open();
 Takein.spin(fwd,100,pct);//取环
//vs_take();
drivelow_distance(34);//更近取环  2
cave.close();


wait(0.25,sec);
drivelow_distance(20);
drivelow_distance(-22);
turn_to_angle(60);//转90度
flag1++;
drivelow_distance(59);//踢环//34

Capture.close();

Takein.stop(coast);
turn_to_angle(-75);//转145度


drivelow_distance(-39*1.414-2);//取庄
thread shi(b);
Capture.open();//取庄

Lock.open();



drive_distance(3*45*1.414-20);//drive_distance();//
Takein.spin(fwd,100,pct);
thread ting(visionadd);
// flag1--;

// thread shijue1(visionadd);
drivelow_distance(140);//20

turn_to_angle(-145);
wait(0.25,sec);
Lock.close();
turn_to_angle(-75);

drivelow_distance(40);

// cave.open();
// drive_distance(8);
// cave.close();
// drive_distance(8);
Lock.open();
turn_to_angle(-175);
Lock.close(); 
turn_to_angle(-75);
// Lock.open();
// turn_to_angle(-145);
// Lock.close();
// turn_to_angle(-75);这段刚改
drive_distance(12);
drive_distance(-10);
drive_distance(12);

///寝室代码


// drive_distance(-16);
// drive_distance(16);
drive_distance(-44);

turn_to_angle(105);
//打庄
drive_distance(-44);
Capture.close();


turn_to_angle(60);

//Arm.spinToPosition(-600, deg, 500, rpm, false);
Arm_L.spinToPosition(-100, degrees, 100, velocityUnits::pct);
            wait(10,msec);
            Arm_L.spinToPosition(-80, degrees, 100, velocityUnits::pct);
            Arm_L.stop(hold);
drive_distance(74*2-5);
//drivelow_distance(20);
wait(2,sec);
turn_to_angle(-30);
drive_distance(-3);
Takein.stop(coast);
 Arm_L.spinToPosition(-600, degrees, 100, velocityUnits::pct);
Arm_L.spinToPosition(-600, degrees, 100, velocityUnits::pct);

drive_distance(-80);



/******************** */

// drive_distance(-17.8);//继续取环
// wait(2,sec);
// drive_distance(1);//后退瞄准
// wait(2,sec);
// turn_to_angle(78);//转角40度
// wait(2,sec);
// drive_distance(47);

//
//drive_direct_distance(5);
}
void f(){
    
    Takein.spin(fwd,100,pct);
}
void sjtu_auto(){
    thread t1(c);
Capture.close();
drive_distance(-99.0);//80
drivelow_distance(-6.2);//先取庄
Capture.open();
wait(0.25,sec);
thread t4(f);
    drive_distance(72);

Capture.close();
drive_distance(40);
turn_to_angle(-120);
drivelow_distance(-74);
Capture.open();
drive_distance(135+30);
drive_distance(-30);
turn_to_angle(-210);
drive_distance(80);
wait(1,sec);


    Capture.open();
drive_distance(-90);
turn_to_angle(-75);//75
Lock.open();
Takein.stop(coast);
drive_distance(70);
turn_to_angle(-175);//-25
Lock.close();
turn_to_angle(-255);//-105
drive_distance(-30);
Capture.close();
turn_to_angle(-265);
drive_distance(150);
drivelow_distance(50);
}
void odometry_auto1()//lanfang
{
  thread t1(c);
Capture.close();
drive_distance(-104);//80
drivelow_distance(-10.5);//先取庄

Capture.open();//取庄
wait(0.25,sec);


turn_to_angle(30);//转30度
wait(0.25,sec);
//cave.open();
 Takein.spin(fwd,100,pct);//取环
//vs_take();
drivelow_distance(34);//更近取环  2
cave.close();


wait(0.25,sec);
drivelow_distance(20);
drivelow_distance(-22);
turn_to_angle(-60);//转90度
flag1++;
drivelow_distance(59);//踢环//34

Capture.close();

Takein.stop(coast);
turn_to_angle(75);//转145度


drivelow_distance(-39*1.414-2+7.4-0.02);//取庄//-0
thread shi(b);
Capture.open();//取庄
wait(0.25,sec);
Lock.open();



drive_distance(3*45*1.414-29);//drive_distance();//29
Takein.spin(fwd,100,pct);
thread ting(visionadd);
// flag1--;

// thread shijue1(visionadd);
drivelow_distance(140);//20

turn_to_angle(5);

Lock.close();
turn_to_angle(75);

drivelow_distance(40);

// cave.open();
// drive_distance(8);
// cave.close();
// drive_distance(8);
Lock.open();
turn_to_angle(5);
Lock.close(); 
turn_to_angle(75);
// Lock.open();
// turn_to_angle(-145);
// Lock.close();
// turn_to_angle(-75);这段刚改
drive_distance(10);
drive_distance(-8);
drive_distance(10);

///寝室代码


// drive_distance(-16);
// drive_distance(16);
drive_distance(-44);

turn_to_angle(-105);
//打庄
drive_distance(-44);
Capture.close();


turn_to_angle(-60);

//Arm.spinToPosition(-600, deg, 500, rpm, false);
 Arm_L.spinToPosition(-100, degrees, 100, velocityUnits::pct);
            wait(10,msec);
            Arm_L.spinToPosition(-100, degrees, 100, velocityUnits::pct);
            Arm_L.stop(hold);

drive_distance(74*2-10);
drivelow_distance(10);
//wait(1,sec);
turn_to_angle(30);
drivelow_distance(10);
Takein.stop(coast);
drivelow_distance(-3);
 Arm_L.spinToPosition(-600, degrees, 100, velocityUnits::pct);
 drivelow_distance(-30);
 Arm_L.spinToPosition(0, degrees, 100, velocityUnits::pct);
//wait(2,sec);
drivelow_distance(-70);
// left_swing_to_angle(-180);

//  drive_distance(-150);
// turn_to_angle(-20);
// drive_distance(-90);
// turn_to_angle(-75);
 //Arm.spinToPosition(600, deg, 500, rpm, false);







/******************** */

// drive_distance(-17.8);//继续取环
// wait(2,sec);
// drive_distance(1);//后退瞄准
// wait(2,sec);
// turn_to_angle(78);//转角40度
// wait(2,sec);
// drive_distance(47);

//
//drive_direct_distance(5);
}
void auto_geometry(){
    drive_distance_geometry(2,5);
}void auto_test(){
    vs_take();
   
   }