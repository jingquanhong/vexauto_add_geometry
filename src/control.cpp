#include "vex.h"
using namespace vex;

bool flag= 0;



void inite() {
    // Slide.setPosition(0, deg);  // 设置传送带的初始位置
    Arm_L.setPosition(0, deg);
    
}



void chassisControl(){ 
    double left_c,right_c;
    int i=1;
    
    
    left_c= Controller.Axis3.position(percent)*i-(Controller.Axis1.position(percent)*0.8);
    right_c=Controller.Axis3.position(percent)*i+(Controller.Axis1.position(percent)*0.8);
    
    
    if(Controller.Axis3.position(pct)>5||Controller.Axis3.position(pct)<-5||Controller.Axis1.position(pct)>5||Controller.Axis1.position(pct)<-5)
    {
    L.spin(directionType::rev,left_c/100*30,volt);
    R.spin(directionType::rev,right_c/100*30,volt);
    }
    
    
    else{
        L.stop(brakeType::coast);
        R.stop(brakeType::coast);
        
        
      }
    
}
void chassisControl1(){ 
    double left_c,right_c;
    int i=1;
    
    
    left_c= -Controller.Axis3.position(percent)*i+(Controller.Axis1.position(percent)*0.65);
    right_c=-Controller.Axis3.position(percent)*i-(Controller.Axis1.position(percent)*0.65);
    
    
    if(Controller.Axis3.position(pct)>5||Controller.Axis3.position(pct)<-5||Controller.Axis1.position(pct)>5||Controller.Axis1.position(pct)<-5)
    {
    L.spin(directionType::rev,left_c/100*30,volt);
    R.spin(directionType::rev,right_c/100*30,volt);
    }
    
    
    else{
        L.stop(brakeType::coast);
        R.stop(brakeType::coast);
        
        
      }
    
}

void P1()
{
    
    if(Controller.ButtonY.pressing())
        flag = !flag;    
    Capture.set(flag);
    while(Controller.ButtonY.pressing());   
    wait(10,msec);
}

void P3()
{
    // 按住时设置气动为1，松开时为0
    Lock.set(Controller.ButtonRight.pressing());
    wait(10, msec); // 防止 CPU 过载
}
void list_control() {
    
    static bool prevR1 = false;
    static bool prevR2 = false;
    static int buttonState = 0;

    bool currR1 = Controller.ButtonR1.pressing();
    bool currR2 = Controller.ButtonR2.pressing();

    // 优先检查 ButtonLeft 是否被按下
    if (Controller.ButtonLeft.pressing()) {
       buttonState = 4;
    }
    else {
       
        bool bothPressedThisCycle = currR1 && currR2 ;

        if (bothPressedThisCycle) {
            
            buttonState = 3;
        }
        else if (currR1 && !currR2) {
           
            buttonState = 1;
        }
        else if (currR2 && !currR1) {
           
            buttonState = 2;
        }
        else if (!currR1 && !currR2) {
           
            buttonState = 0;
        }
       
    }

    
    switch(buttonState) {
        case 4:
            Arm_L.setPosition(0, deg);
            break;
        case 3:
        
            // Arm_L.spin(fwd,100,pct);
             spin_to_angle(300);
            // Arm_L.spinToPosition(-125, degrees, 100, velocityUnits::pct);
            // wait(10,msec);
            // Arm_L.spinToPosition(-125, degrees, 100, velocityUnits::pct);
            // Arm_L.stop(hold);
           
            break;
        case 2:
            
            Arm_L.spin(forward, 100, percent);
           
            break;
        
        case 1:
            
           
            Arm_L.spin(reverse, 100, percent);
            
            
            break;
        
        default:
            if(Arm_L.position(degrees) >= -80){
            Arm_L.stop(brake);}
            else{
            Arm_L.stop(hold);}
            
            break;
    }
    prevR1 = currR1;
    prevR2 = currR2;

    if (Controller.ButtonL1.pressing()) {
        Takein_R.spin(fwd, 80, percent);
        Takein_L.spin(fwd, 100, percent);  
     }
     
    
    
    else if (Controller.ButtonL2.pressing()) {
        Takein_R.spin(reverse, 80, percent);
        Takein_L.spin(reverse, 100, percent); 
    }
     
    else if(!Controller.ButtonL1.pressing() && !Controller.ButtonL2.pressing())
    {   

       
        Takein_R.stop(coast); 
        Takein_L.stop(coast);             

    }
    
}
// 主控制函数
void control() {
    
    
    //list_control();
    vex::thread SHANG(list_control);

    //chassisControl();
    vex::thread DIPAN(chassisControl);
    
    P1();
    
    P3();
    
    

    
}
