#include "vex.h"
using namespace vex;

int on = 0;
double pumpback = -500;
int mode = 4;

bool switch1=false;
int flag2=0;//传送带初始转速flag
int vtk=0;
int flag3=0;//回抽flag
double t1=0;
double t2=0;

double z=0;
//说明：需要确定一个传送带的初始化位置


void opcontrol()
{
    //autoplay();
    while(1)
    {
        Vision1.takeSnapshot(SIG_1);
        if (Vision1.largestObject.exists&&Vision1.largestObject.width*Vision1.largestObject.height>=500){
            out.open();
        }else{
            out.close();
        }
        buttonControl();
        chassis();
        takein();
        lowtake();
        vision_function();
        capture();
        arm();
        //feedback();
        wait(20, msec);
    }
}

void buttonControl()
{
    Controller.ButtonDown.pressed(onevent_ButtonDown_pressed);
    Controller.ButtonRight.pressed(onevent_ButtonRight_pressed);
    Controller.ButtonY.pressed(onevent_ButtonY_pressed);
    Controller.ButtonX.pressed(onevent_ButtonX_pressed);
    Controller.ButtonA.pressed(a1);

}

void onevent_ButtonB_pressed()
{
    Capture.set(!Capture.value());
}
void onevent_ButtonY_pressed()
{
    mode = 0;
}   
void onevent_ButtonRight_pressed()
{
    mode = 1;
}
void onevent_ButtonDown_pressed()
{
    gudingjuli(150);//利用距离传感器走到固定距离
    mode = 2;
}
void onevent_ButtonX_pressed()
{
    Lock.set(!Lock.value());
}


void chassis()
{
    
        L.spin(fwd, vL(), pct);
        R.spin(fwd, vR(), pct);
    
}
double vL()
{
    double v = (-(double)Controller.Axis3.position(pct) / 100) * 100;
    double w = ((double)Controller.Axis1.position(pct) / 100) * 100;
    return Capture.value()? (v + w) : (-v + w);
    //return v+w;
}
double vR()
{
    double v = (-(double)Controller.Axis3.position(pct) / 100) * 100;
    double w = ((double)Controller.Axis1.position(pct) / 100) * 100;
    return Capture.value()? (v - w) : (-v - w);
    //return v-w;
}

void takein()
{//68

    int direction = 0;
    if(Controller.ButtonR1.pressing())
    {
        direction = -1;
        
    }
    else if(Controller.ButtonR2.pressing()){
        direction = 1;
    }
    Takein.spin(fwd, 68 * direction, pct);
}

void capture(){
    Controller.ButtonB.pressed(onevent_ButtonB_pressed);
}


void arm(){
    switch (mode) {
    case 0:
        Arm.spinToPosition(10, deg, 500, rpm, false);
        if(Arm.position(deg) > 5 && Arm.position(deg) < 15){
            mode = 4;
            }
        break;
    case 1:
        Arm.spinToPosition(130, deg, 500, rpm, false);
        if(Arm.position(deg) > 125 && Arm.position(deg) < 135){
            mode = 4;
        }
        break;
    case 2:
        Arm.spinToPosition(850, deg, 500, rpm, false);
        if(Arm.position(deg) > 845 && Arm.position(deg) < 855){
            mode = 4;
        }
        break;
    default:
        if(Controller.ButtonL1.pressing()&&!Controller.ButtonL2.pressing()){
            if(Arm.position(deg) < 1000){
                Arm_L.spin(fwd, 60, pct);
                Arm_R.spin(fwd, 60, pct);
            }
        }
        else if(!Controller.ButtonL1.pressing()&&Controller.ButtonL2.pressing()){
            if(Arm.position(deg) > 0){
                Arm_L.spin(reverse, 60, pct);
                Arm_R.spin(reverse, 60, pct);
            }
        }
        else{
            Arm_L.stop(hold);
            Arm_R.stop(hold);
        }
        break;
    }
}

void take()
{
    vtk=68;
    // if(flag2<10000)//防止一开始就检测到转速低于10pct先跑个40遍程序
    // {
    //     Takein.spin(fwd,68,pct);
    //     flag2++;
    // }
    // else if(Takein.velocity(rpm)<20)//回抽防卡
    // {
    //     if(flag3==0)
    //     { 
    //         t1=time0.value()+0.2;
    //         flag3=1;
    //     }
    //     if(time0.value()<t1)
    //     {
    //        vtk=-50;
    //     }
    //     else
    //     {
    //         vtk=68;
    //     }
    // }
    // else
    // flag3=0;   
    
}

void lowtake()
    {//68
    if(Controller.ButtonLeft.pressing())
    {
        Takein.spin(fwd, 80, pct);
    }
    
}


void feedback(){
}
void a1(){
    gudingjuli(140);
    //mode=2;
}