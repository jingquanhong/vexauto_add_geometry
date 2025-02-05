#include "vex.h"
using namespace vex;



void vision_function(){
    Cheese();
    Kick();
}

int count = 0;
double queue[100] = {-10000};
bool complete = 1;

void Cheese(){
    if(complete){
        Vision1.takeSnapshot(SIG_1);
        if(Vision1.objects[0].height * Vision1.objects[0].width >= 50000){
            count++;
            queue[count] = Takein.position(deg);
            complete = 0;
        }
    }
}

void Kick(){
    if(Takein.position(deg) > queue[count] && Takein.position(deg) < queue[count] + 600){
        out.open();
        complete = 1;
    }else{
        out.close();
    }
}



double vodom=60;//这个是初始化的传送带的位置到第一个钩子到识别区域的距离
double flag4=0;
double flag5=0;//一个位置只识别一次的flag
double flag6=0;//取瞬时时间

void vs()//视觉
{
    if(Takein.position(deg)>=vodom&&flag5==0)
    {
        Vision1.takeSnapshot(SIG_1);
        if(Vision1.objects[0].height*Vision1.objects[0].width>=8000)//1000是面积要改一改
        {
            flag4=1;
        }
        flag5=1;
    }
    
}

void go()//剔除错色环
{
    if(flag4==1&&Takein.position(deg)<=vodom+149)//张开多少秒，如果不太行就改成编码器在一定范围开就行
    {
        out.open();
    }
    else
    {
        out.close();
    }
    if(Takein.position(deg)>=vodom+149)//这个是剔除完错误环以后的值，
    {
        vodom+=149;//1000是两个钩子之间的间隔，相当于刷新到下一个钩子
        flag5=0;
        flag4=0;
        flag6=0;/*这四个相当于三个标志位重新初始化，但这有个bug是必须完成对一个环的剔除之后，
                才会刷新开始检测下一个环，要看间距够不够，我今天来看感觉是够的，要是够的话就ok，不够的话就得重新换算法*/
    }
}
// void vs_take(){
//     Takein.spin(fwd,100,pct);
//     Vision1.takeSnapshot(SIG_1);
//     Brain.Screen.print('1');
//     if(Vision1.largestObject.exists&&Vision1.largestObject.height*Vision1.largestObject.width>=210*210){
//      out.open();
//      Brain.Screen.print('2');
//     }else{
//     out.close();
//     Brain.Screen.print('3');
//     }
// }

void vs_take(){
    
    Brain.Screen.clearLine();
  Vision1.takeSnapshot(SIG_1);
  if (Vision1.largestObject.exists){
    
    Brain.Screen.print("Vision Sensor: x: %d", Vision1.largestObject.originX);
    Brain.Screen.print(" Y: %d", Vision1.largestObject.originY);
    Brain.Screen.print(" W %d", Vision1.largestObject.width);
    Brain.Screen.print(" H: %d", Vision1.largestObject.height);
    wait(1,sec);

}
 else
  {
     
    Brain.Screen.print("Vision Sensor: Color Signature Not Found!");
    wait(1,sec);
    } 
}