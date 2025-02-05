// #include "vex.h"
// using namespace vex;

// brain Brain;
// controller Controller;
// timer time0;
// //主控以及手柄

// motor L1(PORT16, ratio18_1);
// motor L2(PORT17, ratio18_1, true);
// motor L3(PORT18, ratio18_1);
// motor L4(PORT19, ratio18_1, true);
// motor L5(PORT20, ratio18_1, true);
// motor R1(PORT11, ratio18_1);
// motor R2(PORT12, ratio18_1);
// motor R3(PORT13, ratio18_1);
// motor R4(PORT14, ratio18_1, true);
// motor R5(PORT15, ratio18_1, true);
// motor_group L(L1, L2, L3, L4, L5);
// motor_group R(R1, R2, R3, R4, R5);
// //十电机底盘

// motor Takein_L(PORT9, ratio6_1, true);
// motor Takein_R(PORT10, ratio6_1);
// motor_group Takein(Takein_L, Takein_R);
// //吸盘取环传送装置

// motor Arm_L(PORT5, ratio6_1);
// motor Arm_R(PORT6, ratio6_1, true);
// motor_group Arm(Arm_L, Arm_R);
// //前臂

// pneumatics Capture(Brain.ThreeWirePort.H);
// pneumatics out(Brain.ThreeWirePort.G);
// pneumatics Lock(Brain.ThreeWirePort.F);
// //尖桩夹持、剔环及自锁

// inertial Inertial(PORT7);
// //惯性传感器

// vex::vision::signature SIG_1 = vex::vision::signature (1, 7233, 9967, 8600, -939, -197, -568, 3, 0);
// vex::vision::signature SIG_2 = vex::vision::signature (2, -4757, -3855, -4306, 7159, 9885, 8522, 3.6, 0);
// vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision Vision1 = vex::vision (vex::PORT8, 50, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
// //视觉传感器初始化



//红车端口
// #include "vex.h"
// using namespace vex;

// brain Brain;
// controller Controller;
// timer time0;
// //涓绘帶浠ュ強鎵嬫焺

// motor L1(PORT18, ratio18_1,true);
// motor L2(PORT17, ratio18_1);
// motor L3(PORT16, ratio18_1,true);
// motor L4(PORT15, ratio18_1);
// motor L5(PORT14, ratio18_1,true);
// motor R1(PORT13, ratio18_1,true);
// motor R2(PORT12, ratio18_1);
// motor R3(PORT11, ratio18_1);
// motor R4(PORT7, ratio18_1,true);
// motor R5(PORT6, ratio18_1);
// motor_group L(L1, L2, L3, L4, L5);
// motor_group R(R1, R2, R3, R4, R5);
// //鍗佺數鏈哄簳鐩?

// motor Takein_L(PORT5, ratio6_1, true);
// motor Takein_R(PORT20, ratio6_1,false);
// motor_group Takein(Takein_L, Takein_R);
// //鍚哥洏鍙栫幆浼犻€佽缃?

// motor Arm_L(PORT9, ratio6_1,false);
// motor Arm_R(PORT8, ratio6_1, true);
// motor_group Arm(Arm_L, Arm_R);
// //鍓嶈噦

// pneumatics Capture(Brain.ThreeWirePort.H);
// pneumatics out(Brain.ThreeWirePort.A);
// pneumatics Lock(Brain.ThreeWirePort.F);
// //灏栨々澶规寔銆佸墧鐜強鑷攣

// inertial Inertial(PORT4);
// //鎯€т紶鎰熷櫒
// distance distance1(PORT3);
// vex::rotation leftEncoder(PORT10);
// vex::rotation rightEncoder(PORT2);
// vex::vision::signature SIG_1 = vex::vision::signature (1, 7233, 9967, 8600, -939, -197, -568, 3, 0);//3
// vex::vision::signature SIG_2 = vex::vision::signature (2, -4115, -3625, -3870, 6163, 7021, 6592, 7, 0);
// vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 20.5, 0);
// vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision Vision1 = vex::vision (vex::PORT19, 50, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
// //CQU-RBF
// #include "vex.h"
// using namespace vex;

// vex::brain Brain;
// timer time0;
// controller Controller;

// motor Takein_L(PORT1,ratio18_1,true);
// motor Takein_R(PORT9,ratio18_1,false);
// motor_group Takein(Takein_L,Takein_R);/////////吸环

// motor L1 = motor(PORT11, ratio18_1, false);
// motor L2 = motor(PORT12, ratio18_1, false);
// motor L3 = motor(PORT13, ratio18_1, false);
// motor L4 = motor(PORT14, ratio18_1, false);
// motor L5 = motor(PORT15, ratio18_1, true);
// motor_group R(L1,L2,L3,L4,L5);
// motor R1 = motor(PORT16, ratio18_1, true);
// motor R2 = motor(PORT17, ratio18_1, false);
// motor R3 = motor(PORT18, ratio18_1, true);
// motor R4 = motor(PORT19, ratio18_1, true);
// motor R5 = motor(PORT20, ratio18_1, true);
// motor_group L(R1,R2,R3,R4,R5);//////底盘

// motor Arm_L = motor(PORT2, ratio18_1, true);/////摇臂
// motor Arm_R = motor(PORT10, ratio18_1, false);
// motor_group Arm(Arm_L, Arm_R);



// pneumatics Capture(Brain.ThreeWirePort.H); ///夹环气动
// pneumatics Lock(Brain.ThreeWirePort.B);
// pneumatics out(Brain.ThreeWirePort.C);



// inertial Inertial = inertial(PORT5);//惯性传感器
// distance distance1=distance(PORT5);    //距离传感器
// rotation leftEncoder=rotation(PORT6);//编码器
// rotation rightEncoder=rotation(PORT2);

// drivetrain Drivetrain = drivetrain(L, R, 314.15, 295, 40, mm, 1);
// vex::vision::signature SIG_1 = vex::vision::signature (1, 7233, 9967, 8600, -939, -197, -568, 3, 0);//3
// vex::vision::signature SIG_2 = vex::vision::signature (2, -4115, -3625, -3870, 6163, 7021, 6592, 7, 0);
// vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 20.5, 0);
// vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
// vex::vision Vision1 = vex::vision (vex::PORT8, 50, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);

//cqu-cjh
#include "vex.h"
using namespace vex;

vex::brain Brain;
timer time0;
controller Controller;

motor Takein_L(PORT6,ratio6_1,true);
motor Takein_R(PORT7,ratio18_1,true);
motor_group Takein(Takein_L,Takein_R);/////////吸环



motor R1 = motor(PORT11, ratio6_1, true);
motor R2 = motor(PORT12, ratio6_1, false);
motor R3 = motor(PORT13, ratio6_1, true);
motor R4 = motor(PORT14, ratio6_1, false);
motor R5 = motor(PORT15, ratio6_1, true);
motor_group L(L1,L2,L3,L4,L5);

motor L1 = motor(PORT16, ratio6_1, false);
motor L2 = motor(PORT17, ratio6_1, true);
motor L3 = motor(PORT18, ratio6_1, false);
motor L4 = motor(PORT19, ratio6_1, true);
motor L5 = motor(PORT20, ratio6_1, false);
motor_group R(R1,R2,R3,R4,R5);//////底盘


motor Arm_L = motor(PORT8, ratio36_1, true);/////摇臂就这一个
motor Arm_R = motor(PORT10, ratio36_1, false);
motor_group Arm(Arm_L, Arm_R);

pneumatics Capture(Brain.ThreeWirePort.A); ///夹环气动
pneumatics out(Brain.ThreeWirePort.D);
pneumatics Lock(Brain.ThreeWirePort.C);
pneumatics cave(Brain.ThreeWirePort.B);


rotation arm_rotation=rotation(PORT9);  

inertial Inertial = inertial(PORT21);//惯性传感器
distance distance1=distance(PORT5);    //距离传感器
rotation leftEncoder=rotation(PORT6);//编码器
rotation rightEncoder=rotation(PORT2);
drivetrain Drivetrain = drivetrain(L, R, 314.15, 295, 40, mm, 1);
vex::vision::signature SIG_1 = vex::vision::signature (1, 7051, 8725, 7888, -1579, 175, -702, 4.8, 0);//3
vex::vision::signature SIG_2 = vex::vision::signature (2, 7051, 8725, 7888, -1579, 175, -702, 4.8, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 20.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision Vision1 = vex::vision (vex::PORT19, 50, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);