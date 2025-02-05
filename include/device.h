#pragma once
using namespace vex;

extern brain Brain;
extern controller Controller;
extern timer time0;
extern rotation arm_rotation;
extern motor L1;
extern motor L2;
extern motor L3;
extern motor L4;
extern motor L5;
extern motor R1;
extern motor R2;
extern motor R3;
extern motor R4;
extern motor R5;
extern motor_group L;
extern motor_group R;

extern motor Takein_L;
extern motor Takein_R;
extern motor_group Takein;

extern motor Arm_L;
extern motor Arm_R;
extern motor_group Arm;

extern pneumatics Capture;
extern pneumatics out;
extern pneumatics Lock;
extern pneumatics cave;
extern inertial Inertial;
extern distance distance1;

extern vision::signature SIG_1;
extern vision::signature SIG_2;
extern vision::signature SIG_3;
extern vision::signature SIG_4;
extern vision::signature SIG_5;
extern vision::signature SIG_6;
extern vision::signature SIG_7;
extern vision Vision1;
extern vex::rotation leftEncoder;
extern vex::rotation rightEncoder;