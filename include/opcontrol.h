#pragma once

extern void opcontrol();
extern void chassis();
extern double vL();
extern double vR();
extern void takein();
extern void capture();
extern void arm();
extern void buttonControl();
extern void onevent_ButtonDown_pressed();
extern void onevent_ButtonY_pressed();
extern void onevent_ButtonRight_pressed();
extern void onevent_ButtonB_pressed();
extern void onevent_ButtonX_pressed();
extern void feedback();

extern int on;
extern int mode;
extern double pumpback;
extern void lowtake();
void a1();