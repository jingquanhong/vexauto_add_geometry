#include"vex.h"
#define the_radius_of_encoderwheel 4.0*0.393700787//按英寸算的
#define M_PI 3.14159265358979323846
#define wide_of_vexrobot 12.0*0.393700787//按英寸算的
Kalman_Filter_t *kf;
double desire_heading;

float wheel_diameter=4;
float wheel_ratio=1.75;
float drive_in_to_deg_ratio=(wheel_ratio/360.0*M_PI*wheel_diameter);

float drive_max_voltage=10;
float drive_kp=1.5;//1.5//2.5
float drive_ki=0.12;
float drive_kd=8;
float drive_starti=0;

float drive_settle_error=1.5;
float drive_settle_time=100;
float drive_timeout=2000;

float angle_max_voltage=6;
float angle_kp=0.3;
float angle_ki=0;
float angle_kd=0;
float angle_starti=0;//角度环

float heading_max_voltage=6;
float heading_kp=0.62;//0.42
float heading_ki=0.12;//0
float heading_kd=0.7;
float heading_starti=0;

float swing_max_voltage=12;
float swing_kp=0.3;
float swing_ki=0.001;
float swing_kd=2;
float swing_starti=15;

float swing_settle_error=1;
float swing_settle_time=300;
float swing_timeout=500;

float turn_max_voltage=20;
float turn_settle_error=1;
float turn_settle_time=100;
float turn_timeout=800;
float turn_kp=0.4;    //0.5  0.001  4
float turn_ki=0.1;
float turn_kd=5.5;
float turn_starti=15;
  
float desired_angle;
float desired_heading;
float DesiredHeading;
float gets_absolute_heading(){ 
  return( reduce_0_to_360( Inertial.rotation()) ); 
}

float get_absolute_heading(){ 
  return( reduce_0_to_360( Inertial.rotation() ) ); 
}

void drive_with_voltage(float leftVoltage, float rightVoltage){
  L.spin(fwd, leftVoltage, volt);
  R.spin(fwd, rightVoltage,volt);
}
void start_with_voltage(float Arm_L_Voltage){
  Arm_L.spin(fwd,Arm_L_Voltage,volt);
}

float get_left_position_in(){
  return( L.position(deg)*drive_in_to_deg_ratio );
}

float get_right_position_in(){
  return( R.position(deg)*drive_in_to_deg_ratio );
}

//如果不使用里程计，这是一段很好的代码
void drive_distance(float distance){
  drive_distance(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  heading=get_absolute_heading();
  desire_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - gets_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);
    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
    drive_with_voltage(drive_output + heading_output, drive_output - heading_output);
    task::sleep(10);}
  L.stop(coast);R.stop(coast);
}
/**************************geometry_ku*****************************************/
/*******************************************************************/
void drive_distance_another(float distance){
  drive_distance_another(distance, DesiredHeading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}
void drive_distance_another(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
   heading=get_absolute_heading();
  desire_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - gets_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = leftEncoder.position(vex::rotationUnits::deg)/360.0*(2*M_PI*the_radius_of_encoderwheel);
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = leftEncoder.position(vex::rotationUnits::deg)/360.0*(2*M_PI*the_radius_of_encoderwheel);
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);
    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
    drive_with_voltage(drive_output + heading_output, drive_output - heading_output);
    task::sleep(10);}
  L.stop(brake);R.stop(brake);
}



void drive_direct_distance(float distance){
  drive_direct_distance(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void drive_direct_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  heading=get_absolute_heading();
  desire_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - gets_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output, drive_output);
    task::sleep(10);
  }
  
  L.stop(brake);
  R.stop(brake);
}

// float turn_max_voltage=12;
// float turn_settle_error=1;
// float turn_settle_time=100;
// float turn_timeout=800;
// float turn_kp=0.5;    //0.5  0.001  4
// float turn_ki=0.001;
// float turn_kd=4;
// float turn_starti=15;
/*********转向pid******** */
void spin_to_angle(float angle){
  spin_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void spin_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  desired_angle = angle;
  PID turnPID(reduce_negative_180_to_180(angle - get_arm_angle()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_arm_angle());
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    start_with_voltage(output);
    task::sleep(10);
  }
  Arm_L.stop(brake);
  Arm_R.stop(brake);
}

void coast_drive_distance(float distance){
  coast_drive_distance(distance, desired_heading, 8, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void coast_drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  heading=get_absolute_heading();
  desire_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - gets_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output, drive_output);
    
  }
  L.stop(brake);
  R.stop(brake);
  

}

void drive_firstdistance(float distance){
  drive_distance(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void drive_firstdistance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  heading=get_absolute_heading();
  desire_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - gets_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output , drive_output );
    //drive_with_voltage(drive_output + heading_output, drive_output - heading_output);
    if(drive_error<=20)
    drive_max_voltage=3;
  }
  L.stop(brake);
  R.stop(brake);
}

void firstdistance(double e)
{
  float target= (get_left_position_in()+get_right_position_in())/2.0+e * drive_in_to_deg_ratio;
  while((get_left_position_in()+get_right_position_in())/2.0<target*3/4)
    drive_with_voltage(-10 , -10 );
  while((get_left_position_in()+get_right_position_in())/2.0/2>=target*3/4
        &&(get_left_position_in()+get_right_position_in())/2.0<target)
    drive_with_voltage(-5 , -5 );
  L.stop(brake);
  R.stop(brake);
}

void left_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    L.spin(fwd, output, volt);
    R.stop(hold);
    task::sleep(10);
  }
  L.stop(brake);
  R.stop(brake);
}

void left_swing_to_angle(float angle){
  left_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    R.spin(reverse, output, volt);
    L.stop(hold);
    task::sleep(10);
  }
  L.stop(brake);
  R.stop(brake);
}

void right_swing_to_angle(float angle){
  right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  desired_heading = angle;
  /*****如果不用kalman该这三段就行******/
  // float angle_test;
  // update(kf, get_absolute_heading());
  // angle_test=get_estimate(kf);
  /***********/
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    task::sleep(10);
  }
  // PID turnPID(reduce_negative_180_to_180(angle - angle_test), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  // while(turnPID.is_settled() == false){
  //   float error = reduce_negative_180_to_180(angle - angle_test);
  //   float output = turnPID.compute(error);
  //   output = clamp(output, -turn_max_voltage, turn_max_voltage);
  //   drive_with_voltage(output, -output);
  //   task::sleep(10);
  // }
  L.stop(brake);
  R.stop(brake);
}

void turn_to_angle(float angle){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void turnlow_to_angle(float angle){
  turn_to_angle(angle, 3, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}


void drivelow_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  heading=get_absolute_heading();
  desire_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - gets_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }
  L.stop(brake);
  R.stop(brake);
}

void drivelow_distance(float distance){
  drive_distance(distance, desired_heading,3, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void drive1_distance(float distance){
  drive_distance(distance, desired_heading,10, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

//右1
void qianjing(double du)
{
  do{
    L.spin(fwd,36,pct);
    R.spin(fwd,72,pct);
  }while(Inertial.yaw()>du);
  L.stop(brake);
  R.stop(brake);
}

void qianjing2(double du)
{
  do{
    L.spin(fwd,60,pct);
    R.spin(fwd,45,pct);
  }while(Inertial.yaw()<du);
  L.stop(brake);
  R.stop(brake);
}

void qianjing3(double du)
{
  do{
    L.spin(fwd,50,pct);
    R.spin(fwd,60,pct);
  }while(Inertial.yaw()>du);
  L.stop(brake);
  R.stop(brake);
}

//右1
// void daoche(double du)
// {
//   do{
//     L.spin(reverse,80,pct);
//     R.spin(reverse,70,pct);
//   }while(Inertial.rotation(deg)<du);
//   L.stop(brake);
//   R.stop(brake);
// }



void gudingjuli(float dis){//走到固定距离 dis设定距离  
  int a=0;
  int Vec=60;//移动速度
  int errordis=10;//容许距离波动误差
  while(a!=1){
    L.setVelocity(Vec,pct);
    R.setVelocity(Vec,pct);
    L.spin(reverse);
    R.spin(reverse);
  if(distance1.objectDistance(mm)<dis-errordis){
    L.spin(fwd);
    R.spin(fwd);
    
  }else if(distance1.objectDistance(mm)>dis+errordis){
    L.spin(reverse);
    R.spin(reverse);
    
  }else if(distance1.objectDistance(mm)<dis+errordis&&distance1.objectDistance(mm)>dis-errordis){
    L.stop(brake);
    R.stop(brake);
    a=1;
  }}
}