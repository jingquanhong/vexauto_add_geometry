里程计控制原理
基于上交2023赛季开源代码

### 获取当前机器人全局位置

在此之前需要知道以下这两个函数是如何获取左右定位轮走过的距离的

```cpp
/**
 * @brief 左定位轮走过距离 cm
 * 
 * @return float 
 */
void Position::updateLMileage()
{
    lastLMileage = curLMileage;

    curLMileage = deg2rad(-EncoderL.position(degrees)) * TrackingWheelRadius;
}

/**
 * @brief 右定位轮走过距离 cm
 * 
 * @return float 
 */
void Position::updateRMileage()
{
    lastRMileage = curRMileage;

    curRMileage = deg2rad(-EncoderR.position(degrees)) * TrackingWheelRadius;
}
```

直接通过编码器的position函数直接计算角度该角度是弧度制，然后乘上路子的半径就是轮子走过的弧长，也就是定位轮走过的距离，在这里并没有经过pid的计算

```cpp
/**
 * @brief 返回陀螺仪朝向，正前方为0，顺时针为正，范围 0 ~ 2PI
 * 
 * @return double 
 */
void Position::updateInertialHeading()
{
    lastIMUHeading = curIMUHeading;
    curIMUHeading = deg2rad(IMUHeading());
}
```

这里需要通过陀螺仪获取当前朝向，也是用的弧度制

```cpp
double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

```

然而根据发现，上交程序获取当前位置是通过更新获取的，而这个更新的距离就是通过速度乘以时间获取的，然而这个速度是当前速度，也是通过速度计算的

之前有个函数提到了左右编码轮的当前走过的距离，这里通过计算也是基于这个距离，通过读取采样时间之前的距离和采样时间之后的距离乘以1000除以采样时间，就是计算当前左右编码轮的速度，加上将速度的单位在转换成cm/s，然后通过滤波转换当前速度，最后就是计算世界坐标系下的x，y分速度以及机器人坐标系下的速度

```cpp
/**
 * @brief 左定位轮速度 cm/s
 * 
 * @return float 
 */
void Position::updateLSpeed()
{
    lastLSpeed = curLSpeed;

    double ret = (curLMileage - lastLMileage) * 1000 / sampleTime;                      //前向差分法
    if (abs(ret) > 1000 || abs(ret) < 0.001) ret = 0;
    
    // 滤波前的速度
    LSpeed[2] = LSpeed[1];
    LSpeed[1] = LSpeed[0];
    LSpeed[0] = ret;

    // 滤波后的速度
    FiltLSpeed[2] = FiltLSpeed[1];
    FiltLSpeed[1] = FiltLSpeed[0];
    FiltLSpeed[0] = (fliter_b[0] * LSpeed[0] + fliter_b[1] * LSpeed[1] + fliter_b[2] * LSpeed[2] - fliter_a[1] * FiltLSpeed[1] - fliter_a[2] * FiltLSpeed[2]) / fliter_a[0]; 
    if (abs(FiltLSpeed[0]) < 0.001) FiltLSpeed[0] = 0;

    curLSpeed = FiltLSpeed[0]; 
    // curLSpeed = ret;
}

/**
 * @brief 右定位轮速度 cm/s
 * 
 * @return float 
 */
void Position::updateRSpeed()
{
    lastRSpeed = curRSpeed;

    double ret = (curRMileage - lastRMileage) * 1000 / sampleTime;                      //前向差分法
    if (abs(ret) > 1000 || abs(ret) < 0.001) ret = 0;                                   //速度过大或过小，认为是噪声

    // 滤波前的速度
    RSpeed[2] = RSpeed[1];
    RSpeed[1] = RSpeed[0];
    RSpeed[0] = ret;

    // 滤波后的速度
    FiltRSpeed[2] = FiltRSpeed[1];
    FiltRSpeed[1] = FiltRSpeed[0];
    FiltRSpeed[0] = (fliter_b[0] * RSpeed[0] + fliter_b[1] * RSpeed[1] + fliter_b[2] * RSpeed[2] - fliter_a[1] * FiltRSpeed[1] - fliter_a[2] * FiltRSpeed[2]) / fliter_a[0];
    if (abs(FiltRSpeed[0]) < 0.001) FiltRSpeed[0] = 0;

    curRSpeed = FiltRSpeed[0];
    // curRSpeed = ret;
}

/**
 * @brief 自身坐标系y轴速度 cm/s
 * 
 * @return double 
 */
void Position::updateSelfYSpeed()
{
    double D = -cos(REncoderAngle) * sin(LEncoderAngle) - cos(LEncoderAngle) * sin(REncoderAngle);
    double Dy = curLSpeed * -cos(REncoderAngle) - curRSpeed * cos(LEncoderAngle);
    selfYSpeed = Dy / D;

    #ifdef debug
    double caliberLSpeed = 0;
    double caliberRSpeed = 0;

    if (TestCaliberFlag)
    {
        if (angleSpeed < 0)
        {
            caliberLSpeed = curRSpeed * (leftCCCoefficient/rightCCCoefficient) * sign(curLSpeed);
            caliberRSpeed = curRSpeed;
        }
        else if (angleSpeed > 0)
        {
            caliberRSpeed = curLSpeed * (rightCLCoefficient/leftCLCoefficient) * sign(curRSpeed);
            caliberLSpeed = curLSpeed;
        }
        else
        {
            caliberLSpeed = curLSpeed;
            caliberRSpeed = curRSpeed;
        }

        cout<<setiosflags(ios::fixed)<<setprecision(2)<<curRSpeed<<" "<<curLSpeed<<" "<<caliberRSpeed<<" "<<caliberLSpeed<<endl;
    }
    #endif
}

/**
 * @brief 自身坐标系x轴速度 cm/s
 * 
 * @return double 
 */
void Position::updateSelfXSpeed()
{
    double D = -cos(REncoderAngle) * sin(LEncoderAngle) - cos(LEncoderAngle) * sin(REncoderAngle);
    double Dx = curRSpeed * sin(LEncoderAngle) - curLSpeed * sin(REncoderAngle);
    selfXSpeed = Dx / D;
}

/**
 * @brief 世界坐标系y轴速度 cm/s
 * 
 * @return float 
 */
void Position::updateGlobalYSpeed()
{
    lastglobalYSpeed = globalYSpeed;
    globalYSpeed = selfYSpeed * cos(curIMUHeading) - selfXSpeed * sin(curIMUHeading);
    if (abs(globalYSpeed) < 0.01) globalYSpeed = 0;
    if (abs(globalYSpeed) > 250) globalYSpeed = lastglobalYSpeed;
}

/**
 * @brief 世界坐标系x轴速度 cm/s
 * 
 * @return float 
 */
void Position::updateGlobalXSpeed()
{
    lastglobalXSpeed = globalXSpeed;
    globalXSpeed = selfYSpeed * sin(curIMUHeading) + selfXSpeed * cos(curIMUHeading);
    if (abs(globalXSpeed) < 0.01) globalXSpeed = 0;
    if (abs(globalXSpeed) > 250) globalXSpeed = lastglobalXSpeed;

    // # ifdef debug
    //     if (CollectFlag)
    //     {
    //         cout <<setiosflags(ios::fixed)<<setprecision(2)<< globalXSpeed << " " << globalYSpeed << "\n";
    //     }
    // # endif
}
```

获取当前位置就是基于这样一个速度计算，如下代码可知，优先计算世界坐标系下的y轴位置，因为之后我们战队估计这个里程计发展时间不会太久，所以现在暂时先用个正常状态，之后再选择优化一下，所以我们暂时只考虑世界坐标系下的机器人坐标

首先是y轴位置，对应的就是世界坐标系下的Y速度乘以采样时间再乘以1000，就是以m为单位了，处理完成的是当前瞬时的位移，所以向量相加可以直接加，因为误差并不会很大，最后再加上一个误差处理

目前就需要解释一下这个globalspeed是怎么来的了，因为我们要用的也是这个世界坐标系的速度

```cpp
/**
 * @brief 更新世界坐标系机器y轴位置（积分器）
 * 
 */
void Position::updateGlobalY()
{
    double d = globalYSpeed * sampleTime / 1000;                            //前向差分法
    // double d = (globalYSpeed + lastglobalYSpeed) * sampleTime / 1000 / 2;   //双线性变换法
    if (abs(d) < 0.001)
        return;
    else
        globalY = globalY + d;
}

/**
 * @brief 更新世界坐标系机器x轴位置（积分器）
 * 
 * @param speed 
 */
void Position::updateGlobalX()
{
    double d = globalXSpeed * sampleTime / 1000;                            //前向差分法
    // double d = (globalXSpeed + lastglobalXSpeed) * sampleTime / 1000 / 2;   //双线性变换法       
    if (abs(d) < 0.001)
        return;
    else
        globalX = globalX + d;
}

/**
 * @brief 更新世界坐标系下机器位置（每tick调用）
 * 
 */
void Position::updatePos(){
    sampleTime = (Timer.getTimeDouble() - lastTime) * 1000;
    lastTime = Timer.getTimeDouble();

    updateInertialHeading();
    updateAngleSpeed();
    updateLMileage();
    updateRMileage();
    updateLSpeed();
    updateRSpeed();
    updateSelfYSpeed();
    updateSelfXSpeed();
    updateGlobalYSpeed();
    updateGlobalXSpeed();
    updateGlobalY();
    updateGlobalX();
}

```

下面我们把这段代码单独拿出来看

```jsx
/**
 * @brief 世界坐标系y轴速度 cm/s
 * 
 * @return float 
 */
void Position::updateGlobalYSpeed()
{
    lastglobalYSpeed = globalYSpeed;
    globalYSpeed = selfYSpeed * cos(curIMUHeading) - selfXSpeed * sin(curIMUHeading);
    if (abs(globalYSpeed) < 0.01) globalYSpeed = 0;
    if (abs(globalYSpeed) > 250) globalYSpeed = lastglobalYSpeed;
}
```

这段代码首先的输入参数有一个selfYspeed，这个参数就是以自己为主坐标系下的坐标，也就是通常所说的机器人系下的坐标

‘下面来解释一下这个速度是怎么来的，debug部分就不看了，战队目前状况还是不太需要用debug

```jsx
void Position::updateSelfYSpeed()
{
    double D = -cos(REncoderAngle) * sin(LEncoderAngle) - cos(LEncoderAngle) * sin(REncoderAngle);
    double Dy = curLSpeed * -cos(REncoderAngle) - curRSpeed * cos(LEncoderAngle);
    selfYSpeed = Dy / D;

    #ifdef debug
    double caliberLSpeed = 0;
    double caliberRSpeed = 0;

    if (TestCaliberFlag)
    {
        if (angleSpeed < 0)
        {
            caliberLSpeed = curRSpeed * (leftCCCoefficient/rightCCCoefficient) * sign(curLSpeed);
            caliberRSpeed = curRSpeed;
        }
        else if (angleSpeed > 0)
        {
            caliberRSpeed = curLSpeed * (rightCLCoefficient/leftCLCoefficient) * sign(curRSpeed);
            caliberLSpeed = curLSpeed;
        }
        else
        {
            caliberLSpeed = curLSpeed;
            caliberRSpeed = curRSpeed;
        }

        cout<<setiosflags(ios::fixed)<<setprecision(2)<<curRSpeed<<" "<<curLSpeed<<" "<<caliberRSpeed<<" "<<caliberLSpeed<<endl;
    }
    #endif
}
```

![image.png](attachment:bc4651a9-adf8-420f-a9ff-ee8ddb2f824b:image.png)

![image.png](attachment:91b2204f-6a44-4734-a5ae-0a82878df73f:image.png)

根据上交的机型得出里程计排布图

![image.png](attachment:03ec6aef-8290-48aa-9f70-55e2b0248d0e:image.png)

同时由提供的代码部分

```cpp
const double LEncoderAngle = acos(157.6291667 / 220);
const double REncoderAngle = acos(155.4553333 / 220);
```

由计算可知157.6291667 / 220=0.7165~root2/2

所以这两个角度极有可能是45度

然后根据左右编码的位置可知因为底数是一样的都是220证明这是他们的track wheel

于是我们继续看如何计算selfYspeed

```jsx
double D = -cos(REncoderAngle) * sin(LEncoderAngle) - cos(LEncoderAngle) * sin(REncoderAngle);
double Dy = curLSpeed * -cos(REncoderAngle) - curRSpeed * cos(LEncoderAngle);
selfYSpeed = Dy / D;//这个是机器人坐标系vy的计算
double D = -cos(REncoderAngle) * sin(LEncoderAngle) - cos(LEncoderAngle) * sin(REncoderAngle);
double Dx = curRSpeed * sin(LEncoderAngle) - curLSpeed * sin(REncoderAngle);
selfXSpeed = Dx / D;//这个是机器人坐标系vx的计算
```

![img_v3_02j4_32d00de5-a959-4a08-8932-9e92a4f64b1g.jpg](attachment:ec77ec9f-484c-4d87-864d-64f00618c38b:img_v3_02j4_32d00de5-a959-4a08-8932-9e92a4f64b1g.jpg)

经过计算可知，最后的计算结果如上图方程式计算

可知手动计算的角度有问题，实际上的citaR，和citaL分别是对应轮子的另一边，客观上展示了这两边速度的计算原理

计算完成后的debug就不解释了

然后计算世界坐标系vy

```jsx
void Position::updateGlobalYSpeed()
{
    lastglobalYSpeed = globalYSpeed;
    globalYSpeed = selfYSpeed * cos(curIMUHeading) - selfXSpeed * sin(curIMUHeading);
    if (abs(globalYSpeed) < 0.01) globalYSpeed = 0;
    if (abs(globalYSpeed) > 250) globalYSpeed = lastglobalYSpeed;
}
```

数学计算过程如下

![image.png](attachment:c9114428-70b6-4223-b284-631616e90d95:image.png)

得出全局速度，然后进行轻微滤波，避免微小的变化影响全局定位

最后更新当前全局坐标，也就是速度乘以取样时间，最后除以1000把单位变成m，然后用积分法累加位移，得出最新的全局Y

```jsx
/**
 * @brief 更新世界坐标系机器y轴位置（积分器）
 * 
 */
void Position::updateGlobalY()
{
    double d = globalYSpeed * sampleTime / 1000;                            //前向差分法
    // double d = (globalYSpeed + lastglobalYSpeed) * sampleTime / 1000 / 2;   //双线性变换法
    if (abs(d) < 0.001)
        return;
    else
        globalY = globalY + d;
}
```

同理X坐标的定位也是如此

最后将这些更新的函数存储再一个函数里面，其中取样时间的设定是根据两次取样的差值时间确定的，这样在一定程度上避免了计时器不稳定的问题

```cpp
/**
 * @brief 更新世界坐标系下机器位置（每tick调用）
 * 
 */
void Position::updatePos(){
    sampleTime = (Timer.getTimeDouble() - lastTime) * 1000;
    lastTime = Timer.getTimeDouble();

    updateInertialHeading();
    updateAngleSpeed();
    updateLMileage();
    updateRMileage();
    updateLSpeed();
    updateRSpeed();
    updateSelfYSpeed();
    updateSelfXSpeed();
    updateGlobalYSpeed();
    updateGlobalXSpeed();
    updateGlobalY();
    updateGlobalX();
}
```

### 实现串级pid控制机器人走直线或者转动固定距离

首先解释以一定速度走一个固定的直线式如何实现的

```cpp
void quickMoveToWithHeading(double _x, double _y, double _tarAng, double _maxSpeed){
    Point _tarPos = Point(_x, _y);
    Point pos = Position::getInstance()->getPos();
    double dis = (_tarPos - pos).mod();
    double time = 1000 * dis / 100;
    if (time < 1500) time = 1500;
    while (_tarAng < 0)
        _tarAng += 360;
    Trajectory *traj = TrajectoryFactory::getQuickTraj(_tarPos, _tarAng, _maxSpeed);
    MyTimer mytimer;
    while (!traj->targetArrived() && mytimer.getTime() <= time+500)
    {
        traj->update();
        // cout << Position::getInstance()->getPos()._x << " " << Position::getInstance()->getPos()._y << " " << IMUHeading() << endl;
        // cout << "Vel: " << traj->getVelocity()._x << " " << traj->getVelocity()._y << " " << traj->getVelocityR() << endl;
        Chassis::getInstance()->autoSetWorldVel(traj->getVelocity(), traj->getVelocityR());
        this_thread::sleep_for(50);
    }
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    this_thread::sleep_for(30);
}
```

首先输入你要去往目的地的全局位置坐标，输入你要旋转的角度（就是如果你已经经过一系列的操作了，通过陀螺仪很难计算出当前的相对朝向，而且也没有必要，所以需要编程手直接算出角度，随后再输入一个最大速度确定速度范围）

然后通过Poisition中的函数获取getpos()函数获取当前的位置输入为pos

随后进行dis的计算，这个dis实际上就是两点之间位置的计算，只是在他们代码里面封装了加减乘除的点计算，计算完成后的相对位移点再取模长就是两点之间的距离

```jsx
 double time = 1000 * dis / 100;
    if (time < 1500) time = 1500;
    while (_tarAng < 0)
        _tarAng += 360;
    Trajectory *traj = TrajectoryFactory::getQuickTraj(_tarPos, _tarAng, _maxSpeed);
    MyTimer mytimer;
```

这段暂且不管，暂时不知道什么意思

然后就是跟踪函数，跟踪函数就是初始化一些pid，其中因为上交的底盘是全向轮底盘，所以有行驶的线速度和旋转速度的pid设置，这里我们不管到时候看，其中貌似可以看到dis≤=20和dis>20的pid计算还有区别，调试的时候注意一下

```cpp
Trajectory* TrajectoryFactory::getQuickTraj(Point _tarPos, double _tarAng, double _maxSpeed){
    Trajectory* traj;
    Point pos = Position::getInstance()->getPos();
    double dis = (_tarPos - pos).mod();
    if (dis <= 20){
        traj = new PIDTrajectory(_tarPos, _tarAng, _maxSpeed);
        traj->setPIDR(0.88, 1, 0, 7, 10);
        traj->setPIDV(2.2, 1, 2.5, 20, 10);
        traj->setJumpTime(0);
        traj->setAngTol(3);
        traj->setTranTol(5);
    }
    else{
        traj = new NormalTrajectory(_tarPos, _tarAng, _maxSpeed);
        traj->setPIDR(0.88, 1, 0, 7, 10);
        traj->setPIDV(2.2, 1, 2.5, 20, 10);
        traj->setJumpTime(0);
        traj->setAngTol(3);
        traj->setTranTol(5);
    }
    return traj;
}
```

其次就是进入while循环了

首先有一个traj->update();

这个函数有一个函数updateStage，这个函数用于获取当前状态，状态包括首先的pos也就是当前全局坐标系下的位置坐标，这几个更新的参数暂时没有找到对应的处理机制，暂时保留

```cpp
/**
 * @brief 更新状态
 * 
 */
void NormalTrajectory::updateStage()
{
    // cout << "N updateStage" << endl;
    Point pos = Position::getInstance()->getPos();
    double leftLength = (targetPos - pos).mod();
    double accLength = (targetPos - initPos).mod() * accPctv;
    double passedLength = (pos - initPos).mod();
    double cstLength = (targetPos - initPos).mod() * decPctv;
    switch (stageV){
        case BEGIN:
            stageV = ACCEL;
            // cout << "stageV ACCEL" << endl;
            break;
        case ACCEL:{
            if (passedLength >= accLength || velocity.mod() >= maxSpeed){
                stageV = CONSTSPEED;
                // cout << "stageV CONSTSPEED" << endl;
            }
            break;
        }
        case CONSTSPEED:{
            if (leftLength <= cstLength){
                stageV = DECEL;
                // cout << "stageV DECEL" << endl;
            }
            break;
        }
        case DECEL:
            if (pidv.targetArrived()){
                stageV = STOP;
                // cout << "stageV STOP" << endl;
            }
            break;
        case STOP:
            break;
        default:
            break;
    }
    
    double leftAng = (targetAng - IMUHeading());
    if (leftAng > 180) leftAng -= 360;
    if (leftAng < -180) leftAng += 360;
    double da = (targetAng - initAng);
    if (da > 180) da -= 360;
    if (da < -180) da += 360;
    double passedAng = (IMUHeading() - initAng);
    if (passedAng > 180) passedAng -= 360;
    if (passedAng < -180) passedAng += 360;
    double accAng = da * accPctr;
    double cstAng = da * decPctr;

    switch (stageR){
        case BEGIN:
            stageR = DECEL;
            break;
        case DECEL:
            break;
        case STOP:
            break;
        default:
            break;
    }
}

```

再有一个就是更新速度状态，整车的状态分为BEGIN  ACCEL  CONSTSPEED  DECEL  STOP

```jsx
double leftAng = (targetAng - IMUHeading());
    if (leftAng > 180) leftAng -= 360;
    if (leftAng < -180) leftAng += 360;
    double da = (targetAng - initAng);
    if (da > 180) da -= 360;
    if (da < -180) da += 360;
    double passedAng = (IMUHeading() - initAng);
    if (passedAng > 180) passedAng -= 360;
    if (passedAng < -180) passedAng += 360;
    double accAng = da * accPctr;
    double cstAng = da * decPctr;
```

然后上一段部分有一个角度的调整，但是输出的leftAng和passedAng的值没有用到

使用switch语句，如果状态是BEGIN就将他变成ACCEL，如果是ACCEL就执行软启动函数，等等这几个状态可以看懂我就不说了

```cpp
/**
 * @brief 状态转移函数的速度部分，更新速度值
 *
 * @details 自动机设定了整个运动过程分为 开始 - 加速 - 匀速 - 减速 - 结束 阶段
 *
 *
 */
void NormalTrajectory::updateVelocity(){
    // cout << "N updateV" << endl;
    double x1 = initPos._x, y1 = initPos._y;
    double x2 = targetPos._x, y2 = targetPos._y;
    Point curPos = Position::getInstance()->getPos();
    double x3 = curPos._x, y3 = curPos._y;
    double A = y1 - y2;
    double B = x2 - x1;
    double C = x1 * y2 - x2 * y1;
    double M = (A != 0 || B != 0) ? abs(A * x3 + B * y3 + C) / sqrt(A * A + B * B) : 0;
    Vector fixVel = Vector(((x3*(x1 - x2) + y3*(y1 - y2))*(x1 - x2))/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2) - ((y2*(x1 - x2) - x2*(y1 - y2))*(y1 - y2))/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2) - x3,
                           ((y2*(x1 - x2) - x2*(y1 - y2))*(x1 - x2))/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2) + ((x3*(x1 - x2) + y3*(y1 - y2))*(y1 - y2))/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2) - y3);
    pidt.update(-M);
    if (fixVel.mod() != 0){
        fixVel = fixVel / fixVel.mod();
        double output = pidt.getOutput();
        if (output >= 20) output = 20;
        fixVel = fixVel * output;
    }
    switch (stageV)
    {
        case BEGIN: // calculate durations and path lengths
            break;
        case ACCEL:
            softStartV();
            break;
        case CONSTSPEED:
            constSpeedV();
            break;
        case DECEL: // m_pid.setTarget(totalPath);
            pidControlV(pidv);
            break;
        case STOP:
            velocity = Vector(0, 0);
            break;
        default:
            break;
    }
    switch (stageR)
    {
        case BEGIN: // calculate durations and path lengths
            break;
        case DECEL: // m_pid.setTarget(totalPath);
            pidControlR(pidr);
            break;
        case STOP:
            velocityR = 0;
            break;
        default:
            break;
    }
    // cout << "fixVel: " << fixVel._x << " " << fixVel._y << endl;
    if (stageV == ACCEL || stageV == CONSTSPEED)
    {
        if (M >= 5) velocity = velocity + fixVel;
        if (velocity.mod() >= maxSpeed){
        velocity = velocity / velocity.mod() * maxSpeed;
        }
    }

    if (abs(velocityR) >= 60)
        velocityR = sign(velocityR) * 60;
    if (targetArrived()){
        velocity = Vector(0, 0);
        velocityR = 0;
    }
    updateArriveState();
    
}
```

然后就是对底盘进行控制

```jsx
 Chassis::getInstance()->autoSetWorldVel(traj->getVelocity(), traj->getVelocityR());
```

这个对底盘的控制是修改chassis这个类的相关参数来实现控制底盘，而底盘的输出速度是一直都有的。下面是这个控制函数，他要输入一个线速度和一个旋转速度，这些速度经过一系列的计算加入机器人速度中，然后通过速度标志输入轮子上

```cpp
/// @brief 【自动控制】通过世界坐标系速度驱动底盘
/// @param Vel 世界坐标系下速度向量，速度向量大小（模）在 0-100 之间
/// @param VelR 机器人角速度，范围在-100 - 100
void Chassis::autoSetWorldVel(Vector Vel, double VelR)
{
    if (Vel.mod() > 100){
        Vel = Vel / Vel.mod() * 100;
    }
    if (abs(VelR) > 100)
        VelR = 100 * sign(VelR);
    autoVel = calcRobotVel(Vel);
    autoVelR = VelR;
    robotVel = manualVel + autoVel;
    robotVelR = manualVelR + autoVelR;
    if (robotVel.mod() > 100){
        robotVel = robotVel / robotVel.mod() * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}
```

然后再到达后就停下，这个是整个对驱使前进的操作

### 将坐标变换融入到战队的代码中

我现在就在上交赛的代码上修改，并且逐步记录

首先说明我的一些观点，如果单论或者转角的话，我们的代码已经非常足够了，唯一的缺点就是我们的场地定位做得不好，实际上就是不能够再以电机内置编码器的转动速度来作为编码器的测量值，我们想要通过底盘下面的里程计取测量计算行驶距离，并且我们需要使用场地定位，就是我们输入的值不再是位置距离，而是目标点的坐标。明白上面的观点很重要，至少你得确定我们为什么要使用里程计，并不是因为上交用了我们就用，而是本质上我们其实有比上交做得好的地方

首先我们新建两个文件，pos.h和pos.cpp来储存和计算当前坐标

```cpp
#pragma once
#include "vex.h"
using namespace vex;
typedef struct Point{
    float x = 0;
    float y = 0;
};
class MyTimer{//计时器
private:
    double startTime;
public:
    MyTimer();
    MyTimer(double);
    void reset();
    int getTime() const;
    double getTimeDouble() const;
};

void rotation_init(void);
void updateLMileage(void);
void updateRMileage(void);
double deg2rad(double deg);
void updateLSpeed(void);
void updateRSpeed(void);
void updateLAngle(void);
void updateRAngle(void);
void updateLAngleSpeed(void);
void updateRAngleSpeed(void);
void updateSelfXSpeed(void);
void updateSelfYSpeed(void);
void updateGlobalX(void);
void updateGlobalY(void);
void updateGlobalXSpeed(void);
void updateGlobalYSpeed(void);
void updateIMUHeading(void);
void updateInertialHeading(void);
void update_all(void);
float return_globalX(void);
float return_globalY(void);
void update_global_point(void);

/************geometry_ku****************/
void drive_distance_geometry(float aimpos_x,float aimpos_y);
void drive_distance_geometry(float aimpos_x,float aimpos_y, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

/***************************************/
```

理论上你只要将这两段代码的内容融入你的工程文件中就可以了

```cpp
#include "vex.h"
#include <math.h>
#define the_radius_of_encoderwheel 4.0*0.393700787//cm
using namespace vex;
float IMUCoefficient = 3600 / 3594;//参数可改，而且必须要改
float lastLMileage = 0;
float curLMileage = 0;
float lastRMileage = 0;
float curRMileage = 0;
float lastLSpeed = 0;
float curLSpeed = 0;
float lastRSpeed = 0;
float curRSpeed = 0;
float selfXSpeed = 0;
float selfYSpeed = 0;
float globalXSpeed = 0;
float globalYSpeed = 0;
float lastglobalXSpeed = 0;
float lastglobalYSpeed = 0;
float sampleTime = 0;
float lastTime = 0;//用于更新采样时间
float LSpeed[3] = {0, 0, 0};
float RSpeed[3] = {0, 0, 0};
float FiltLSpeed[3] = {0, 0, 0};
float FiltRSpeed[3] = {0, 0, 0};
float LEncoderAngle = 0;
float REncoderAngle = 0;
float fliter_b[3] = {0.0675, 0.1349, 0.0675};
float fliter_a[3] = {1, -1.1429, 0.4128};
float curIMUHeading = 0;
float lastIMUHeading = 0;
float globalX = 0;
float globalY = 0;
Point GlobalPoint;
MyTimer time1;

const float positionRefreshTime = 10; 

float drive_max_voltage_geometry=10;
float drive_kp_geometry=1.5;//1.5//2.5
float drive_ki_geometry=0.12;
float drive_kd_geometry=8;
float drive_starti_geometry=0;
float drive_settle_error_geometry=1.5;
float drive_settle_time_geometry=100;
float drive_timeout_geometry=2000;
float heading_max_voltage_geometry=6;
float heading_kp_geometry=0.62;//0.42
float heading_ki_geometry=0.12;//0
float heading_kd_geometry=0.7;
float heading_starti_geometry=0;
double desire_heading_geometry = 0;
double heading_geometry = 0;
// FiltLSpeed[0] = FiltLSpeed[1] = FiltLSpeed[2] = 0;
//     FiltRSpeed[0] = FiltRSpeed[1] = FiltRSpeed[2] = 0;

//**************计时器相关函数**************************//
MyTimer::MyTimer(){
    startTime = Brain.Timer.value();
}

void MyTimer::reset(){
    startTime = Brain.Timer.value();
}

int MyTimer::getTime() const{
    return floor((Brain.Timer.value() - startTime) * 1000); // return time (msec) from startTime
}

double MyTimer::getTimeDouble() const{
    return Brain.Timer.value() - startTime; // return time (sec) from startTime
}
//*********************************//

// void updateIMUHeading(){
//     lastIMUHeading = curIMUHeading;
//     curIMUHeading = Inertial.heading();
// }

// float reduce_0_to_360(float angle) {
//   while(!(angle >= 0 && angle < 360)) {
//     if( angle < 0 ) { angle += 360; }
//     if(angle >= 360) { angle -= 360; }
//   }
//   return(angle);
// }
// float gets_absolute_heading(){ 
//   return( reduce_0_to_360( Inertial.rotation()) ); 
// }

double IMUHeading(){
    double heading_geometry = Inertial.rotation(rotationUnits::deg);
    heading_geometry = heading_geometry * IMUCoefficient;
    while (heading_geometry < 0) heading_geometry += 360;
    while (heading_geometry > 360) heading_geometry -= 360;
    return heading_geometry;
}

float deg2rad_geometry(float deg){
    return deg * M_PI / 180.0;
}

void updateInertialHeading(){
    lastIMUHeading = curIMUHeading;
    curIMUHeading = deg2rad_geometry(IMUHeading());
}

void rotation_init(){
    leftEncoder.resetPosition();
    rightEncoder.resetPosition();//里程计初始化
}
//updateInertialHeading();
//updateLMileage();
//updateRMileage();
//updateLSpeed();
//updateRSpeed();
//updateSelfXSpeed();
//updateSelfYSpeed();
//updateGlobalXSpeed();
//updateGlobalYSpeed();

void updateLMileage()
{
    lastLMileage = curLMileage;

    curLMileage = deg2rad_geometry(-leftEncoder.position(degrees)) * the_radius_of_encoderwheel;
}
void updateRMileage()
{
    lastRMileage = curRMileage;

    curRMileage = deg2rad_geometry(-rightEncoder.position(degrees)) * the_radius_of_encoderwheel;
}
/**
 * @brief 左定位轮速度 cm/s
 * 
 * @return float 
 */
void updateLSpeed(){
    lastLSpeed = curLSpeed;

    double ret = (curLMileage - lastLMileage) * 1000 / sampleTime;                      //前向差分法
    if (std::abs(ret) > 1000 || std::abs(ret) < 0.001) ret = 0;
    
    // 滤波前的速度
    LSpeed[2] = LSpeed[1];
    LSpeed[1] = LSpeed[0];
    LSpeed[0] = ret;

    // 滤波后的速度
    FiltLSpeed[2] = FiltLSpeed[1];
    FiltLSpeed[1] = FiltLSpeed[0];
    FiltLSpeed[0] = (fliter_b[0] * LSpeed[0] + fliter_b[1] * LSpeed[1] + fliter_b[2] * LSpeed[2] - fliter_a[1] * FiltLSpeed[1] - fliter_a[2] * FiltLSpeed[2]) / fliter_a[0]; 
    if (std::abs(FiltLSpeed[0]) < 0.001) FiltLSpeed[0] = 0;

    curLSpeed = FiltLSpeed[0]; 
    // curLSpeed = ret;
}

/**
 * @brief 右定位轮速度 cm/s
 * 
 * @return float 
 */
void updateRSpeed(){
    lastRSpeed = curRSpeed;

    double ret = (curRMileage - lastRMileage) * 1000 / sampleTime;                      //前向差分法
    if (std::abs(ret) > 1000 || std::abs(ret) < 0.001) ret = 0;                                   //速度过大或过小，认为是噪声
    // 伪kalman滤波
    // 滤波前的速度
    RSpeed[2] = RSpeed[1];
    RSpeed[1] = RSpeed[0];
    RSpeed[0] = ret;

    // 滤波后的速度
    FiltRSpeed[2] = FiltRSpeed[1];
    FiltRSpeed[1] = FiltRSpeed[0];
    FiltRSpeed[0] = (fliter_b[0] * RSpeed[0] + fliter_b[1] * RSpeed[1] + fliter_b[2] * RSpeed[2] - fliter_a[1] * FiltRSpeed[1] - fliter_a[2] * FiltRSpeed[2]) / fliter_a[0];
    if (std::abs(FiltRSpeed[0]) < 0.001) FiltRSpeed[0] = 0;

    curRSpeed = FiltRSpeed[0];
    // curRSpeed = ret;
}

/**
 * @brief 自身坐标系y轴速度 cm/s
 * 
 * @return double 
 */
void updateSelfYSpeed(){
    selfYSpeed = curLSpeed;
    //如果里程计不是按照正交形式摆放，那么就用下面那段代码
    // double D = -cos(REncoderAngle) * sin(LEncoderAngle) - cos(LEncoderAngle) * sin(REncoderAngle);
    // double Dy = curLSpeed * -cos(REncoderAngle) - curRSpeed * cos(LEncoderAngle);
    // selfYSpeed = Dy / D;

}

/**
 * @brief 自身坐标系x轴速度 cm/s
 * 
 * @return double 
 */
void updateSelfXSpeed(){
    selfXSpeed = curRSpeed;
    // double D = -cos(REncoderAngle) * sin(LEncoderAngle) - cos(LEncoderAngle) * sin(REncoderAngle);
    // double Dx = curRSpeed * sin(LEncoderAngle) - curLSpeed * sin(REncoderAngle);
    // selfXSpeed = Dx / D;
}

/**
 * @brief 世界坐标系y轴速度 cm/s
 * 
 * @return float 
 */
void updateGlobalYSpeed(){
    curIMUHeading = get_absolute_heading();
    lastglobalYSpeed = globalYSpeed;
    globalYSpeed = selfYSpeed * cos(curIMUHeading) - selfXSpeed * sin(curIMUHeading);
    if (std::abs(globalYSpeed) < 0.01) globalYSpeed = 0;
    if (std::abs(globalYSpeed) > 250) globalYSpeed = lastglobalYSpeed;
}

/**
 * @brief 世界坐标系x轴速度 cm/s
 * 
 * @return float 
 */
void updateGlobalXSpeed(){
    curIMUHeading = get_absolute_heading();
    lastglobalXSpeed = globalXSpeed;
    globalXSpeed = selfYSpeed * sin(curIMUHeading) + selfXSpeed * cos(curIMUHeading);
    if (std::abs(globalXSpeed) < 0.01) globalXSpeed = 0;
    if (std::abs(globalXSpeed) > 250) globalXSpeed = lastglobalXSpeed;

    // # ifdef debug
    //     if (CollectFlag)
    //     {
    //         cout <<setiosflags(ios::fixed)<<setprecision(2)<< globalXSpeed << " " << globalYSpeed << "\n";
    //     }
    // # endif
}
void updateGlobalX(){
    double d = globalYSpeed * sampleTime / 1000;                            //前向差分法
    // double d = (globalYSpeed + lastglobalYSpeed) * sampleTime / 1000 / 2;   //双线性变换法
    if (std::abs(d) < 0.001)
        return;
    else
        globalY = globalY + d;
}

void updateGlobalY(){
    double d = globalXSpeed * sampleTime / 1000;                            //前向差分法
    // double d = (globalXSpeed + lastglobalXSpeed) * sampleTime / 1000 / 2;   //双线性变换法
    if (std::abs(d) < 0.001)
        return;
    else
        globalX = globalX + d;
}

void update_all(){
    sampleTime = (time1.getTimeDouble() - lastTime) * 1000;
    lastTime = time1.getTimeDouble();//更新采样时间
    updateInertialHeading();
    updateLMileage();
    updateRMileage();
    updateLSpeed();
    updateRSpeed();
    updateSelfXSpeed();
    updateSelfYSpeed();
    updateGlobalXSpeed();
    updateGlobalYSpeed();
    updateGlobalX();
    updateGlobalY();
}

float return_globalX(){
    update_all();
    return globalX;
}
float return_globalY(){
    update_all();
    return globalY;
}
void update_global_point(){
    while(true){
    update_all();
    GlobalPoint.x = globalX;
    GlobalPoint.y = globalY;
    this_thread::sleep_for(positionRefreshTime);}
}

/************geometry_ku****************/
void drive_distance_geometry(float aimpos_x,float aimpos_y){
    drive_distance_geometry(aimpos_x, aimpos_y, get_absolute_heading(), drive_max_voltage_geometry, heading_max_voltage_geometry, drive_settle_error_geometry, drive_settle_time_geometry, drive_timeout_geometry, drive_kp_geometry, drive_ki_geometry, drive_kd_geometry, drive_starti_geometry, heading_kp_geometry, heading_ki_geometry, heading_kd_geometry, heading_starti_geometry);
}

void drive_distance_geometry(float aimpos_x,float aimpos_y, float heading_geometry, float drive_max_voltage_geometry, float heading_max_voltage_geometry, float drive_settle_error_geometry, float drive_settle_time_geometry, float drive_timeout_geometry, float drive_kp_geometry, float drive_ki_geometry, float drive_kd_geometry, float drive_starti_geometry, float heading_kp_geometry, float heading_ki_geometry, float heading_kd_geometry, float heading_starti_geometry){
    float curpos_X = GlobalPoint.x;
    float curpos_Y = GlobalPoint.y;
    float distance = sqrt((aimpos_x-curpos_X)*(aimpos_x-curpos_X)+(aimpos_y-curpos_Y)*(aimpos_y-curpos_Y));
    heading_geometry=get_absolute_heading();
    desire_heading_geometry = heading_geometry;
    PID drivePID(distance, drive_kp_geometry, drive_ki_geometry, drive_kd_geometry, drive_starti_geometry, drive_settle_error_geometry, drive_settle_time_geometry, drive_timeout_geometry);
    PID headingPID(reduce_negative_180_to_180(heading_geometry - gets_absolute_heading()), heading_kp_geometry, heading_ki_geometry, heading_kd_geometry, heading_starti_geometry);
    float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float average_position = start_average_position;
    while(drivePID.is_settled() == false){
        average_position = (get_left_position_in()+get_right_position_in())/2.0;
        float drive_error_geometry = distance+start_average_position-average_position;
        float heading_error_geometry = reduce_negative_180_to_180(heading_geometry - get_absolute_heading());
        float drive_output = drivePID.compute(drive_error_geometry);
        float heading_output = headingPID.compute(heading_error_geometry);
        drive_output = clamp(drive_output, -drive_max_voltage_geometry, drive_max_voltage_geometry);
        heading_output = clamp(heading_output, -heading_max_voltage_geometry, heading_max_voltage_geometry);
        drive_with_voltage(drive_output + heading_output, drive_output - heading_output);
        task::sleep(10);
        }
        L.stop(coast);R.stop(coast);
}

/***************************************/
```

理论上你只需要将这两个文件添加如你的工程文件中就完成了移植，在这里我只以pos.h文件为例子介绍相关函数的作用及功能

```cpp
#pragma once
#include "vex.h"
using namespace vex;
typedef struct Point{
    float x = 0;
    float y = 0;
};
class MyTimer{//计时器
private:
    double startTime;
public:
    MyTimer();
    MyTimer(double);
    void reset();
    int getTime() const;
    double getTimeDouble() const;
};

void rotation_init(void);
void updateLMileage(void);
void updateRMileage(void);
double deg2rad(double deg);
void updateLSpeed(void);
void updateRSpeed(void);
void updateSelfXSpeed(void);
void updateSelfYSpeed(void);
void updateGlobalX(void);
void updateGlobalY(void);
void updateGlobalXSpeed(void);
void updateGlobalYSpeed(void);
void updateIMUHeading(void);
void updateInertialHeading(void);
void update_all(void);
float return_globalX(void);
float return_globalY(void);
void update_global_point(void);

/************geometry_ku****************/
void drive_distance_geometry(float aimpos_x,float aimpos_y);
void drive_distance_geometry(float aimpos_x,float aimpos_y, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

/***************************************/
```

首先定义一个point结构体，储存坐标x和坐标y，因为包括全局坐标和机器人系统坐标都包括用到了点位，所以需要在最开始就添加一个点结构体

其次，就是计时器的类，这个计时器重要的功能是获取当前时间并将当前时间转换成double类型别的功能都是多余的，也就是函数中的getTimedouble函数

其次便是每个计算的函数，rotation_init();这个函数是用来初始化里程计的，因为在开电摆位置时可能会造成里程计的滚动，所以要在运行程序之前初始化里程计的位置

然后是更新两个里程计的行驶距离，用于计算x，y的速度，更新惯导朝向最后计算全局坐标，然后就是计算目标点的距离函数，最后完成。

下面这个文件是最终融入里程计代码的

[rblue (2).zip](attachment:948c64fa-875b-4766-8d53-b808e33ef60c:rblue_(2).zip)
