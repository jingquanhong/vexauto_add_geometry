#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "device.h"
#include "opcontrol.h"
#include "PID.h"
#include "vision_function.h"
#include "driver.h"
#include "util.h"
#include "auto.h"
#include "odometry.h"
#include "kalman_filter.h"
#include "control.h"
#include "anglePID.h"
#include "pos.h"
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)