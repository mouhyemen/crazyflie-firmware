/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#define DEBUG_MODULE "STAB"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#include "estimator.h"
#include "usddeck.h"
#include "quatcompress.h"
#include "statsCnt.h"
#include "static_mem.h"

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

#define PROPTEST_NBR_OF_VARIANCE_VALUES   100
static bool startPropTest = false;

uint32_t inToOutLatency;


/**
 * Supporting and utility functions
 */

static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
  { configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }



/* Variables for computing transforms:
Tor - Transform of robot pose in odometry frame (use stateFlow)
Ror - Attitude matrix of robot pose in odometry frame (use stateFlow.q)
tor - Position of robot pose in odometry frame (use stateFlow.x/y/z)

Twr - Transform of robot pose in global frame (use stateSweep)
Rwr - Attitude matrix of robot pose in global frame (use stateSweep.q)
twr - Position of robot pose in global frame (use stateSweep.x/y/z)
*/
static float tmp[4*4];
static arm_matrix_instance_f32 Tor = {4, 4, tmp};
static arm_matrix_instance_f32 Tor_inv = {4, 4, tmp};
static arm_matrix_instance_f32 I = {4, 4, tmp};

// static arm_matrix_instance_f32 Twr;

/*
  Computes a 4x4 homogeneous transformation matrix T 
  from 3x3 attitude matrix R and 3x1 position vector t.
  T = [ R   t ; 0 0 0 1]
  T = [ R00 R01 R02 t0
        R10 R11 R12 t1
        R20 R21 R22 t2      
         0   0   0   1 ]
*/
static void computeTransform(arm_matrix_instance_f32* T, const state_t* state) {
  double qw;  // quaternion constant
  double qx;  // quaternion along x
  double qy;  // quaternion along y
  double qz;  // quaternion along z
  qw = state->attitudeQuaternion.w;
  qx = state->attitudeQuaternion.x;
  qy = state->attitudeQuaternion.y;
  qz = state->attitudeQuaternion.z;

  // Compute R using state's quaternion estimates
  double R[3][3];  // attitude matrix
  R[0][0] = qw * qw + qx * qx - qy * qy - qz * qz;
  R[0][1] = 2 * qx * qy - 2 * qw * qz;
  R[0][2] = 2 * qx * qz + 2 * qw * qy;

  R[1][0] = 2 * qx * qy + 2 * qw * qz;
  R[1][1] = qw * qw - qx * qx + qy * qy - qz * qz;
  R[1][2] = 2 * qy * qz - 2 * qw * qx;

  R[2][0] = 2 * qx * qz - 2 * qw * qy;
  R[2][1] = 2 * qy * qz + 2 * qw * qx;
  R[2][2] = qw * qw - qx * qx - qy * qy + qz * qz;

  // Compute t using state's position estimates
  double t[3];     // position vector
  t[0] = state->position.x;
  t[1] = state->position.y;
  t[2] = state->position.z;

  // Compute T transform using R and t
  const float32_t T_array[16] = {
    R[0][0] , R[0][1] , R[0][2] , t[0]  ,
    R[1][0] , R[1][1] , R[1][2] , t[1]  ,
    R[2][0] , R[2][1] , R[2][2] , t[2]  ,
      0     ,   0     ,   0     ,   1   ,
  };  
  
  arm_mat_init_f32(T, 4, 4, (float32_t *)T_array);
}


/*
  Computes a 4x4 homogeneous transformation matrix T inverse
  from 3x3 attitude matrix R and 3x1 position vector t.
  Tinv = [ R' -R'*t; 0 0 0 1]
  Tinv = [R00 R10 R20 
          R01 R11 R21  -R'*t
          R02 R12 R22       
           0   0   0    1 ]

*/
static void computeTransformInverse(arm_matrix_instance_f32* Tinv, const state_t* state) {
  double qw;  // quaternion constant
  double qx;  // quaternion along x
  double qy;  // quaternion along y
  double qz;  // quaternion along z
  qw = state->attitudeQuaternion.w;
  qx = state->attitudeQuaternion.x;
  qy = state->attitudeQuaternion.y;
  qz = state->attitudeQuaternion.z;

  // Compute R using state's quaternion estimates
  double R[3][3];  // attitude matrix
  R[0][0] = qw * qw + qx * qx - qy * qy - qz * qz;
  R[0][1] = 2 * qx * qy - 2 * qw * qz;
  R[0][2] = 2 * qx * qz + 2 * qw * qy;

  R[1][0] = 2 * qx * qy + 2 * qw * qz;
  R[1][1] = qw * qw - qx * qx + qy * qy - qz * qz;
  R[1][2] = 2 * qy * qz - 2 * qw * qx;

  R[2][0] = 2 * qx * qz - 2 * qw * qy;
  R[2][1] = 2 * qy * qz + 2 * qw * qx;
  R[2][2] = qw * qw - qx * qx - qy * qy + qz * qz;

  // Compute t using state's position estimates
  double t[3];     // position vector
  t[0] = state->position.x;
  t[1] = state->position.y;
  t[2] = state->position.z;

  // Compute R' * t
  double Rt[3];
  Rt[0] = R[0][0]*t[0] + R[1][0]*t[1] + R[2][0]*t[2] ;
  Rt[1] = R[0][1]*t[0] + R[1][1]*t[1] + R[2][1]*t[2] ;
  Rt[2] = R[0][2]*t[0] + R[1][2]*t[1] + R[2][2]*t[2] ;

  // Compute Tinv transform using R and t
  const float32_t Tinv_array[16] = {
    R[0][0] , R[1][0] , R[2][0] , -Rt[0],
    R[0][1] , R[1][1] , R[2][1] , -Rt[1],
    R[0][2] , R[1][2] , R[2][2] , -Rt[2],
      0     ,   0     ,   0     ,   1   ,
  };
  arm_mat_init_f32(Tinv, 4, 4, (float32_t *)Tinv_array);
}




// static void getStatefromTransform(state_t* state, const arm_matrix_instance_f32* T) {
//   // initialize R 3x3 matrix and position vector t
//   // double R[3][3];
//   double t[3];
  
//   // create a temporary transform matrix
//   static arm_matrix_instance_f32 T_tmp = {4, 4, tmp};
//   memcpy(&T_tmp, T, sizeof(T_tmp));

//   // populate R and t 
//   // R[0][0] = T_tmp->pData[0];
//   // R[0][1] = T_tmp->pData[1];
//   // R[0][2] = T_tmp->pData[2];
//   t[0]    = T_tmp.pData[3];
//   // t[0]    = T->pData[3];
  
//   // R[1][0] = T_tmp->pData[4];
//   // R[1][1] = T_tmp->pData[5];
//   // R[1][2] = T_tmp->pData[6];
//   t[1]    = T_tmp.pData[7];
//   // t[1]    = T->pData[7];
  
//   // R[2][0] = T_tmp->pData[8];
//   // R[2][1] = T_tmp->pData[9];
//   // R[2][2] = T_tmp->pData[10];
//   t[2]    = T_tmp.pData[11];
//   // t[2]    = T->pData[11];

//   state->position.x = t[0];
//   state->position.y = t[1];
//   state->position.z = t[2];

// }




// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
// static state_t state;


/*
  Variables for creating two separate state estimates (from lighthouse & flowdeck).
  Also, for fusing the two measurements and creating a corrected state estimate.
  A buffer is created for storing past state estimates - this is for emulating latencies.
*/
#define NUM_OF_STATES_IN_BUFFER 1 // number of states stored relates to latency in milliseconds
#define BUFFER_TICK_INTERVAL 1 // tick interval of states in buffer
#define DELAYED_STATE (NUM_OF_STATES_IN_BUFFER - 1) // latency = # states in buffer * buffer_tick_interval
static state_t stateBuffer[NUM_OF_STATES_IN_BUFFER]; // buffer of state estimates from t-N up to t

static state_t stateSweep;    // same as `state` except it does not include flow measurements
static state_t stateFlow;     // same as `state` except it does not include sweep measurements
static state_t stateLag;      // delayed state estimate, earliest state in the stateBuffer
// static state_t stateCorrect;  // corrected state estimate using transform compensation

static control_t control;

static StateEstimatorType estimatorType;
static ControllerType controllerType;

typedef enum { configureAcc, measureNoiseFloor, measureProp, testBattery, restartBatTest, evaluateResult, testDone } TestState;
#ifdef RUN_PROP_TEST_AT_STARTUP
  static TestState testState = configureAcc;
#else
  static TestState testState = testDone;
#endif

static STATS_CNT_RATE_DEFINE(stabilizerRate, 500);

// static struct {
//   // position - mm
//   int16_t x;
//   int16_t y;
//   int16_t z;
//   // velocity - mm / sec
//   int16_t vx;
//   int16_t vy;
//   int16_t vz;
//   // acceleration - mm / sec^2
//   int16_t ax;
//   int16_t ay;
//   int16_t az;
//   // compressed quaternion, see quatcompress.h
//   int32_t quat;
//   // angular velocity - milliradians / sec
//   int16_t rateRoll;
//   int16_t ratePitch;
//   int16_t rateYaw;
// } stateCompressed;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
} stateCompressedSweep;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
} stateCompressedFlow;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
} setpointCompressed;

static float accVarX[NBR_OF_MOTORS];
static float accVarY[NBR_OF_MOTORS];
static float accVarZ[NBR_OF_MOTORS];
// Bit field indicating if the motors passed the motor test.
// Bit 0 - 1 = M1 passed
// Bit 1 - 1 = M2 passed
// Bit 2 - 1 = M3 passed
// Bit 3 - 1 = M4 passed
static uint8_t motorPass = 0;
static uint16_t motorTestCount = 0;

STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);

static void stabilizerTask(void* param);
static void testProps(sensorData_t *sensors);

static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

// static void compressState()
// {
//   stateCompressed.x = state.position.x * 1000.0f;
//   stateCompressed.y = state.position.y * 1000.0f;
//   stateCompressed.z = state.position.z * 1000.0f;

//   stateCompressed.vx = state.velocity.x * 1000.0f;
//   stateCompressed.vy = state.velocity.y * 1000.0f;
//   stateCompressed.vz = state.velocity.z * 1000.0f;

//   stateCompressed.ax = state.acc.x * 9.81f * 1000.0f;
//   stateCompressed.ay = state.acc.y * 9.81f * 1000.0f;
//   stateCompressed.az = (state.acc.z + 1) * 9.81f * 1000.0f;

//   float const q[4] = {
//     state.attitudeQuaternion.x,
//     state.attitudeQuaternion.y,
//     state.attitudeQuaternion.z,
//     state.attitudeQuaternion.w};
//   stateCompressed.quat = quatcompress(q);

//   float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
//   stateCompressed.rateRoll = sensorData.gyro.x * deg2millirad;
//   stateCompressed.ratePitch = -sensorData.gyro.y * deg2millirad;
//   stateCompressed.rateYaw = sensorData.gyro.z * deg2millirad;
// }

static void compressStateSweep()
{
  stateCompressedSweep.x = stateSweep.position.x * 1000.0f;
  stateCompressedSweep.y = stateSweep.position.y * 1000.0f;
  stateCompressedSweep.z = stateSweep.position.z * 1000.0f;

  stateCompressedSweep.vx = stateSweep.velocity.x * 1000.0f;
  stateCompressedSweep.vy = stateSweep.velocity.y * 1000.0f;
  stateCompressedSweep.vz = stateSweep.velocity.z * 1000.0f;

  stateCompressedSweep.ax = stateSweep.acc.x * 9.81f * 1000.0f;
  stateCompressedSweep.ay = stateSweep.acc.y * 9.81f * 1000.0f;
  stateCompressedSweep.az = (stateSweep.acc.z + 1) * 9.81f * 1000.0f;

  float const q[4] = {
    stateSweep.attitudeQuaternion.x,
    stateSweep.attitudeQuaternion.y,
    stateSweep.attitudeQuaternion.z,
    stateSweep.attitudeQuaternion.w};
  stateCompressedSweep.quat = quatcompress(q);

  float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
  stateCompressedSweep.rateRoll = sensorData.gyro.x * deg2millirad;
  stateCompressedSweep.ratePitch = -sensorData.gyro.y * deg2millirad;
  stateCompressedSweep.rateYaw = sensorData.gyro.z * deg2millirad;
}

static void compressStateFlow()
{
  stateCompressedFlow.x = stateFlow.position.x * 1000.0f;
  stateCompressedFlow.y = stateFlow.position.y * 1000.0f;
  stateCompressedFlow.z = stateFlow.position.z * 1000.0f;

  stateCompressedFlow.vx = stateFlow.velocity.x * 1000.0f;
  stateCompressedFlow.vy = stateFlow.velocity.y * 1000.0f;
  stateCompressedFlow.vz = stateFlow.velocity.z * 1000.0f;

  stateCompressedFlow.ax = stateFlow.acc.x * 9.81f * 1000.0f;
  stateCompressedFlow.ay = stateFlow.acc.y * 9.81f * 1000.0f;
  stateCompressedFlow.az = (stateFlow.acc.z + 1) * 9.81f * 1000.0f;

  float const q[4] = {
    stateFlow.attitudeQuaternion.x,
    stateFlow.attitudeQuaternion.y,
    stateFlow.attitudeQuaternion.z,
    stateFlow.attitudeQuaternion.w};
  stateCompressedFlow.quat = quatcompress(q);

  float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
  stateCompressedFlow.rateRoll = sensorData.gyro.x * deg2millirad;
  stateCompressedFlow.ratePitch = -sensorData.gyro.y * deg2millirad;
  stateCompressedFlow.rateYaw = sensorData.gyro.z * deg2millirad;
}

static void compressSetpoint()
{
  setpointCompressed.x = setpoint.position.x * 1000.0f;
  setpointCompressed.y = setpoint.position.y * 1000.0f;
  setpointCompressed.z = setpoint.position.z * 1000.0f;

  setpointCompressed.vx = setpoint.velocity.x * 1000.0f;
  setpointCompressed.vy = setpoint.velocity.y * 1000.0f;
  setpointCompressed.vz = setpoint.velocity.z * 1000.0f;

  setpointCompressed.ax = setpoint.acceleration.x * 1000.0f;
  setpointCompressed.ay = setpoint.acceleration.y * 1000.0f;
  setpointCompressed.az = setpoint.acceleration.z * 1000.0f;
}

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAny);
  powerDistributionInit();
  sitAwInit();
  estimatorType = getStateEstimator();
  controllerType = getControllerType();

  STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= controllerTest();
  pass &= powerDistributionTest();

  return pass;
}

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  DEBUG_PRINT("Wait for sensor calibration...\n");

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  DEBUG_PRINT("Ready to fly.\n");

  unsigned int state_counter_index = 0;
  // bool use_laggy_state = false;

  while(1) {
    // The sensor should unlock at 1kHz
    sensorsWaitDataReady();

    if (startPropTest != false) {
      // TODO: What happens with estimator when we run tests after startup?
      testState = configureAcc;
      startPropTest = false;
    }

    if (testState != testDone) {
      sensorsAcquire(&sensorData, tick);
      testProps(&sensorData);
    } else {
      // allow to update estimator dynamically
      if (getStateEstimator() != estimatorType) {
        stateEstimatorSwitchTo(estimatorType);
        estimatorType = getStateEstimator();
      }
      // allow to update controller dynamically
      if (getControllerType() != controllerType) {
        controllerInit(controllerType);
        controllerType = getControllerType();
      }


      // stateEstimator(&state, &sensorData, &control, tick);
      stateEstimator(&stateSweep, &stateFlow, &sensorData, &control, tick);
      compressStateSweep();
      compressStateFlow();

      /* Introduce latency in the state estimate. Create a buffer
      to store values of the state estimate. Based on the latency,
      use an earlier state estimate and feed it to the controller.
      Then use the next state estimate in the next iteration.
      */
      if (tick % BUFFER_TICK_INTERVAL == 0) {
        memcpy( &stateBuffer[state_counter_index % NUM_OF_STATES_IN_BUFFER], 
                &stateSweep, 
                sizeof(state_t));
      }
      if (state_counter_index > DELAYED_STATE) {
        memcpy( &stateLag, 
                &stateBuffer[(state_counter_index - DELAYED_STATE) % NUM_OF_STATES_IN_BUFFER], 
                sizeof(state_t)); // load laggy state from the buffer
        memcpy(&stateSweep, &stateLag, sizeof(state_t));   // use laggy state instead of real-time state
      }
      if (tick % BUFFER_TICK_INTERVAL == 0) {
        state_counter_index++;
      }

      // Compute transforms Tor, Twr
      computeTransform(&Tor, &stateFlow);   // pose in odometry frame using stateFlow
      // computeTransform(&Twr, &stateSweep);  // pose in world frame using stateSweep

      // // Check elements of transform matrix Tor
      // if (tick % 2500 == 0) {
      //   double Tor_11 = Tor.pData[0];
      //   double Tor_12 = Tor.pData[1];
      //   double Tor_13 = Tor.pData[2];
      //   double Tor_14 = Tor.pData[3];
      //   double Tor_21 = Tor.pData[4];
      //   double Tor_22 = Tor.pData[5];
      //   double Tor_23 = Tor.pData[6];
      //   double Tor_24 = Tor.pData[7];
      //   double Tor_31 = Tor.pData[8];
      //   double Tor_32 = Tor.pData[9];
      //   double Tor_33 = Tor.pData[10];
      //   double Tor_34 = Tor.pData[11];
      //   double Tor_41 = Tor.pData[12];
      //   double Tor_42 = Tor.pData[13];
      //   double Tor_43 = Tor.pData[14];
      //   double Tor_44 = Tor.pData[15];
      //   double xFl = stateFlow.position.x;
      //   double yFl = stateFlow.position.y;
      //   double zFl = stateFlow.position.z;
      //   consolePrintf("Tor: \n");
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n", Tor_11, Tor_12, Tor_13, Tor_14);
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n", Tor_21, Tor_22, Tor_23, Tor_24);
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n", Tor_31, Tor_32, Tor_33, Tor_34);
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n\n", Tor_41, Tor_42, Tor_43, Tor_44);
      //   consolePrintf(" stateFlow:  \n");
      //   consolePrintf(" %5.6f %5.6f %5.6f \n\n", xFl, yFl, zFl);
      // }


      computeTransformInverse(&Tor_inv, &stateFlow);  // computes inverse of Tor
      
      // // Check elements of transform inverse matrix Tor_inv
      // if (tick % 2500 == 0) {
      //   double Tor_inv_11 = Tor_inv.pData[0];
      //   double Tor_inv_12 = Tor_inv.pData[1];
      //   double Tor_inv_13 = Tor_inv.pData[2];
      //   double Tor_inv_14 = Tor_inv.pData[3];
      //   double Tor_inv_21 = Tor_inv.pData[4];
      //   double Tor_inv_22 = Tor_inv.pData[5];
      //   double Tor_inv_23 = Tor_inv.pData[6];
      //   double Tor_inv_24 = Tor_inv.pData[7];
      //   double Tor_inv_31 = Tor_inv.pData[8];
      //   double Tor_inv_32 = Tor_inv.pData[9];
      //   double Tor_inv_33 = Tor_inv.pData[10];
      //   double Tor_inv_34 = Tor_inv.pData[11];
      //   double Tor_inv_41 = Tor_inv.pData[12];
      //   double Tor_inv_42 = Tor_inv.pData[13];
      //   double Tor_inv_43 = Tor_inv.pData[14];
      //   double Tor_inv_44 = Tor_inv.pData[15];
      //   consolePrintf("Tor_inv: \n");
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n", Tor_inv_11, Tor_inv_12, Tor_inv_13, Tor_inv_14);
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n", Tor_inv_21, Tor_inv_22, Tor_inv_23, Tor_inv_24);
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n", Tor_inv_31, Tor_inv_32, Tor_inv_33, Tor_inv_34);
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n\n", Tor_inv_41, Tor_inv_42, Tor_inv_43, Tor_inv_44);
      // }


      // get state from transform [mainly position and euler angles]
      // getStatefromTransform(&stateCorrect, &Tor);
      // // Check elements of stateCorrect
      // if (tick % 2500 == 0) {
      //   double xC = stateCorrect.position.x;
      //   double yC = stateCorrect.position.y;
      //   double zC = stateCorrect.position.z;
      //   consolePrintf("stateCorrect: \n");
      //   consolePrintf(" %5.6f %5.6f %5.6f \n\n", xC, yC, zC);
      // }

      // multiply Tor and Tor_inv
      mat_mult(&Tor, &Tor_inv, &I);

      // if (tick % 2500 == 0) {
      //   double I_11 = I.pData[0];
      //   double I_12 = I.pData[1];
      //   double I_13 = I.pData[2];
      //   double I_14 = I.pData[3];
      //   double I_21 = I.pData[4];
      //   double I_22 = I.pData[5];
      //   double I_23 = I.pData[6];
      //   double I_24 = I.pData[7];
      //   double I_31 = I.pData[8];
      //   double I_32 = I.pData[9];
      //   double I_33 = I.pData[10];
      //   double I_34 = I.pData[11];
      //   double I_41 = I.pData[12];
      //   double I_42 = I.pData[13];
      //   double I_43 = I.pData[14];
      //   double I_44 = I.pData[15];
      //   consolePrintf("I: \n");
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n", I_11, I_12, I_13, I_14);
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n", I_21, I_22, I_23, I_24);
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n", I_31, I_32, I_33, I_34);
      //   consolePrintf(" %5.6f %5.6f %5.6f %5.6f\n\n", I_41, I_42, I_43, I_44);
      // }


      commanderGetSetpoint(&setpoint, &stateFlow);
      compressSetpoint();

      sitAwUpdateSetpoint(&setpoint, &sensorData, &stateFlow);

      controller(&control, &setpoint, &sensorData, &stateFlow, tick);

      checkEmergencyStopTimeout();

      if (emergencyStop) {
        powerStop();
      } else {
        powerDistribution(&control);
      }

      // Log data to uSD card if configured
      if (   usddeckLoggingEnabled()
          && usddeckLoggingMode() == usddeckLoggingMode_SynchronousStabilizer
          && RATE_DO_EXECUTE(usddeckFrequency(), tick)) {
        usddeckTriggerLogging();
      }
      
    }
    calcSensorToOutputLatency(&sensorData);
    tick++;
    STATS_CNT_RATE_EVENT(&stabilizerRate);
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

static float variance(float *buffer, uint32_t length)
{
  uint32_t i;
  float sum = 0;
  float sumSq = 0;

  for (i = 0; i < length; i++)
  {
    sum += buffer[i];
    sumSq += buffer[i] * buffer[i];
  }

  return sumSq - (sum * sum) / length;
}

/** Evaluate the values from the propeller test
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
static bool evaluateTest(float low, float high, float value, uint8_t motor)
{
  if (value < low || value > high)
  {
    DEBUG_PRINT("Propeller test on M%d [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                motor + 1, (double)low, (double)high, (double)value);
    return false;
  }

  motorPass |= (1 << motor);

  return true;
}


static void testProps(sensorData_t *sensors)
{
  static uint32_t i = 0;
  static float accX[PROPTEST_NBR_OF_VARIANCE_VALUES];
  static float accY[PROPTEST_NBR_OF_VARIANCE_VALUES];
  static float accZ[PROPTEST_NBR_OF_VARIANCE_VALUES];
  static float accVarXnf;
  static float accVarYnf;
  static float accVarZnf;
  static int motorToTest = 0;
  static uint8_t nrFailedTests = 0;
  static float idleVoltage;
  static float minSingleLoadedVoltage[NBR_OF_MOTORS];
  static float minLoadedVoltage;

  if (testState == configureAcc)
  {
    motorPass = 0;
    sensorsSetAccMode(ACC_MODE_PROPTEST);
    testState = measureNoiseFloor;
    minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
    minSingleLoadedVoltage[MOTOR_M1] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M2] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M3] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M4] = minLoadedVoltage;
  }
  if (testState == measureNoiseFloor)
  {
    accX[i] = sensors->acc.x;
    accY[i] = sensors->acc.y;
    accZ[i] = sensors->acc.z;

    if (++i >= PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      i = 0;
      accVarXnf = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarYnf = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarZnf = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
      DEBUG_PRINT("Acc noise floor variance X+Y:%f, (Z:%f)\n",
                  (double)accVarXnf + (double)accVarYnf, (double)accVarZnf);
      testState = measureProp;
    }

  }
  else if (testState == measureProp)
  {
    if (i < PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      accX[i] = sensors->acc.x;
      accY[i] = sensors->acc.y;
      accZ[i] = sensors->acc.z;
      if (pmGetBatteryVoltage() < minSingleLoadedVoltage[motorToTest])
      {
        minSingleLoadedVoltage[motorToTest] = pmGetBatteryVoltage();
      }
    }
    i++;

    if (i == 1)
    {
      motorsSetRatio(motorToTest, 0xFFFF);
    }
    else if (i == 50)
    {
      motorsSetRatio(motorToTest, 0);
    }
    else if (i == PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      accVarX[motorToTest] = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarY[motorToTest] = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarZ[motorToTest] = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
      DEBUG_PRINT("Motor M%d variance X+Y:%f (Z:%f)\n",
                   motorToTest+1, (double)accVarX[motorToTest] + (double)accVarY[motorToTest],
                   (double)accVarZ[motorToTest]);
    }
    else if (i >= 1000)
    {
      i = 0;
      motorToTest++;
      if (motorToTest >= NBR_OF_MOTORS)
      {
        i = 0;
        motorToTest = 0;
        testState = evaluateResult;
        sensorsSetAccMode(ACC_MODE_FLIGHT);
      }
    }
  }
  else if (testState == testBattery)
  {
    if (i == 0)
    {
      minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
    }
    if (i == 1)
    {
      motorsSetRatio(MOTOR_M1, 0xFFFF);
      motorsSetRatio(MOTOR_M2, 0xFFFF);
      motorsSetRatio(MOTOR_M3, 0xFFFF);
      motorsSetRatio(MOTOR_M4, 0xFFFF);
    }
    else if (i < 50)
    {
      if (pmGetBatteryVoltage() < minLoadedVoltage)
        minLoadedVoltage = pmGetBatteryVoltage();
    }
    else if (i == 50)
    {
      motorsSetRatio(MOTOR_M1, 0);
      motorsSetRatio(MOTOR_M2, 0);
      motorsSetRatio(MOTOR_M3, 0);
      motorsSetRatio(MOTOR_M4, 0);
//      DEBUG_PRINT("IdleV: %f, minV: %f, M1V: %f, M2V: %f, M3V: %f, M4V: %f\n", (double)idleVoltage,
//                  (double)minLoadedVoltage,
//                  (double)minSingleLoadedVoltage[MOTOR_M1],
//                  (double)minSingleLoadedVoltage[MOTOR_M2],
//                  (double)minSingleLoadedVoltage[MOTOR_M3],
//                  (double)minSingleLoadedVoltage[MOTOR_M4]);
      DEBUG_PRINT("%f %f %f %f %f %f\n", (double)idleVoltage,
                  (double)(idleVoltage - minLoadedVoltage),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M1]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M2]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M3]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M4]));
      testState = restartBatTest;
      i = 0;
    }
    i++;
  }
  else if (testState == restartBatTest)
  {
    if (i++ > 2000)
    {
      testState = configureAcc;
      i = 0;
    }
  }
  else if (testState == evaluateResult)
  {
    for (int m = 0; m < NBR_OF_MOTORS; m++)
    {
      if (!evaluateTest(0, PROPELLER_BALANCE_TEST_THRESHOLD,  accVarX[m] + accVarY[m], m))
      {
        nrFailedTests++;
        for (int j = 0; j < 3; j++)
        {
          motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
          vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
          motorsBeep(m, false, 0, 0);
          vTaskDelay(M2T(100));
        }
      }
    }
#ifdef PLAY_STARTUP_MELODY_ON_MOTORS
    if (nrFailedTests == 0)
    {
      for (int m = 0; m < NBR_OF_MOTORS; m++)
      {
        motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
        vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
        motorsBeep(m, false, 0, 0);
        vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
      }
    }
#endif
    motorTestCount++;
    testState = testDone;
  }
}




PARAM_GROUP_START(health)
PARAM_ADD(PARAM_UINT8, startPropTest, &startPropTest)
PARAM_GROUP_STOP(health)


PARAM_GROUP_START(stabilizer)
PARAM_ADD(PARAM_UINT8, estimator, &estimatorType)
PARAM_ADD(PARAM_UINT8, controller, &controllerType)
PARAM_ADD(PARAM_UINT8, stop, &emergencyStop)
PARAM_GROUP_STOP(stabilizer)

LOG_GROUP_START(health)
LOG_ADD(LOG_FLOAT, motorVarXM1, &accVarX[0])
LOG_ADD(LOG_FLOAT, motorVarYM1, &accVarY[0])
LOG_ADD(LOG_FLOAT, motorVarXM2, &accVarX[1])
LOG_ADD(LOG_FLOAT, motorVarYM2, &accVarY[1])
LOG_ADD(LOG_FLOAT, motorVarXM3, &accVarX[2])
LOG_ADD(LOG_FLOAT, motorVarYM3, &accVarY[2])
LOG_ADD(LOG_FLOAT, motorVarXM4, &accVarX[3])
LOG_ADD(LOG_FLOAT, motorVarYM4, &accVarY[3])
LOG_ADD(LOG_UINT8, motorPass, &motorPass)
LOG_ADD(LOG_UINT16, motorTestCount, &motorTestCount)
LOG_GROUP_STOP(health)

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, x, &setpoint.position.x)
LOG_ADD(LOG_FLOAT, y, &setpoint.position.y)
LOG_ADD(LOG_FLOAT, z, &setpoint.position.z)

LOG_ADD(LOG_FLOAT, vx, &setpoint.velocity.x)
LOG_ADD(LOG_FLOAT, vy, &setpoint.velocity.y)
LOG_ADD(LOG_FLOAT, vz, &setpoint.velocity.z)

LOG_ADD(LOG_FLOAT, ax, &setpoint.acceleration.x)
LOG_ADD(LOG_FLOAT, ay, &setpoint.acceleration.y)
LOG_ADD(LOG_FLOAT, az, &setpoint.acceleration.z)

LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(ctrltargetZ)
LOG_ADD(LOG_INT16, x, &setpointCompressed.x)   // position - mm
LOG_ADD(LOG_INT16, y, &setpointCompressed.y)
LOG_ADD(LOG_INT16, z, &setpointCompressed.z)

LOG_ADD(LOG_INT16, vx, &setpointCompressed.vx) // velocity - mm / sec
LOG_ADD(LOG_INT16, vy, &setpointCompressed.vy)
LOG_ADD(LOG_INT16, vz, &setpointCompressed.vz)

LOG_ADD(LOG_INT16, ax, &setpointCompressed.ax) // acceleration - mm / sec^2
LOG_ADD(LOG_INT16, ay, &setpointCompressed.ay)
LOG_ADD(LOG_INT16, az, &setpointCompressed.az)
LOG_GROUP_STOP(ctrltargetZ)

// LOG_GROUP_START(stabilizer)
// LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
// LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
// LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
// LOG_ADD(LOG_FLOAT, thrust, &control.thrust)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &stateSweep.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &stateSweep.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &stateSweep.attitude.yaw)
LOG_ADD(LOG_FLOAT, thrust, &control.thrust)

STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)
LOG_ADD(LOG_UINT32, intToOut, &inToOutLatency)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

// LOG_GROUP_START(stateEstimate)
// LOG_ADD(LOG_FLOAT, x, &state.position.x)
// LOG_ADD(LOG_FLOAT, y, &state.position.y)
// LOG_ADD(LOG_FLOAT, z, &state.position.z)

// LOG_ADD(LOG_FLOAT, vx, &state.velocity.x)
// LOG_ADD(LOG_FLOAT, vy, &state.velocity.y)
// LOG_ADD(LOG_FLOAT, vz, &state.velocity.z)

// LOG_ADD(LOG_FLOAT, ax, &state.acc.x)
// LOG_ADD(LOG_FLOAT, ay, &state.acc.y)
// LOG_ADD(LOG_FLOAT, az, &state.acc.z)

// LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
// LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
// LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)

// LOG_ADD(LOG_FLOAT, qx, &state.attitudeQuaternion.x)
// LOG_ADD(LOG_FLOAT, qy, &state.attitudeQuaternion.y)
// LOG_ADD(LOG_FLOAT, qz, &state.attitudeQuaternion.z)
// LOG_ADD(LOG_FLOAT, qw, &state.attitudeQuaternion.w)
// LOG_GROUP_STOP(stateEstimate)



LOG_GROUP_START(stateEstimate)
LOG_ADD(LOG_FLOAT, x_Lh, &stateSweep.position.x)
LOG_ADD(LOG_FLOAT, y_Lh, &stateSweep.position.y)
LOG_ADD(LOG_FLOAT, z_Lh, &stateSweep.position.z)

LOG_ADD(LOG_FLOAT, x_Fl, &stateFlow.position.x)
LOG_ADD(LOG_FLOAT, y_Fl, &stateFlow.position.y)
LOG_ADD(LOG_FLOAT, z_Fl, &stateFlow.position.z)

LOG_ADD(LOG_FLOAT, vx, &stateSweep.velocity.x)
LOG_ADD(LOG_FLOAT, vy, &stateSweep.velocity.y)
LOG_ADD(LOG_FLOAT, vz, &stateSweep.velocity.z)

LOG_ADD(LOG_FLOAT, ax, &stateSweep.acc.x)
LOG_ADD(LOG_FLOAT, ay, &stateSweep.acc.y)
LOG_ADD(LOG_FLOAT, az, &stateSweep.acc.z)

LOG_ADD(LOG_FLOAT, roll_Lh, &stateSweep.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch_Lh, &stateSweep.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw_Lh, &stateSweep.attitude.yaw)

LOG_ADD(LOG_FLOAT, roll_Fl, &stateFlow.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch_Fl, &stateFlow.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw_Fl, &stateFlow.attitude.yaw)

LOG_ADD(LOG_FLOAT, qx, &stateSweep.attitudeQuaternion.x)
LOG_ADD(LOG_FLOAT, qy, &stateSweep.attitudeQuaternion.y)
LOG_ADD(LOG_FLOAT, qz, &stateSweep.attitudeQuaternion.z)
LOG_ADD(LOG_FLOAT, qw, &stateSweep.attitudeQuaternion.w)
LOG_GROUP_STOP(stateEstimate)



// LOG_GROUP_START(stateEstimateZ)
// LOG_ADD(LOG_INT16, x, &stateCompressed.x)                 // position - mm
// LOG_ADD(LOG_INT16, y, &stateCompressed.y)
// LOG_ADD(LOG_INT16, z, &stateCompressed.z)

// LOG_ADD(LOG_INT16, vx, &stateCompressed.vx)               // velocity - mm / sec
// LOG_ADD(LOG_INT16, vy, &stateCompressed.vy)
// LOG_ADD(LOG_INT16, vz, &stateCompressed.vz)

// LOG_ADD(LOG_INT16, ax, &stateCompressed.ax)               // acceleration - mm / sec^2
// LOG_ADD(LOG_INT16, ay, &stateCompressed.ay)
// LOG_ADD(LOG_INT16, az, &stateCompressed.az)

// LOG_ADD(LOG_UINT32, quat, &stateCompressed.quat)           // compressed quaternion, see quatcompress.h

// LOG_ADD(LOG_INT16, rateRoll, &stateCompressed.rateRoll)   // angular velocity - milliradians / sec
// LOG_ADD(LOG_INT16, ratePitch, &stateCompressed.ratePitch)
// LOG_ADD(LOG_INT16, rateYaw, &stateCompressed.rateYaw)
// LOG_GROUP_STOP(stateEstimateZ)