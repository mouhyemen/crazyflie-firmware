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

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
// static state_t state;
static state_t stateSweep;
static state_t stateFlow;
static state_t stateLag; // delayed state estimate, earliest state in the stateBuffer
#define NUM_OF_STATES_IN_BUFFER 1 // number of states stored relates to latency in milliseconds
#define BUFFER_TICK_INTERVAL 1 // tick interval of states in buffer
#define DELAYED_STATE (NUM_OF_STATES_IN_BUFFER - 1) // latency = # states in buffer * buffer_tick_interval
static state_t stateBuffer[NUM_OF_STATES_IN_BUFFER]; // buffer of state estimates from t-N up to t
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
      
      // combine stateSweep and stateFlow into stateCorrected

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

// LOG_GROUP_START(stateEstimateSweep)
// LOG_ADD(LOG_FLOAT, x, &stateSweep.position.x)
// LOG_ADD(LOG_FLOAT, y, &stateSweep.position.y)
// LOG_ADD(LOG_FLOAT, z, &stateSweep.position.z)

// LOG_ADD(LOG_FLOAT, vx, &stateSweep.velocity.x)
// LOG_ADD(LOG_FLOAT, vy, &stateSweep.velocity.y)
// LOG_ADD(LOG_FLOAT, vz, &stateSweep.velocity.z)

// LOG_ADD(LOG_FLOAT, ax, &stateSweep.acc.x)
// LOG_ADD(LOG_FLOAT, ay, &stateSweep.acc.y)
// LOG_ADD(LOG_FLOAT, az, &stateSweep.acc.z)

// LOG_ADD(LOG_FLOAT, roll, &stateSweep.attitude.roll)
// LOG_ADD(LOG_FLOAT, pitch, &stateSweep.attitude.pitch)
// LOG_ADD(LOG_FLOAT, yaw, &stateSweep.attitude.yaw)

// LOG_ADD(LOG_FLOAT, qx, &stateSweep.attitudeQuaternion.x)
// LOG_ADD(LOG_FLOAT, qy, &stateSweep.attitudeQuaternion.y)
// LOG_ADD(LOG_FLOAT, qz, &stateSweep.attitudeQuaternion.z)
// LOG_ADD(LOG_FLOAT, qw, &stateSweep.attitudeQuaternion.w)
// LOG_GROUP_STOP(stateEstimateSweep)


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


// LOG_GROUP_START(stateEstimateFlow)
// LOG_ADD(LOG_FLOAT, x, &stateFlow.position.x)
// LOG_ADD(LOG_FLOAT, y, &stateFlow.position.y)
// LOG_ADD(LOG_FLOAT, z, &stateFlow.position.z)

// LOG_ADD(LOG_FLOAT, vx, &stateFlow.velocity.x)
// LOG_ADD(LOG_FLOAT, vy, &stateFlow.velocity.y)
// LOG_ADD(LOG_FLOAT, vz, &stateFlow.velocity.z)

// LOG_ADD(LOG_FLOAT, ax, &stateFlow.acc.x)
// LOG_ADD(LOG_FLOAT, ay, &stateFlow.acc.y)
// LOG_ADD(LOG_FLOAT, az, &stateFlow.acc.z)

// LOG_ADD(LOG_FLOAT, roll, &stateFlow.attitude.roll)
// LOG_ADD(LOG_FLOAT, pitch, &stateFlow.attitude.pitch)
// LOG_ADD(LOG_FLOAT, yaw, &stateFlow.attitude.yaw)

// LOG_ADD(LOG_FLOAT, qx, &stateFlow.attitudeQuaternion.x)
// LOG_ADD(LOG_FLOAT, qy, &stateFlow.attitudeQuaternion.y)
// LOG_ADD(LOG_FLOAT, qz, &stateFlow.attitudeQuaternion.z)
// LOG_ADD(LOG_FLOAT, qw, &stateFlow.attitudeQuaternion.w)
// LOG_GROUP_STOP(stateEstimateFlow)


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

// LOG_GROUP_START(stateEstimateZSweep)
// LOG_ADD(LOG_INT16, x, &stateCompressedSweep.x)                 // position - mm
// LOG_ADD(LOG_INT16, y, &stateCompressedSweep.y)
// LOG_ADD(LOG_INT16, z, &stateCompressedSweep.z)

// LOG_ADD(LOG_INT16, vx, &stateCompressedSweep.vx)               // velocity - mm / sec
// LOG_ADD(LOG_INT16, vy, &stateCompressedSweep.vy)
// LOG_ADD(LOG_INT16, vz, &stateCompressedSweep.vz)

// LOG_ADD(LOG_INT16, ax, &stateCompressedSweep.ax)               // acceleration - mm / sec^2
// LOG_ADD(LOG_INT16, ay, &stateCompressedSweep.ay)
// LOG_ADD(LOG_INT16, az, &stateCompressedSweep.az)

// LOG_ADD(LOG_UINT32, quat, &stateCompressedSweep.quat)           // compressed quaternion, see quatcompress.h

// LOG_ADD(LOG_INT16, rateRoll, &stateCompressedSweep.rateRoll)   // angular velocity - milliradians / sec
// LOG_ADD(LOG_INT16, ratePitch, &stateCompressedSweep.ratePitch)
// LOG_ADD(LOG_INT16, rateYaw, &stateCompressedSweep.rateYaw)
// LOG_GROUP_STOP(stateEstimateZ)


// LOG_GROUP_START(stateEstimateZFlow)
// LOG_ADD(LOG_INT16, x, &stateCompressedFlow.x)                 // position - mm
// LOG_ADD(LOG_INT16, y, &stateCompressedFlow.y)
// LOG_ADD(LOG_INT16, z, &stateCompressedFlow.z)

// LOG_ADD(LOG_INT16, vx, &stateCompressedFlow.vx)               // velocity - mm / sec
// LOG_ADD(LOG_INT16, vy, &stateCompressedFlow.vy)
// LOG_ADD(LOG_INT16, vz, &stateCompressedFlow.vz)

// LOG_ADD(LOG_INT16, ax, &stateCompressedFlow.ax)               // acceleration - mm / sec^2
// LOG_ADD(LOG_INT16, ay, &stateCompressedFlow.ay)
// LOG_ADD(LOG_INT16, az, &stateCompressedFlow.az)

// LOG_ADD(LOG_UINT32, quat, &stateCompressedFlow.quat)           // compressed quaternion, see quatcompress.h

// LOG_ADD(LOG_INT16, rateRoll, &stateCompressedFlow.rateRoll)   // angular velocity - milliradians / sec
// LOG_ADD(LOG_INT16, ratePitch, &stateCompressedFlow.ratePitch)
// LOG_ADD(LOG_INT16, rateYaw, &stateCompressedFlow.rateYaw)
// LOG_GROUP_STOP(stateEstimateZ)

