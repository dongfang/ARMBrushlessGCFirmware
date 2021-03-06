/*
 *  engine.c
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */
#include <stdint.h>
#include <math.h>
#include "engine.h"
#include "adc.h"
#include "gyro.h"
#include "utils.h"
#include "config.h"
#include "pwm.h"
#include "rc.h"
#include "comio.h"
#include "stopwatch.h"
#include "i2c.h"
#include "definitions.h"
#include "usb.h"
#include "main.h"

int debugPrint   = 0;
int debugPerf    = 0;
int debugSense   = 0;
int debugCnt     = 0;
int debugRC      = 0;
int debugOrient  = 0;
int debugSetpoints = 0;
int debugGravityVector = 0;
int debugAutoPan = 0;

float /*pitch, Gyro_Pitch_angle,*/ pitch_setpoint = 0.0f, pitch_Error_last = 0.0f,  pitch_angle_correction;
float /*roll,  Gyro_Roll_angle,*/  roll_setpoint  = 0.0f,  roll_Error_last = 0.0f,   roll_angle_correction;
float /*yaw,   Gyro_Yaw_angle,*/   yaw_setpoint   = 0.0f,   yaw_Error_last = 0.0f,    yaw_angle_correction;

//float ADC1Ch13_yaw;

static float rollRCOffset = 0.0f, pitchRCOffset = 0.0f, yawRCOffset = 0.0f;

static int printcounter = 0;

float Output[EULER];
float CameraOrient[EULER];
//float AccAngleSmooth[EULER];

float AccData[NUMAXES]  = {0.0f, 0.0f, 0.0f};
float GyroData[NUMAXES] = {0.0f, 0.0f, 0.0f};
float ApproxGravityVector[NUMAXES]  = {0.0f, 0.0f, 1.0f};

float Step[NUMAXES]     = {0.0f, 0.0f, 0.0f};
float RCSmooth[NUMAXES] = {0.0f, 0.0f, 0.0f};

//void mergeACC(float* v, float* acc, float alpha);

void roll_PID(void)
{
    float Error_current = roll_setpoint + CameraOrient[ROLL] * 1000.0;
    float KP = Error_current * ((float)configData[1] / 1000.0);
    float KD = ((float)configData[4] / 100.0) * (Error_current - roll_Error_last);

    roll_Error_last = Error_current;

    Output[ROLL] = KD + KP;
    SetRollMotor(KP + KD, configData[7]);
}

void pitch_PID(void)
{
    float Error_current = pitch_setpoint + CameraOrient[PITCH] * 1000.0;
    float KP = Error_current * ((float)configData[0] / 1000.0);
    float KD = ((float)configData[3] / 100.0) * (Error_current - pitch_Error_last);

    pitch_Error_last = Error_current;

    Output[PITCH] = KD + KP;
    SetPitchMotor(KP + KD, configData[6]);
}

void yaw_PID(void)
{
    float Error_current = yaw_setpoint + CameraOrient[YAW] * 1000.0;
    float KP = Error_current * ((float)configData[2] / 1000.0);
    float KD = ((float)configData[5] / 100.0) * (Error_current - yaw_Error_last);

    yaw_Error_last = Error_current;

    Output[YAW] = KD + KP;
    SetYawMotor(KP + KD, configData[8]);
}

float constrain(float value, float low, float high)
{
    if (value < low)
        return low;

    if (value > high)
        return high;

    return value;
}

/*
  Limits the Pitch angle
*/
float Limit_Pitch(float step, float pitch)
{
    if (pitch < PITCH_UP_LIMIT && step > 0)
    {
        step = 0.0;
    }

    if (pitch > PITCH_DOWN_LIMIT && step < 0)
    {
        step = 0.0;
    }

    return step;
}

/*
void Init_Orientation()
{
    int init_loops = 150;
    float AccAngle[NUMAXES];
    int i;

    for (i = 0; i < init_loops; i++)
    {
        MPU6050_ACC_get(AccData); //Getting Accelerometer data

        AccAngle[ROLL]  = -(atan2f(AccData[X_AXIS], AccData[Z_AXIS]));   //Calculating roll ACC angle
        AccAngle[PITCH] = +(atan2f(AccData[Y_AXIS], AccData[Z_AXIS]));   //Calculating pitch ACC angle

        AccAngleSmooth[ROLL]  = ((AccAngleSmooth[ROLL] * (float)(init_loops - 1))  + AccAngle[ROLL])  / (float)init_loops; //Averaging roll ACC values
        AccAngleSmooth[PITCH] = ((AccAngleSmooth[PITCH] * (float)(init_loops - 1)) + AccAngle[PITCH]) / (float)init_loops; //Averaging pitch  ACC values
        Delay_ms(1);
    }

    CameraOrient[PITCH] = AccAngleSmooth[PITCH];
    CameraOrient[ROLL]  = AccAngleSmooth[ROLL];
    CameraOrient[YAW]   = 0.0f;
}
*/

/*
void Get_Orientation(float *SmoothAcc, float *Orient, float *AccData, float *GyroData, float dt) {
    float AccAngle[EULER];
    float GyroRate[EULER];

    //AccAngle[ROLL]  = -(atan2f(AccData[X_AXIS], AccData[Z_AXIS]));   //Calculating roll ACC angle
    AccAngle[ROLL]  = -(atan2f(AccData[X_AXIS], sqrtf( AccData[Z_AXIS] * AccData[Z_AXIS] + AccData[Y_AXIS] * AccData[Y_AXIS])));   //Calculating roll ACC angle
    AccAngle[PITCH] = +(atan2f(AccData[Y_AXIS], AccData[Z_AXIS]));   //Calculating pitch ACC angle

    SmoothAcc[ROLL]  = ((SmoothAcc[ROLL] * 99.0f)  + AccAngle[ROLL])  / 100.0f; //Averaging roll ACC values
    SmoothAcc[PITCH] = ((SmoothAcc[PITCH] * 99.0f) + AccAngle[PITCH]) / 100.0f; //Averaging pitch  ACC values

    GyroRate[PITCH] =  GyroData[X_AXIS];
    Orient[PITCH]   = (Orient[PITCH] + GyroRate[PITCH] * dt) + 0.0002f * (SmoothAcc[PITCH] - Orient[PITCH]);  //Pitch Horizon

    GyroRate[ROLL] = -GyroData[Z_AXIS] * sinf(Orient[PITCH]) + GyroData[Y_AXIS] * cosf(fabsf(Orient[PITCH]));
    Orient[ROLL]   = (Orient[ROLL] + GyroRate[ROLL] * dt)    + 0.0002f * (SmoothAcc[ROLL] - Orient[ROLL]); //Roll Horizon

    GyroRate[YAW]  = -GyroData[Z_AXIS] * cosf(fabsf(Orient[PITCH])) - GyroData[Y_AXIS] * sinf(Orient[PITCH]); //presuming Roll is horizontal
    Orient[YAW]    = (Orient[YAW] + GyroRate[YAW] * dt); //Yaw
}
*/

// The usual small-angle approximation.
void rotateV(float* v, float* rates, float dt) {
  float tmp[NUMAXES];
  // The small rotation rate deltas.
  float deltas[NUMAXES];
  uint8_t i;
  for (i=0; i<3; i++) {
	  deltas[i] = rates[i]*dt;
	  tmp[i] = v[i];
  }

  v[Z_AXIS] -= deltas[ROLL]  * tmp[X_AXIS] + deltas[PITCH] * tmp[Y_AXIS];
  v[X_AXIS] += deltas[ROLL]  * tmp[Z_AXIS] - deltas[YAW]   * tmp[Y_AXIS];
  v[Y_AXIS] += deltas[PITCH] * tmp[Z_AXIS] + deltas[YAW]   * tmp[X_AXIS];
}

void MergeAcc(float alpha) {
  uint8_t axis;
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  //  if (( 36 < accMag && accMag < 196 ) || disableAccGtest) {
    for (axis = 0; axis < 3; axis++) {
      //utilLP_float(&EstG.A[axis], accLPF[axis], AccComplFilterConst);
      ApproxGravityVector[axis] = ApproxGravityVector[axis] * (1.0 - alpha) +
    		  AccData[axis] * alpha;
    }
  //  }
}

void ComputeEulerAngles(float dt) {
    // Normal attitude angles come from the rotation from global in order yaw, pitch, roll (roll on pitch on yaw)
    // Camera gimbals are typically pitch on roll on yaw. Not the same!
    // Therefore, we reverse the definitions and domains of pitch and roll. Formulas too.
	CameraOrient[ROLL] = atan2(ApproxGravityVector[X_AXIS] ,
			sqrtf(ApproxGravityVector[Z_AXIS]*ApproxGravityVector[Z_AXIS] +
					ApproxGravityVector[Y_AXIS]*ApproxGravityVector[Y_AXIS]));
	CameraOrient[PITCH] = atan2(ApproxGravityVector[Y_AXIS], ApproxGravityVector[Z_AXIS]);
	// TODO: Completely wrong. This is the integral of the sensor yaw rate. NOT the gimbal yaw.
	CameraOrient[YAW] -= GyroData[YAW] * dt;
}

void Init_Orientation()
{
    int init_loops = 150;
    int i;

    for (i = 0; i < init_loops; i++)
    {
        MPU6050_ACC_get(AccData); //Getting Accelerometer data
        MergeAcc(0.1);
        Delay_ms(1);
    }

    CameraOrient[YAW]   = 0.0f;
    ComputeEulerAngles(0);
}

void Update_Orientation(float dt) {
	// First, apply small-angle approximation to our estimated gravity vector:
	rotateV(ApproxGravityVector, GyroData, dt);
	// Then complementary-filter that together with the acc. meter's data:
	MergeAcc(0.002); // TODO: Make this magic number configurable It is very useful to
	// have different values to choose from. High->IMU returns quickly to a good attitude
	// after having been knocked and gyros saturated. Low->less sensitive to lateral accelerations,
	// such as a fixed-wing plane at high power or in a turn.
	ComputeEulerAngles(dt);
}

//---------------------YAW autopan----------------------//
//#define ANGLE2SETPOINT -1000
#define DEADBAND 2.0f //in radians with respect to one motor pole (actual angle is (DEADBAND / numberPoles) * R2D)
#define MOTORPOS2SETPNT 0.45f //scaling factor for how fast it should move
#define AUTOPANSMOOTH 40.0f
//#define LPFTIMECONSTANT 20 //change this to adjust sensitivity

//float yawAngleLPF=0;
float centerPoint = 0.0f;
float stepSmooth  = 0.0f;
float step        = 0.0f;

float autoPan(float motorPos, float setpoint)
{
    if (motorPos < centerPoint - DEADBAND)
    {
        centerPoint = (+DEADBAND);
        step = MOTORPOS2SETPNT * motorPos; //dampening
    }
    else if (motorPos > centerPoint + DEADBAND)
    {
        centerPoint = (-DEADBAND);
        step = MOTORPOS2SETPNT * motorPos; //dampening
    }
    else
    {
        step = 0.0f;
        centerPoint = 0.0f;
    }
    stepSmooth = (stepSmooth * (AUTOPANSMOOTH - 1.0f) + step) / AUTOPANSMOOTH;
    return (setpoint -= stepSmooth);
}

//--------------------Engine Process-----------------------------//
void engineProcess(float dt)
{
    static int loopCounter;
    tStopWatch sw;

    loopCounter++;
    LEDon();
    DEBUG_LEDoff();

    StopWatchInit(&sw);
    MPU6050_ACC_get(AccData); // Getting Accelerometer data.
    // dongfang comment: We don't need to do this all that often really.
    unsigned long tAccGet = StopWatchLap(&sw);

    MPU6050_Gyro_get(GyroData); // Getting Gyroscope data
    unsigned long tGyroGet = StopWatchLap(&sw);

    // dongfang: Major change here.
    Update_Orientation(dt);
    unsigned long tAccAngle = StopWatchLap(&sw);

    // if we enable RC control
    if (configData[9] == '1')
    {
        // Get the RX values and Averages
        Get_RC_Step(Step, RCSmooth); // Get RC movement on all three AXIS
        Step[PITCH] = Limit_Pitch(Step[PITCH], CameraOrient[PITCH]); // limit pitch to defined limits in header
    }

    // Pitch adjustments
    // pitch_setpoint += Step[PITCH];
    // This is where the dreaded RC integration takes place.
    pitchRCOffset += Step[PITCH] / 1000.0;

    pitch_angle_correction = constrain((CameraOrient[PITCH] + pitchRCOffset) * R2D, -CORRECTION_STEP, CORRECTION_STEP);
    pitch_setpoint += pitch_angle_correction; // Pitch return to zero after collision

    // Roll Adjustments
    // roll_setpoint += Step[ROLL];
    // This is where the dreaded RC integration takes place.
    rollRCOffset += Step[ROLL] / 1000.0;

    // include the config roll offset which is scaled to 0 = -10.0 degrees, 100 = 0.0 degrees, and 200 = 10.0 degrees
    roll_angle_correction = constrain((CameraOrient[ROLL] + rollRCOffset + Deg2Rad((configData[11] - 100) / 10.0)) * R2D, -CORRECTION_STEP, CORRECTION_STEP);
    roll_setpoint += roll_angle_correction; //Roll return to zero after collision

    // if we enabled AutoPan on Yaw
    if (configData[10] == '0')
    {
        //ADC1Ch13_yaw = ((ADC1Ch13_yaw * 99.0) + ((float)(readADC1(13) - 2000) / 4000.0)) / 100.0;  // Average ADC value
        //CameraOrient[YAW] = CameraOrient[YAW] + 0.01 * (ADC1Ch13_yaw - CameraOrient[YAW]);
        yaw_setpoint = autoPan(Output[YAW], yaw_setpoint);
    }
    else
    {
        // Yaw Adjustments
        yaw_setpoint += Step[YAW];
        yawRCOffset += Step[YAW] / 1000.0;
    }

#if 0
    yaw_angle_correction = constrain((CameraOrient[YAW] + yawRCOffset) * R2D, -CORRECTION_STEP, CORRECTION_STEP);
    yaw_setpoint += yaw_angle_correction; // Yaw return to zero after collision
#endif

    unsigned long tCalc = StopWatchLap(&sw);

    pitch_PID();
    roll_PID();
    yaw_PID();

    unsigned long tPID = StopWatchLap(&sw);
    unsigned long tAll = StopWatchTotal(&sw);

    printcounter++;

    //if (printcounter >= 500 || dt > 0.0021)
    if (printcounter >= 200)
    {
        if (debugPrint)
        {
            print("Loop: %7d, I2CErrors: %d, angles: roll %7.2f, pitch %7.2f, yaw %7.2f\r\n",
                  loopCounter, I2Cerrorcount, Rad2Deg(CameraOrient[ROLL]),
                  Rad2Deg(CameraOrient[PITCH]), Rad2Deg(CameraOrient[YAW]));
        }

        if (debugSense)
        {
            print(" dt %f, AccData: %8.3f | %8.3f | %8.3f, GyroData %7.3f | %7.3f | %7.3f \r\n",
                  dt, AccData[X_AXIS], AccData[Y_AXIS], AccData[Z_AXIS], GyroData[ROLL], GyroData[PITCH], GyroData[YAW]);
        }

        if (debugPerf)
        {
            print("idle: %5.2f%%, time[µs]: attitude est. %4d, IMU acc %4d, gyro %4d, angle %4d, calc %4d, PID %4d\r\n",
                  GetIdlePerf(), tAll, tAccGet, tGyroGet, tAccAngle, tCalc, tPID);
        }

        if (debugRC)
        {
            print(" RC2avg: %7.2f |  RC3avg: %7.2f |  RC4avg: %7.2f | RStep:%7.3f  PStep: %7.3f  YStep: %7.3f\r\n",
                  RCSmooth[ROLL], RCSmooth[PITCH], RCSmooth[YAW], Step[ROLL], Step[PITCH], Step[YAW]);
        }

        if (debugOrient)
        {
            print("Roll:%12.4f | Pitch:%12.4f | Yaw:%12.4f\r\n",
                  CameraOrient[ROLL]*R2D, CameraOrient[PITCH]*R2D, CameraOrient[YAW]*R2D);
        }

        if (debugGravityVector)
        {
            print("X:%12.4f | Y:%12.4f | Z:%12.4f\r\n",
                  ApproxGravityVector[0], ApproxGravityVector[1], ApproxGravityVector[2]);
        }

        if (debugSetpoints)
        {
            print("Roll_setpoint:%12.4f | Pitch_setpoint:%12.4f | Yaw_setpoint:%12.4f\r\n",
                  roll_setpoint, pitch_setpoint, yaw_setpoint);
        }

        if (debugCnt)
        {
            print("Counter min %3d, %3d, %3d,  max %4d, %4d, %4d, count %3d, %3d, %3d, usbOverrun %4d\r\n",
                  MinCnt[ROLL], MinCnt[PITCH], MinCnt[YAW],
                  MaxCnt[ROLL], MaxCnt[PITCH], MaxCnt[YAW],
                  IrqCnt[ROLL], IrqCnt[PITCH], IrqCnt[YAW],
                  usbOverrun());
        }

        if (debugAutoPan)
        {
            print("Pitch_output:%3.2f | Roll_output:%3.2f | Yaw_output:%3.2f | centerpoint:%4.4f\n\r",
                  Output[PITCH],
                  Output[ROLL],
                  Output[YAW],
                  centerPoint);
        }

        printcounter = 0;
    }

    LEDoff();
}

