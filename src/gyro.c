/*
 *  gyro.c
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */
#include <stdint.h>
#include <math.h>
#include "gyro.h"
#include "i2c.h"
#include "utils.h"
#include "pins.h"
#include "pwm.h"
#include "definitions.h"
#include "engine.h"

static float gyroADCOffset[3];
static short int gyroADC[3];

// TODO! Make confgurable.

// This is my (dongfang) setup. I have the board vertically, as if on the side of the camera.
// The connector is towards the rear, the board is on the left hand side of the camera with
// the MPU6050's top facing towards it.
//static const MPU_6050_PLANE_t MPU_6050_PLANE = CAMERA_SIDE;
//static const MPU_6050_TURN_t MPU_6050_TURN = NO_TURN;
//static const MPU_6050_FLIP_t MPU_6050_FLIP = NO_FLIP;

// Here is the default setting: Board as if on top of the camera, with the MPU6050's top
// facing up and the connector to the right.
static const MPU_6050_PLANE_t MPU_6050_PLANE = CAMERA_TOP;
static const MPU_6050_TURN_t MPU_6050_TURN = NO_TURN;
static const MPU_6050_FLIP_t MPU_6050_FLIP = NO_FLIP;

int MPU6050_Init(void)
{
    uint8_t mpu_adr;

    // Check to make sure there is a device out there and its on the
    // correct address
    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x75); // Who Am I
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(1);

    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFF));//ff-1(Read)
    I2C1_WaitAck();

    mpu_adr = I2C1_ReceiveByte();//receive

    I2C1_NoAck();
    I2C1_Stop();

    // if wrong address or no device then bail out with an error
    if (mpu_adr != 0x68)
    {
        return -1;
    }

    Delay_ms(5);

    // force a device reset
    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x6B); // Force a reset
    I2C1_WaitAck();
    I2C1_SendByte(0x80);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(150);

    // set the internal clock to be the Z AXIS gyro
    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x6B);
    I2C1_WaitAck();
    I2C1_SendByte(0x03); // clock source AKA - changed from 0x00 (internal clock)
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    //  turn off all sleep modes
    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x6C);
    I2C1_WaitAck();
    I2C1_SendByte(0x00); // wake up ctrl
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    //  Set the sample rate on the accel and refresh rate on the gyro
    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x19); // Sample output rate
    I2C1_WaitAck();
    I2C1_SendByte(0x00);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    // turn on the built in LPF
    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x1A);
    I2C1_WaitAck();
    I2C1_SendByte(0x00);    //low pass disable AKA - was 0x02 for 98hz
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    // set the gyro scale
    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x1B);
    I2C1_WaitAck();
    I2C1_SendByte(0x00); //set to 250LSB/Deg/s
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    // set the accel scale
    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x1C);
    I2C1_WaitAck();
    I2C1_SendByte(0x00); //set to accel to +/-2g scale AKA - was 0x08 for +/-4g
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    //  configure the interrupt(s) pin because we don't use it
    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x37); // init pin config
    I2C1_WaitAck();
    I2C1_SendByte(0x00);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    // disable the interrupt pin(s)
    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x38); // init enable
    I2C1_WaitAck();
    I2C1_SendByte(0x00);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

/*
    // this was bad code and was removed
    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x6A);
    I2C1_WaitAck();
    I2C1_SendByte(0x01); // reset signal paths
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);
*/

    return 0;
}

void MPU6050_get(int cmd, uint8_t read[6])
{
    I2Cerror = 0;

    I2C1_Start();
    I2C1_SendByte((MPU6050_ADDR & 0xFE));//fe-0(Write)
    I2C1_WaitAck();

    if (I2Cerror == 0)
    {
        I2C1_SendByte(cmd);
        I2C1_WaitAck();

        if (I2Cerror == 0)
        {
            I2C1_Stop();
            I2C1_Start();
            I2C1_SendByte((MPU6050_ADDR & 0xFF));//ff-1(Read)
            I2C1_WaitAck();

            if (I2Cerror == 0)
            {
                read[0] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                read[1] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                read[2] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                read[3] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                read[4] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                read[5] = I2C1_ReceiveByte(); //receive
                I2C1_NoAck();
                I2C1_Stop();
            }
        } else {
        	I2C1_Stop();
        }
    }
}

void MPU6050_ACC_get(float *AccData)
{
    uint8_t read[6];
    float accDataUntransformed[3];
    MPU6050_get(0x3B, read);
    if (I2Cerror == 0)
    {
    	accDataUntransformed[0] = (short)((read[0] << 8) | read[1]);
    	accDataUntransformed[1] = (short)((read[2] << 8) | read[3]);
    	accDataUntransformed[2] = (short)((read[4] << 8) | read[5]);

        // The chip has its topside in which plane (3 possibilities)
        switch(MPU_6050_PLANE) {
        case CAMERA_TOP:
        	AccData[X_AXIS] = -accDataUntransformed[0];
        	AccData[Y_AXIS] =  accDataUntransformed[1];
        	AccData[Z_AXIS] =  accDataUntransformed[2];
        	break;
        case CAMERA_SIDE:
        	// TODO! Signs.
        	AccData[X_AXIS] =  accDataUntransformed[2];
        	AccData[Y_AXIS] = -accDataUntransformed[0];
        	AccData[Z_AXIS] =  accDataUntransformed[1];
        	break;
        case FOCAL_PLANE:
        	// TODO! Not sure the signs are right!
        	AccData[X_AXIS] = accDataUntransformed[1];
        	AccData[Y_AXIS] = accDataUntransformed[2];
        	AccData[Z_AXIS] = accDataUntransformed[0];
        	break;
        }

        switch(MPU_6050_FLIP) {
        case NO_FLIP: break;
        case FLIPPED:
        	// Z acc gets reversed and (assuming the flip is around X axis) X.
        	accDataUntransformed[0] = -accDataUntransformed[0];
        	accDataUntransformed[2] = -accDataUntransformed[2];
        	break;
        }

        float tmp;
        // The chip has the dot at which corner (4 possibilities)
        switch(MPU_6050_TURN) {
        case NO_TURN: break;
        case CW90:
        	tmp = accDataUntransformed[0];
        	accDataUntransformed[0] = accDataUntransformed[1];
        	accDataUntransformed[1] = -tmp;
        	break;
        case CW180:
        	accDataUntransformed[0] = -accDataUntransformed[0];
        	accDataUntransformed[1] = -accDataUntransformed[1];
    		break;
        case CW270:
    		tmp = accDataUntransformed[0];
    		accDataUntransformed[0] = -accDataUntransformed[1];
    		accDataUntransformed[1] = tmp;
    		break;
        }
    }
}

void MPU6050_Gyro_get(float *GyroData)
{
    uint8_t read[6];
    MPU6050_get(0x43, read);

    if (I2Cerror == 0) // TODO: We need to deal with a jammed I2C bus eventually.
    {
        gyroADC[0] = (short)((read[0] << 8) | read[1]);
        gyroADC[1] = (short)((read[2] << 8) | read[3]);
        gyroADC[2] = (short)((read[4] << 8) | read[5]);

        float gyroScaleFactor = 7505.747116f;// 8000.0f;//     2.0F/131.0F * M_PI/180.0F;
        float gyroDataUntransformed[3];
        uint8_t i;

        for (i=0; i<3; i++) {
        	gyroDataUntransformed[i] = ((float)gyroADC[i] - gyroADCOffset[i]) / gyroScaleFactor;
        }

        // Transform stuff depending on hardware orientation.
        // There are 24 possible orientations (at right angles), all are covered here (2*4*3).
        // Whether all the transforms are consistent with those for acceleration - hmmm :) TBD.

        // The chip has its topside in which plane (3 possibilities)
        switch(MPU_6050_PLANE) {
        case CAMERA_TOP:
        	GyroData[PITCH] = gyroDataUntransformed[0];
        	GyroData[ROLL]  = gyroDataUntransformed[1];
        	GyroData[YAW]   = gyroDataUntransformed[2];
        	break;
        case CAMERA_SIDE:
        	GyroData[PITCH] = -gyroDataUntransformed[2];
        	GyroData[ROLL]  = -gyroDataUntransformed[0];
        	GyroData[YAW]   = gyroDataUntransformed[1];
        	break;
        case FOCAL_PLANE:
        	GyroData[PITCH] = gyroDataUntransformed[1];
        	GyroData[ROLL]  = gyroDataUntransformed[2];
        	GyroData[YAW]   = gyroDataUntransformed[0];
        	break;
        }

        // Is the chip upside down (2 possibilities)
        switch(MPU_6050_FLIP) {
        case NO_FLIP: break;
        case FLIPPED:
        	// Z gyro gets reversed and (assuming the flip is around X axis) Y.
    		gyroDataUntransformed[1] = -gyroDataUntransformed[1];
    		gyroDataUntransformed[2] = -gyroDataUntransformed[2];
    		break;
        }

        float tmp;
        // The chip has the dot at which corner (4 possibilities)
        switch(MPU_6050_TURN) {
        case NO_TURN: break;
        case CW90:
        	tmp = gyroDataUntransformed[0];
        	gyroDataUntransformed[0] = gyroDataUntransformed[1];
        	gyroDataUntransformed[1] = -tmp;
        	break;
        case CW180:
    		gyroDataUntransformed[0] = -gyroDataUntransformed[0];
    		gyroDataUntransformed[1] = -gyroDataUntransformed[1];
    		break;
        case CW270:
    		tmp = gyroDataUntransformed[0];
    		gyroDataUntransformed[0] = -gyroDataUntransformed[1];
    		gyroDataUntransformed[1] = tmp;
    		break;
        }
    }
}

void MPU6050_Gyro_calibration(void)
{
    uint8_t i;
    int loops = 150;

    float dummy[3];

    gyroADCOffset[0] = 0;
    gyroADCOffset[1] = 0;
    gyroADCOffset[2] = 0;

    for (i = 0; i < loops; i++)
    {
        MPU6050_Gyro_get(dummy);
        gyroADCOffset[0] += gyroADC[0];
        gyroADCOffset[1] += gyroADC[1];
        gyroADCOffset[2] += gyroADC[2];
        Delay_ms(2);
    }

    gyroADCOffset[0] /= loops;
    gyroADCOffset[1] /= loops;
    gyroADCOffset[2] /= loops;

    Delay_ms(5);
}
