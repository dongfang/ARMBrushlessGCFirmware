/*
 * definitions.h
 *
 *  Created on: Aug 14, 2013
 *      Author: Paul Phillips
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

typedef enum
{
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    NUMAXES // is 3
} tAxisEnum;


typedef enum
{
    ROLL,
    PITCH,
    YAW,
    EULER // is 3
} tEulerEnum;

#define R2D 57.3F
#define D2R 0.01745329

#endif /* DEFINITIONS_H_ */
