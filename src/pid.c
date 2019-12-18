/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * pid.c - implementation of the PID regulator
 */

#include "pid.h"
//#include "num.h"
#include <math.h>
#include <float.h>


#include <stdint.h>
#define IIR_SHIFT         8

#ifndef M_PI_F
#define M_PI_F   (3.14159265358979323846f)
#define M_1_PI_F (0.31830988618379067154f)
#define M_PI_2_F (1.57079632679f)
#endif



void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
float lpf2pApply(lpf2pData* lpfData, float sample);
float lpf2pReset(lpf2pData* lpfData, float sample);

float constrain(float value, const float minVal, const float maxVal);




void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter)
{
  pid->error         = 0;
  pid->prevError     = 0;
  pid->integ         = 0;
  pid->deriv         = 0;
  pid->desired       = desired;
  pid->kp            = kp;
  pid->ki            = ki;
  pid->kd            = kd;
  pid->iLimit        = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->outputLimit   = DEFAULT_PID_OUTPUT_LIMIT;
  pid->dt            = dt;
  pid->enableDFilter = enableDFilter;
  /*if (pid->enableDFilter)
  {
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
  }*/
}



float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output = 0.0f;
    float deriv;
	
    if (updateError)  //true
    {
        pid->error = pid->desired - measured;
    }

    pid->outP = pid->kp * pid->error;
    output += pid->outP;

    deriv = (pid->error - pid->prevError) / pid->dt;
    if (pid->enableDFilter)
    {
      pid->deriv = lpf2pApply(&pid->dFilter, deriv);
    } else {
      pid->deriv = deriv;
    }
    if (isnan(pid->deriv)) {
      pid->deriv = 0;
    }
    pid->outD = pid->kd * pid->deriv;
    output += pid->outD;

    pid->integ += pid->error * pid->dt;

    // Constrain the integral (unless the iLimit is zero)
    if(pid->iLimit != 0)
    {
    	pid->integ = constrain(pid->integ, -pid->iLimit, pid->iLimit);
    }

    pid->outI = pid->ki * pid->integ;
    output += pid->outI;

    // Constrain the total PID output (unless the outputLimit is zero)
    if(pid->outputLimit != 0)
    {
      output = constrain(output, -pid->outputLimit, pid->outputLimit);
    }


    pid->prevError = pid->error;

    return output;
}


void pidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->iLimit = limit;
}


void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}



void pidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
  pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
  return pid->desired;
}

bool pidIsActive(PidObject* pid)
{
  bool isActive = true;

  if (pid->kp < 0.0001f && pid->ki < 0.0001f && pid->kd < 0.0001f)
  {
    isActive = false;
  }

  return isActive;
}

void pidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}
void pidSetDt(PidObject* pid, const float dt) {
    pid->dt = dt;
}


/**
 * 2-Pole low pass filter
 */
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
  //if (lpfData == NULL || cutoff_freq <= 0.0f) {
    //return;
  //}

  lpf2pSetCutoffFreq(lpfData, sample_freq, cutoff_freq);
}

void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
  float fr = sample_freq/cutoff_freq;
  float ohm = tanf(M_PI_F/fr);
  float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm;
  lpfData->b0 = ohm*ohm/c;
  lpfData->b1 = 2.0f*lpfData->b0;
  lpfData->b2 = lpfData->b0;
  lpfData->a1 = 2.0f*(ohm*ohm-1.0f)/c;
  lpfData->a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
  lpfData->delay_element_1 = 0.0f;
  lpfData->delay_element_2 = 0.0f;
}

float lpf2pApply(lpf2pData* lpfData, float sample)
{
	float output;
  float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
  if (!isfinite(delay_element_0)) {
    // don't allow bad values to propigate via the filter
    delay_element_0 = sample;
  }

  output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

  lpfData->delay_element_2 = lpfData->delay_element_1;
  lpfData->delay_element_1 = delay_element_0;
  return output;
}

float lpf2pReset(lpf2pData* lpfData, float sample)
{
  float dval = sample / (lpfData->b0 + lpfData->b1 + lpfData->b2);
  lpfData->delay_element_1 = dval;
  lpfData->delay_element_2 = dval;
  return lpf2pApply(lpfData, sample);
}

float constrain(float value, const float minVal, const float maxVal)
{
  return fminf(maxVal, fmaxf(minVal,value));
}
