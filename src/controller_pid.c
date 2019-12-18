
//#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
//#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_pid.h"

//#include "log.h"
#include "param.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE) //ATTITUDE_RATE = 500Hz

uint32_t tick;
static bool tiltCompensationEnabled = false;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

void controllerPidInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

/*bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}*/

void controllerPid(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{ 
	
	// controllerPid(&control, &setpoint, &sensorData, &state, tick);
	
  /*if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) // ATTITUDE_RATE = 500Hz
  {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) 
    {
       attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
      while (attitudeDesired.yaw > 180.0f)
        attitudeDesired.yaw -= 360.0f;
      while (attitudeDesired.yaw < -180.0f)
        attitudeDesired.yaw += 360.0f;
    } 
    else 
    {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }
  }*/

  if (RATE_DO_EXECUTE(POSITION_RATE, tick))  //POSITION_RATE = 100Hz
  {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick))  // ATTITUDE_RATE = 500Hz
  {
    // Switch between manual and automatic position control
    /*if (setpoint->mode.z == modeDisable) 
    {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) 
    {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }*/
		 
		attitudeDesired.yaw = 0;

		attitudeDesired.roll = 0;
		
    attitudeDesired.pitch = 0;

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    /*if (setpoint->mode.roll == modeVelocity) 
    {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) 
    {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }*/

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;
  }

 // if (tiltCompensationEnabled) // probably not enabled
 // {
  //  control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
 // }
 // else
  {
    control->thrust = actuatorThrust;
  }

if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}


