/**
 ******************************************************************************
 *
 * @file       vtolpathfollower.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      This module compared @ref PositionActuatl to @ref ActiveWaypoint 
 * and sets @ref AttitudeDesired.  It only does this when the FlightMode field
 * of @ref ManualControlCommand is Auto.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/**
 * Input object: ActiveWaypoint
 * Input object: PositionActual
 * Input object: ManualControlCommand
 * Output object: AttitudeDesired
 *
 * This module will periodically update the value of the AttitudeDesired object.
 *
 * The module executes in its own thread in this example.
 *
 * Modules have no API, all communication to other modules is done through UAVObjects.
 * However modules may use the API exposed by shared libraries.
 * See the OpenPilot wiki for more details.
 * http://www.openpilot.org/OpenPilot_Application_Architecture
 *
 */

#include "openpilot.h"
#include "paths.h"

#include "vtolpathfollower.h"
#include "accels.h"
#include "attitudeactual.h"
#include "hwsettings.h"
#include "pathdesired.h"        // object that will be updated by the module
#include "positionactual.h"
#include "manualcontrol.h"
#include "flightstatus.h"
#include "gpsvelocity.h"
#include "gpsposition.h"
#include "vtolpathfollowersettings.h"
#include "nedaccel.h"
#include "nedposition.h"
#include "stabilizationdesired.h"
#include "stabilizationsettings.h"
#include "systemsettings.h"
#include "velocitydesired.h"
#include "velocityactual.h"
#include "CoordinateConversions.h"

// Private constants
#define MAX_QUEUE_SIZE 4
#define STACK_SIZE_BYTES 1548
#define TASK_PRIORITY (tskIDLE_PRIORITY+2)
#define F_PI 3.14159265358979323846f

// Private types

// Private variables
static xTaskHandle pathfollowerTaskHandle;
static PathDesiredData pathDesired;
static VtolPathFollowerSettingsData guidanceSettings;

// Private functions
static void vtolPathFollowerTask(void *parameters);
static void SettingsUpdatedCb(UAVObjEvent * ev);
static void updateNedAccel();
static void updatePathVelocity();
static void updateEndpointVelocity();
static void updateVtolDesiredAttitude();
static float bound(float val, float min, float max);
static bool vtolpathfollower_enabled;

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t VtolPathFollowerStart()
{
	if (vtolpathfollower_enabled) {
		// Start main task
		xTaskCreate(vtolPathFollowerTask, (signed char *)"VtolPathFollower", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &pathfollowerTaskHandle);
		TaskMonitorAdd(TASKINFO_RUNNING_PATHFOLLOWER, pathfollowerTaskHandle);
	}

	return 0;
}

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t VtolPathFollowerInitialize()
{
	uint8_t optionalModules[HWSETTINGS_OPTIONALMODULES_NUMELEM];
	
	HwSettingsOptionalModulesGet(optionalModules);
	
	if (optionalModules[HWSETTINGS_OPTIONALMODULES_VTOLPATHFOLLOWER] == HWSETTINGS_OPTIONALMODULES_ENABLED) {
		VtolPathFollowerSettingsInitialize();
		NedAccelInitialize();
		PathDesiredInitialize();
		VelocityDesiredInitialize();
		vtolpathfollower_enabled = true;
	} else {
		vtolpathfollower_enabled = false;
	}
	
	return 0;
}

MODULE_INITCALL(VtolPathFollowerInitialize, VtolPathFollowerStart)

static float northVelIntegral = 0;
static float eastVelIntegral = 0;
static float downVelIntegral = 0;

static float northPosIntegral = 0;
static float eastPosIntegral = 0;
static float downPosIntegral = 0;

static float throttleOffset = 0;
/**
 * Module thread, should not return.
 */
static void vtolPathFollowerTask(void *parameters)
{
	SystemSettingsData systemSettings;
	FlightStatusData flightStatus;

	portTickType lastUpdateTime;
	
	VtolPathFollowerSettingsConnectCallback(SettingsUpdatedCb);
	PathDesiredConnectCallback(SettingsUpdatedCb);
	
	VtolPathFollowerSettingsGet(&guidanceSettings);
	PathDesiredGet(&pathDesired);
	
	// Main task loop
	lastUpdateTime = xTaskGetTickCount();
	while (1) {

		// Conditions when this runs:
		// 1. Must have VTOL type airframe
		// 2. Flight mode is PositionHold and PathDesired.Mode is Endpoint  OR
		//    FlightMode is PathPlanner and PathDesired.Mode is Endpoint or Path

		SystemSettingsGet(&systemSettings);
		if ( (systemSettings.AirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_VTOL) &&
			(systemSettings.AirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_QUADP) &&
			(systemSettings.AirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_QUADP) &&
			(systemSettings.AirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_QUADX) &&
			(systemSettings.AirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_HEXA) &&
			(systemSettings.AirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_HEXAX) &&
			(systemSettings.AirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_HEXACOAX) &&
			(systemSettings.AirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_OCTO) &&
			(systemSettings.AirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_OCTOV) &&
			(systemSettings.AirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_OCTOCOAXP) &&
			(systemSettings.AirframeType != SYSTEMSETTINGS_AIRFRAMETYPE_TRI) )
		{
			AlarmsSet(SYSTEMALARMS_ALARM_GUIDANCE,SYSTEMALARMS_ALARM_WARNING);
			vTaskDelay(1000);
			continue;
		}

		// Continue collecting data if not enough time
		vTaskDelayUntil(&lastUpdateTime, guidanceSettings.UpdatePeriod / portTICK_RATE_MS);

		// Convert the accels into the NED frame
		updateNedAccel();
		
		FlightStatusGet(&flightStatus);

		// Check the combinations of flightmode and pathdesired mode
		switch(flightStatus.FlightMode) {
			case FLIGHTSTATUS_FLIGHTMODE_RTH:
				if (pathDesired.Mode == PATHDESIRED_MODE_ENDPOINT) {
					updateEndpointVelocity();
					updateVtolDesiredAttitude();
				} else {
					AlarmsSet(SYSTEMALARMS_ALARM_GUIDANCE,SYSTEMALARMS_ALARM_ERROR);
				}
				break;
			case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
				if (pathDesired.Mode == PATHDESIRED_MODE_ENDPOINT) {
					updateEndpointVelocity();
					updateVtolDesiredAttitude();
				} else {
					AlarmsSet(SYSTEMALARMS_ALARM_GUIDANCE,SYSTEMALARMS_ALARM_ERROR);
				}
				break;
			case FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER:
				if (pathDesired.Mode == PATHDESIRED_MODE_ENDPOINT) {
					updateEndpointVelocity();
					updateVtolDesiredAttitude();
				} else if (pathDesired.Mode == PATHDESIRED_MODE_PATH) {
					updatePathVelocity();
					updateVtolDesiredAttitude();
				} else {
					AlarmsSet(SYSTEMALARMS_ALARM_GUIDANCE,SYSTEMALARMS_ALARM_ERROR);
					break;
				}
				break;
			default:
				// Be cleaner and get rid of global variables
				northVelIntegral = 0;
				eastVelIntegral = 0;
				downVelIntegral = 0;
				northPosIntegral = 0;
				eastPosIntegral = 0;
				downPosIntegral = 0;

				// Track throttle before engaging this mode.  Cheap system ident
				StabilizationDesiredData stabDesired;
				StabilizationDesiredGet(&stabDesired);
				throttleOffset = stabDesired.Throttle;

				break;
		}
	}
}

/**
 * Compute desired velocity from the current position and path
 *
 * Takes in @ref PositionActual and compares it to @ref PathDesired 
 * and computes @ref VelocityDesired
 */
static void updatePathVelocity()
{
	float dT = guidanceSettings.UpdatePeriod / 1000.0f;
	float downCommand;

	PositionActualData positionActual;
	PositionActualGet(&positionActual);
	
	float cur[3] = {positionActual.North, positionActual.East, positionActual.Down};
	struct path_status progress;
	
	path_progress(pathDesired.Start, pathDesired.End, cur, &progress);
	
	float groundspeed = pathDesired.StartingVelocity + 
	    (pathDesired.EndingVelocity - pathDesired.StartingVelocity) * progress.fractional_progress;
	if(progress.fractional_progress > 1)
		groundspeed = 0;
	
	VelocityDesiredData velocityDesired;
	velocityDesired.North = progress.path_direction[0] * groundspeed;
	velocityDesired.East = progress.path_direction[1] * groundspeed;
	
	float error_speed = progress.error * guidanceSettings.HorizontalPosPI[VTOLPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_KP];
	float correction_velocity[2] = {progress.correction_direction[0] * error_speed, 
	    progress.correction_direction[1] * error_speed};
	
	float total_vel = sqrtf(powf(correction_velocity[0],2) + powf(correction_velocity[1],2));
	float scale = 1;
	if(total_vel > guidanceSettings.HorizontalVelMax)
		scale = guidanceSettings.HorizontalVelMax / total_vel;

	velocityDesired.North += progress.correction_direction[0] * error_speed * scale;
	velocityDesired.East += progress.correction_direction[1] * error_speed * scale;
	
	float altitudeSetpoint = pathDesired.Start[2] + (pathDesired.End[2] - pathDesired.Start[2]) *
	    bound(progress.fractional_progress,0,1);

	float downError = altitudeSetpoint - positionActual.Down;
	downPosIntegral = bound(downPosIntegral + downError * dT * guidanceSettings.VerticalPosPI[VTOLPATHFOLLOWERSETTINGS_VERTICALPOSPI_KI],
							-guidanceSettings.VerticalPosPI[VTOLPATHFOLLOWERSETTINGS_VERTICALPOSPI_ILIMIT],
							guidanceSettings.VerticalPosPI[VTOLPATHFOLLOWERSETTINGS_VERTICALPOSPI_ILIMIT]);
	downCommand = (downError * guidanceSettings.VerticalPosPI[VTOLPATHFOLLOWERSETTINGS_VERTICALPOSPI_KP] + downPosIntegral);
	velocityDesired.Down = bound(downCommand,
								 -guidanceSettings.VerticalVelMax,
								 guidanceSettings.VerticalVelMax);

	VelocityDesiredSet(&velocityDesired);
}

/**
 * Compute desired velocity from the current position
 *
 * Takes in @ref PositionActual and compares it to @ref PositionDesired 
 * and computes @ref VelocityDesired
 */
void updateEndpointVelocity()
{
	float dT = guidanceSettings.UpdatePeriod / 1000.0f;

	PositionActualData positionActual;
	VelocityDesiredData velocityDesired;
	
	PositionActualGet(&positionActual);
	VelocityDesiredGet(&velocityDesired);
	
	float northError;
	float eastError;
	float downError;
	float northCommand;
	float eastCommand;
	float downCommand;
	
	float northPos = 0, eastPos = 0, downPos = 0;
	switch (guidanceSettings.PositionSource) {
		case VTOLPATHFOLLOWERSETTINGS_POSITIONSOURCE_EKF:
			northPos = positionActual.North;
			eastPos = positionActual.East;
			downPos = positionActual.Down;
			break;
		case VTOLPATHFOLLOWERSETTINGS_POSITIONSOURCE_GPSPOS:
		{
			NEDPositionData nedPosition;
			NEDPositionGet(&nedPosition);
			northPos = nedPosition.North;
			eastPos = nedPosition.East;
			downPos = nedPosition.Down;
		}
			break;
		default:
			PIOS_Assert(0);
			break;
	}

	// Compute desired north command
	northError = pathDesired.End[PATHDESIRED_END_NORTH] - northPos;
	northPosIntegral = bound(northPosIntegral + northError * dT * guidanceSettings.HorizontalPosPI[VTOLPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_KI], 
			      -guidanceSettings.HorizontalPosPI[VTOLPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_ILIMIT],
			      guidanceSettings.HorizontalPosPI[VTOLPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_ILIMIT]);
	northCommand = (northError * guidanceSettings.HorizontalPosPI[VTOLPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_KP] +
			northPosIntegral);
	
	eastError = pathDesired.End[PATHDESIRED_END_EAST] - eastPos;
	eastPosIntegral = bound(eastPosIntegral + eastError * dT * guidanceSettings.HorizontalPosPI[VTOLPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_KI], 
				 -guidanceSettings.HorizontalPosPI[VTOLPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_ILIMIT],
				 guidanceSettings.HorizontalPosPI[VTOLPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_ILIMIT]);
	eastCommand = (eastError * guidanceSettings.HorizontalPosPI[VTOLPATHFOLLOWERSETTINGS_HORIZONTALPOSPI_KP] +
		       eastPosIntegral);
	
	// Limit the maximum velocity
	float total_vel = sqrtf(powf(northCommand,2) + powf(eastCommand,2));
	float scale = 1;
	if(total_vel > guidanceSettings.HorizontalVelMax)
		scale = guidanceSettings.HorizontalVelMax / total_vel;

	velocityDesired.North = northCommand * scale;
	velocityDesired.East = eastCommand * scale;

	downError = pathDesired.End[PATHDESIRED_END_DOWN] - downPos;
	downPosIntegral = bound(downPosIntegral + downError * dT * guidanceSettings.VerticalPosPI[VTOLPATHFOLLOWERSETTINGS_VERTICALPOSPI_KI], 
				-guidanceSettings.VerticalPosPI[VTOLPATHFOLLOWERSETTINGS_VERTICALPOSPI_ILIMIT],
				guidanceSettings.VerticalPosPI[VTOLPATHFOLLOWERSETTINGS_VERTICALPOSPI_ILIMIT]);
	downCommand = (downError * guidanceSettings.VerticalPosPI[VTOLPATHFOLLOWERSETTINGS_VERTICALPOSPI_KP] + downPosIntegral);
	velocityDesired.Down = bound(downCommand,
				     -guidanceSettings.VerticalVelMax, 
				     guidanceSettings.VerticalVelMax);
	
	VelocityDesiredSet(&velocityDesired);	
}

/**
 * Compute desired attitude from the desired velocity
 *
 * Takes in @ref NedActual which has the acceleration in the 
 * NED frame as the feedback term and then compares the 
 * @ref VelocityActual against the @ref VelocityDesired
 */
static void updateVtolDesiredAttitude()
{
	float dT = guidanceSettings.UpdatePeriod / 1000.0f;

	VelocityDesiredData velocityDesired;
	VelocityActualData velocityActual;
	StabilizationDesiredData stabDesired;
	AttitudeActualData attitudeActual;
	NedAccelData nedAccel;
	VtolPathFollowerSettingsData guidanceSettings;
	StabilizationSettingsData stabSettings;
	SystemSettingsData systemSettings;

	float northError;
	float northCommand;
	
	float eastError;
	float eastCommand;

	float downError;
	float downCommand;
		
	SystemSettingsGet(&systemSettings);
	VtolPathFollowerSettingsGet(&guidanceSettings);
	
	VelocityActualGet(&velocityActual);
	VelocityDesiredGet(&velocityDesired);
	StabilizationDesiredGet(&stabDesired);
	VelocityDesiredGet(&velocityDesired);
	AttitudeActualGet(&attitudeActual);
	StabilizationSettingsGet(&stabSettings);
	NedAccelGet(&nedAccel);
	
	float northVel = 0, eastVel = 0, downVel = 0;
	switch (guidanceSettings.VelocitySource) {
		case VTOLPATHFOLLOWERSETTINGS_VELOCITYSOURCE_EKF:
			northVel = velocityActual.North;
			eastVel = velocityActual.East;
			downVel = velocityActual.Down;
			break;
		case VTOLPATHFOLLOWERSETTINGS_VELOCITYSOURCE_NEDVEL:
		{
			GPSVelocityData gpsVelocity;
			GPSVelocityGet(&gpsVelocity);
			northVel = gpsVelocity.North;
			eastVel = gpsVelocity.East;
			downVel = gpsVelocity.Down;
		}
			break;
		case VTOLPATHFOLLOWERSETTINGS_VELOCITYSOURCE_GPSPOS:
		{
			GPSPositionData gpsPosition;
			GPSPositionGet(&gpsPosition);
			northVel = gpsPosition.Groundspeed * cosf(gpsPosition.Heading * F_PI / 180.0f);
			eastVel = gpsPosition.Groundspeed * sinf(gpsPosition.Heading * F_PI / 180.0f);
			downVel = velocityActual.Down;
		}
			break;
		default:
			PIOS_Assert(0);
			break;
	}
	
	// Testing code - refactor into manual control command
	ManualControlCommandData manualControlData;
	ManualControlCommandGet(&manualControlData);
	stabDesired.Yaw = stabSettings.MaximumRate[STABILIZATIONSETTINGS_MAXIMUMRATE_YAW] * manualControlData.Yaw;	
	
	// Compute desired north command
	northError = velocityDesired.North - northVel;
	northVelIntegral = bound(northVelIntegral + northError * dT * guidanceSettings.HorizontalVelPID[VTOLPATHFOLLOWERSETTINGS_HORIZONTALVELPID_KI], 
			      -guidanceSettings.HorizontalVelPID[VTOLPATHFOLLOWERSETTINGS_HORIZONTALVELPID_ILIMIT],
			      guidanceSettings.HorizontalVelPID[VTOLPATHFOLLOWERSETTINGS_HORIZONTALVELPID_ILIMIT]);
	northCommand = (northError * guidanceSettings.HorizontalVelPID[VTOLPATHFOLLOWERSETTINGS_HORIZONTALVELPID_KP] +
			northVelIntegral -
			nedAccel.North * guidanceSettings.HorizontalVelPID[VTOLPATHFOLLOWERSETTINGS_HORIZONTALVELPID_KD] +
			velocityDesired.North * guidanceSettings.VelocityFeedforward);
	
	// Compute desired east command
	eastError = velocityDesired.East - eastVel;
	eastVelIntegral = bound(eastVelIntegral + eastError * dT * guidanceSettings.HorizontalVelPID[VTOLPATHFOLLOWERSETTINGS_HORIZONTALVELPID_KI], 
			     -guidanceSettings.HorizontalVelPID[VTOLPATHFOLLOWERSETTINGS_HORIZONTALVELPID_ILIMIT],
			     guidanceSettings.HorizontalVelPID[VTOLPATHFOLLOWERSETTINGS_HORIZONTALVELPID_ILIMIT]);
	eastCommand = (eastError * guidanceSettings.HorizontalVelPID[VTOLPATHFOLLOWERSETTINGS_HORIZONTALVELPID_KP] + 
		       eastVelIntegral - 
		       nedAccel.East * guidanceSettings.HorizontalVelPID[VTOLPATHFOLLOWERSETTINGS_HORIZONTALVELPID_KD] +
			   velocityDesired.East * guidanceSettings.VelocityFeedforward);
	
	// Compute desired down command
	downError = velocityDesired.Down - downVel;
	// Must flip this sign 
	downError = -downError;
	downVelIntegral = bound(downVelIntegral + downError * dT * guidanceSettings.VerticalVelPID[VTOLPATHFOLLOWERSETTINGS_VERTICALVELPID_KI], 
			      -guidanceSettings.VerticalVelPID[VTOLPATHFOLLOWERSETTINGS_VERTICALVELPID_ILIMIT],
			      guidanceSettings.VerticalVelPID[VTOLPATHFOLLOWERSETTINGS_VERTICALVELPID_ILIMIT]);	
	downCommand = (downError * guidanceSettings.VerticalVelPID[VTOLPATHFOLLOWERSETTINGS_VERTICALVELPID_KP] +
		       downVelIntegral -
		       nedAccel.Down * guidanceSettings.VerticalVelPID[VTOLPATHFOLLOWERSETTINGS_VERTICALVELPID_KD]);
	
	stabDesired.Throttle = bound(downCommand + throttleOffset, 0, 1);
	
	// Project the north and east command signals into the pitch and roll based on yaw.  For this to behave well the
	// craft should move similarly for 5 deg roll versus 5 deg pitch
	stabDesired.Pitch = bound(-northCommand * cosf(attitudeActual.Yaw * M_PI / 180) + 
				      -eastCommand * sinf(attitudeActual.Yaw * M_PI / 180),
				      -guidanceSettings.MaxRollPitch, guidanceSettings.MaxRollPitch);
	stabDesired.Roll = bound(-northCommand * sinf(attitudeActual.Yaw * M_PI / 180) + 
				     eastCommand * cosf(attitudeActual.Yaw * M_PI / 180),
				     -guidanceSettings.MaxRollPitch, guidanceSettings.MaxRollPitch);
	
	if(guidanceSettings.ThrottleControl == VTOLPATHFOLLOWERSETTINGS_THROTTLECONTROL_FALSE) {
		// For now override throttle with manual control.  Disable at your risk, quad goes to China.
		ManualControlCommandData manualControl;
		ManualControlCommandGet(&manualControl);
		stabDesired.Throttle = manualControl.Throttle;
	}
	
	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] = STABILIZATIONDESIRED_STABILIZATIONMODE_RATE;
	
	StabilizationDesiredSet(&stabDesired);
}

/**
 * Keep a running filtered version of the acceleration in the NED frame
 */
static void updateNedAccel()
{
	float accel[3];
	float q[4];
	float Rbe[3][3];
	float accel_ned[3];

	// Collect downsampled attitude data
	AccelsData accels;
	AccelsGet(&accels);		
	accel[0] = accels.x;
	accel[1] = accels.y;
	accel[2] = accels.z;
	
	//rotate avg accels into earth frame and store it
	AttitudeActualData attitudeActual;
	AttitudeActualGet(&attitudeActual);
	q[0]=attitudeActual.q1;
	q[1]=attitudeActual.q2;
	q[2]=attitudeActual.q3;
	q[3]=attitudeActual.q4;
	Quaternion2R(q, Rbe);
	for (uint8_t i=0; i<3; i++){
		accel_ned[i]=0;
		for (uint8_t j=0; j<3; j++)
			accel_ned[i] += Rbe[j][i]*accel[j];
	}
	accel_ned[2] += 9.81f;
	
	NedAccelData accelData;
	NedAccelGet(&accelData);
	accelData.North = accel_ned[0];
	accelData.East = accel_ned[1];
	accelData.Down = accel_ned[2];
	NedAccelSet(&accelData);
}

/**
 * Bound input value between limits
 */
static float bound(float val, float min, float max)
{
	if (val < min) {
		val = min;
	} else if (val > max) {
		val = max;
	}
	return val;
}

static void SettingsUpdatedCb(UAVObjEvent * ev)
{
	VtolPathFollowerSettingsGet(&guidanceSettings);
	PathDesiredGet(&pathDesired);
}

