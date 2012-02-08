/**
 ******************************************************************************
 *
 * @file       guidance.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
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
#include "altitudeholdsettings.h"
#include "altitudeholddesired.h"	// object that will be updated by the module
#include "positionactual.h"
#include "stabilizationdesired.h"

// Private constants
#define MAX_QUEUE_SIZE 1
#define STACK_SIZE_BYTES 1024
#define TASK_PRIORITY (tskIDLE_PRIORITY+2)
// Private types

// Private variables
static xTaskHandle altitudeHoldTaskHandle;
static xQueueHandle queue;

// Private functions
static void altitudeHoldTask(void *parameters);
static void SettingsUpdatedCb(UAVObjEvent * ev);

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AltitudeHoldStart()
{
	// Start main task
	xTaskCreate(altitudeHoldTask, (signed char *)"AltitudeHold", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &altitudeHoldTaskHandle);
//	TaskMonitorAdd(TASKINFO_RUNNING_GUIDANCE, guidanceTaskHandle);

	return 0;
}

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AltitudeHoldInitialize()
{
	AltitudeHoldSettingsInitialize();
	AltitudeHoldDesiredInitialize();

	// Create object queue
	queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
	
	// Listen for updates.
	AltitudeHoldDesiredConnectQueue(queue);
	
	AltitudeHoldSettingsConnectCallback(&SettingsUpdatedCb);
	
	return 0;
}
MODULE_INITCALL(AltitudeHoldInitialize, AltitudeHoldStart)

static float throttleIntegral = 0;

/**
 * Module thread, should not return.
 */
static void altitudeHoldTask(void *parameters)
{
	AltitudeHoldSettingsData altitudeHoldSettings;
	AltitudeHoldDesiredData altitudeHoldDesired;
	PositionActualData positionActual;
	StabilizationDesiredData stabilizationDesired;
			
	portTickType thisTime;
	portTickType lastSysTime;
	UAVObjEvent ev;
	
	// Force update of the settings
	SettingsUpdatedCb(&ev);
	
	// Main task loop
	lastSysTime = xTaskGetTickCount();
	while (1) {

		// Wait until the AttitudeRaw object is updated, if a timeout then go to failsafe
		if ( xQueueReceive(queue, &ev, 100 / portTICK_RATE_MS) != pdTRUE )
		{
			// Todo: Add alarm if it should be running
			throttleIntegral = 0;
			continue;
		} else {
			PositionActualGet(&positionActual);
			StabilizationDesiredGet(&stabilizationDesired);
			float dT;
			
			thisTime = xTaskGetTickCount();
			if(thisTime > lastSysTime) // reuse dt in case of wraparound
				dT = (thisTime - lastSysTime) / portTICK_RATE_MS / 1000.0f;
			lastSysTime = thisTime;

			static float altitude;
			const float altitudeTau = 0.1;
			
			
			// Flipping sign on error since altitude is "down"
			float error = - (altitudeHoldDesired.Down - positionActual.Down);
			static float 
			throttleIntegral += error * altitudeHoldSettings.Ki * dT * 1000;
			if(throttleIntegral > altitudeHoldSettings.ILimit)
				throttleIntegral = altitudeHoldSettings.ILimit;
			else if (throttleIntegral < 0)
				throttleIntegral = 0;
			stabilizationDesired.Throttle = error * altitudeHoldSettings.Kp + throttleIntegral;
			if(stabilizationDesired.Throttle > 1)
				stabilizationDesired.Throttle = 1;
			else if (stabilizationDesired.Throttle < 0)
				stabilizationDesired.Throttle = 0;
				
			stabilizationDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
			stabilizationDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
			stabilizationDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] = STABILIZATIONDESIRED_STABILIZATIONMODE_AXISLOCK;		
			stabilizationDesired.Roll = altitudeHoldDesired.Roll;
			stabilizationDesired.Pitch = altitudeHoldDesired.Pitch;
			stabilizationDesired.Yaw = altitudeHoldDesired.Yaw;
			StabilizationDesiredSet(&stabilizationDesired);
		}
	
	}
}

static void SettingsUpdatedCb(UAVObjEvent * ev)
{
	AltitudeHoldDesiredGet(&altitudeHoldDesired);
	AltitudeHoldSettingsGet(&altitudeHoldSettings);

	const float fakeDt = 0.0025;
	if(settings.GyroTau < 0.0001)
		gyro_alpha = 0;   // not trusting this to resolve to 0
	else
		gyro_alpha = expf(-fakeDt  / settings.GyroTau);

}