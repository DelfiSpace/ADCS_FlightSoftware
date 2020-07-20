/*
 * ADCS.h
 *
 *  Created on: 23 Jul 2019
 *      Author: stefanosperett
 */

#ifndef ADCS_H_
#define ADCS_H_

#include <AlgorithmService.h>
#include <driverlib.h>
#include "SLOT_SELECT.h"
#include "SoftwareUpdateService.h"
#include "Bootloader.h"
#include "msp.h"
#include "DelfiPQcore.h"
#include "PQ9Bus.h"
#include "PQ9Frame.h"
#include "PQ9Message.h"
#include "DWire.h"
#include "INA226.h"
#include "Console.h"
#include "CommandHandler.h"
#include "PingService.h"
#include "ResetService.h"
#include "Task.h"
#include "PeriodicTask.h"
#include "TaskManager.h"
#include "HousekeepingService.h"
#include "ADCSTelemetryContainer.h"
#include "TMP100.h"
#include "DSPI.h"
#include "MB85RS.h"
#include "TestService.h"
#include "PeriodicTaskNotifier.h"
#include "HWMonitor.h"
#include "ADCManager.h"

#define FCLOCK 48000000

#define ADCS_ADDRESS     5

// callback functions
void acquireTelemetry(ADCSTelemetryContainer *tc);
void periodicTask();

#endif /* ADCS_H_ */
