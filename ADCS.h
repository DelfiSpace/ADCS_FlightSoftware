/*
 * ADCS.h
 *
 *  Created on: 23 Jul 2019
 *      Author: stefanosperett
 */

#ifndef ADCS_H_
#define ADCS_H_

#include <driverlib.h>
#include "msp.h"
#include "PQ9Bus.h"
#include "PQ9Frame.h"
#include "DWire.h"
#include "INA226.h"
#include "DSerial.h"
#include "PQ9CommandHandler.h"
#include "PingService.h"
#include "ResetService.h"
#include "HousekeepingService.h"
#include "ADCSHousekeepingService.h"
#include "ADCSTelemetryContainer.h"
#include "TMP100.h"

#define FCLOCK 48000000

#define ADCS_ADDRESS     5

#endif /* ADCS_H_ */
