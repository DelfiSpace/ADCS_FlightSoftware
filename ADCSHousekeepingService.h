/*
 * ADCSTelemetryService.h
 *
 *  Created on: 4 Aug 2019
 *      Author: stefanosperett
 */

#ifndef ADCSHOUSEKEEPINGSERVICE_H_
#define ADCSHOUSEKEEPINGSERVICE_H_

#include "ADCSTelemetryContainer.h"
#include "HousekeepingService.h"

class ADCSHousekeepingService: public HousekeepingService
{
protected:
    ADCSTelemetryContainer telemetryContainer[2];

public:
    virtual TelemetryContainer* getContainerToRead()
    {
        return &(telemetryContainer[(telemetryIndex + 1) % 2]);
    };

    virtual TelemetryContainer* getContainerToWrite()
    {
        return &(telemetryContainer[telemetryIndex]);
    };
};
#endif /* ADCSHOUSEKEEPINGSERVICE_H_ */
