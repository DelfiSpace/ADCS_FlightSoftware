/*
 * TestService.cpp
 *
 *  Created on: 6 Aug 2019
 *      Author: stefanosperett
 */

#include "TestService.h"

extern DSerial serial;
extern MB85RS fram;

bool TestService::process(PQ9Frame &command, PQ9Bus &interface, PQ9Frame &workingBuffer)
{
    if (command.getPayload()[0] == 0)
    {
        serial.println("TestService");

        if (command.getPayload()[1] == 0)
        {
            // initialize FRAM
            fram.init();

            //Read status
            unsigned char stat = fram.read_Status();
            serial.print("Value of status register: ");
            serial.print(stat, DEC);
            serial.println();

            serial.print("Ping: ");
            serial.print(fram.ping(), DEC);
            serial.println();
        } else if (command.getPayload()[1] == 1)
        {
            serial.println("Write");
            serial.print("Address: ");
            serial.print(command.getPayload()[2], DEC);
            serial.println();
            serial.print("Value: ");
            serial.print(command.getPayload()[3], DEC);
            serial.println();
            fram.write(command.getPayload()[2], &command.getPayload()[3], 1);

        } else if (command.getPayload()[1] == 2)
        {
            unsigned char v;
            fram.read(command.getPayload()[2], &v, 1);
            serial.println("Read");
            serial.print("Address: ");
            serial.print(command.getPayload()[2], DEC);
            serial.println();
            serial.print("Value: ");
            serial.print(v, DEC);
            serial.println();
        } else if (command.getPayload()[1] == 3)
        {
            serial.println("Erase all");
            fram.erase_All();
        }

        return true;
    }
    return false;
}
