/*
 * BurnService.cpp
 *
 *  Created on: 23 Jul 2020
 *      Author: Casper
 */

#include "CoilService.h"


CoilService::CoilService()
{
    //set GPIO settings

    //Enable
    MAP_GPIO_setOutputLowOnPin(TORQ_ENABLE_PORT, TORQ_ENABLE_PIN);
    MAP_GPIO_setAsOutputPin(TORQ_ENABLE_PORT, TORQ_ENABLE_PIN);

    //TorqX:
    MAP_GPIO_setOutputLowOnPin(TORQ_X_1_PORT, TORQ_X_1_PIN);
    MAP_GPIO_setOutputLowOnPin(TORQ_X_2_PORT, TORQ_X_2_PIN);
    MAP_GPIO_setOutputLowOnPin(TORQ_X_3_PORT, TORQ_X_3_PIN);
    MAP_GPIO_setAsOutputPin(TORQ_X_1_PORT, TORQ_X_1_PIN);
    MAP_GPIO_setAsOutputPin(TORQ_X_2_PORT, TORQ_X_2_PIN);
    MAP_GPIO_setAsOutputPin(TORQ_X_3_PORT, TORQ_X_3_PIN);

    //TorqY:
    MAP_GPIO_setOutputLowOnPin(TORQ_Y_1_PORT, TORQ_Y_1_PIN);
    MAP_GPIO_setOutputLowOnPin(TORQ_Y_2_PORT, TORQ_Y_2_PIN);
    MAP_GPIO_setOutputLowOnPin(TORQ_Y_3_PORT, TORQ_Y_3_PIN);
    MAP_GPIO_setAsOutputPin(TORQ_Y_1_PORT, TORQ_Y_1_PIN);
    MAP_GPIO_setAsOutputPin(TORQ_Y_2_PORT, TORQ_Y_2_PIN);
    MAP_GPIO_setAsOutputPin(TORQ_Y_3_PORT, TORQ_Y_3_PIN);

    //TorqX:
    MAP_GPIO_setOutputLowOnPin(TORQ_Z_1_PORT, TORQ_Z_1_PIN);
    MAP_GPIO_setOutputLowOnPin(TORQ_Z_2_PORT, TORQ_Z_2_PIN);
    MAP_GPIO_setOutputLowOnPin(TORQ_Z_3_PORT, TORQ_Z_3_PIN);
    MAP_GPIO_setAsOutputPin(TORQ_Z_1_PORT, TORQ_Z_1_PIN);
    MAP_GPIO_setAsOutputPin(TORQ_Z_2_PORT, TORQ_Z_2_PIN);
    MAP_GPIO_setAsOutputPin(TORQ_Z_3_PORT, TORQ_Z_3_PIN);

    //set all in break mode
    setStatus(CoilSelect::TorqX,CoilState::Brake);
    setStatus(CoilSelect::TorqY,CoilState::Brake);
    setStatus(CoilSelect::TorqZ,CoilState::Brake);
}


bool CoilService::process( DataMessage &command, DataMessage &workingBuffer )
{
    if(command.getService() == COILSERVICE)
    {
        workingBuffer.setService(COILSERVICE);
        workingBuffer.setMessageType(SERVICE_RESPONSE_REPLY);

        Console::log("CoilService!");
        switch(command.getDataPayload()[0])
        {
        case COILSERVICE_ENABLE:
            if(command.getPayloadSize() == 2)
            {
                Console::log("CoilService: Enable: %s", command.getDataPayload()[1] ? "TRUE" : "FALSE");
                enableCoils(command.getDataPayload()[1]);
                workingBuffer.getDataPayload()[0] = COILSERVICE_NO_ERROR;
                workingBuffer.setPayloadSize(1);
            }
            else
            {
                //unknown command
                Console::log("CoilService: InvalidCommand");
                workingBuffer.getDataPayload()[0] = COILSERVICE_UNKNOWN_CMD;
                workingBuffer.setPayloadSize(1);
            }
            break;
        case COILSERVICE_SET_COIL_STATE:
            if(command.getPayloadSize() == 3)
            {
                Console::log("CoilService targetCoil: %d, targetState: %d", command.getDataPayload()[1], command.getDataPayload()[2]);
                setStatus(command.getDataPayload()[1],command.getDataPayload()[1]);
                workingBuffer.getDataPayload()[0] = COILSERVICE_NO_ERROR;
                workingBuffer.setPayloadSize(1);
            }
            else
            {
                //unknown command
                Console::log("CoilService: InvalidCommand");
                workingBuffer.getDataPayload()[0] = COILSERVICE_UNKNOWN_CMD;
                workingBuffer.setPayloadSize(1);
            }
            break;
        default:
            Console::log("CoilService: InvalidCommand");
            workingBuffer.getDataPayload()[0] = command.getDataPayload()[0];
            workingBuffer.getDataPayload()[1] = COILSERVICE_UNKNOWN_CMD;
            workingBuffer.setPayloadSize(2);
            break;
        }
        return true;
    }
    else
    {
        return false;
    }
};

void CoilService::enableCoils(bool enabled){
    if(enabled){
        this->torqEnabled = true;
        MAP_GPIO_setOutputHighOnPin(TORQ_ENABLE_PORT, TORQ_ENABLE_PIN);

    }else{
        this->torqEnabled = false;
        MAP_GPIO_setOutputLowOnPin(TORQ_ENABLE_PORT, TORQ_ENABLE_PIN);
    }
}

void CoilService::setStatus(uint8_t targetCoil, uint8_t targetState){
    switch(targetCoil){
    case CoilSelect::TorqX:
        Console::log("X COIL");
        switch(targetState){
        case CoilState::Coast:
            torqXState = targetState;
            Console::log("COAST STATE");
            //nsleep low, rest is ignored
            MAP_GPIO_setOutputLowOnPin(TORQ_X_3_PORT, TORQ_X_3_PIN);
            break;
        case CoilState::Brake:
            torqXState = targetState;
            Console::log("BRAKE STATE");
            //nsleep HIGH and enable LOW
            MAP_GPIO_setOutputHighOnPin(TORQ_X_3_PORT, TORQ_X_3_PIN);
            MAP_GPIO_setOutputLowOnPin(TORQ_X_1_PORT, TORQ_X_1_PIN);
            break;
        case CoilState::Reverse:
            torqXState = targetState;
            Console::log("REVERSE STATE");
            //nsleep, enable and polarity HIGH
            MAP_GPIO_setOutputHighOnPin(TORQ_X_1_PORT, TORQ_X_1_PIN);
            MAP_GPIO_setOutputHighOnPin(TORQ_X_2_PORT, TORQ_X_2_PIN);
            MAP_GPIO_setOutputHighOnPin(TORQ_X_3_PORT, TORQ_X_3_PIN);
            break;
        case CoilState::Forward:
            torqXState = targetState;
            Console::log("FORWARD STATE");
            //nsleep, enable HIGH and polarity LOW
            MAP_GPIO_setOutputHighOnPin(TORQ_X_1_PORT, TORQ_X_1_PIN);
            MAP_GPIO_setOutputLowOnPin(TORQ_X_2_PORT, TORQ_X_2_PIN);
            MAP_GPIO_setOutputHighOnPin(TORQ_X_3_PORT, TORQ_X_3_PIN);
            break;
        default:
            Console::log("INVALID STATE");
            break;
        }
        break;
    case CoilSelect::TorqY:
        Console::log("Y COIL");

        switch(targetState){
        case CoilState::Coast:
            torqYState = targetState;
            Console::log("COAST STATE");
            //nsleep low, rest is ignored
            MAP_GPIO_setOutputLowOnPin(TORQ_Y_3_PORT, TORQ_Y_3_PIN);
            break;
        case CoilState::Brake:
            torqYState = targetState;
            Console::log("BRAKE STATE");
            //nsleep HIGH and enable LOW
            MAP_GPIO_setOutputHighOnPin(TORQ_Y_3_PORT, TORQ_Y_3_PIN);
            MAP_GPIO_setOutputLowOnPin(TORQ_Y_1_PORT, TORQ_Y_1_PIN);
            break;
        case CoilState::Reverse:
            torqYState = targetState;
            Console::log("REVERSE STATE");
            //nsleep, enable and polarity HIGH
            MAP_GPIO_setOutputHighOnPin(TORQ_Y_1_PORT, TORQ_Y_1_PIN);
            MAP_GPIO_setOutputHighOnPin(TORQ_Y_2_PORT, TORQ_Y_2_PIN);
            MAP_GPIO_setOutputHighOnPin(TORQ_Y_3_PORT, TORQ_Y_3_PIN);
            break;
        case CoilState::Forward:
            torqYState = targetState;
            Console::log("FORWARD STATE");
            //nsleep, enable HIGH and polarity LOW
            MAP_GPIO_setOutputHighOnPin(TORQ_Y_1_PORT, TORQ_Y_1_PIN);
            MAP_GPIO_setOutputLowOnPin(TORQ_Y_2_PORT, TORQ_Y_2_PIN);
            MAP_GPIO_setOutputHighOnPin(TORQ_Y_3_PORT, TORQ_Y_3_PIN);
            break;
        default:
            Console::log("INVALID STATE");
            break;
        }
        break;
    case CoilSelect::TorqZ:
        Console::log("Z COIL");
        switch(targetState){
        case CoilState::Coast:
            torqZState = targetState;
            Console::log("COAST STATE");
            //nsleep low, rest is ignored
            MAP_GPIO_setOutputLowOnPin(TORQ_Z_3_PORT, TORQ_Z_3_PIN);
            break;
        case CoilState::Brake:
            torqZState = targetState;
            Console::log("BRAKE STATE");
            //nsleep HIGH and enable LOW
            MAP_GPIO_setOutputHighOnPin(TORQ_Z_3_PORT, TORQ_Z_3_PIN);
            MAP_GPIO_setOutputLowOnPin(TORQ_Z_1_PORT, TORQ_Z_1_PIN);
            break;
        case CoilState::Reverse:
            torqZState = targetState;
            Console::log("REVERSE STATE");
            //nsleep, enable and polarity HIGH
            MAP_GPIO_setOutputHighOnPin(TORQ_Z_1_PORT, TORQ_Z_1_PIN);
            MAP_GPIO_setOutputHighOnPin(TORQ_Z_2_PORT, TORQ_Z_2_PIN);
            MAP_GPIO_setOutputHighOnPin(TORQ_Z_3_PORT, TORQ_Z_3_PIN);
            break;
        case CoilState::Forward:
            torqZState = targetState;
            Console::log("FORWARD STATE");
            //nsleep, enable HIGH and polarity LOW
            MAP_GPIO_setOutputHighOnPin(TORQ_Z_1_PORT, TORQ_Z_1_PIN);
            MAP_GPIO_setOutputLowOnPin(TORQ_Z_2_PORT, TORQ_Z_2_PIN);
            MAP_GPIO_setOutputHighOnPin(TORQ_Z_3_PORT, TORQ_Z_3_PIN);
            break;
        default:
            Console::log("INVALID STATE");
            break;
        }
        break;
    default:
        Console::log("INVALID COIL");
        break;
    }
}




