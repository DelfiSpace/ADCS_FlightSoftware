/*
 * CoilService.h
 *
 *  Created on: 20 Sep 2020
 *      Author: Casper
 *
 * Description:
 */
#ifndef COILSERVICE_H_
#define COILSERVICE_H_

#include "Service.h"
#include "Console.h"

#define COILSERVICE                  74
#define COILSERVICE_NO_ERROR         0
#define COILSERVICE_UNKNOWN_CMD      55

#define COILSERVICE_ENABLE           1
#define COILSERVICE_SET_COIL_STATE   2

#define TORQ_ENABLE_PORT             GPIO_PORT_P10
#define TORQ_ENABLE_PIN              GPIO_PIN5

#define TORQ_X_1_PORT     GPIO_PORT_P7
#define TORQ_X_2_PORT     GPIO_PORT_P7
#define TORQ_X_3_PORT     GPIO_PORT_P7
#define TORQ_X_1_PIN      GPIO_PIN6
#define TORQ_X_2_PIN      GPIO_PIN5
#define TORQ_X_3_PIN      GPIO_PIN4

#define TORQ_Y_1_PORT     GPIO_PORT_P8
#define TORQ_Y_2_PORT     GPIO_PORT_P8
#define TORQ_Y_3_PORT     GPIO_PORT_P7
#define TORQ_Y_1_PIN      GPIO_PIN1
#define TORQ_Y_2_PIN      GPIO_PIN0
#define TORQ_Y_3_PIN      GPIO_PIN7

#define TORQ_Z_1_PORT     GPIO_PORT_P3
#define TORQ_Z_2_PORT     GPIO_PORT_P3
#define TORQ_Z_3_PORT     GPIO_PORT_P3
#define TORQ_Z_1_PIN      GPIO_PIN2
#define TORQ_Z_2_PIN      GPIO_PIN1
#define TORQ_Z_3_PIN      GPIO_PIN0


typedef enum CoilState {Coast = 0, Brake = 1, Reverse = 2, Forward = 3} CoilState;
typedef enum CoilSelect {TorqX = 0, TorqY = 1, TorqZ = 2} CoilSelect;


class CoilService: public Service
{
private:
    bool torqEnabled = false;
    uint8_t torqXState = 0;
    uint8_t torqYState = 0;
    uint8_t torqZState = 0;

public:
    CoilService();

    bool process( DataMessage &command, DataMessage &workingBbuffer );

    void enableCoils(bool enabled);
    void setStatus(uint8_t targetCoil, uint8_t targetState);
};



#endif /* COILSERVICE_H_ */
