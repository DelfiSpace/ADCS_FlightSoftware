
#include "ADCS.h"

// I2C busses
DWire I2Cinternal(0);

// voltage / current sensors
INA226 powerBus(I2Cinternal, 0x40);
INA226 torquerX(I2Cinternal, 0x41);
INA226 torquerY(I2Cinternal, 0x42);
INA226 torquerZ(I2Cinternal, 0x43);

// temperature sensor
TMP100 temp(I2Cinternal, 0x48);

// CDHS bus handler
PQ9Bus pq9bus(3, GPIO_PORT_P9, GPIO_PIN0);

// debug console handler
DSerial serial;

// services running in the system
PingService ping;
ResetService reset( GPIO_PORT_P4, GPIO_PIN0 );
HousekeepingService<ADCSTelemetryContainer> hk;
Service* services[] = { &ping, &reset, &hk };

// ADCS board tasks
PQ9CommandHandler cmdHandler(pq9bus, services, 3);
PeriodicTask timerTask(FCLOCK, periodicTask);
Task* tasks[] = { &cmdHandler, &timerTask };

// system uptime
unsigned long uptime = 0;

// TODO: remove when bug in CCS has been solved
void kickWatchdog(PQ9Frame &newFrame)
{
    cmdHandler.received(newFrame);
}

void validCmd(void)
{
    reset.kickInternalWatchDog();
}

void periodicTask()
{
    // increase the timer, this happens every second
    uptime++;

    // collect telemetry
    hk.acquireTelemetry(acquireTelemetry);

    // refresh the watch-dog configuration to make sure that, even in case of internal
    // registers corruption, the watch-dog is capable of recovering from an error
    reset.refreshConfiguration();

    // kick hardware watch-dog after every telemetry collection happens
    reset.kickExternalWatchDog();
}

void acquireTelemetry(ADCSTelemetryContainer *tc)
{
    unsigned short v;
    signed short i, t;

    // set uptime in telemetry
    tc->setUpTime(uptime);

    // measure the power bus
    tc->setBusStatus((!powerBus.getVoltage(v)) & (!powerBus.getCurrent(i)));
    tc->setBusVoltage(v);
    tc->setBusCurrent(i);

    // measure the torquer X
    tc->setTorquerXStatus((!torquerX.getVoltage(v)) & (!torquerX.getCurrent(i)));
    tc->setTorquerXVoltage(v);
    tc->setTorquerXCurrent(i);

    // measure the torquer Y
    tc->setTorquerYStatus((!torquerY.getVoltage(v)) & (!torquerY.getCurrent(i)));
    tc->setTorquerYVoltage(v);
    tc->setTorquerYCurrent(i);

    // measure the torquer Z
    tc->setTorquerZStatus((!torquerZ.getVoltage(v)) & (!torquerZ.getCurrent(i)));
    tc->setTorquerZVoltage(v);
    tc->setTorquerZCurrent(i);

    // acquire board temperature
    tc->setTmpStatus(!temp.getTemperature(t));
    tc->setTemperature(t);
}

/**
 * main.c
 */
void main(void)
{
    // initialize the MCU:
    // - clock source
    // - clock tree
    DelfiPQcore::initMCU();

    // initialize the reset handler:
    // - prepare the watch-dog
    // - initialize the pins for the hardware watch-dog
    // - prepare the pin for power cycling the system
    reset.init();

    // Initialize I2C master
    I2Cinternal.setFastMode();
    I2Cinternal.begin();

    // initialize the shunt resistor
    powerBus.setShuntResistor(40);
    torquerX.setShuntResistor(40);
    torquerY.setShuntResistor(40);
    torquerZ.setShuntResistor(40);

    temp.init();

    serial.begin( );                        // baud rate: 9600 bps
    pq9bus.begin(115200, ADCS_ADDRESS);     // baud rate: 115200 bps
                                            // address ADCS (5)

    // link the command handler to the PQ9 bus:
    // every time a new command is received, it will be forwarded to the command handler
    // TODO: put back the lambda function after bug in CCS has been fixed
    //pq9bus.setReceiveHandler([](PQ9Frame &newFrame){ cmdHandler.received(newFrame); });
    pq9bus.setReceiveHandler(&kickWatchdog);

    // every time a command is correctly processed, call the watch-dog
    // TODO: put back the lambda function after bug in CCS has been fixed
    //cmdHandler.onValidCommand([]{ reset.kickInternalWatchDog(); });
    cmdHandler.onValidCommand(&validCmd);

    serial.println("ADCS booting...");

    TaskManager::start(tasks, 2);
}
