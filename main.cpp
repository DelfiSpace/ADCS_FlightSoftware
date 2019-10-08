
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
ADCSHousekeepingService hk;
Service* services[] = { &ping, &reset, &hk };

// command handler, dealing with all CDHS requests and responses
PQ9CommandHandler cmdHandler(pq9bus, services, 3);

// ADCS board tasks
Task cmdTask(commandTask);
PeriodicTask timerTask(FCLOCK, periodicTask);
Task* tasks[] = { &cmdTask, &timerTask };

// system uptime
unsigned long uptime = 0;

void onReceive( PQ9Frame &newFrame )
{
    cmdHandler.received(newFrame);
    cmdTask.notify();
}

// handler used for the execution of received commands
void commandTask()
{
    if (cmdHandler.handleCommands())
    {
        // if a correct command has been received, clear the watch-dog
        // if no command is received before the watch-dog triggers,
        // the board is power-cycled
        reset.kickInternalWatchDog();
    }
}

void periodicTask()
{
    // increase the timer, this happens every second
    uptime++;

    // collect telemetry
    ADCSTelemetryContainer *tc = static_cast<ADCSTelemetryContainer*>(hk.getContainerToWrite());

    // acquire telemetry
    acquireTelemetry(tc);

    // telemetry collected, store the values and prepare for next collection
    hk.stageTelemetry();

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
    tc->setBusStatus(!powerBus.getVoltage(v));
    tc->setBusVoltage(v);
    tc->setBusStatus(!powerBus.getCurrent(i));
    tc->setBusCurrent(i);

    // measure the torquer X
    tc->setTorquerXStatus(!torquerX.getVoltage(v));
    tc->setTorquerXVoltage(v);
    tc->setTorquerXStatus(!torquerX.getCurrent(i));
    tc->setTorquerXCurrent(i);

    // measure the torquer Y
    tc->setTorquerYStatus(!torquerY.getVoltage(v));
    tc->setTorquerYVoltage(v);
    tc->setTorquerYStatus(!torquerY.getCurrent(i));
    tc->setTorquerYCurrent(i);

    // measure the torquer Z
    tc->setTorquerZStatus(!torquerZ.getVoltage(v));
    tc->setTorquerZVoltage(v);
    tc->setTorquerZStatus(!torquerZ.getCurrent(i));
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
    // prepare the pin for power cycling the system
    reset.init();

    // Initialize I2C master
    I2Cinternal.setFastMode();
    I2Cinternal.begin();

    // initialize the shunt resistor
    powerBus.setShuntResistor(0.04);
    torquerX.setShuntResistor(0.04);
    torquerY.setShuntResistor(0.04);
    torquerZ.setShuntResistor(0.04);

    temp.init(RESOLUTION_12_BIT);

    serial.begin( );                        // baud rate: 9600 bps
    pq9bus.begin(115200, ADCS_ADDRESS);     // baud rate: 115200 bps
                                            // address ADCS (5)

    // initialize the command handler: from now on, commands can be processed
    pq9bus.setReceiveHandler(&onReceive);

    serial.println("ADCS booting...");

    DelfiPQcore::startTaskManager(tasks, 2);
}
