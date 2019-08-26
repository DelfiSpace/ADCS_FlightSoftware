
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

// system uptime
unsigned long uptime = 0;
volatile int counter = 0;

void timerHandler(void)
{
    MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE);
    counter++;
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

    // init the reset handler:
    // - prepare the watchdog
    // - initialize the pins for the hardware watchdog
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
    cmdHandler.init();

    // Configuring Timer32 to FCLOCK (1s) of MCLK in periodic mode
    MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
            TIMER32_PERIODIC_MODE);
    MAP_Timer32_registerInterrupt(TIMER32_0_INTERRUPT, &timerHandler);
    MAP_Timer32_setCount(TIMER32_0_BASE, FCLOCK);
    MAP_Timer32_startTimer(TIMER32_0_BASE, false);


    serial.println("Hello World");

    while(true)
    {
        if (cmdHandler.commandLoop())
        {
            // if a correct command has been received, clear the watchdog
            reset.kickInternalWatchDog();
        }

        // hack to simulate timer to acquire telemetry approximately once per second
        if (counter != 0)
        {
            uptime ++;
            counter = 0;

            ADCSTelemetryContainer *tc = static_cast<ADCSTelemetryContainer*>(hk.getContainerToWrite());

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

            // telemetry collected, store the values and prepare for next collection
            hk.stageTelemetry();

            reset.kickExternalWatchDog();
        }

        //counter++;
        //MAP_PCM_gotoLPM0();
    }
}
