
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
ADCSHousekeepingService hk;
Service* services[] = { &ping, &hk };

// command handler, dealing with all CDHS requests and responses
PQ9CommandHandler cmdHandler(pq9bus, services, 2);

/**
 * main.c
 */
void main(void)
{
    // Disabling the Watchdog timer
    MAP_WDT_A_holdTimer( );

    // Configuring pins for HF XTAL
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Starting HFXT in non-bypass mode without a timeout. Before we start
    // we have to change VCORE to 1 to support the 48MHz frequency
    MAP_CS_setExternalClockSourceFrequency(0, FCLOCK);
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    MAP_CS_startHFXT(false);

    // Configure clocks that we need
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_2);

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

    serial.println("Hello World");

    int counter = 0;
    while(true)
    {
        cmdHandler.commandLoop();

        // hack to simulate timer to acquire telemetry approximately once per second
        if (counter >= 1400000)
        {
            counter = 0;
            serial.println("Acquiring telemetry....");

            ADCSTelemetryContainer *tc = static_cast<ADCSTelemetryContainer*>(hk.getContainerToWrite());

            unsigned short v;
            signed short i, t;

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
        }

        counter++;
    }
}
