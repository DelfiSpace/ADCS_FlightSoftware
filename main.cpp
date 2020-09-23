#include "ADCS.h"

// I2C bus
DWire I2Cinternal(0);
DWire BMXI2C(1);
INA226 powerBus(I2Cinternal, 0x40);
INA226 torquerX(I2Cinternal, 0x41);
INA226 torquerY(I2Cinternal, 0x42);
INA226 torquerZ(I2Cinternal, 0x43);
TMP100 temp(I2Cinternal, 0x48);
BMX055 bmx055(BMXI2C, 0x18, 0x68, 0x12);

// SPI bus
DSPI spi(3);
MB85RS fram(spi, GPIO_PORT_P1, GPIO_PIN0, MB85RS::MB85RS1MT );

// HardwareMonitor
HWMonitor hwMonitor(&fram);

// Bootloader
Bootloader bootLoader = Bootloader(fram);

// CDHS bus handler
PQ9Bus pq9bus(3, GPIO_PORT_P9, GPIO_PIN0);

// services running in the system
TestService test;
PingService ping;
ResetService reset( GPIO_PORT_P4, GPIO_PIN0, GPIO_PORT_P4, GPIO_PIN2 );

CoilService coilServ;
FRAMService framService(fram);

#ifndef SW_VERSION
SoftwareUpdateService SWupdate(fram);
#else
SoftwareUpdateService SWupdate(fram, (uint8_t*)xtr(SW_VERSION));
#endif


HousekeepingService<ADCSTelemetryContainer> hk;
Service* services[] = { &ping, &reset, &hk, &test, &SWupdate, &coilServ, &framService };

// ADCS board tasks
CommandHandler<PQ9Frame,PQ9Message> cmdHandler(pq9bus, services, 7);
PeriodicTask timerTask(1000, periodicTask);
PeriodicTask* periodicTasks[] = {&timerTask};
PeriodicTaskNotifier taskNotifier = PeriodicTaskNotifier(periodicTasks, 1);
Task* tasks[] = { &cmdHandler, &timerTask };

// system uptime
unsigned long uptime = 0;
FRAMVar<unsigned long> totalUptime;


// TODO: remove when bug in CCS has been solved
void receivedCommand(DataFrame &newFrame)
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
    uptime += 1;
    totalUptime += 1;

//    bmx055.test();
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
    unsigned short v, c;
    signed short i, t;
    unsigned char uc;
    unsigned long ul;
    //Set Telemetry:

    //HouseKeeping Header:
    tc->setStatus(Bootloader::getCurrentSlot());
    fram.read(FRAM_RESET_COUNTER + Bootloader::getCurrentSlot(), &uc, 1);
    tc->setBootCounter(uc);
    tc->setResetCause(hwMonitor.getResetStatus());
    tc->setUptime(uptime);
    tc->setTotalUptime((unsigned long) totalUptime);
    tc->setVersionNumber(2);
    tc->setMCUTemp(hwMonitor.getMCUTemp());

    // acquire board Sensors
    tc->setINAStatus(!(powerBus.getVoltage(v)) & !(powerBus.getCurrent(i)));
    tc->setVoltage(v);
    tc->setCurrent(i);
    tc->setTMPStatus(!temp.getTemperature(t));
    tc->setTemperature(t);

//    Console::log("%d", v);


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

    // initialize the ADC
    // - ADC14 and FPU Module
    // - MEM0 for internal temperature measurements
    ADCManager::initADC();

    // Initialize I2C master
    I2Cinternal.setFastMode();
    I2Cinternal.begin();

    BMXI2C.setFastMode();
    BMXI2C.begin();

    // initialize the shunt resistor
    powerBus.setShuntResistor(33);
    torquerX.setShuntResistor(40);
    torquerY.setShuntResistor(40);
    torquerZ.setShuntResistor(40);

    bmx055.init();

    // initialize temperature sensor
    temp.init();

    // Initialize SPI master
    spi.initMaster(DSPI::MODE0, DSPI::MSBFirst, 1000000);

    // Initialize fram and fram-variables
    fram.init();
    totalUptime.init(fram, FRAM_TOTAL_UPTIME);

    // initialize the console
    Console::init( 115200 );                // baud rate: 115200 bps
    pq9bus.begin(115200, ADCS_ADDRESS);     // baud rate: 115200 bps
                                            // address ADCS (5)

    //InitBootLoader!
    bootLoader.JumpSlot();

    // initialize the reset handler:
    // - prepare the watch-dog
    // - initialize the pins for the hardware watch-dog
    // - prepare the pin for power cycling the system
    reset.init();

    // initialize Task Notifier
    taskNotifier.init();

    // initialize HWMonitor readings
    hwMonitor.readResetStatus();
    hwMonitor.readCSStatus();


    // link the command handler to the PQ9 bus:
    // every time a new command is received, it will be forwarded to the command handler
    // TODO: put back the lambda function after bug in CCS has been fixed
    //pq9bus.setReceiveHandler([](PQ9Frame &newFrame){ cmdHandler.received(newFrame); });
    pq9bus.setReceiveHandler(&receivedCommand);

    // every time a command is correctly processed, call the watch-dog
    // TODO: put back the lambda function after bug in CCS has been fixed
    //cmdHandler.onValidCommand([]{ reset.kickInternalWatchDog(); });
    cmdHandler.onValidCommand(&validCmd);

    Console::log("ADCS booting...SLOT: %d", (int) Bootloader::getCurrentSlot());

    if(HAS_SW_VERSION == 1){
        Console::log("SW_VERSION: %s", (const char*)xtr(SW_VERSION));
    }

    Console::log("BMX ACC NUMBER: 0x%x", bmx055.getAccId());
    Console::log("BMX GYR NUMBER: 0x%x", bmx055.getGyroId());
    Console::log("BMX MAG NUMBER: 0x%x", bmx055.getMagId());

//    Console::log("ENABLE P10.5");
//    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P10, GPIO_PIN5);
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN5);

    TaskManager::start(tasks, 2);
}
