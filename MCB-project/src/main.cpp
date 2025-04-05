#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

#include "robots/RobotControl.hpp"

#include "drivers.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();
    drivers->refSerial.initialize();
    // drivers->cvBoard.initialize();
    // drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
    drivers->bmi088.initialize(500, 0.1f, 0.0f);
    drivers->bmi088.requestRecalibration();
}

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    // drivers->cvBoard.updateSerial();
    drivers->remote.read();
}

src::Drivers drivers;

RobotControl control{&drivers};

int main()
{

    Board::initialize();
    initializeIo(&drivers);
    control.initialize();   
    tap::buzzer::silenceBuzzer(&(drivers.pwm));

    tap::arch::PeriodicMilliTimer refreshTimer(2);

    while (1)
    {
        // do this as fast as you can
        updateIo(&drivers);

        if (refreshTimer.execute())
        {
            //tap::buzzer::playNote(&(drivers.pwm), 493);

            drivers.bmi088.periodicIMUUpdate();
            control.update();
            drivers.commandScheduler.run();
            drivers.djiMotorTxHandler.encodeAndSendCanData();
            // drivers.terminalSerial.update();
        }

        // prevent looping too fast
        modm::delay_us(10);
    }
    return 0;
}