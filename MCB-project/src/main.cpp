#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "robots/RobotControl.hpp"

#include "drivers.hpp"

uint16_t testvar = 0;
float testvar2 = 0.0f;
// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers) {
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();
    drivers->refSerial.initialize();
    drivers->i2c.initialize();
    modm::delay_ms(2);
    drivers->i2c.refresh();
    modm::delay_ms(20);
    drivers->i2c.refresh();


    // drivers->cvBoard.initialize();
    // drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
    drivers->bmi088.initialize(500, 0.1f, 0.0f);
    drivers->bmi088.requestCalibration();

}

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers) {
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

int main() {
    Board::initialize();
    initializeIo(&drivers);
    testvar = drivers.i2c.encoder.getRawAngle();
    testvar2 = drivers.i2c.encoder.getAngle();
    control.initialize();
    tap::buzzer::silenceBuzzer(&(drivers.pwm));

    tap::arch::PeriodicMilliTimer refreshTimer(2);

    while (1) {
        // do this as fast as you can
        updateIo(&drivers);

        if (refreshTimer.execute()) {
            // tap::buzzer::playNote(&(drivers.pwm), 493);

            drivers.bmi088.periodicIMUUpdate();
            drivers.bmi088.read();
            drivers.i2c.refresh();
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