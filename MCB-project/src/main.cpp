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

    //things we need to check controller
    drivers->remote.initialize();
    drivers->analog.init();
    drivers->digital.init();
    drivers->leds.init();

    //if controller is on when the robot turns on, wait for it to be off.
    //This is to prevent the shredding of wires
    modm::delay_ms(1000);
    drivers->leds.set(tap::gpio::Leds::Red, true);
    int i = 0;
    while(i < 5000){
        drivers->remote.read();
        if(drivers->remote.isConnected())
            i = 0;
        else
            i++;
        modm::delay_us(10);
    }

    drivers->leds.set(tap::gpio::Leds::Blue, true);

    drivers->pwm.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->refSerial.initialize();

    drivers->i2c.initialize();
    //try waiting 9 clock pulses? 
    drivers->i2c.refresh();
    drivers->uart.initialize();


    drivers->terminalSerial.initialize(); //needs to be commented for cv to work?
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
    
    drivers->leds.set(tap::gpio::Leds::Red, false);
    drivers->bmi088.initialize(500, 0.0f, 0.0f);
    drivers->bmi088.setCalibrationSamples(2000);
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

    drivers->remote.read();
}

src::Drivers drivers;

RobotControl control{&drivers};

int main() {
    Board::initialize();
    initializeIo(&drivers);
    testvar = drivers.i2c.encoder.getRawAngle();
    testvar2 = drivers.i2c.encoder.getAngle();
    tap::buzzer::silenceBuzzer(&(drivers.pwm));

    control.initialize();

    tap::arch::PeriodicMilliTimer refreshTimer(2);
    tap::arch::MilliTimeout waitForBmi088(4000);

    bool imuIsReady = false;

    while (1) {
        // do this as fast as you can
        updateIo(&drivers);
        drivers.i2c.refresh();
        drivers.uart.updateSerial();
        testvar = drivers.i2c.encoder.getRawAngle();
        testvar2 = drivers.i2c.encoder.getAngle();

        if (refreshTimer.execute()) {
            // tap::buzzer::playNote(&(drivers.pwm), 493);

            drivers.bmi088.periodicIMUUpdate();
            drivers.bmi088.read();
            if (waitForBmi088.isExpired() && !imuIsReady) { // do everything except things that do things if IMU isn't done
                imuIsReady = true;
                drivers.leds.set(tap::gpio::Leds::Blue, false);
            }
            if(imuIsReady){
                drivers.commandScheduler.run();
                control.update();
            }
            drivers.djiMotorTxHandler.encodeAndSendCanData();

            drivers.terminalSerial.update(); //needs to be commented for cv to work?
        } 

        // prevent looping too fast
        modm::delay_us(10);
    }
    return 0;
}