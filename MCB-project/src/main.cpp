#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "robots/RobotControl.hpp"

#include "drivers.hpp"

const int RECALIBRATION_THRESHOLD_TIME = 15; //s, when there are fewer than this many seconds remaining in the game stage, run scheduled recalibrations
const int AUTORECALIBRATION_THRESHOLD_TIME = 60*3000; //ms, set to 3 min. if the controller has been off for this long, recalibrate

src::Drivers drivers;
RobotControl control{&drivers};
tap::arch::MilliTimeout autoRecalibrateTimeout;



float adctest;

void checkAutoRecalibrate() {
    if(drivers.remote.isConnected()){
        //we have a controller, stop counting
        autoRecalibrateTimeout.stop(); 
    } else {
        //no controller, start timer if needed, if timer is done, we should execute recalibration
        if(autoRecalibrateTimeout.isStopped()){
            autoRecalibrateTimeout.restart(AUTORECALIBRATION_THRESHOLD_TIME);
        } 
        if(autoRecalibrateTimeout.execute()){
            autoRecalibrateTimeout.restart(AUTORECALIBRATION_THRESHOLD_TIME);
            drivers.recal.forceCalibration();
        }
    }
}


//if ctrl r (not ctrl shift r) and it is time
bool shouldExecuteScheduledRecalibration() {
    RefSerialData::Rx::GameStage currentGameStage = drivers.refSerial.getGameData().gameStage;
    return drivers.recal.isRequestingRecalibration() &&
           drivers.refSerial.getRefSerialReceivingData() &&
           (currentGameStage == RefSerial::Rx::GameStage::SETUP || currentGameStage == RefSerial::Rx::GameStage::INITIALIZATION) &&
           drivers.refSerial.getGameData().stageTimeRemaining < RECALIBRATION_THRESHOLD_TIME;
}

void adc1_pa6_init() {
    // 1. Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // 2. Configure PA6 as analog (MODER = 11)
    GPIOA->MODER |= (3U << (6 * 2));
    GPIOA->PUPDR &= ~(3U << (6 * 2)); // no pull-up/down

    // 3. ADC prescaler (common safe value: PCLK2 / 8)
    ADC->CCR &= ~(3U << 16);
    ADC->CCR |=  (1U << 16); // ADCPRE = 01 -> PCLK2/4 (or use 10 for /6, 11 for /8)

    // 4. ADC configuration
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    // Right alignment
    ADC1->CR2 &= ~ADC_CR2_ALIGN;

    // 5. Single conversion mode (default)
    ADC1->CR2 &= ~ADC_CR2_CONT;

    // 6. Sampling time for channel 6 (PA6 -> IN6 in SMPR2)
    // choose 144 cycles for stability
    ADC1->SMPR2 &= ~(7U << (6 * 3));
    ADC1->SMPR2 |=  (7U << (6 * 3));

    // 7. Regular sequence length = 1 conversion
    ADC1->SQR1 &= ~(0xF << 20);

    // 8. Channel 6 as first conversion in sequence
    ADC1->SQR3 = 6;

    // 9. Enable ADC
    // Power on ADC
    ADC1->CR2 |= ADC_CR2_ADON;

    // mandatory stabilization delay
    for (volatile int i = 0; i < 10000; i++);

    // trigger a dummy conversion (VERY important)
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    (void)ADC1->DR;
}

uint16_t adc1_pa6_read() {
    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait for EOC
    while (!(ADC1->SR & ADC_SR_EOC));

    // Read result clears EOC
    return (uint16_t)ADC1->DR;
}



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
    modm::delay_ms(3000);
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


    // drivers->terminalSerial.initialize(); //interferes with jetson because uses the same uart port. Previously believed to be necessary for ui to work, turns out it isn't
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();



    drivers->leds.set(tap::gpio::Leds::Red, false);
    drivers->bmi088.initialize(1000, 0.0f, 0.000f);
    drivers->bmi088.setTargetTemperature(35.0f);
    drivers->bmi088.setCalibrationSamples(4000);
    adc1_pa6_init();
        //ADC testing
        //modm::platform::Adc1::connect<modm::platform::GpioA6::In6>();
        //modm::platform::Adc1::initialize<
          //  Board::SystemClock,      
         //   modm::MHz(21)            //prescaler of 4 hopefully
        //>();
        //modm::platform::Adc1::setChannel(
        //   modm::platform::Adc1::Channel::Channel6,
         //   modm::platform::Adc1::SampleTime::Cycles56 // or your preferred sample time
        //);
        //
    drivers->executeCalibration();
    drivers->recal.setIsFirstCalibrating();
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

int main() {
    Board::initialize();
    initializeIo(&drivers);
    tap::buzzer::silenceBuzzer(&(drivers.pwm));

    control.initialize();

    tap::arch::PeriodicMilliTimer refreshTimer(1);
    tap::arch::MilliTimeout waitForRobotToStopMoving{};
    waitForRobotToStopMoving.stop();

    while (1) {
        // do this as fast as you can
        updateIo(&drivers);
        drivers.i2c.refresh();
        drivers.uart.updateSerial();

        if (refreshTimer.execute()) {
            
            adctest = adc1_pa6_read();;
            //adctest = drivers.readPa6Adc();
            // tap::buzzer::playNote(&(drivers.pwm), 493);
            
            checkAutoRecalibrate(); //would make drivers.recal.isForcingRecalibration() return true
            bool goingToRecalibrate = drivers.recal.isForcingRecalibration() || shouldExecuteScheduledRecalibration();
            if(goingToRecalibrate){
                control.stopForImuRecal();
                drivers.recal.setIsWaiting();
                drivers.leds.set(tap::gpio::Leds::Blue, true);
                drivers.leds.set(tap::gpio::Leds::Green, true);
                drivers.leds.set(tap::gpio::Leds::Red, false);
                waitForRobotToStopMoving.restart(6000);
            }

            if(waitForRobotToStopMoving.timeRemaining()<1000){
                drivers.recal.setJustBeforeSecondCalibrating();
            }

            if(waitForRobotToStopMoving.isExpired()){
                waitForRobotToStopMoving.stop();
                drivers.recal.setIsSecondCalibrating();
                drivers.leds.set(tap::gpio::Leds::Green, false);
                drivers.executeCalibration();
            }


            drivers.bmi088.periodicIMUUpdate();
            drivers.bmi088.read();

            //only turn blue led off once in case someone elsewhere wants it on
            //if I think I am calibrating and it is done
            if (drivers.recal.getIsCalibrating() && drivers.bmi088.getImuState()==tap::communication::sensors::imu::AbstractIMU::ImuState::IMU_CALIBRATED) { // do everything except things that do things if IMU isn't done
                drivers.recal.setIsDoneCalibrating();
                drivers.leds.set(tap::gpio::Leds::Blue, false);
                if(drivers.recal.isAfterSecondCalibration())
                    control.resumeAfterImuRecal(); //when it turned on, it flipped 180
            }
            if(drivers.recal.getIsImuReady()){
                drivers.commandScheduler.run();
                control.update();
            }

            drivers.djiMotorTxHandler.encodeAndSendCanData();

            // drivers.terminalSerial.update();
        }
        // prevent looping too fast
        modm::delay_us(5);
    }

    return 0;
}


