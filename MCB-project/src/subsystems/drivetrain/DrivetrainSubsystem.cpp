#include "DrivetrainSubsystem.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"

#include "DrivetrainSubsystemConstants.hpp"

namespace subsystems {
using namespace tap::communication::serial;

DrivetrainSubsystem::DrivetrainSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* motorOne, tap::motor::DjiMotor* motorTwo, tap::motor::DjiMotor* motorThree, tap::motor::DjiMotor* motorFour)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      motorArray{motorOne, motorTwo, motorThree, motorFour},
      powerLimit(DEFAULT_POWER_LIMIT),
      rotationPIDController(drivetrainPIDConfig) {}

void DrivetrainSubsystem::initialize() {
    drivers->commandScheduler.registerSubsystem(this);

    for (tap::motor::DjiMotor* m : motorArray) m->initialize();
}

// guaranteed to be called
void DrivetrainSubsystem::refresh() {
    // if (!drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getRobotData().robotPower & RefSerialData::Rx::RobotPower::CHASSIS_HAS_POWER) {
    // need to actually fix this yay
    imuAngle = (drivers->bmi088.getYaw() - 180) * PI / 180;

    if (drivers->refSerial.getRefSerialReceivingData())  // check for uart disconnected
        powerLimit = std::min((uint16_t)120, drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

    for (int i = 0; i < 4; i++) {
        motorVel[i] = motorArray[i]->getShaftRPM() * PI / 30.0f;  // in rad/s
    }
    // }
}
#if defined(drivetrain_sysid)
const float SAT_SECONDS = 5.0f, MAX_CURRENT = 1.5f, RAMP_TIME = 90.0f;
float time, current;
float motor1Vel, motor2Vel, motor3Vel, motor4Vel;
float calculateCurrent(float time) {
    if (time < 0) {
        return 0.0;
    } else if (time < SAT_SECONDS) {
        return MAX_CURRENT;
    } else if (time < SAT_SECONDS + RAMP_TIME) {
        // Linearly decrease from MAX_CURRENT to 0
        float progress = (time - SAT_SECONDS) / RAMP_TIME;
        return MAX_CURRENT * (1.0 - progress);
    } else if (time < 2 * SAT_SECONDS + RAMP_TIME) {
        return 0.0;
    } else if (time < 3 * SAT_SECONDS + RAMP_TIME) {
        return -MAX_CURRENT;
    } else if (time < 3 * SAT_SECONDS + 2 * RAMP_TIME) {
        // Linearly increase from -MAX_CURRENT to 0
        float progress = (time - (3 * SAT_SECONDS + RAMP_TIME)) / RAMP_TIME;
        return -MAX_CURRENT * (1.0 - progress);
    } else {
        return 0.0;  // Stay at 0 after the full cycle
    }
}
#endif
void DrivetrainSubsystem::setTargetTranslation(Pose2d drive, bool shouldBoost) {
    lastDrive = drive;

#if defined(drivetrain_sysid)
    current = calculateCurrent(time);
    time += 0.002f;
    for (int i = 0; i < 4; i++) motorCurrent[i] = current;
    motor1Vel = motorVel[0];
    motor2Vel = motorVel[1];
    motor3Vel = motorVel[2];
    motor4Vel = motorVel[3];
#elif defined(yaw_sysid)

    for (int i = 0; i < 4; i++) motorCurrent[i] = 0;

#else
    boost = shouldBoost && (drivers->refSerial.getRobotData().chassis.powerBuffer > 20) ? 30.0f : 0.0f;
    throttle = (drivers->refSerial.getRobotData().chassis.powerBuffer <= 10) ? 10.0f : 0.0f;
    controller.calculate(lastDrive, powerLimit + boost - throttle, imuAngle, motorVel, motorCurrent);

#endif
    for (int i = 0; i < 4; i++) {
        float adjustedCurrent = std::clamp(motorCurrent[i], -20.0f, 20.0f) * 819.2f;

        motorArray[i]->setDesiredOutput(static_cast<int32_t>(adjustedCurrent));
    }
}

// fix function
void DrivetrainSubsystem::stopMotors() {
#if defined(drivetrain_sysid)
    time = 0;
#endif

    for (int i = 0; i < 4; i++) motorCurrent[i] = 0;

    for (int i = 0; i < 4; i++) {
        float adjustedCurrent = std::clamp(motorCurrent[i], -20.0f, 20.0f) * 819.2f;

        motorArray[i]->setDesiredOutput(static_cast<int32_t>(adjustedCurrent));
    }
    rotationPIDController.reset();
}

float DrivetrainSubsystem::calculateRotationPID(float error) {
    while (error > PI) {
        error -= 2 * PI;
    }
    while (error < -PI) {
        error += 2 * PI;
    }
    rotationPIDController.runControllerDerivateError(error, 0.002f);
    return rotationPIDController.getOutput();
}
}  // namespace subsystems