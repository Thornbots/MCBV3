#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"

#include "drivers.hpp"

using namespace tap::motor;
using namespace tap::can;

namespace robots
{
//class for standard robot. This has hardware and subsystems that are robot specific. This means we can have multiple StandardControl for different control schemes
class HeroHardware
{
public:
    HeroHardware(src::Drivers* drivers) : drivers(drivers) {
        float sqrt2over2 = std::sqrt(2.0f) / 2.0f;
        drivers->bmi088.setOrientationQuaternion(sqrt2over2, 0, sqrt2over2, 0);
    }

    //drivers
    src::Drivers* drivers;

    //motors 
    DjiMotor flywheelMotor1{drivers, MotorId::MOTOR3, CanBus::CAN_BUS2, false, "Flywheel"};
    DjiMotor flywheelMotor2{drivers, MotorId::MOTOR5, CanBus::CAN_BUS2, true, "Flywheel"};

    DjiMotor yawMotor{drivers, MotorId::MOTOR7, CanBus::CAN_BUS1, false, "Yaw"};
    DjiMotor pitchMotor{drivers, MotorId::MOTOR6, CanBus::CAN_BUS2, false, "Pitch"};

    DjiMotor indexTopMotor{drivers, MotorId::MOTOR7, CanBus::CAN_BUS2, false, "IndexerTop"};
    DjiMotor indexBottomMotor{drivers, MotorId::MOTOR1, CanBus::CAN_BUS2, false, "IndexerBottom"};

    DjiMotor driveMotor1{drivers, MotorId::MOTOR1, CanBus::CAN_BUS1, true, "Motor 1"};
    DjiMotor driveMotor2{drivers, MotorId::MOTOR3, CanBus::CAN_BUS1, true, "Motor 2"};
    DjiMotor driveMotor3{drivers, MotorId::MOTOR4, CanBus::CAN_BUS1, true, "Motor 3"};
    DjiMotor driveMotor4{drivers, MotorId::MOTOR2, CanBus::CAN_BUS1, true, "Motor 4"};

};

}  // namespace ThornBots
