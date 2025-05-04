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
    HeroHardware(src::Drivers* drivers) : drivers(drivers) {}

    //drivers
    src::Drivers* drivers;

    //motors 
    DjiMotor flywheelMotor1{drivers, MotorId::MOTOR3, CanBus::CAN_BUS2, true, "Flywheel", 0, 0};
    DjiMotor flywheelMotor2{drivers, MotorId::MOTOR5, CanBus::CAN_BUS2, false, "Flywheel", 0, 0};

    DjiMotor yawMotor{drivers, MotorId::MOTOR7, CanBus::CAN_BUS1, false, "Yaw", 0, 0};
    DjiMotor pitchMotor{drivers, MotorId::MOTOR6, CanBus::CAN_BUS2, false, "Pitch", 0, 0};

    DjiMotor indexTopMotor{drivers, MotorId::MOTOR1, CanBus::CAN_BUS2, false, "IndexerTop", 0, 0};
    DjiMotor indexBottomMotor{drivers, MotorId::MOTOR7, CanBus::CAN_BUS2, false, "IndexerBottom", 0, 0};

    DjiMotor driveMotor1{drivers, MotorId::MOTOR1, CanBus::CAN_BUS1, true, "Motor 1", 0, 0};
    DjiMotor driveMotor2{drivers, MotorId::MOTOR3, CanBus::CAN_BUS1, true, "Motor 2", 0, 0};
    DjiMotor driveMotor3{drivers, MotorId::MOTOR4, CanBus::CAN_BUS1, true, "Motor 3", 0, 0};
    DjiMotor driveMotor4{drivers, MotorId::MOTOR2, CanBus::CAN_BUS1, true, "Motor 4", 0, 0};

};

}  // namespace ThornBots
