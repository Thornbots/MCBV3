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
        drivers->i2c.encoder.setAngleInvertedTrue();
        drivers->bmi088.setMountingTransform(imuTransform);
    }

    

    tap::algorithms::CMSISMat<3, 1> translation{{0,0,0}};
    tap::algorithms::CMSISMat<3, 3> rotation{{0,0,-1, 0,1,0, 1,0,0}};
    tap::algorithms::transforms::Transform imuTransform{translation, rotation};

    //drivers
    src::Drivers* drivers;

    //motors 
    DjiMotor flywheelMotor1{drivers, MotorId::MOTOR8, CanBus::CAN_BUS2, false, "Flywheel1"};
    DjiMotor flywheelMotor2{drivers, MotorId::MOTOR7, CanBus::CAN_BUS2, true, "Flywheel2"};

    DjiMotor yawMotor{drivers, MotorId::MOTOR5, CanBus::CAN_BUS1, false, "Yaw"};
    DjiMotor pitchMotor{drivers, MotorId::MOTOR6, CanBus::CAN_BUS2, false, "Pitch"};

    DjiMotor indexTopMotor{drivers, MotorId::MOTOR2, CanBus::CAN_BUS2, true, "IndexerTop"};
    DjiMotor indexBottomMotor{drivers, MotorId::MOTOR6, CanBus::CAN_BUS1, true, "IndexerBottom"};

    DjiMotor driveMotor1{drivers, MotorId::MOTOR1, CanBus::CAN_BUS1, false, "Motor 1"};
    DjiMotor driveMotor2{drivers, MotorId::MOTOR2, CanBus::CAN_BUS1, false, "Motor 2"};
    DjiMotor driveMotor3{drivers, MotorId::MOTOR3, CanBus::CAN_BUS1, false, "Motor 3"};
    DjiMotor driveMotor4{drivers, MotorId::MOTOR4, CanBus::CAN_BUS1, false, "Motor 4"};


};

}  // namespace ThornBots
