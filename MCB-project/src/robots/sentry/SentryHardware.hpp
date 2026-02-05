#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"


#include "drivers.hpp"

using namespace tap::motor;
using namespace tap::can;
namespace robots
{
//class for sentry robot. This has hardware and subsystems that are robot specific
class SentryHardware
{
public:
    SentryHardware(src::Drivers* drivers) : drivers(drivers) {
        drivers->i2c.encoder.setAngleInvertedTrue();
    }

    //drivers
    src::Drivers* drivers;

    //motors 
    DjiMotor flywheelMotor1{drivers, MotorId::MOTOR3, CanBus::CAN_BUS2, true, "TopFlywheel"};
    DjiMotor flywheelMotor2{drivers, MotorId::MOTOR2, CanBus::CAN_BUS2, false, "BottomFlywheel"};


    DjiMotor odoMotor{drivers, MotorId::MOTOR5, CanBus::CAN_BUS1, true, "Odo"};

    DjiMotor yawMotor{drivers, MotorId::MOTOR7, CanBus::CAN_BUS1, false, "Yaw"};
    DjiMotor pitchMotor{drivers, MotorId::MOTOR5, CanBus::CAN_BUS2, false, "Pitch"};

    DjiMotor indexMotor{drivers, MotorId::MOTOR6, CanBus::CAN_BUS2, true, "indexer"};

    DjiMotor driveMotor1{drivers, MotorId::MOTOR4, CanBus::CAN_BUS1, false, "Motor 4 as 1"};
    DjiMotor driveMotor2{drivers, MotorId::MOTOR1, CanBus::CAN_BUS1, false, "Motor 1 as 2"};
    DjiMotor driveMotor3{drivers, MotorId::MOTOR2, CanBus::CAN_BUS1, false, "Motor 2 as 3"};
    DjiMotor driveMotor4{drivers, MotorId::MOTOR3, CanBus::CAN_BUS1, false, "Motor 3 as 4"};


};

}  // namespace ThornBots
