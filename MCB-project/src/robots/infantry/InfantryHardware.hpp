#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"

#include "drivers.hpp"

using namespace tap::motor;
using namespace tap::can;

namespace robots
{
//class for standard robot. This has hardware and subsystems that are robot specific. This means we can have multiple StandardControl for different control schemes
class InfantryHardware
{
public:
    InfantryHardware(src::Drivers* drivers) : drivers(drivers) {}

    //drivers
    src::Drivers* drivers;
    Servo servo{drivers, tap::gpio::Pwm::C1, 0.3f, 0.2f, 0.01f};

    //motors 
    DjiMotor flywheelMotor1{drivers, MotorId::MOTOR5, CanBus::CAN_BUS2, false, "Flywheel"};
    DjiMotor flywheelMotor2{drivers, MotorId::MOTOR8, CanBus::CAN_BUS2, true, "Flywheel 2"};

    DjiMotor yawMotor{drivers, MotorId::MOTOR7, CanBus::CAN_BUS1, false, "Yaw"};
    DjiMotor pitchMotor{drivers, MotorId::MOTOR6, CanBus::CAN_BUS2, false, "Pitch"};

    DjiMotor indexMotor{drivers, MotorId::MOTOR7, CanBus::CAN_BUS2, false, "Indexer2"};

    DjiMotor driveMotor1{drivers, MotorId::MOTOR1, CanBus::CAN_BUS1, false, "Motor 1"};
    DjiMotor driveMotor2{drivers, MotorId::MOTOR2, CanBus::CAN_BUS1, false, "Motor 2"};
    DjiMotor driveMotor3{drivers, MotorId::MOTOR3, CanBus::CAN_BUS1, false, "Motor 3"};
    DjiMotor driveMotor4{drivers, MotorId::MOTOR4, CanBus::CAN_BUS1, false, "Motor 4"};


};

}  // namespace ThornBots
