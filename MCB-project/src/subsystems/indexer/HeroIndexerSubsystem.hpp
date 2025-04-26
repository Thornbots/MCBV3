#pragma once
#include "IndexerSubsystem.hpp"

namespace subsystems
{

template<class T>
class HeroIndexerSubsystem : public IndexerSubsystem
{

public:
    // Additional Motor Constants (if necessary)
    tap::motor::DjiMotor* bottomIndexer;

    HeroIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index1, tap::motor::DjiMotor* index2);

    ~HeroIndexerSubsystem() {}

    void initialize() override;
    void refresh() override;
    void indexAtRate(float ballsPerSecond) override;
    void indexAtMaxRate() override;
    void setTargetMotor2RPM(int targetMotorRPM);
    bool readBreakBeam();

private:
tap::algorithms::SmoothPid indexPIDController2;
int32_t indexerVoltage2 = 0;

};

} // namespace subsystems
