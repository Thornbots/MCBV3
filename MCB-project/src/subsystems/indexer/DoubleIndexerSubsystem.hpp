#pragma once
#include "IndexerSubsystem.hpp"

namespace subsystems
{

class DoubleIndexerSubsystem : public IndexerSubsystem
{

public:
    // Additional Motor Constants (if necessary)
    tap::motor::DjiMotor* motorIndexer2;

    DoubleIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index1, tap::motor::DjiMotor* index2);

    ~DoubleIndexerSubsystem() {}

    void initialize() override;
    void refresh() override;
    void stopIndex() override;
    void unjam() override;
    void indexAtRate(float ballsPerSecond) override;
    void setTargetMotorRPM(int targetMotorRPM) override;

private:
tap::algorithms::SmoothPid indexPIDController2;
int32_t indexerVoltage2 = 0;

};

} // namespace subsystems
