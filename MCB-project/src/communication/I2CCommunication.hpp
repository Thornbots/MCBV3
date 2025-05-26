#pragma once
#include "tap/board/board.hpp"

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"
#include "modm/processing/protothread.hpp"
#include "modm/processing/resumable.hpp"

#include "mt6701.hpp"
#include "LocalizationPico.hpp"

#include <modm/math/units.hpp>

namespace communication {

class I2CCommunication {
public:
    inline void initialize() {
        I2cMaster2::connect<Board::DigitalInPinPF0::Sda, Board::DigitalInPinPF1::Scl>(I2cMaster2::PullUps::Internal);
        I2cMaster2::initialize<Board::SystemClock, 400'000>();
    }

    void refresh() { 
        encoder.run(); 
        // odom.run();
    }

    MT6701<I2cMaster2> encoder{};
    LocalizationPico<I2cMaster2> odom{};

private:
};
};  // namespace communication