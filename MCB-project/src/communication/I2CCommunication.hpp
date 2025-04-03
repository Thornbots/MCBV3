
#include "tap/board/board.hpp"

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"
#include "modm/processing/protothread.hpp"
#include "modm/processing/resumable.hpp"
#include "mt6701.hpp"

namespace communication {

using I2cSda = Board::DigitalInPinPF0;
using I2cScl = Board::DigitalInPinPF1;
using I2cMaster = I2cMaster2;

class I2CCommunication {

public:
    inline void initialize() {
        I2cMaster::connect<I2cSda::Sda, I2cScl::Scl>(I2cMaster::PullUps::Internal);
        I2cMaster::initialize<Board::SystemClock>();
    }

    void refresh(){

    }

    float getAngle(){
        return encoder.getAngle();
    }


private:

    MT6701<I2cMaster> encoder{};

};
};  // namespace communication