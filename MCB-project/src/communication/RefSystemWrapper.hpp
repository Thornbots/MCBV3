#pragma once

#include "tap/drivers.hpp"

namespace communication {
class RefSystemWrapper {
public:
    RefSystemWrapper(tap::Drivers *drivers) : drivers(drivers) {}

    #if defined(faked_ref)
    
    #else

    #endif

private:
    tap::Drivers *drivers;
};

};  // namespace communication