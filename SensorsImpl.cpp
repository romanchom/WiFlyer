
#include "EspWait.hpp"
#include "I2CBus.hpp"

#include <icarus/MPU9255_impl.hpp>
#include <icarus/BMP180_impl.hpp>
#include <icarus/AK8963_impl.hpp>
#include <icarus/I2CRegisterBank.hpp>

template class icarus::MPU9255<icarus::I2CRegisterBank<I2CBus>>;
template class icarus::BMP180<icarus::I2CRegisterBank<I2CBus>>;
template class icarus::AK8963<EspWait, icarus::I2CRegisterBank<I2CBus>>;

