#pragma once

#include "tap/communication/serial/ref_serial.hpp"

// Referee gates for autonomy. Only an RMUL 3v3 match holds robots by game stage;
// with no referee or another game type, nothing is gated.
namespace referee_match {
using RefSerial = tap::communication::serial::RefSerial;
using GameStage = tap::communication::serial::RefSerialData::Rx::GameStage;

inline bool inRmul3v3(const RefSerial& ref) {
    return ref.getRefSerialReceivingData() && ref.getGameData().gameType == tap::communication::serial::RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3;
}

// True only during an RMUL 3v3 match that is in `stage`.
inline bool rmul3v3StageIs(const RefSerial& ref, GameStage stage) { return inRmul3v3(ref) && ref.getGameData().gameStage == stage; }
}  // namespace referee_match
