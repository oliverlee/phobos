#pragma once
#include "messages.pb.h"
#include <array>

namespace packet {
namespace serialize {

uint8_t encode_bicycle_pose(const BicyclePose& bp, uint8_t* buffer, uint8_t buffer_size);
bool decode_bicycle_pose(const uint8_t* buffer, BicyclePose* bp, uint8_t buffer_size);

} // namespace serialize
} // namespace packet
