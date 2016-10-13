#pragma once
#include <cstdint>

namespace packet {
namespace framing {

void stuff(const uint8_t* source, uint8_t* dest, uint8_t length);
void unstuff(const uint8_t* source, uint8_t* dest, uint8_t length);

} // namespace framing
} // namespace packet
