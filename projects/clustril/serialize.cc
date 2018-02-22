#include "packet/serialize.h"
#include "simulation.pb.h"

namespace packet {
namespace serialize {

// TODO: autogenerate these template specializations
template <> const pb_field_t* message_field<pbSimulation>::type = pbSimulation_fields;

} // namespace serialize
} // namespace packet
