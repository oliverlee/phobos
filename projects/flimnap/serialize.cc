#include "packet/serialize.h"
#include "pose.pb.h"

namespace packet {
namespace serialize {

// TODO: autogenerate these template specializations
template <> const pb_field_t* message_field<BicyclePoseMessage>::type = BicyclePoseMessage_fields;

} // namespace serialize
} // namespace packet
