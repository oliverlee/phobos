#include "packet/serialize.h"
#include "txrx.pb.h"

namespace packet {
namespace serialize {

// The only protobuf message we want to serialize and transmit is pbTxPackage
template <> const pb_field_t* message_field<pbTxPackage>::type = pbTxPackage_fields;

} // namespace serialize
} // namespace packet
