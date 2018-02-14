#include "packet/serialize.h"
#include "txrx.pb.h"

namespace packet {
namespace serialize {

// The only protobuf message we want to serialize and transmit is pbTxMaster
template <> const pb_field_t* message_field<pbTxMaster>::type = pbTxMaster_fields;

} // namespace serialize
} // namespace packet
