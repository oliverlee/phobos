#include "packet/serialize.h"
#include "clustril.pb.h"

namespace packet {
namespace serialize {

// TODO: autogenerate these template specializations
template <> const pb_field_t* message_field<ClustrilMessage>::type = ClustrilMessage_fields;
template <> const pb_field_t* message_field<BicycleStateMessage>::type = BicycleStateMessage_fields;
template <> const pb_field_t* message_field<BicycleInputMessage>::type = BicycleInputMessage_fields;
template <> const pb_field_t* message_field<BicyclePoseMessage>::type = BicyclePoseMessage_fields;
template <> const pb_field_t* message_field<SensorMessage>::type = SensorMessage_fields;
template <> const pb_field_t* message_field<ActuatorMessage>::type = ActuatorMessage_fields;
template <> const pb_field_t* message_field<SymmetricStateMatrixMessage>::type = SymmetricStateMatrixMessage_fields;
template <> const pb_field_t* message_field<SymmetricOutputMatrixMessage>::type = SymmetricOutputMatrixMessage_fields;
template <> const pb_field_t* message_field<KalmanGainMatrixMessage>::type = KalmanGainMatrixMessage_fields;
template <> const pb_field_t* message_field<BicycleKalmanMessage>::type = BicycleKalmanMessage_fields;
template <> const pb_field_t* message_field<SecondOrderMatrixMessage>::type = SecondOrderMatrixMessage_fields;
template <> const pb_field_t* message_field<StateMatrixMessage>::type = StateMatrixMessage_fields;
template <> const pb_field_t* message_field<InputMatrixMessage>::type = InputMatrixMessage_fields;
template <> const pb_field_t* message_field<OutputMatrixMessage>::type = OutputMatrixMessage_fields;
template <> const pb_field_t* message_field<FeedthroughMatrixMessage>::type = FeedthroughMatrixMessage_fields;
template <> const pb_field_t* message_field<BicycleModelMessage>::type = BicycleModelMessage_fields;

} // namespace serialize
} // namespace packet
