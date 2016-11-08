#include "packet/serialize.h"
#include "messages.pb.h"

namespace packet {
namespace serialize {

// TODO: autogenerate these template specializations
template <> const pb_field_t* message_field<ClustrilMessage>::type = ClustrilMessage_fields;
template <> const pb_field_t* message_field<BicycleState>::type = BicycleState_fields;
template <> const pb_field_t* message_field<BicycleInput>::type = BicycleInput_fields;
template <> const pb_field_t* message_field<BicyclePose>::type = BicyclePose_fields;
template <> const pb_field_t* message_field<Sensors>::type = Sensors_fields;
template <> const pb_field_t* message_field<Actuators>::type = Actuators_fields;
template <> const pb_field_t* message_field<SymmetricStateMatrix>::type = SymmetricStateMatrix_fields;
template <> const pb_field_t* message_field<SymmetricOutputMatrix>::type = SymmetricOutputMatrix_fields;
template <> const pb_field_t* message_field<KalmanGainMatrix>::type = KalmanGainMatrix_fields;
template <> const pb_field_t* message_field<Kalman>::type = Kalman_fields;
template <> const pb_field_t* message_field<SecondOrderMatrix>::type = SecondOrderMatrix_fields;
template <> const pb_field_t* message_field<StateMatrix>::type = StateMatrix_fields;
template <> const pb_field_t* message_field<InputMatrix>::type = InputMatrix_fields;
template <> const pb_field_t* message_field<OutputMatrix>::type = OutputMatrix_fields;
template <> const pb_field_t* message_field<FeedthroughMatrix>::type = FeedthroughMatrix_fields;
template <> const pb_field_t* message_field<BicycleModel>::type = BicycleModel_fields;

} // namespace serialize
} // namespace packet
