#include "messageutil.h"
#include "gitsha1.h"
#include "ch.h"

namespace message {

void set_build_config(pbBuildConfig* pb) {
#if defined(PROJECT_FLIMNAP)
    pb->project = pbProject.FLIMNAP;
#endif

    std::memcpy(pb->gitsha1.f, g_GITSHA1, sizeof(pb->gitsha1.f)*sizeof(pb->gitsha1.f[0]));

// The CMake project defines the following if the build type is NOT release
// add_definitions("-DOSAL_DBG_ENABLE_ASSERTS=true -DOSAL_DBG_ENABLE_CHECKS=true")
// which can be used to determine build type
    pb->build_type =
#if defined(NDEBUG)
        OSAL_DBG_ENABLE_ASSERTS ? pbBuildType_RELWITHDEBINFO : pbBuildType_RELEASE;
#else
            pbBuildType_DEBUG;
#endif

    if (CH_CFG_ST_RESOLUTION == 16) {
        pb->ch_st_resolution = pbSystemTickResolution_BITS_16;
    } else if (CH_CFG_ST_RESOLUTION == 32) {
        pb->ch_st_resolution = pbSystemTickResolution_BITS_32;
    }

    pb->ch_st_frequency = CH_CFG_ST_FREQUENCY;

    pb->ch_rtc_frequency = STM32_SYSCLK;
}

void set_state_matrix(pbStateMatrix* pb, const model::Bicycle::state_matrix_t& m) {
    impl::set_matrix(pb, m);
}

void set_input_matrix(pbInputMatrix* pb, const model::Bicycle::input_matrix_t& m) {
    impl::set_matrix(pb, m);
}

void set_output_matrix(pbOutputMatrix* pb, const model::Bicycle::output_matrix_t& m) {
    impl::set_matrix(pb, m);
}

void set_feedthrough_matrix(pbFeedthroughMatrix* pb, const model::Bicycle::feedthrough_matrix_t& m) {
    impl::set_matrix(pb, m);
}

void set_second_order_matrix(pbSecondOrderMatrix* pb, const model::Bicycle::second_order_matrix_t& m) {
    impl::set_matrix(pb, m);
}

void set_model_state(pbModelState* pb, const model::Bicycle::state_t& m) {
    impl::set_matrix(pb, m);
}

void set_model_input(pbModelInput* pb, const model::Bicycle::input_t& m) {
    impl::set_matrix(pb, m);
}

void set_model_auxiliary_state(pbModelAuxiliaryState* pb, const model::Bicycle::auxiliary_state_t& m) {
    impl::set_matrix(pb, m);
}

void set_model_state_space(pbModelStateSpace* pb, const model::Bicycle& bicycle) {
    impl::set_matrix(&pb->A, bicycle.A());
    impl::set_matrix(&pb->B, bicycle.B());
    impl::set_matrix(&pb->C, bicycle.C());
    impl::set_matrix(&pb->D, bicycle.D());
}

} // namespace message
