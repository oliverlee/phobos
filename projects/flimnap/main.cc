/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

#include "analog.h"
#include "encoder.h"
#include "encoderfoaw.h"
#include "simulation.pb.h"
#include "messageutil.h"
#include "filter/movingaverage.h"
#include "foaw.h"

#include "blink.h"
#include "saconfig.h"
#include "utility.h"
#include "sautility.h"

#include "parameters.h"

#include <array>
#include <type_traits>

#include "simbicycle.h"
#include "transmitter.h"
#include "receiver.h"

#if defined(USE_BICYCLE_KINEMATIC_MODEL)
#include "haptic.h"
#include "bicycle/kinematic.h" // simplified bicycle model
#else // No Whipple model simplifications
#include "bicycle/whipple.h"
#endif

namespace {
#if defined(USE_BICYCLE_KINEMATIC_MODEL)
    using model_t = model::BicycleKinematic;
    using haptic_drive_t = haptic::Handlebar0;
#else
    using model_t = model::BicycleWhipple;
#endif
    using bicycle_t = sim::Bicycle<model_t>;

    // sensors
    Analog<10> analog; // per channel buffer depth of 10

    Encoder encoder_steer(sa::RLS_ROLIN_ENC, sa::RLS_ROLIN_ENC_INDEX_CFG);
    EncoderFoaw<float, 32> encoder_rear_wheel(sa::RLS_GTS35_ENC,
                                              sa::RLS_GTS35_ENC_CFG,
                                              MS2ST(1),
                                              3.0f);
#if defined(USE_BICYCLE_KINEMATIC_MODEL)
    filter::MovingAverage<float, 5> velocity_filter;
#else
    constexpr float fixed_v = 6.0f; // fixed bicycle velocity
#endif

    // pose calculation loop
    constexpr systime_t pose_loop_period = US2ST(8333); // update pose at 120 Hz

    // dynamics loop
    constexpr systime_t dynamics_loop_period = MS2ST(1); // 1 ms -> 1 kHz
    constexpr float dt = static_cast<float>(dynamics_loop_period)/CH_CFG_ST_FREQUENCY;
    static_assert(dt > 0, "'dt' must be greater than zero. \
Verify 'dynamics_loop_period is greater than 1 ms.");

    // feedback and feedforward parameters
    constexpr float k_p = 150.0f;
    constexpr float k_d = k_p*0.15f;
    constexpr float m = sa::ASSEMBLY_INERTIA;

    Foaw<float, 8> error_filter(dynamics_loop_period, 3.0f);

    // Suspends the invoking thread until the system time arrives to the
    // specified value.
    // The system time is assumed to be between prev and time else the call is
    // assumed to have been called outside the allowed time interval, in this
    // case the thread yields the time slot and no sleep is performed.
    systime_t chThdSleepUntilWindowedOrYield(systime_t prev, systime_t next) {
        systime_t time;

        chSysLock();

        time = chVTGetSystemTimeX();
        if (chVTIsTimeWithinX(time, prev, next)) {
            chThdSleepS(next - time);
        } else {
            chSchDoYieldS();
        }

        chSysUnlock();

        return next;
    }

    struct pose_thread_arg {
        bicycle_t& bicycle;
        message::Transmitter& transmitter;
    };
    THD_WORKING_AREA(wa_pose_thread, 2048);
    THD_FUNCTION(pose_thread, arg) {
        pose_thread_arg* a = static_cast<pose_thread_arg*>(arg);

        chRegSetThreadName("pose");
        systime_t deadline = chVTGetSystemTime();
        while (true) {
            a->bicycle.update_kinematics();
            pbTxSmallPackage* msg = a->transmitter.alloc_smallpackage_message();
            if (msg != nullptr) {
                msg->which_value = pbTxSmallPackage_set_pose_tag;
                message::set_pose(reinterpret_cast<pbPose*>(&msg->value), a->bicycle);
                if (a->transmitter.transmit(msg) != MSG_OK) {
                    // Discard pose message if transmission fails. As the
                    // receiver requires the most recent pose, we should not
                    // wait to queue a pose message.
                    a->transmitter.free(msg);
                }
            }

            deadline = chThdSleepUntilWindowedOrYield(deadline, deadline + pose_loop_period);
        }
    }

    void initialize_sensors_actuators() {
        // Start sensors.
        // Encoder:
        //   Initialize encoder driver 5 on pins PA0, PA1 (EXT2-4, EXT2-8).
        //   Pins for encoder driver 3 are already set in board.h.
        palSetLineMode(LINE_TIM5_CH1, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
        palSetLineMode(LINE_TIM5_CH2, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
        encoder_steer.start();
        encoder_rear_wheel.start();
        analog.start(10000); // trigger ADC conversion at 10 kHz

        //Set torque measurement enable line low.
        //The output of the Kistler torque sensor is not valid until after a falling edge
        //on the measurement line and it is held low. The 'LINE_TORQUE_MEAS_EN' line is
        //reversed due to NPN switch Q1.
        palClearLine(LINE_TORQUE_MEAS_EN);
        chThdSleepMilliseconds(1);
        palSetLine(LINE_TORQUE_MEAS_EN);

        // Start DAC1 driver and set output pin as analog as suggested in Reference Manual.
        // The default line configuration is OUTPUT_OPENDRAIN_PULLUP for SPI1_ENC1_NSS
        // and must be changed to use as analog output.
        palSetLineMode(LINE_KOLLM_ACTL_TORQUE, PAL_MODE_INPUT_ANALOG);
        dacStart(sa::KOLLM_DAC, sa::KOLLM_DAC_CFG);
        sa::set_kollmorgen_torque(0.0f);
    }
} // namespace


int main(void) {
    // System initializations.
    // - HAL initialization, this also initializes the configured device drivers
    //   and performs the board-specific initializations.
    // - Kernel initialization, the main() function becomes a thread and the
    //   RTOS is active.
    halInit();
    chSysInit();

    // Create LED blink thread
    chBlinkThreadCreateStatic();

    initialize_sensors_actuators();

    // Initialize bicycle.
    bicycle_t bicycle(
#if defined(USE_BICYCLE_KINEMATIC_MODEL)
            0.0f,
#else
            fixed_v,
#endif
            static_cast<model::real_t>(dt));

#if defined(USE_BICYCLE_KINEMATIC_MODEL)
    // Initialize handlebar object to calculate motor drive feedback torque
    haptic_drive_t haptic_drive(bicycle.model());
#endif

    // Initialize time measurements
    time_measurement_t computation_time_measurement;
    chTMObjectInit(&computation_time_measurement);

    // Initialize USB data transmission
    message::Receiver receiver;
    message::Transmitter transmitter;

    receiver.start(NORMALPRIO + 2);
    transmitter.start(NORMALPRIO + 1);

    {
        // TODO Transmit initial message containing gitsha1 and model data.
        pbSimulation* msg = transmitter.alloc_simulation_message();
        *msg = pbSimulation_init_zero;
        message::set_simulation_model(msg, bicycle);
        transmitter.transmit(msg);
    }

    // Start running pose calculation thread
    pose_thread_arg a{bicycle, transmitter};
    chThdCreateStatic(wa_pose_thread, sizeof(wa_pose_thread),
            NORMALPRIO - 1, pose_thread, static_cast<void*>(&a));

    // Normal main() thread activity. This is the dynamics simulation loop.
    systime_t deadline = chVTGetSystemTime();
    while (true) {
        systime_t starttime = chVTGetSystemTime();
        chTMStartMeasurementX(&computation_time_measurement);
        constexpr float roll_torque = 0.0f;

        // positive motor torque will rotate the steering shaft clockwise
        // positive kistler torque equal to positive motor torque will stop rotation
        // m2 * xdd = T_s + T_a = 0
        //
        // then
        // m1 * xdd = T_delta + -T_s = 0
        //
        // m1: upper mass
        // m2: lower mass
        // T_delta: rider applied steer torque
        // T_s: kistler torque (sensor measurement)
        // T_a: motor torque (actuator command)
        //
        // NOTE: we neglect inertia torque after finding it to be negligible
        // compared to the sensor torque
        const float steer_torque = sa::get_kistler_sensor_torque(analog);

#if defined(USE_BICYCLE_KINEMATIC_MODEL)
        const float v = velocity_filter.output(
                -sa::REAR_WHEEL_RADIUS*(util::encoder_rate(encoder_rear_wheel)));
        bicycle.set_v(v);

        const float yaw_angle = util::wrap(
                model_t::get_full_state_element(
                    bicycle.full_state(),
                    model_t::full_state_index_t::yaw_angle)
                ); // use previous state
        const float steer_angle = util::encoder_count<float>(encoder_steer);

        bicycle.update_dynamics(
                (model_t::input_t() << roll_torque, steer_torque).finished(),
                (model_t::measurement_t() << yaw_angle, steer_angle).finished());

        const float desired_torque = haptic_drive.torque(
                model_t::get_state_part(bicycle.full_state()));
        const dacsample_t handlebar_reference_dac =
            sa::set_kollmorgen_torque(desired_torque);
#else // defined(USE_BICYCLE_KINEMATIC_MODEL)
        constexpr float v = fixed_v;
        bicycle.set_v(v);

        bicycle.update_dynamics(roll_torque, steer_torque);

        const float desired_position = model_t::get_full_state_element(
                bicycle.full_state(),
                model_t::full_state_index_t::steer_angle);

        const float steer_angle = util::encoder_count<float>(encoder_steer);
        const float error = desired_position - steer_angle;

        error_filter.add_position(error);
        const float error_derivative = error_filter.estimate_velocity();

        const float feedback_torque = k_p*error + k_d*error_derivative;

        const model_t& model = bicycle.model();
        const model_t::state_t state_deriv =
            model.A()*model_t::get_state_part(bicycle.full_state()) +
            model.B()*bicycle.input();
        const float steer_accel = model_t::get_state_element(
                state_deriv,
                model_t::state_index_t::steer_rate);
        const float feedforward_torque = m*steer_accel;

        const dacsample_t handlebar_reference_dac =
            sa::set_kollmorgen_torque(feedback_torque + feedforward_torque);
#endif // defined(USE_BICYCLE_KINEMATIC_MODEL)
        chTMStopMeasurementX(&computation_time_measurement);

        {   // prepare message for transmission
            pbSimulation* msg = transmitter.alloc_simulation_message();
            if (msg != nullptr) {
                *msg = pbSimulation_init_zero;
                message::set_simulation_model(msg, bicycle);

                msg->actuators.kollmorgen_command_torque = handlebar_reference_dac;

                // The sensor values are sampled again at a slightly different time
                // but should be roughly the same is the ones used in computation
                msg->sensors.kistler_measured_torque = analog.get_kistler_sensor();
                msg->sensors.kollmorgen_measured_torque = analog.get_kollmorgen_motor();
                msg->sensors.steer_encoder_count = encoder_steer.count();
                msg->sensors.rear_wheel_encoder_count = encoder_rear_wheel.count();

                msg->timing.starttime = starttime;
                msg->timing.computation = computation_time_measurement.last;

#if not defined(USE_BICYCLE_KINEMATIC_MODEL)
                msg->controller.feedback.torque = feedback_torque;
                msg->controller.feedback.error = error;
                msg->controller.feedback.error_derivative = error_derivative;

                msg->controller.feedforward.torque = feedforward_torque;
                msg->controller.feedforward.acceleration = steer_accel;
#endif
                if (transmitter.transmit(msg) != MSG_OK) {
                    // Discard simulation message if it cannot be processed quickly enough.
                    transmitter.free(msg);
                }
            }
        }
        deadline = chThdSleepUntilWindowedOrYield(
                deadline,
                deadline + dynamics_loop_period);
    }
}
