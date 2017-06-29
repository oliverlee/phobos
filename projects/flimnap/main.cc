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

#include "blink.h"
#include "saconfig.h"
#include "utility.h"

#include "parameters.h"

#include <array>
#include <type_traits>

#include "haptic.h"
#include "simbicycle.h"
#include "transmitter.h"

#if defined(USE_BICYCLE_KINEMATIC_MODEL)
#include "bicycle/kinematic.h" // simplified bicycle model
#else // defined(USE_BICYCLE_KINEMATIC_MODEL)
#include "bicycle/whipple.h" // whipple bicycle model
#include "kalman.h" // Kalman filter observer
#include "interpolated_lqr.h" // LQR controller
#endif // defined(USE_BICYCLE_KINEMATIC_MODEL)

namespace {
#if defined(USE_BICYCLE_KINEMATIC_MODEL)
    using model_t = model::BicycleKinematic;
    using observer_t = std::nullptr_t;
#else // defined(USE_BICYCLE_KINEMATIC_MODEL)
    using model_t = model::BicycleWhipple;
    using observer_t = observer::Kalman<model_t>;
    using lqr_t = controller::InterpolatedLqr<model_t>;
#endif // defined(USE_BICYCLE_KINEMATIC_MODEL)
    using bicycle_t = sim::Bicycle<model_t, observer_t>;

    // sensors
    Analog analog;
    Encoder encoder_steer(sa::RLS_ROLIN_ENC, sa::RLS_ROLIN_ENC_INDEX_CFG);
    EncoderFoaw<float, 32> encoder_rear_wheel(sa::RLS_GTS35_ENC,
                                              sa::RLS_GTS35_ENC_CFG,
                                              MS2ST(1), 3.0f);
    filter::MovingAverage<float, 5> velocity_filter;

    // pose calculation loop
    constexpr systime_t pose_loop_period = US2ST(8333); // update pose at 120 Hz

    // dynamics loop
    constexpr systime_t dynamics_loop_period = MS2ST(1); // 1 ms -> 1 kHz

    // virtual roll and steer torque assistance enabled for
    constexpr float assistance_velocity_limit = 1.0f; // [m/s] values less than this
    // we gradually increase/decrease torque assistance over this period
    // after the velocity crosses the velocity limit
    constexpr systime_t assistance_fade_period = MS2ST(50)/dynamics_loop_period; // in iterations
    systime_t assistance_fade_counter = 0;

    // motor cooldown
    // we compute the moving average of motor torque values
    bool cooldown_active = false;
    constexpr systime_t cooldown_period = MS2ST(1500);  // this duration is also used to trigger cooldown
                                                        // value is chosen based on collected data
    constexpr uint32_t cooldown_subsample_rate = 100;
    uint32_t cooldown_subsample_counter = 0;
    filter::MovingAverage<float,
        cooldown_period/dynamics_loop_period/cooldown_subsample_rate> cooldown_torque_average;

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

    dacsample_t set_handlebar_velocity(float velocity, float measured_torque) {
        cooldown_subsample_counter = (cooldown_subsample_counter + 1) % cooldown_subsample_rate;
        if (cooldown_subsample_counter == 0) {
            cooldown_torque_average.output(measured_torque);
        }

        if (cooldown_active) {
            cooldown_active = std::abs(cooldown_torque_average.output()) < 0.1f*sa::MAX_KOLLMORGEN_TORQUE;
            palClearLine(LINE_MOTOR1_EN);
        } else {
            cooldown_active = std::abs(cooldown_torque_average.output()) > 0.9f*sa::MAX_KOLLMORGEN_TORQUE;
            palSetLine(LINE_MOTOR1_EN);
        }

        // limit velocity to a maximum magnitude of 100 deg/s
        // input is in units of rad/s
        const float saturated_velocity = util::clamp(velocity,
                -sa::MAX_KOLLMORGEN_VELOCITY, sa::MAX_KOLLMORGEN_VELOCITY);
        const dacsample_t aout =
            saturated_velocity/sa::MAX_KOLLMORGEN_VELOCITY*sa::DAC_HALF_RANGE + sa::DAC_HALF_RANGE;
        // Calculate channel value based on DAC device params
        // regshift = 0 -> CH1 -> channel value 0
        // regshift = 16 -> CH2 -> channel value 1
        static const dacchannel_t channel = sa::KOLLM_DAC->params->regshift/16;
        dacPutChannelX(sa::KOLLM_DAC, channel, aout);
        return aout;
    }

    constexpr float adc_to_nm(adcsample_t value, adcsample_t adc_zero, float magnitude) {
        // Convert torque from ADC samples to Nm.
        // ADC samples are 12 bits.
        // It's not clear when scaling should be applied as data was never saved after the scale
        // factors were determined.
        const int16_t shifted_value = static_cast<int16_t>(value) - static_cast<int16_t>(adc_zero);
        return static_cast<float>(shifted_value)*magnitude/static_cast<float>(sa::ADC_HALF_RANGE);
    }

    template <typename T>
    struct observer_initializer{
        template <typename S = T>
        typename std::enable_if<std::is_same<typename S::observer_t, observer::Kalman<model_t>>::value, void>::type
            initialize(S& bicycle) {
            typename S::observer_t& observer = bicycle.observer();
            observer.set_Q(parameters::defaultvalue::kalman::Q(observer.dt()));
            // Reduce steer measurement noise covariance
            observer.set_R(parameters::defaultvalue::kalman::R/1000);

            // prime the Kalman gain matrix
            bicycle.prime_observer();

            // We start with steer angle equal to the measurement and all other state elements at zero.
            model_t::state_t x0 = model_t::state_t::Zero();
            model_t::set_state_element(x0, model_t::state_index_t::steer_angle,
                    util::encoder_count<float>(encoder_steer));
            observer.set_x(x0);
        }
        template <typename S = T>
        typename std::enable_if<!std::is_same<typename S::observer_t, observer::Kalman<model_t>>::value, void>::type
            initialize(S& bicycle) {
            // no-op
            (void)bicycle;
        }

        template <typename S = T>
        typename std::enable_if<std::is_same<typename S::observer_t, observer::Kalman<model_t>>::value, void>::type
            set_message(S& bicycle, SimulationMessage* msg) {
            message::set_kalman_gain(&msg->kalman, bicycle.observer());
            msg->has_kalman = true;
        }
        template <typename S = T>
        typename std::enable_if<!std::is_same<typename S::observer_t, observer::Kalman<model_t>>::value, void>::type
            set_message(S& bicycle, SimulationMessage* msg) {
            // no-op
            (void)bicycle;
            (void)msg;
        }
    };

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
            BicyclePoseMessage* msg = a->transmitter.alloc_pose_message();
            if (msg != nullptr) {
                *msg = a->bicycle.pose();
                if (a->transmitter.transmit_async(msg) != MSG_OK) {
                    // Discard pose message if transmission fails. As the
                    // receiver requires the most recent pose, we should not
                    // wait to queue a pose message.
                    a->transmitter.free_message(msg);
                }
            }

            deadline = chThdSleepUntilWindowedOrYield(deadline, deadline + pose_loop_period);
        }
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
    // We also use MOTOR1_EN to enable the motor.
    palSetLineMode(LINE_KOLLM_ACTL_TORQUE, PAL_MODE_INPUT_ANALOG);
    dacStart(sa::KOLLM_DAC, sa::KOLLM_DAC_CFG);
    palSetLine(LINE_MOTOR1_EN);
    set_handlebar_velocity(0.0f, 0.0f);

    // Initialize bicycle. The initial velocity is important as we use it to prime
    // the Kalman gain matrix.
    bicycle_t bicycle(0.0, static_cast<model::real_t>(dynamics_loop_period)/CH_CFG_ST_FREQUENCY);

#if !defined(USE_BICYCLE_KINEMATIC_MODEL)
    // At low speed, we add an assistive roll torque to stabilize the bicycle.
    // Gains calculated with script calculate_lqr_gain.py.
    lqr_t controller(
            (lqr_t::feedback_gain_t() << // v = 0
             0, -1648.90829472, -48.43076882, -516.76805995, -14.59295354,
             0, -50.23751925, -42.32812576, -26.53589725, -11.70135889).finished(),
            (lqr_t::feedback_gain_t() << // v = 1 = assistance_velocity_limit
             0, -238.99395251, -8.0805282, -71.5451519, -1.21535324,
             0, 624.96912408, -16.89860361, 176.94881285, -6.47514655).finished()
    );
#endif

    // Initialize HandlebarDynamic object to estimate torque due to handlebar inertia.
    // TODO: naming here is poor
#if !defined(FLIMNAP_ZERO_INPUT)
    haptic::HandlebarDynamic handlebar_model(bicycle.model(), sa::UPPER_ASSEMBLY_INERTIA_PHYSICAL);
#endif  // !defined(FLIMNAP_ZERO_INPUT)

    observer_initializer<observer_t> oi;
    oi.initialize(bicycle);

    // Initialize time measurements
    time_measurement_t computation_time_measurement;
    time_measurement_t transmission_time_measurement;
    chTMObjectInit(&computation_time_measurement);
    chTMObjectInit(&transmission_time_measurement);

    // Initialize USB data transmission
    message::Transmitter transmitter;
    {   // Transmit initial message containing gitsha1, model, and observer data.
        SimulationMessage* msg = transmitter.alloc_simulation_message();
        *msg = SimulationMessage_init_zero;
        msg->timestamp = chVTGetSystemTime();
        message::set_simulation_full_model_observer(msg, bicycle);
        transmitter.transmit(msg); // This blocks until USB data starts getting read
    }
    transmitter.start(NORMALPRIO + 1); // start transmission thread

    // Start running pose calculation thread
    pose_thread_arg a{bicycle, transmitter};
    chThdCreateStatic(wa_pose_thread, sizeof(wa_pose_thread),
            NORMALPRIO - 1, pose_thread, static_cast<void*>(&a));

    // Normal main() thread activity. This is the dynamics simulation loop.
    systime_t deadline = chVTGetSystemTime();
    while (true) {
        systime_t starttime = chVTGetSystemTime();
        chTMStartMeasurementX(&computation_time_measurement);
        float roll_torque = 0.0f;

        // get sensor measurements
        const float kistler_torque = adc_to_nm(analog.get_adc12(),
                sa::KISTLER_ADC_ZERO_OFFSET, sa::MAX_KISTLER_TORQUE);
        const float motor_torque = adc_to_nm(analog.get_adc13(),
                sa::KOLLMORGEN_ADC_ZERO_OFFSET, sa::MAX_KOLLMORGEN_TORQUE);
        const float steer_angle = util::encoder_count<float>(encoder_steer);
        const float rear_wheel_angle = std::fmod(-util::encoder_count<float>(encoder_rear_wheel),
                                                 constants::two_pi);
        const float v = velocity_filter.output(
                -sa::REAR_WHEEL_RADIUS*(util::encoder_rate(encoder_rear_wheel)));

        // yaw angle, just use previous state value
        const float yaw_angle = util::wrap(bicycle.pose().yaw);

        // calculate rider applied torque
#if defined(FLIMNAP_ZERO_INPUT)
        float steer_torque = 0.0f;
        (void)kistler_torque;
#else // defined(FLIMNAP_ZERO_INPUT)
        const float inertia_torque = -handlebar_model.torque(model_t::get_state_part(bicycle.full_state()));
        float steer_torque = kistler_torque - inertia_torque;
#endif // defined(FLIMNAP_ZERO_INPUT)

        // simulate bicycle
        bicycle.set_v(v);
#if !defined(USE_BICYCLE_KINEMATIC_MODEL)
        if (bicycle.v() < assistance_velocity_limit) {
            const float interp = bicycle.v() / assistance_velocity_limit;
            const model_t::input_t u = controller.control_calculate(bicycle.observer().state(), interp);
            if (assistance_fade_counter < assistance_fade_period) {
                ++assistance_fade_counter;
            }
            const float fade = static_cast<float>(assistance_fade_counter)/assistance_fade_period;
            roll_torque += fade * model_t::get_input_element(u, model_t::input_index_t::roll_torque);
            steer_torque += fade * model_t::get_input_element(u, model_t::input_index_t::steer_torque);
        } else if (assistance_fade_counter != 0) {
            const model_t::input_t u = controller.control_calculate(bicycle.observer().state(), 1.0f);
            const float fade = static_cast<float>(assistance_fade_counter--)/assistance_fade_period;
            roll_torque += fade * model_t::get_input_element(u, model_t::input_index_t::roll_torque);
            steer_torque += fade * model_t::get_input_element(u, model_t::input_index_t::steer_torque);
        }
#endif
        bicycle.update_dynamics(roll_torque, steer_torque, yaw_angle, steer_angle, rear_wheel_angle);

        // generate handlebar torque output
        const float desired_velocity =
            model_t::get_full_state_element(bicycle.full_state(),
                                            model_t::full_state_index_t::steer_rate);
        const dacsample_t handlebar_velocity_dac = set_handlebar_velocity(desired_velocity, motor_torque);
        chTMStopMeasurementX(&computation_time_measurement);

        {   // prepare message for transmission
            SimulationMessage* msg = transmitter.alloc_simulation_message();
            if (msg != nullptr) {
                *msg = SimulationMessage_init_zero;
                msg->timestamp = starttime;
                msg->model.v = bicycle.v();
                msg->model.has_v = true;
                msg->has_model = true;
                message::set_bicycle_input(&msg->input,
                        (model_t::input_t() << roll_torque, steer_torque).finished());
                msg->has_input = true;
                message::set_simulation_state(msg, bicycle);
                message::set_simulation_auxiliary_state(msg, bicycle);
                oi.set_message(bicycle, msg);
                message::set_simulation_actuators(msg, handlebar_velocity_dac);
                message::set_simulation_sensors(msg,
                        analog.get_adc12(), analog.get_adc13(),
                        encoder_steer.count(), encoder_rear_wheel.count());
                message::set_simulation_timing(msg,
                        computation_time_measurement.last, transmission_time_measurement.last);
                if (transmitter.transmit_async(msg) != MSG_OK) {
                    // Discard simulation message if it cannot be processed quickly enough.
                    transmitter.free_message(msg);
                }
            }
        }
        deadline = chThdSleepUntilWindowedOrYield(deadline, deadline + dynamics_loop_period);
    }
}
