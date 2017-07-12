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

#include "gitsha1.h"
#include "blink.h"
#include "usbconfig.h"
#include "saconfig.h"
#include "utility.h"

#include "parameters.h"

#include <Eigen/Core>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

namespace {
    constexpr systime_t loop_period = MS2ST(1);
    constexpr float dt = static_cast<float>(ST2MS(loop_period))/1000.0f; // seconds
    constexpr float k = 3.14f; // N-m/rad

    // sensors
    Analog analog;
    Encoder encoder_steer(sa::RLS_ROLIN_ENC, sa::RLS_ROLIN_ENC_INDEX_CFG);

    dacsample_t set_handlebar_reference(float reference) {
#if defined(LIMTOC_VELOCITY_MODE)
        constexpr float MAX_REF_VALUE = sa::MAX_KOLLMORGEN_VELOCITY;
#else // defined(LIMTOC_VELOCITY_MODE)
        constexpr float MAX_REF_VALUE = sa::MAX_KOLLMORGEN_TORQUE;
#endif // defined(LIMTOC_VELOCITY_MODE)
        const float saturated_reference = util::clamp(reference, -MAX_REF_VALUE, MAX_REF_VALUE);
        const dacsample_t aout = saturated_reference/MAX_REF_VALUE*sa::DAC_HALF_RANGE + sa::DAC_HALF_RANGE;
        dacPutChannelX(sa::KOLLM_DAC, 0, aout); // TODO: don't hardcode zero but find the DAC channel constant
        return aout;
    }

    // This is a simple mass-spring system so the equation of motion is:
    //   m*x_ddot = -k*x
    // and the state space formulation is
    // [x_dot ] = [  0  1][x    ]
    // [x_ddot]   [ -k  0][x_dot]
#if defined(LIMTOC_VELOCITY_MODE)
    float prev_steer_angle = 0.0f;
    using state_t = Eigen::Matrix<float, 2, 1>;
    boost::numeric::odeint::runge_kutta_dopri5<
        state_t,
        float,
        state_t,
        float,
        boost::numeric::odeint::vector_space_algebra> stepper;

    float get_velocity_reference(const state_t& x) {
        state_t xout = x;
        stepper.do_step([](const state_t& x, state_t& dxdt, const float t) -> void {
                (void)t; // system is time-independent

                dxdt[0] = x[1];
                dxdt[1] = -k*x[0];
                }, xout, 0.0f, dt); // newly obtained state written in place
        return xout[1];
    }
#else // defined(LIMTOC_VELOCITY_MODE)
    float get_torque_reference(float angle) {
        return -k*angle;
    }
#endif // defined(LIMTOC_VELOCITY_MODE)
} // namespace

/*
 * Application entry point.
 */
int main(void) {

    //
    // System initializations.
    // - HAL initialization, this also initializes the configured device drivers
    //   and performs the board-specific initializations.
    // - Kernel initialization, the main() function becomes a thread and the
    //   RTOS is active.
    halInit();
    chSysInit();

    // Initialize a serial-over-USB CDC driver.
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    // Activate the USB driver and then the USB bus pull-up on D+.
    // Note, a delay is inserted in order to not have to disconnect the cable
    // after a reset.
    board_usb_lld_disconnect_bus();   //usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    board_usb_lld_connect_bus();      //usbConnectBus(serusbcfg.usbp);

    // create the blink thread 
    chBlinkThreadCreateStatic();

    // Start sensors.
    // Encoder:
    //   Initialize encoder driver 5 on pins PA0, PA1 (EXT2-4, EXT2-8).
    //   Pins for encoder driver 3 are already set in board.h.
    palSetLineMode(LINE_TIM5_CH1, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    palSetLineMode(LINE_TIM5_CH2, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    encoder_steer.start();
    analog.start(10000); /* trigger ADC conversion at 10 kHz */

    // Set torque measurement enable line low.
    // The output of the Kistler torque sensor is not valid until after a falling edge
    // on the measurement line and it is held low. The 'LINE_TORQUE_MEAS_EN' line is
    // reversed due to NPN switch Q1.
    palClearLine(LINE_TORQUE_MEAS_EN);
    chThdSleepMilliseconds(1);
    palSetLine(LINE_TORQUE_MEAS_EN);

    // Normal main() thread activity
    systime_t deadline = chVTGetSystemTime();
    while (true) {
        const float kistler_torque = util::adc_to_Nm(analog.get_adc12(),
                sa::KISTLER_ADC_ZERO_OFFSET, sa::MAX_KISTLER_TORQUE);
        const float motor_torque = util::adc_to_Nm(analog.get_adc13(),
                sa::KOLLMORGEN_ADC_ZERO_OFFSET, sa::MAX_KOLLMORGEN_TORQUE);
        const float steer_angle = util::encoder_count<float>(encoder_steer);

        // generate motor command to simulate a spring
#if defined(LIMTOC_VELOCITY_MODE)
        constexpr char modechar = 'v';
        const float steer_rate = (steer_angle - prev_steer_angle)/dt;
        prev_steer_angle = steer_angle;
        state_t x;
        x << steer_angle, steer_rate;
        const float feedback_reference = get_velocity_reference(x);
#else // defined(LIMTOC_VELOCITY_MODE)
        constexpr char modechar = 't';
        const float feedback_reference = get_torque_reference(steer_angle);
#endif  // defined(LIMTOC_VELOCITY_MODE)
        set_handlebar_reference(feedback_reference);

        printf("%c[%u] kistler: %8.3f Nm\tkollmorgen: %8.3f Nm\tsteer: %8.3f rad\r\n",
               modechar, chSysGetRealtimeCounterX(), kistler_torque, motor_torque, steer_angle);
        deadline = chThdSleepUntilWindowed(deadline, deadline + loop_period);
    }
}
