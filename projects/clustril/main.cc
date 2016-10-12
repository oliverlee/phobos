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

#include "blink.h"
#include "usbconfig.h"
#include "printf.h"
#include "gitsha1.h"

#include "bicycle.h"
#include "kalman.h"
#include "parameters.h"

#include "analog.h"
#include "encoder.h"

#include "printstate.h"
#include "packet/serialize.h"
#include "packet/framing.h"
#include "angle.h"

#include "messages.pb.h"

namespace {
    using bicycle_t = model::Bicycle;
    using kalman_t = observer::Kalman<bicycle_t>;

    const float fs = 200; // sample rate [Hz]
    const float dt = 1.0/fs; // sample time [s]
    const float v0 = 5.0; // forward speed [m/s]

    /* Kalman filter variance values */
    const float sigma0 = 1 * constants::as_radians; // yaw angle measurement noise variance
    const float sigma1 = 0.008 * constants::as_radians; // steer angle measurement noise variance

    bicycle_t::input_t u; /* roll torque, steer torque */
    bicycle_t::state_t x; /* yaw angle, roll angle, steer angle, roll rate, steer rate */
    bicycle_t::output_t z; /* yaw angle, steer angle */
    bicycle_t::auxiliary_state_t x_aux; /* rear contact x, rear contact y, pitch angle */

    /* sensors */
    Analog analog;
    Encoder encoder(&GPTD5, /* CH1, CH2 connected to PA0, PA1 and NOT enabled by board.h */
            {PAL_NOLINE, /* no index channel */
             152000, /* counts per revolution */
             EncoderConfig::filter_t::CAPTURE_64}); /* 64 * 42 MHz (TIM3 on APB1) = 1.52 us
                                                     * for valid edge */

     const float max_kistler_torque = 25.0f; /* maximum measured steer torque */
     /*
      * The voltage output of the Kistler torque sensor is ±10V. With the 12-bit ADC,
      * resolution for LSB is 4.88 mV/bit or 12.2 mNm/bit.
      */
     const float max_kollmorgen_torque = 10.0f; /* max torque at 1.00 Arms/V */

    const DACConfig dac1cfg1 = {
        .init       = 2047U, // max value is 4095 (12-bit)
        .datamode   = DAC_DHRM_12BIT_RIGHT
    };

     std::array<uint8_t, BicyclePose_size> encode_buffer;
     std::array<uint8_t, BicyclePose_size + 1> frame_buffer;
} // namespace

/*
 * Application entry point.
 */
int main(void) {

    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    /*
     * Initialize a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activate the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    board_usb_lld_disconnect_bus();   //usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    board_usb_lld_connect_bus();      //usbConnectBus(serusbcfg.usbp);

    /* create the blink thread and print state monitor */
    chBlinkThreadCreateStatic();
    /*
     * Use LINE_TIM4_CH2 (PB7, EXT1-15, J4-B) as a button by
     * connecting/disconnecting it to ground.
     * */
    palSetLineMode(LINE_TIM4_CH2, PAL_MODE_INPUT_PULLUP);
    enablePrintStateMonitor(LINE_TIM4_CH2);

    /*
     * Start sensors.
     * Encoder:
     *   Initialize encoder driver 5 on pins PA0, PA1 (EXT2-4, EXT2-8).
     */
    palSetLineMode(LINE_TIM5_CH1, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    palSetLineMode(LINE_TIM5_CH2, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    encoder.start();
    analog.start(1000); /* trigger ADC conversion at 1 kHz */


    /*
     * Set torque measurement enable line low.
     * The output of the Kistler torque sensor is not valid until after a falling edge
     * on the measurement line and it is held low. The 'LINE_TORQUE_MEAS_EN' line is
     * reversed due to NPN switch Q1.
     */
    palClearLine(LINE_TORQUE_MEAS_EN);
    chThdSleepMilliseconds(1);
    palSetLine(LINE_TORQUE_MEAS_EN);

    /*
     * Start DAC1 driver and set output pin as analog as suggested in Reference Manual.
     * The default line configuration is OUTPUT_OPENDRAIN_PULLUP  for SPI1_ENC1_NSS
     * and must be changed to use as analog output.
     */
    palSetLineMode(LINE_KOLLM_ACTL_TORQUE, PAL_MODE_INPUT_ANALOG);
    dacStart(&DACD1, &dac1cfg1);

    /* initialize bicycle model and states */
    model::Bicycle bicycle(v0, dt);
    x.setZero();

    /* initialize bicycle auxiliary states */
    x_aux.setZero(); /* set x, y to zero */
    x_aux[2] = bicycle.solve_constraint_pitch(x, 30 * constants::as_radians);

    /* initialize Kalman filter */
    kalman_t kalman(bicycle,
            parameters::defaultvalue::kalman::Q(dt), /* process noise cov */
            (kalman_t::measurement_noise_covariance_t() <<
             sigma0,      0,
                  0, sigma1).finished(),
            bicycle_t::state_t::Zero(), /* set initial state estimate to zero */
            std::pow(x[1]/2, 2) * bicycle_t::state_matrix_t::Identity()); /* error cov */

    /*
     * Normal main() thread activity, in this demo it simulates the bicycle
     * dynamics in real-time (roughly).
     */
    rtcnt_t kalman_update_time = 0;
    bool print_version_string = true;
    u.setZero(); /* set both roll and steer torques to zero */
    BicyclePose pose = BicyclePose_init_zero;
    while (true) {
        u[1] = static_cast<float>(analog.get_adc12()*2.0f*max_kistler_torque/4096 -
                max_kistler_torque);
        //float motor_torque = static_cast<float>(
        //        analog.get_adc13()*2.0f*max_kollmorgen_torque/4096 -
        //        max_kollmorgen_torque);

        /* set measurement vector */
        z[0] = angle::wrap(x[0]); /* yaw angle, just use previous state value */

        /*
         * Steer angle, read from encoder (enccnt_t is uint32_t)
         * Convert angle from enccnt_t (unsigned) to corresponding signed type and use negative
         * values for any count over half a revolution.
         */
        auto position = static_cast<std::make_signed<enccnt_t>::type>(encoder.count());
        auto rev = static_cast<std::make_signed<enccnt_t>::type>(encoder.config().counts_per_rev);
        if (position > rev / 2) {
            position -= rev;
        }
        z[1] = static_cast<float>(position) / rev * constants::two_pi;

        /* observer time/measurement update (~80 us with real_t = float) */
        kalman_update_time = chSysGetRealtimeCounterX();
        kalman.time_update(u);
        kalman.measurement_update(z);
        x = kalman.x();
        x[0] = angle::wrap(x[0]);
        x[1] = angle::wrap(x[1]);
        x[2] = angle::wrap(x[2]);
        x_aux = bicycle.x_aux_next(x, x_aux);

        /* generate an example torque output for testing */
        float torque = 10.0f * std::sin(constants::two_pi *
                ST2S(static_cast<float>(chVTGetSystemTime())));
        dacsample_t aout = static_cast<dacsample_t>(
                (torque/21.0f * 2048) + 2048); /* reduce output to half of full range */
        dacPutChannelX(&DACD1, 0, aout);

        uint8_t bytes_written;
        pose.x = x_aux[0];
        pose.y = x_aux[1];
        pose.yaw = x[0];
        pose.roll = x[1];
        pose.steer = x[2];
        bytes_written = packet::serialize::encode(pose, encode_buffer.data(), encode_buffer.size());
        packet::framing::stuff(encode_buffer.data(), frame_buffer.data(), bytes_written);

        printst_t s = getPrintState();
        if (s == printst_t::VERSION) {
            if (print_version_string) {
                printf("Running firmware version %.7s\r\n", g_GITSHA1);
                print_version_string = false;
            }
        } else if (s == printst_t::NORMAL) {
            packet::framing::unstuff(frame_buffer.data(), encode_buffer.data(), bytes_written + 1);
            pose = BicyclePose_init_zero;
            if (packet::serialize::decode(encode_buffer.data(), &pose, bytes_written)) {
                /*
                 * total computation time (kalman update, x_aux calculation, packet framing and serialization)
                 * = ~270 us
                 * TODO: calculate x_aux in a separate loop at a slower update rate.
                 */
                kalman_update_time = chSysGetRealtimeCounterX() - kalman_update_time;
                printf("bicycle pose:\r\n"
                        "\tx:\t%0.3f m\r\n"
                        "\ty:\t%0.3f m\r\n",
                        pose.x, pose.y);
                printf("\tyaw:\t%0.3f deg\r\n"
                        "\troll:\t%0.3f deg\r\n"
                        "\tsteer:\t%0.3f deg\r\n",
                        pose.yaw*constants::as_degrees,
                        pose.roll*constants::as_degrees,
                        pose.steer*constants::as_degrees);
                printf("computation time: %U us\r\n",
                        RTC2US(STM32_SYSCLK, kalman_update_time));
            }
            //printf("encoder count:\t%u\r\n", encoder.count());
            //printf("sensors:\t%0.3f\t%0.3f\t%0.3f\t%0.3f\r\n",
            //        u[1], motor_torque, rad_to_deg(z[0]), rad_to_deg(z[1]));
            //printf("state:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\r\n",
            //        rad_to_deg(x[0]), rad_to_deg(x[1]), rad_to_deg(x[2]), rad_to_deg(x[3]), rad_to_deg(x[4]));
            //printf("kalman update time: %U us\r\n",
            //        RTC2US(STM32_SYSCLK, kalman_update_time));
        } else if (s == printst_t::NONE) {
            /* reset printing of version string */
            print_version_string = true;
        }
        chThdSleepMilliseconds(static_cast<systime_t>(1000*dt));
    }
}
