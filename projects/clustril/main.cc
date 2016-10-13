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
#include "ff.h"

#include "blink.h"
#include "usbconfig.h"
#include "printf.h"
#include "gitsha1.h"

#include "analog.h"
#include "encoder.h"

#include "printstate.h"
#include "packet/serialize.h"
#include "packet/framing.h"
#include "angle.h"

#include "parameters.h"
#include "virtualbicycle.h"

#include "messages.pb.h"

/*===========================================================================*/
/* Card insertion monitor.                                                   */
/*===========================================================================*/

#define POLLING_INTERVAL 10
#define POLLING_DELAY 10

/**
 * @brief   Card monitor timer.
 */
static virtual_timer_t tmr;

/**
 * @brief   Debounce counter.
 */
static unsigned cnt;

/**
 * @brief   Card event sources.
 */
static event_source_t inserted_event, removed_event;

/**
 * @brief   Insertion monitor timer callback function.
 *
 * @param[in] p         pointer to the @p BaseBlockDevice object
 *
 * @notapi
 */
static void tmrfunc(void *p) {
  //BaseBlockDevice *bbdp = p;
  BaseBlockDevice *bbdp = reinterpret_cast<BaseBlockDevice*>(p);

  chSysLockFromISR();
  if (cnt > 0) {
    if (blkIsInserted(bbdp)) {
      if (--cnt == 0) {
        chEvtBroadcastI(&inserted_event);
      }
    }
    else
      cnt = POLLING_INTERVAL;
  }
  else {
    if (!blkIsInserted(bbdp)) {
      cnt = POLLING_INTERVAL;
      chEvtBroadcastI(&removed_event);
    }
  }
  chVTSetI(&tmr, MS2ST(POLLING_DELAY), tmrfunc, bbdp);
  chSysUnlockFromISR();
}

/**
 * @brief   Polling monitor start.
 *
 * @param[in] p         pointer to an object implementing @p BaseBlockDevice
 *
 * @notapi
 */
static void tmr_init(void *p)
{
  chEvtObjectInit(&inserted_event);
  chEvtObjectInit(&removed_event);
  chSysLock();
  cnt = POLLING_INTERVAL;
  chVTSetI(&tmr, MS2ST(POLLING_DELAY), tmrfunc, p);
  chSysUnlock();
}

/*===========================================================================*/
/* FatFs related.                                                            */
/*===========================================================================*/

/**
 * @brief FS object.
 */
static FATFS SDC_FS;

/* FS mounted and ready.*/
static bool fs_ready = FALSE;

/* Generic large buffer.*/
static uint8_t fbuff[1024];

/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

/*
 * Card insertion event.
 */
static void InsertHandler(eventid_t id) {
  FRESULT err;

  (void)id;
  /*
   * On insertion SDC initialization and FS mount.
   */
  if (sdcConnect(&SDCD1))
    return;

  err = f_mount(&SDC_FS, "/", 1);
  if (err != FR_OK) {
    sdcDisconnect(&SDCD1);
    return;
  }
  fs_ready = TRUE;
}

/*
 * Card removal event.
 */
static void RemoveHandler(eventid_t id) {

  (void)id;
  sdcDisconnect(&SDCD1);
  fs_ready = FALSE;
}

namespace {
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

    static const evhandler_t evhndl[] = {
      InsertHandler,
      RemoveHandler
    };
    event_listener_t el0, el1;
    sdcStart(&SDCD1, NULL); // Activate SDC driver 1, default configuration
    tmr_init(&SDCD1);       // Activates the card insertion monitor.
    chEvtRegister(&inserted_event, &el0, 0);
    chEvtRegister(&removed_event, &el1, 1);

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

    /* create print state monitor */
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

    VirtualBicycle bicycle;

    /*
     * Normal main() thread activity, in this demo it simulates the bicycle
     * dynamics in real-time (roughly).
     */
    bool print_version_string = true;
    rtcnt_t bicycle_simulation_time = 0;
    while (true) {
        constexpr float roll_torque = 0;

        /* get torque measurements */
        float steer_torque = static_cast<float>(analog.get_adc12()*2.0f*max_kistler_torque/4096 -
                max_kistler_torque);
        float motor_torque = static_cast<float>(
                analog.get_adc13()*2.0f*max_kollmorgen_torque/4096 -
                max_kollmorgen_torque);

        /* get angle measurements */
        float yaw_angle = angle::wrap(bicycle.x()[0]); /* yaw angle, just use previous state value */
        float steer_angle = angle::encoder_count<float>(encoder);

        /* observer time/measurement update (~80 us with real_t = float) */
        bicycle_simulation_time = chSysGetRealtimeCounterX();
        bicycle.update(roll_torque, steer_torque, yaw_angle, steer_angle);
        bicycle.encode_and_stuff_pose();

        /* generate an example torque output for testing */
        float feedback_torque = 10.0f * std::sin(constants::two_pi *
                ST2S(static_cast<float>(chVTGetSystemTime())));
        dacsample_t aout = static_cast<dacsample_t>(
                (feedback_torque/21.0f * 2048) + 2048); /* reduce output to half of full range */
        dacPutChannelX(&DACD1, 0, aout);

        printst_t s = getPrintState();
        if (s == printst_t::VERSION) {
            if (print_version_string) {
                printf("Running firmware version %.7s\r\n", g_GITSHA1);
                print_version_string = false;
            }
        } else if (s == printst_t::NORMAL) {
            packet::framing::unstuff(bicycle.pose_buffer(), encode_buffer.data(), bicycle.pose_buffer_size());
            BicyclePose pose = BicyclePose_init_zero;
            if (packet::serialize::decode(encode_buffer.data(), &pose, bicycle.pose_buffer_size() - 1)) {
                /*
                 * total computation time (kalman update, x_aux calculation, packet framing and serialization)
                 * = ~270 us
                 * TODO: calculate x_aux in a separate loop at a slower update rate.
                 */
                bicycle_simulation_time = chSysGetRealtimeCounterX() - bicycle_simulation_time;
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
                        RTC2US(STM32_SYSCLK, bicycle_simulation_time));
            }
        } else if (s == printst_t::NONE) {
            /* reset printing of version string */
            print_version_string = true;
        }

        if (fs_ready) {
            UINT bw;
            FIL f;
            FRESULT res = f_open(&f, "test.txt", FA_WRITE | FA_OPEN_ALWAYS);
            f_lseek(&f, f_size(&f));
            f_write(&f, encode_buffer.data(),  bicycle.pose_buffer_size() - 1, &bw);
            f_close(&f);
        }
        chEvtDispatch(evhndl, chEvtWaitOneTimeout(ALL_EVENTS, MS2ST(500)));

        chThdSleepMilliseconds(static_cast<systime_t>(1000*bicycle.model().dt()));
    }
}
