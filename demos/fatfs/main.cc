/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

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
#include "test.h"

#include "chprintf.h"
#include "shell.h"

#include "ff.h"

#include "usbconfig.h"

/*===========================================================================*/
/* Card insertion monitor.                                                   */
/*===========================================================================*/

#define POLLING_INTERVAL                10
#define POLLING_DELAY                   10

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
    BaseBlockDevice *bbdp = reinterpret_cast<BaseBlockDevice*>(p);

    chSysLockFromISR();
    if (cnt > 0) {
        if (blkIsInserted(bbdp)) {
            if (--cnt == 0) {
                chEvtBroadcastI(&inserted_event);
            }
        } else {
            cnt = POLLING_INTERVAL;
        }
    } else {
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
static void tmr_init(void *p) {

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

/*
 * Application entry point.
 */
int main(void) {
    static const evhandler_t evhndl[] = {
        InsertHandler,
        RemoveHandler
    };
    event_listener_t el0, el1;

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
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the SDC driver 1 using the default configuration.
     */
    sdcStart(&SDCD1, NULL);

    /*
     * Activates the card insertion monitor.
     */
    tmr_init(&SDCD1);

    /*
     * Normal main() thread activity, in this demo it does nothing except
     * sleeping in a loop and listen for events.
     */
    chEvtRegister(&inserted_event, &el0, 0);
    chEvtRegister(&removed_event, &el1, 1);
    bool file_written = false;
    while (true) {
        if (fs_ready) {
            const char filename[] = "fatfs_test.txt";
            FRESULT fr = f_stat(filename, nullptr);
            const uint8_t test_buffer[] = {0xAB, 0xCD, 0xEF, 0x01};

            if ((fr == FR_NO_FILE) || !file_written) {
                FIL fil;
                UINT bytes_written;

                fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
                if (fr != FR_OK) {
                    break;
                }
                f_write(&fil, test_buffer, sizeof(test_buffer), &bytes_written);
                f_close(&fil);
                file_written = true;
            } else {
                FIL fil;
                uint8_t buffer[4];
                UINT bytes_read;

                fr = f_open(&fil, filename, FA_READ);
                f_read(&fil, &buffer, sizeof(buffer), &bytes_read);
                f_close(&fil);
                if ((bytes_read != sizeof(test_buffer)) &&
                    (buffer[0] != 0xAB) && (buffer[1] != 0xCD) &&
                    (buffer[2] != 0xEF) && (buffer[3] != 0x01)) {
                    chSysHalt("buffer not written/read correctly");
                }
            }
        }
        chEvtDispatch(evhndl, chEvtWaitOneTimeout(ALL_EVENTS, MS2ST(500)));
    }
}
