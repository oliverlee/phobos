#include "filesystem.h"

#include "hal.h"

namespace {
    FATFS SDC_FS;
    bool fs_ready = false;

    const unsigned int sdc_polling_interval = 10;
    const unsigned int sdc_polling_delay = 10;

    virtual_timer_t card_monitor_timer;
    unsigned int debounce_counter;

    /* SD card event sources */
    event_source_t sdc_inserted_event;
    event_source_t sdc_removed_event;

    /* SD card event listeners */
    event_listener_t el0;
    event_listener_t el1;

    void insertion_monitor_timer_callback(void *p) {
        BaseBlockDevice *bbdp = reinterpret_cast<BaseBlockDevice*>(p);

        chSysLockFromISR();
        if (debounce_counter > 0) {
            if (blkIsInserted(bbdp)) {
                if (--debounce_counter == 0) {
                    chEvtBroadcastI(&sdc_inserted_event);
                }
            } else {
                debounce_counter = sdc_polling_interval;
            }
        } else {
            if (!blkIsInserted(bbdp)) {
              debounce_counter = sdc_polling_interval;
              chEvtBroadcastI(&sdc_removed_event);
            }
        }
        chVTSetI(&card_monitor_timer, MS2ST(sdc_polling_delay), insertion_monitor_timer_callback, bbdp);
        chSysUnlockFromISR();
    }

    void polling_monitor_timer_init(void *p) {
        chEvtObjectInit(&sdc_inserted_event);
        chEvtObjectInit(&sdc_removed_event);

        chSysLock();
        debounce_counter = sdc_polling_interval;
        chVTSetI(&card_monitor_timer, MS2ST(sdc_polling_delay), insertion_monitor_timer_callback, p);
        chSysUnlock();
    }

    /* Card insertion event. */
    void InsertHandler(eventid_t id) {
        FRESULT err;

        (void)id;
        /* On insertion SDC initialization and FS mount. */
        if (sdcConnect(&SDCD1))
            return;

        err = f_mount(&SDC_FS, "/", 1);
        if (err != FR_OK) {
            sdcDisconnect(&SDCD1);
            return;
        }
        fs_ready = TRUE;
    }

    /* Card removal event. */
    void RemoveHandler(eventid_t id) {
        (void)id;
        sdcDisconnect(&SDCD1);
        fs_ready = FALSE;
    }
}


namespace filesystem {
    const evhandler_t sdc_eventhandlers[] = {
      InsertHandler,
      RemoveHandler
    };

    void init() {
        sdcStart(&SDCD1, nullptr); // Activate SDC driver 1, default configuration
        polling_monitor_timer_init(&SDCD1); // Activate card insertion monitor
        chEvtRegister(&sdc_inserted_event, &el0, 0);
        chEvtRegister(&sdc_removed_event, &el1, 1);
    }

    bool ready() {
        return fs_ready;
    }
} // filesystem
