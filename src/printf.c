#include "hal.h"
#include "chprintf.h"
#include "memstreams.h"
#include "usbcfg.h"

int chvsnprintf(char *str, size_t size, const char *fmt, va_list ap) {
    MemoryStream ms;
    BaseSequentialStream *chp;
    size_t size_wo_nul;
    int retval;

    if (size > 0)
      size_wo_nul = size - 1;
    else
      size_wo_nul = 0;

    /* Memory stream object to be used as a string writer, reserving one
       byte for the final zero.*/
    msObjectInit(&ms, (uint8_t *)str, size_wo_nul, 0);

    /* Performing the print operation using the common code.*/
    chp = (BaseSequentialStream *)(void *)&ms;
    retval = chvprintf(chp, fmt, ap);

    /* Terminate with a zero, unless size==0.*/
    if (ms.eos < size)
        str[ms.eos] = 0;

    /* Return number of bytes that would have been written.*/
    return retval;
}

int printf(const char *fmt, ...) {
    va_list ap;
    int formatted_bytes;
    static char serial_str[128];

    va_start(ap, fmt);
    formatted_bytes = chvsnprintf(serial_str, sizeof(serial_str)/sizeof(serial_str[0]), fmt, ap);
    va_end(ap);

    return obqWriteTimeout(&SDU1.obqueue, (const uint8_t*)serial_str,
            formatted_bytes, TIME_IMMEDIATE);
}

