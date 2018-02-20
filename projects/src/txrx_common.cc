#include "txrx_common.h"
#include "receiver.h"
#include "ch.h"
#include "usbcfg.h"

namespace {
/**
 IN EP1 state.
 */
USBInEndpointState ep1instate;

/**
 OUT EP1 state.
 */
USBOutEndpointState ep1outstate;

/**
 EP1 initialization structure (both IN and OUT).
 Data transmitted and data received callback are disabled by default.
 */
USBEndpointConfig usbep1cfg = {
    USB_EP_MODE_TYPE_BULK, // ep_mode
    nullptr, // setup_cb
    nullptr, // in_cb
    message::Receiver::data_received_callback, // out_cb
    message::USB_ENDPOINT_MAX_PACKET_SIZE, // in_maxsize
    message::USB_ENDPOINT_MAX_PACKET_SIZE, // out_maxsize
    &ep1instate, // in_state
    &ep1outstate, // out_state
    2, // ep_buffers
    nullptr // setup_buf
};

/**
 IN EP2 state.
 */
USBInEndpointState ep2instate;

/**
 EP2 initialization structure (IN only).
 */
USBEndpointConfig usbep2cfg = {
    USB_EP_MODE_TYPE_INTR, // ep_mode
    nullptr, // setup_cb
    nullptr, // in_cb
    nullptr, // out_cb
    0x0010, // in_maxsize
    0x0000, // out_maxsize
    &ep2instate, // in_state
    nullptr, // out_state
    1, // ep_buffers
    nullptr // setup_buf
};

/**
 Handles the USB driver global events.
 */
void usb_event(USBDriver *usbp, usbevent_t event) {
    switch (event) {
    case USB_EVENT_ADDRESS:
        return;
    case USB_EVENT_CONFIGURED:
        chSysLockFromISR();

        /**
         Enables the endpoints specified into the configuration.
         Note, this callback is invoked from an ISR so I-Class functions
         must be used.
         */
        usbInitEndpointI(usbp, serusbcfg.bulk_out, &usbep1cfg);
        usbInitEndpointI(usbp, serusbcfg.int_in, &usbep2cfg);

        chSysUnlockFromISR();
        return;
    case USB_EVENT_RESET:
      /* Falls into.*/
    case USB_EVENT_UNCONFIGURED:
      /* Falls into.*/
    case USB_EVENT_SUSPEND:
      /* Disconnection event on suspend.*/
    case USB_EVENT_WAKEUP:
      /* Disconnection event on suspend.*/
    case USB_EVENT_STALLED:
      return;
    }
    return;
}

} // namespace

namespace message {

USBConfig usbcfg = {
    usb_event,
    get_descriptor,
    sduRequestsHook, // CDC setup, required for serial port emulation
    nullptr // no SOF handler
};

void usbStart() {
    chSysLock();

    if (usbGetDriverStateI(serusbcfg.usbp) == USB_STOP) {
        // Activate the USB driver and then the USB bus pull-up on D+.
        // Note, a delay is inserted in order to not have to disconnect the cable
        // after a reset.
        board_usb_lld_disconnect_bus();   //usbDisconnectBus(serusbcfg.usbp);
        chThdSleepS(MS2ST(1500));

        // This is copied from usbStart(USBDriver*, const USBConfig*) as we need
        // to do the same thing but while already S-Locked
        chDbgAssert((serusbcfg.usbp->state == USB_STOP) || (serusbcfg.usbp->state == USB_READY),
                      "invalid state");
        serusbcfg.usbp->config = &usbcfg;
        for (unsigned i = 0; i <= (unsigned)USB_MAX_ENDPOINTS; i++) {
            serusbcfg.usbp->epc[i] = nullptr;
        }
        usb_lld_start(serusbcfg.usbp);
        serusbcfg.usbp->state = USB_READY;

        board_usb_lld_connect_bus();      //usbConnectBus(serusbcfg.usbp);

        // Block USB device until ready
        while (usbGetDriverStateI(serusbcfg.usbp) != USB_ACTIVE) {
            chThdSleepS(MS2ST(10));
        }
    }

    chSysUnlock();
}

} // namespace message
