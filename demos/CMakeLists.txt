option(PHOBOS_BUILD_DEMO_USB_SERIAL "Build USB over serial demo" TRUE)
if(PHOBOS_BUILD_DEMO_USB_SERIAL)
    add_subdirectory(usb_serial)
endif()
