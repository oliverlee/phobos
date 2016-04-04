option(PHOBOS_BUILD_DEMO_ADC "Build ADC demo" TRUE)
if(PHOBOS_BUILD_DEMO_ADC)
    add_subdirectory(adc)
endif()

option(PHOBOS_BUILD_DEMO_ENCODER "Build Encoder demo" TRUE)
if(PHOBOS_BUILD_DEMO_ENCODER)
    add_subdirectory(encoder)
endif()

option(PHOBOS_BUILD_DEMO_GPT_ADC "Build GPT-ADC demo" TRUE)
if(PHOBOS_BUILD_DEMO_GPT_ADC)
    add_subdirectory(gpt_adc)
endif()

option(PHOBOS_BUILD_DEMO_EIGEN "Build Eigen demo" TRUE)
if(PHOBOS_BUILD_DEMO_EIGEN)
    add_subdirectory(eigen)
endif()

option(PHOBOS_BUILD_DEMO_USB_SERIAL "Build USB over serial demo" TRUE)
if(PHOBOS_BUILD_DEMO_USB_SERIAL)
    add_subdirectory(usb_serial)
endif()
