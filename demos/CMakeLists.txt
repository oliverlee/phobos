option(PHOBOS_BUILD_DEMO_ADC "Build ADC demo" TRUE)
if(PHOBOS_BUILD_DEMO_ADC)
    add_subdirectory(adc)
endif()

option(PHOBOS_BUILD_DEMO_ENCODER "Build Encoder demo" TRUE)
if(PHOBOS_BUILD_DEMO_ENCODER)
    add_subdirectory(encoder)
endif()

option(PHOBOS_BUILD_DEMO_PWM_ENCODER "Build PWM-Encoder demo" TRUE)
if(PHOBOS_BUILD_DEMO_PWM_ENCODER)
    add_subdirectory(pwm_encoder)
endif()

option(PHOBOS_BUILD_DEMO_ENCODER_INDEX "Build Encoder with index demo" TRUE)
if(PHOBOS_BUILD_DEMO_ENCODER_INDEX)
    add_subdirectory(encoder_index)
endif()

option(PHOBOS_BUILD_DEMO_ENCODER_HOTS "Build Encoder with higher-order time-stamping demo" TRUE)
if(PHOBOS_BUILD_DEMO_ENCODER_HOTS)
    add_subdirectory(encoder_hots)
endif()

option(PHOBOS_BUILD_DEMO_GPT_ADC "Build GPT-ADC demo" TRUE)
if(PHOBOS_BUILD_DEMO_GPT_ADC)
    add_subdirectory(gpt_adc)
endif()

option(PHOBOS_BUILD_DEMO_EIGEN "Build Eigen demo" TRUE)
if(PHOBOS_BUILD_DEMO_EIGEN)
    add_subdirectory(eigen)
endif()

option(PHOBOS_BUILD_DEMO_BICYCLE "Build bicycle demo" TRUE)
if(PHOBOS_BUILD_DEMO_BICYCLE)
    add_subdirectory(bicycle)
endif()

option(PHOBOS_BUILD_DEMO_USB_SERIAL "Build USB over serial demo" TRUE)
if(PHOBOS_BUILD_DEMO_USB_SERIAL)
    add_subdirectory(usb_serial)
endif()
