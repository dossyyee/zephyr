
if(CONFIG_FPU)
    zephyr_library_import(qorvo_lib ${CMAKE_CURRENT_SOURCE_DIR}/libdwt_uwb_driver-m4-hfp-6.0.7.a)
else()
    zephyr_library_import(qorvo_lib ${CMAKE_CURRENT_SOURCE_DIR}/libdwt_uwb_driver-m4-sfp-6.0.7.a)
endif()

# the library needs a custom linker script
zephyr_linker_sources(SECTIONS custom-sections.ld)