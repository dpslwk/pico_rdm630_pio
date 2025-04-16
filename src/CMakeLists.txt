pico_add_library(pico_rdm630_pio)

pico_generate_pio_header(pico_rdm630_pio
        ${CMAKE_CURRENT_LIST_DIR}/pico_rdm630_pio.pio)

target_sources(pico_rdm630_pio INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/pico_rdm630_pio.c
        )

target_include_directories(pico_rdm630_pio INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/include)

pico_mirrored_target_link_libraries(pico_rdm630_pio INTERFACE
        pico_stdlib
        hardware_pio
        hardware_irq
        hardware_dma
        pico_async_context_freertos
        )
