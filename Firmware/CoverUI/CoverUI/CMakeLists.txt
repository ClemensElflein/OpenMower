cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)

set(projname "mower-buttons")
add_executable(${projname})






target_sources(${projname} PRIVATE main.cpp LEDcontrol.cpp statemachine.cpp buttonscan.cpp)

#project(${projname} C CXX ASM)
project(pico_examples C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init() 

#pico_generate_pio_header(${projname} ${CMAKE_CURRENT_LIST_DIR}/GPIO_prg.pio)
pico_generate_pio_header(${projname} ${CMAKE_CURRENT_LIST_DIR}/status_LED.pio)
pico_generate_pio_header(${projname} ${CMAKE_CURRENT_LIST_DIR}/LED_mux.pio)
pico_generate_pio_header(${projname} ${CMAKE_CURRENT_LIST_DIR}/buzzer.pio)


 
#set(Mower_PATH ${PROJECT_SOURCE_DIR})
set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})



# enable usb output, disable uart output
 #   pico_enable_stdio_usb(mower-buttons 0)
 #   pico_enable_stdio_uart(mower-buttons 1)


 

#  main.c
#        extrafunc.c
#        )

target_link_libraries(${projname} pico_stdlib hardware_pio pico_multicore)

pico_add_extra_outputs(${projname})