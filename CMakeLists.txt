cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
include(pico_sdk_import.cmake)
project(cg_grower_device_pico_w C CXX ASM)

pico_sdk_init()
add_executable(cg_grower_device_pico_w cg_grower_device_pico_w.c)
target_include_directories(cg_grower_device_pico_w PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(cg_grower_device_pico_w pico_stdlib hardware_adc
pico_cyw43_arch_lwip_threadsafe_background pico_lwip_http)

pico_add_extra_outputs(cg_grower_device_pico_w)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)