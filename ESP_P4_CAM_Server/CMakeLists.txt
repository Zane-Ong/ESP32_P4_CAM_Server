# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(SDKCONFIG_DEFAULTS
    "sdkconfig.defaults"
    "sdkconfig.defaults.esp32p4")
    
set(EXTRA_COMPONENT_DIRS 
    LED
    ${EXTRA_COMPONENT_DIRS}
)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP_P4_CAM_Server)
