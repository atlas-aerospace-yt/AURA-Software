cmake_minimum_required(VERSION 3.8)

# Use C++ not C for this project
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Define all files to be included
set(EXTRA_COMPONENT_DIRS
    src/main
    src/main/main_core_0
    src/main/main_core_1
    src/common/bus_drivers/I2C
    src/common/bus_drivers/SPI
    src/common/dev_drivers/BMP280
    src/common/dev_drivers/ICM20948
    src/common/dev_drivers/INA219
    src/common/dev_drivers/PCA9685
    src/common/dev_drivers/SD_Card
    src/common/dev_drivers/Flash
    src/common/dev_drivers/FSA8S
    src/common/utility
    lib/Quat
    lib/PID
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(AURA_V_1_0)
