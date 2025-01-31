cmake_minimum_required(VERSION 3.20.0)


# Set the environment variable for UBXLIB_DIR
set(ENV{UBXLIB_DIR} ${CMAKE_CURRENT_LIST_DIR}/modules/ubxlib)

# Append the ubxlib module path to Zephyr modules
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/ubxlib)

# Set the configuration files early, before calling find_package(Zephyr ...)
set(CONF_FILE prj.conf ${CMAKE_CURRENT_LIST_DIR}/modules/ubxlib/port/platform/zephyr/default.conf)

# Now find Zephyr, as all required variables are set
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})


set(gen_dir ${CMAKE_CURRENT_BINARY_DIR}/certs)
zephyr_include_directories(${gen_dir})
generate_inc_file_for_target(
    app
    certs/AmazonRootCA1.pem
    ${gen_dir}/AmazonRootCA1.pem.inc
    )

include(${ZEPHYR_BASE}/samples/net/common/common.cmake)
include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)

project(full-gateway-03)
target_include_directories(app PRIVATE ${CMAKE_SOURCE_DIR}/src)
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources}
src/gateway_BLE/bleAdvConnCentral.c
src/gateway_BLE/blePAwRCentral.c
src/gateway_MODEM/gateway_modem.c 
src/gateway_STORAGE/gateway_storage.c 
src/gateway_STORAGE/nand_flash.c 
src/gateway_STORAGE/lfs.c 
src/gateway_STORAGE/lfs_util.c 
src/gateway_STORAGE/lfs_nand_wrapper.c)
