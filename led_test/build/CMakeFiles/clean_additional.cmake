# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "/home/abivathan/micro_ros_espidf_component/esp32_toolchain.cmake"
  "/home/abivathan/micro_ros_espidf_component/include"
  "/home/abivathan/micro_ros_espidf_component/micro_ros_dev"
  "/home/abivathan/micro_ros_espidf_component/micro_ros_src"
  "bootloader/bootloader.bin"
  "bootloader/bootloader.elf"
  "bootloader/bootloader.map"
  "config/sdkconfig.cmake"
  "config/sdkconfig.h"
  "esp-idf/esptool_py/flasher_args.json.in"
  "esp-idf/mbedtls/x509_crt_bundle"
  "flash_app_args"
  "flash_bootloader_args"
  "flash_project_args"
  "flasher_args.json"
  "ldgen_libraries"
  "ldgen_libraries.in"
  "led_test.bin"
  "led_test.map"
  "project_elf_src_esp32s3.c"
  "x509_crt_bundle.S"
  )
endif()
