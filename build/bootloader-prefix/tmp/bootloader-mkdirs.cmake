# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/15702/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "D:/Source/ESP_IDF/esp_dhtc11/build/bootloader"
  "D:/Source/ESP_IDF/esp_dhtc11/build/bootloader-prefix"
  "D:/Source/ESP_IDF/esp_dhtc11/build/bootloader-prefix/tmp"
  "D:/Source/ESP_IDF/esp_dhtc11/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Source/ESP_IDF/esp_dhtc11/build/bootloader-prefix/src"
  "D:/Source/ESP_IDF/esp_dhtc11/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Source/ESP_IDF/esp_dhtc11/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Source/ESP_IDF/esp_dhtc11/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
