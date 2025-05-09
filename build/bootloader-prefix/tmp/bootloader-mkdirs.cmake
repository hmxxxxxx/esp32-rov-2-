# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/esp/espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "D:/esp/graduate/14/build/bootloader"
  "D:/esp/graduate/14/build/bootloader-prefix"
  "D:/esp/graduate/14/build/bootloader-prefix/tmp"
  "D:/esp/graduate/14/build/bootloader-prefix/src/bootloader-stamp"
  "D:/esp/graduate/14/build/bootloader-prefix/src"
  "D:/esp/graduate/14/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/esp/graduate/14/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/esp/graduate/14/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
