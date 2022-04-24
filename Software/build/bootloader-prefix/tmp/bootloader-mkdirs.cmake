# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/pablo/esp/esp-idf/components/bootloader/subproject"
  "/home/pablo/Documentos/02_Maestria_en_Sistemas_Embebidos/25_Control_Digital/Desarrollo_software/ESP-32-CDSE_Ver6/build/bootloader"
  "/home/pablo/Documentos/02_Maestria_en_Sistemas_Embebidos/25_Control_Digital/Desarrollo_software/ESP-32-CDSE_Ver6/build/bootloader-prefix"
  "/home/pablo/Documentos/02_Maestria_en_Sistemas_Embebidos/25_Control_Digital/Desarrollo_software/ESP-32-CDSE_Ver6/build/bootloader-prefix/tmp"
  "/home/pablo/Documentos/02_Maestria_en_Sistemas_Embebidos/25_Control_Digital/Desarrollo_software/ESP-32-CDSE_Ver6/build/bootloader-prefix/src/bootloader-stamp"
  "/home/pablo/Documentos/02_Maestria_en_Sistemas_Embebidos/25_Control_Digital/Desarrollo_software/ESP-32-CDSE_Ver6/build/bootloader-prefix/src"
  "/home/pablo/Documentos/02_Maestria_en_Sistemas_Embebidos/25_Control_Digital/Desarrollo_software/ESP-32-CDSE_Ver6/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/pablo/Documentos/02_Maestria_en_Sistemas_Embebidos/25_Control_Digital/Desarrollo_software/ESP-32-CDSE_Ver6/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
