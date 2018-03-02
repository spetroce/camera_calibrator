set(SYSTEM_DETECTED ON)
if(UNIX AND NOT APPLE)
  set(CODE_PREFIX_ "/home/$ENV{USER}")
  if(EXISTS "/home/$ENV{USER}/code/src/Qt/5.10.0/gcc_64/lib/cmake/Qt5")
    set(Qt5_DIR "/home/$ENV{USER}/code/src/Qt/5.10.0/gcc_64/lib/cmake/Qt5")
  endif()
elseif(APPLE)
  set(CODE_PREFIX_ "/Users/$ENV{USER}")
  set(Qt5_DIR "/Users/$ENV{USER}/code/src/Qt/5.10.0/clang_64/lib/cmake/Qt5")
else()
  set(SYSTEM_DETECTED OFF)
endif()

if(SYSTEM_DETECTED)
  message(STATUS "CODE_PREFIX_=${CODE_PREFIX_}")

  ## mio
  set(MIO_INCLUDE_DIR "${CODE_PREFIX_}/code/src")

  ## OpenCV
  set(OpenCV_DIR "${CODE_PREFIX_}/code/install/opencv/dev/rel/share/OpenCV")
else()
  message(WARNING "Couldn't detect system type in PkgConfigPath.cmake")
endif()

