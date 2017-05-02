# OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB cpp "*.cpp")
file(GLOB head "*.h")


if(WIN32)
    # Postfix of .lib and .dll
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

add_library(libchessdetector STATIC ${cpp} ${head})