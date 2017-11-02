# OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories( ${CMAKE_SOURCE_DIR}/src/libchessdetector )


if(WIN32)
    # Postfix of .lib and .dll
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

file (GLOB TRACKER_LIB_SRC "*.cpp")
file (GLOB TRACKER_LIB_HEADER "*.h")

add_library (libpatterntracker STATIC ${TRACKER_LIB_SRC} ${TRACKER_LIB_HEADER})

target_link_libraries(libpatterntracker libchessdetector ${OpenCV_LIBS})
