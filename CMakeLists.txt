cmake_minimum_required(VERSION 2.8)

if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
endif()

project(hybrid_marker_track)
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# set(CMAKE_LIBRARY_PATH "C:\\Program Files (x86)\\Windows Kits\\8.0\\Lib\\win8\\um\\x64")

# add_definitions(-DNOMINMAX)

set(OpenCV_STATIC OFF)

# Find OpenCV
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

# find_package(Qt5Widgets REQUIRED)
# set(CMAKE_AUTOMOC ON)


# IF(WIN32)
#     set(PTHREADS_INCLUDE_DIR "" CACHE PATH "Pthreads Include Directory")
#   set(PTHREADS_LIB_DIR "" CACHE PATH "Pthreads Lib Directory")
# ELSEIF(UNIX)
#   find_package(Threads)
# ENDIF()

# Pattern tracker
add_subdirectory(src/libpatterntracker)
add_subdirectory(src/libchessdetector)


include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/libpatterntracker
 # ${libpatterntracker_SOURCE_DIR}
)

# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)


file(GLOB HEADER_FILES include/*.h)
file(GLOB CXX_FILES src/*.cpp)

# file(GLOB UI_FILES src/*.ui)
# qt5_wrap_ui(UISrcs ${UI_FILES} )

add_executable(hybrid_marker_track
		${CXX_FILES} 
		${HEADER_FILES} 
		# ${UISrcs}
)

# qt5_use_modules(arius Core Gui Widgets)

target_link_libraries(hybrid_marker_track
    ${OpenCV_LIBS}
    # ${Matlab_LIBRARIES}
    libpatterntracker
    # frmgrab
)