cmake_minimum_required(VERSION 2.8.3)
project(yl)
#set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

include_directories(include)

set(AA_HEADERS
	include/FindLines.h
    include/FindCircle.h
    include/StandardLine.h
	include/NWPU.h
	include/Bebop.h
	include/Common.h
	include/Control.h
	include/Detections.h
	include/Parameter.h
)

set(AA_SOURCES
	src/main.cpp
	src/FindCircle.cpp
	src/FindLines.cpp
	src/Bebop.cpp
	src/Common.cpp
	src/Control.cpp
	src/Detections.cpp
    src/StandardLine.cpp
	src/Parameter.cpp
)



find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  std_msgs
  bebop_msgs
  image_transport
  cv_bridge
)

catkin_package()



find_package(
  OpenCV
  )
find_package (ZBar0)
#################################
find_package(PkgConfig)
pkg_check_modules(PC_ZBAR QUIET zbar)
set(ZBAR_DEFINITIONS ${PC_ZBAR_CFLAGS_OTHER})
find_library(ZBAR_LIBRARIES NAMES zbar 
             HINTS ${PC_ZBAR_LIBDIR} ${PC_ZBAR_LIBRARY_DIRS} )
find_path(ZBAR_INCLUDE_DIR Decoder.h
          HINTS ${PC_ZBAR_INCLUDEDIR} ${PC_ZBAR_INCLUDE_DIRS}
          PATH_SUFFIXES zbar )
include(FindPackageHandleStandardArgs)
############################
find_package_handle_standard_args(ZBAR  DEFAULT_MSG  ZBAR_LIBRARIES ZBAR_INCLUDE_DIR)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(yl ${AA_SOURCES})
target_link_libraries(
    	yl
	boost_system
	boost_thread
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
	${ZBAR_LIBRARIES}
	${Boost_LIBRARIES}
)

#add_executable(main src/main.cpp)
#target_link_libraries(main ${catkin_LIBRARIES}${OpenCV_LIBRARIES})
#add_dependencies(main flymain_generate_messages_cpp)
add_dependencies(yl  ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_generate_messages_cpp)


