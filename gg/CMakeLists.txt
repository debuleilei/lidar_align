cmake_minimum_required(VERSION 2.8)
project(eight_test)
set(CMAKE_CXX_FLAGS "-std=c++11")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11" )
# set(WORK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..) CMAKE_SOURCE_DIR
set(WORK_ROOT ${CMAKE_SOURCE_DIR})

include(${CMAKE_SOURCE_DIR}/cmake/Packages.cmake)
include_3rd_module(protobuf eigen pcl console_bridge poco boost grpc gflags fastrtps)
include_directories(${CMAKE_SOURCE_DIR}/modules/cybertron/lib/class_loader/include)

add_definitions(-DMODULE_NAME="eight_test")
include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/modules/cybertron/include)
include_directories(${WORK_ROOT}/modules/common/include) #tofind
include_directories(${WORK_ROOT}/modules) #tofind

aux_source_directory(${PROJECT_SOURCE_DIR}/src/component DIR_SRCS)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/parser DIR_SRCS)
message(STATUS "///----------------------------------------------------------------")
message(STATUS "///-PROJECT_SOURCE_DIR:${PROJECT_SOURCE_DIR}")
set(MATH_UTIL_SRC_LIST
    ${WORK_ROOT}/modules/common/src/math/aabox2d.cpp
    ${WORK_ROOT}/modules/common/src/math/aaboxkdtree2d.cpp
    ${WORK_ROOT}/modules/common/src/math/angle.cpp
    ${WORK_ROOT}/modules/common/src/math/box2d.cpp
    ${WORK_ROOT}/modules/common/src/math/vec2d.cpp
    ${WORK_ROOT}/modules/common/src/math/polygon2d.cpp
    ${WORK_ROOT}/modules/common/src/math/segment2d.cpp
    ${WORK_ROOT}/modules/common/src/math/sin_table.cpp
)
#find_package(PCL 1.7 REQUIRED )
#find_package( Boost COMPONENTS system REQUIRED)
find_package(libpointmatcher  REQUIRED)


include_directories(#${PCL_INCLUDE_DIRS}
                    #${Boost_INCLUDE_DIRS}
                    ${libpointmatcher_INCLUDE_DIRS})
#link_directories(#${PCL_LIBRARY_DIRS}
                  #${Boost_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#message(${libpointmatcher_LIBRARIES})

# add_library(${PROJECT_NAME} SHARED ${DIR_SRCS} ${MATH_UTIL_SRC_LIST})
add_library(${PROJECT_NAME} SHARED ${DIR_SRCS} ${CMAKE_SOURCE_DIR}/modules/common/src/libadu_math.so )
target_link_libraries(${PROJECT_NAME}
    ${CMAKE_SOURCE_DIR}/modules/third_party/protobuf/lib/libprotobuf.so.13    
    adu_common
    adu_geo
    adu_kinematics
    adu_math
    ${libpcap_LIBRARIES}
    protobuf
    cybertron
    pcap
    yaml-cpp
    ${YAML_CPP_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    pcl_io
    pcl_common
    pcl_io_ply
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    #/usr/local/lib/libpointmatcher.so.1.3.1
    ${libpointmatcher_LIBRARIES}
)

if(${RELEASE_BUILD})
  set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})
endif()

if(MAKE_TEST)
  add_subdirectory(tests)
endif()

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY conf dag params
    DESTINATION ${CMAKE_INSTALL_PREFIX}/
)
