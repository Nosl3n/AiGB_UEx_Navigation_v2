# drone_bridge: Webots extern controller (libController) + zero-copy DDS media plane (FastDDS, rc::media).
set(MEDIA_TRANSPORT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../active_inference/common/media_transport)
set(MEDIA_TRANSPORT_IDL ${MEDIA_TRANSPORT_DIR}/generated/idl)

LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/camera_publisher.cpp
  ${MEDIA_TRANSPORT_DIR}/media_transport.cpp
  ${MEDIA_TRANSPORT_IDL}/image_framePubSubTypes.cxx
  ${MEDIA_TRANSPORT_IDL}/image_frameTypeObjectSupport.cxx
  ${MEDIA_TRANSPORT_IDL}/image360_framePubSubTypes.cxx
  ${MEDIA_TRANSPORT_IDL}/image360_frameTypeObjectSupport.cxx
  ${MEDIA_TRANSPORT_IDL}/lidar_framePubSubTypes.cxx
  ${MEDIA_TRANSPORT_IDL}/lidar_frameTypeObjectSupport.cxx
  ${MEDIA_TRANSPORT_IDL}/imu_framePubSubTypes.cxx
  ${MEDIA_TRANSPORT_IDL}/imu_frameTypeObjectSupport.cxx
)

LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/camera_publisher.h
  ../src/flight_controller.h
)

include_directories(${MEDIA_TRANSPORT_DIR})

find_package(fastcdr REQUIRED)
find_package(fastdds REQUIRED)

set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

LIST(APPEND LIBS ${LIBS} fastdds fastcdr -L${WEBOTS_HOME}/lib/controller Controller CppController)

add_definitions(-O2 -fmax-errors=1 -fno-char8_t)
