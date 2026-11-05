# drone_bridge: Webots extern controller (libController) + zero-copy DDS media plane (FastDDS, rc::media).
set(MEDIA_TRANSPORT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../active_inference/common/media_transport)
set(MEDIA_TRANSPORT_IDL ${MEDIA_TRANSPORT_DIR}/generated/idl)

LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/camera_publisher.cpp
  ../src/ardupilot_link.cpp
  ../src/mavlink_pilot.cpp
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
  ../src/ardupilot_link.h
  ../src/mavlink_pilot.h
)

include_directories(${MEDIA_TRANSPORT_DIR})

# MAVLink C library (header-only) for the joystick -> ArduPilot pilot (src/mavlink_pilot.*): the copy ArduPilot's
# own SITL build generates, so it always matches the autopilot that is run. Build ArduPilot first:
#   cd $ARDUPILOT_HOME && ./waf configure --board sitl && ./waf copter
if(DEFINED ENV{ARDUPILOT_HOME})
  set(ARDUPILOT_HOME $ENV{ARDUPILOT_HOME})
else()
  set(ARDUPILOT_HOME $ENV{HOME}/robocomp/ardupilot)
endif()
set(MAVLINK_DIR ${ARDUPILOT_HOME}/build/sitl/libraries/GCS_MAVLink/include/mavlink/v2.0)
if(NOT EXISTS ${MAVLINK_DIR}/ardupilotmega/mavlink.h)
  message(FATAL_ERROR "MAVLink headers not found in ${MAVLINK_DIR}: build ArduPilot SITL first (see webots-drone-bridge/README.md)")
endif()
include_directories(${MAVLINK_DIR})

find_package(fastcdr REQUIRED)
find_package(fastdds REQUIRED)

set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

LIST(APPEND LIBS ${LIBS} fastdds fastcdr -L${WEBOTS_HOME}/lib/controller Controller CppController)

add_definitions(-O2 -fmax-errors=1 -fno-char8_t)
