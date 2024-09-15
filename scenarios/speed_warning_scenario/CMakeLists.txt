cmake_minimum_required(VERSION 3.5)

if(NOT TARGET OmnetPP::sim)
    find_package(OmnetPP REQUIRED GLOBAL)
endif()

add_artery_feature(speed_warning_scenario
    SpeedWarningService.cc
    SpeedAlertReceiverService.cc
)

generate_opp_message(DENMMessage.msg TARGET speed_warning_scenario DIRECTORY collision_msgs)

add_opp_run(speed_warning_scenario
    NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(speed_warning_scenario PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(speed_warning_scenario PUBLIC artery)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0148)
    cmake_policy(SET CMP0148 NEW)
endif()