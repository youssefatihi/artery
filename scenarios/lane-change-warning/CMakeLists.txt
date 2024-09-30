cmake_minimum_required(VERSION 3.5)

if(NOT TARGET OmnetPP::sim)
    find_package(OmnetPP REQUIRED GLOBAL)
endif()

add_artery_feature(lane_change_warning
    LaneChangeWarningService.cc
    LaneChangeAlertReceiverService.cc
)

generate_opp_message(DENMMessage.msg TARGET lane_change_warning DIRECTORY lane_change_msgs)

set(CMAKE_NED_FOLDERS 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ARTERY_DIR}/src
    ${ARTERY_DIR}/scenarios
    ${ARTERY_DIR}/scenarios/lane_change_warning
)
add_opp_run(lane_change_warning_scenario
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    CONFIG omnetpp.ini
    NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(lane_change_warning PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(lane_change_warning PUBLIC artery)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0148)
    cmake_policy(SET CMP0148 NEW)
endif()