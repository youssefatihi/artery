cmake_minimum_required(VERSION 3.5)

if(NOT TARGET OmnetPP::sim)
    find_package(OmnetPP REQUIRED GLOBAL)
endif()

add_artery_feature(drowsiness_detection
    DrowsinessDetectionService.cc
    DrowsinessAlertReceiverService.cc
)

generate_opp_message(DENMMessage.msg TARGET drowsiness_detection DIRECTORY drowsiness_msgs)

set(CMAKE_NED_FOLDERS 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ARTERY_DIR}/src
    ${ARTERY_DIR}/scenarios
    ${ARTERY_DIR}/scenarios/drowsiness_warning
)
add_opp_run(drowsiness_warning_scenario
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    CONFIG omnetpp.ini
    NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(drowsiness_detection PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(drowsiness_detection PUBLIC artery)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0148)
    cmake_policy(SET CMP0148 NEW)
endif()