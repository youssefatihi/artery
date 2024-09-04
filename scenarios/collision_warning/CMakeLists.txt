if(NOT TARGET OmnetPP::sim)
    find_package(OmnetPP REQUIRED GLOBAL)
endif()
add_artery_feature(collision_warning
    CollisionWarningService.cc
    CollisionAlertReceiverService.cc
)

generate_opp_message(CollisionWarningMessage.msg TARGET collision_warning DIRECTORY collision_msgs)

set(CMAKE_NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})

add_opp_run(collision_warning_scenario
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    CONFIG omnetpp.ini
    NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories(collision_warning PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(collision_warning PUBLIC artery)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)