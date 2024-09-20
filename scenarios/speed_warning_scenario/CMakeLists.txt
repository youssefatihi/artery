

include_directories(
    ${OMNETPP_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)
# Ajout de la feature speed_warning
add_artery_feature(speed_warning
    SpeedWarningService.cc
    SpeedAlertReceiverService.cc
    Utils.cc
)

# Génération du message DENM (nous gardons le même message)
generate_opp_message(DENMMessage.msg TARGET speed_warning DIRECTORY collision_msgs)

# Ajout de la cible de compilation pour le scénario
add_opp_run(speed_warning_scenario NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})

# Inclusion des répertoires nécessaires
target_include_directories(speed_warning PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Lier avec la bibliothèque artery
target_link_libraries(speed_warning PUBLIC artery)

# Configuration du standard C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Politiques CMake pour éviter les avertissements
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0148)
    cmake_policy(SET CMP0148 NEW)
endif()