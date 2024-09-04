#!/bin/bash

# Assurez-vous que ce script est exécutable (chmod +x run.sh)

# Chemin vers le répertoire de construction d'Artery
ARTERY_BUILD_DIR="~/artery/build"

# Nom de la cible de simulation
TARGET="collision_warning_scenario"

# Exécuter la simulation
cd "$ARTERY_BUILD_DIR" && make run_$TARGET