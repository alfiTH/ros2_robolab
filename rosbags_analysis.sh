#!/bin/bash

# Ruta fija a los rosbags
RUTA="/home/dataset/Scenario-based-dataset/"

# Lista de nombres de rosbags
FICHEROS=("rosbag2_2025_03_17-13_14_45" "rosbag2_2025_03_25-10_51_19" "rosbag2_2025_03_10-10_27_21"  "rosbag2_2025_03_17-13_22_21" "rosbag2_2025_03_26-12_16_48" "rosbag2_2025_03_10-11_21_52" "rosbag2_2025_03_18-11_16_47" "rosbag2_2025_03_26-13_45_54" "rosbag2_2025_03_10-11_30_24" "rosbag2_2025_03_20-12_23_33" "rosbag2_2025_03_27-12_20_57" "rosbag2_2025_03_10-12_27_19" "rosbag2_2025_03_20-12_28_29" "rosbag2_2025_03_27-12_37_31" "rosbag2_2025_03_13-10_54_53"  "rosbag2_2025_03_24-10_24_06" "rosbag2_2025_03_13-12_02_09" "rosbag2_2025_03_24-10_38_38" "rosbag2_2025_03_27-13_43_17" "rosbag2_2025_03_17-11_28_48" "rosbag2_2025_03_25-10_25_28" "rosbag2_2025_03_17-12_20_33" "rosbag2_2025_03_25-10_50_28")

# Bucle para procesar cada rosbag
for fichero in "${FICHEROS[@]}"; do
    echo "Analizando: $fichero"
    
    ros2 bag play "${RUTA}${fichero}" &

    ros2 run sequence_time sequence_time_node
    
    cp sequence_validity.json "${fichero}.json"

    wait
done

