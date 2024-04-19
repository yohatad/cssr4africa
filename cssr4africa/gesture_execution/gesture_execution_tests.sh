# #!/bin/bash

# # Test the gesture execution node

# # Define arrays for x, y, z values
# POINT_X=("600" "3600" "5400") 
# POINT_Y=("3000" "3000" "3000")
# POINT_Z=("820" "820" "820")

# SERVER_STARTED=true

# # Perform an action based on the value of the variable
# if ! $SERVER_STARTED ; then
#     rosrun gesture_execution perform_gesture_server
# fi

# # Get the length of the arrays
# length_x=${#POINT_X[@]}
# length_y=${#POINT_Y[@]}
# length_z=${#POINT_Z[@]}

# # Check if the lengths of the arrays are equal
# if [ $length_x -eq $length_y ] && [ $length_y -eq $length_z ]; then
#     # Iterate over the arrays of points
#     for ((i=0; i<$length_x; i++)); do
#         # Test the gesture execution node with the current point
#         echo "Performing gesture at point (${POINT_X[$i]}, ${POINT_Y[$i]}, ${POINT_Z[$i]})"
#         rosservice call /perform_gesture -- "deictic" "01" "3000" "45" "45" "${POINT_X[$i]}" "${POINT_Y[$i]}" "${POINT_Z[$i]}"
#         sleep 5
#     done
# else
#     echo "Error: The lengths of the arrays are not equal."
#     exit 1
# fi

# #!/bin/bash

# # Test the gesture execution node

# # Define an associative array for points
# declare -A POINTS
# POINTS["robot1"]="600 -3000 820"
# POINTS["robot2"]="3600 -3000 820"
# POINTS["robot3"]="5400 -3000 820"
# # POINTS["robot4"]="600 3000 820"

# SERVER_STARTED=true

# # Perform an action based on the value of the variable
# if ! $SERVER_STARTED ; then
#     rosrun gesture_execution perform_gesture_server
# fi

# # Iterate over the array of points
# for POINT in "${!POINTS[@]}"; do
#     # Split the point into x, y, z
#     IFS=' ' read -ra COORD <<< "${POINTS[$POINT]}"
#     POINT_X="${COORD[0]}"
#     POINT_Y="${COORD[1]}"
#     POINT_Z="${COORD[2]}"

#     # Test the gesture execution node with the current point
#     echo "Performing gesture at point ($POINT_X, $POINT_Y, $POINT_Z)"
#     rosservice call /perform_gesture -- "deictic" "01" "3000" "45" "45" "$POINT_X" "$POINT_Y" "$POINT_Z"
#     # sleep 5
# done



# #!/bin/bash

# # Test the gesture execution node

# # Define an associative array for points
# declare -A POINTS
# POINTS["point1"]="600 3000 820"
# POINTS["point2"]="600 3000 820"

# SERVER_STARTED=true

# # Perform an action based on the value of the variable
# if ! $SERVER_STARTED ; then
#     rosrun gesture_execution perform_gesture_server
# fi

# # Check the number of arguments
# if [ $# -ne 2 ]; then
#     echo "Usage: $0 <name> <id>"
#     exit 1
# fi

# NAME=$1
# ID=$2

# # Perform different actions based on the name and id
# case $NAME in
#     "deictic")
#         if [ "$ID" == "01" ]; then
#             # Iterate over the array of points
#             for POINT in "${!POINTS[@]}"; do
#                 # Split the point into x, y, z
#                 IFS=' ' read -ra COORD <<< "${POINTS[$POINT]}"
#                 POINT_X="${COORD[0]}"
#                 POINT_Y="${COORD[1]}"
#                 POINT_Z="${COORD[2]}"

#                 # Test the gesture execution node with the current point
#                 echo "Performing gesture at point ($POINT_X, $POINT_Y, $POINT_Z)"
#                 rosservice call /perform_gesture -- "$NAME" "$ID" "3000" "45" "45" "$POINT_X" "$POINT_Y" "$POINT_Z"
#                 sleep 5
#             done
#         else
#             echo "Error: Invalid ID for deictic. Expected '01'."
#             exit 1
#         fi
#         ;;
#     "iconic")
#         case $ID in
#             "01")
#                 # Use only the first point
#                 IFS=' ' read -ra COORD <<< "${POINTS["point1"]}"
#                 POINT_X="${COORD[0]}"
#                 POINT_Y="${COORD[1]}"
#                 POINT_Z="${COORD[2]}"

#                 # Test the gesture execution node with the first point
#                 echo "Performing gesture at point ($POINT_X, $POINT_Y, $POINT_Z)"
#                 rosservice call /perform_gesture -- "$NAME" "$ID" "3000" "45" "45" "$POINT_X" "$POINT_Y" "$POINT_Z"
#                 ;;
#             "02")
#                 # Use only the second point
#                 IFS=' ' read -ra COORD <<< "${POINTS["point2"]}"
#                 POINT_X="${COORD[0]}"
#                 POINT_Y="${COORD[1]}"
#                 POINT_Z="${COORD[2]}"

#                 # Test the gesture execution node with the second point
#                 echo "Performing gesture at point ($POINT_X, $POINT_Y, $POINT_Z)"
#                 rosservice call /perform_gesture -- "$NAME" "$ID" "3000" "45" "45" "$POINT_X" "$POINT_Y" "$POINT_Z"
#                 ;;
#             *)
#                 echo "Error: Invalid ID for iconic. Expected '01' or '02'."
#                 exit 1
#                 ;;
#         esac
#         ;;
#     *)
#         echo "Error: Invalid name. Expected 'deictic' or 'iconic'."
#         exit 1
#         ;;
# esac


#!/bin/bash

# Test the gesture execution node

# Define an associative array for points
declare -A POINTS
# POINTS["robot1"]="600 -3000 820"
# POINTS["robot2"]="3600 -3000 820"
POINTS["robot3"]="5400 3000 820"
POINTS["robot5"]="1200 -3000 820"
POINTS["robot6"]="5400 -3000 820"
POINTS["robot4"]="1200 3000 820"

# Define an array for angles
ANGLES=("15" "30" "45")

SERVER_STARTED=true

# Perform an action based on the value of the variable
if ! $SERVER_STARTED ; then
    rosrun gesture_execution perform_gesture_server
fi

# Check the number of arguments
if [ $# -ne 2 ]; then
    echo "Usage: $0 <name> <id>"
    exit 1
fi

NAME=$1
ID=$2

# Perform different actions based on the name and id
case $NAME in
    "deictic")
        if [ "$ID" == "01" ]; then
            # Iterate over the array of points
            for POINT in "${!POINTS[@]}"; do
                # Split the point into x, y, z
                IFS=' ' read -ra COORD <<< "${POINTS[$POINT]}"
                POINT_X="${COORD[0]}"
                POINT_Y="${COORD[1]}"
                POINT_Z="${COORD[2]}"

                # Test the gesture execution node with the current point
                echo "Performing gesture at point ($POINT_X, $POINT_Y, $POINT_Z)"
                rosservice call /perform_gesture -- "$NAME" "$ID" "1000" "45" "45" "$POINT_X" "$POINT_Y" "$POINT_Z"
                # sleep 5
            done
        else
            echo "Error: Invalid ID for deictic. Expected '01'."
            exit 1
        fi
        ;;
    "iconic")
        case $ID in
            "02")
                # # Use only the second point
                # IFS=' ' read -ra COORD <<< "${POINTS["point2"]}"
                # POINT_X="${COORD[0]}"
                # POINT_Y="${COORD[1]}"
                # POINT_Z="${COORD[2]}"

                # Iterate over the array of angles
                for ANGLE in "${ANGLES[@]}"; do
                    # Test the gesture execution node with the second point and current angle
                    echo "Performing bow gesture with angle $ANGLE"
                    rosservice call /perform_gesture -- "$NAME" "$ID" "2000" "$ANGLE" "$ANGLE" "3000" "3000" "820"
                    # sleep 5
                done
                ;;
            "03")
                # Iterate over the array of angles
                for ANGLE in "${ANGLES[@]}"; do
                    # Test the gesture execution node with the second point and current angle
                    echo "Performing nod gesture with angle $ANGLE"
                    rosservice call /perform_gesture -- "$NAME" "$ID" "2000" "$ANGLE" "$ANGLE" "3000" "3000" "820"
                    sleep 5
                done
                ;;
            *)
                echo "Error: Invalid ID for iconic. Expected '02', or '03'."
                exit 1
                ;;
        esac
        ;;
    *)
        echo "Error: Invalid name. Expected 'deictic' or 'iconic'."
        exit 1
        ;;
esac




# ./gesture_execution_tests.sh iconic 02