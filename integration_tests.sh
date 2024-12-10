#!/bin/bash

display_help() {
    echo "TESTING SCRIPT FOR ARTEFACT"
    echo "made by gr1team5"
    echo ""
    echo "----------------------------------------------------"
    echo ""
    echo "This script executes integration tests for the robot"
    echo ""
    echo ""
    echo "- HELP"
    echo "Command name: help | -h"
    echo "Description: display everything this script does."
    echo "Arguments: none"
    echo ""
    echo "Example:"
    echo "./test_integration.sh help"
    echo ""
    echo ""
    echo "- CONFIGURATION"
    echo "Command name: config | -conf"
    echo "Description: this part sets up the environment in order to use the robot and its capacities."
    echo "Arguments: none"
    echo ""
    echo "Example:"
    echo "./test_integration.sh config"
    echo ""
    echo ""
    echo "- MOTORS"
    echo "Command name: motors | -m"
    echo "Description: this part tests if the motors are behaving as instructed."
    echo "Arguments: separated by commas"
    echo "  -rDIST:    makes the robot move the given distance DIST (in cm) (DIST>0 -> forward, DIST<0 -> backward)"
    echo "  -aANG:     makes the robot turn the given angle ANG (in degree) (ANG>0 -> turns right, ANG<0 -> turns left)"
    echo ""
    echo "Example:"
    echo "./test_integration.sh motors r50,a45,r-50,a90,r50"
    echo ""
    echo ""
    echo "- CAMERA"
    echo "Command name: camera | -c"
    echo "Description: this part tests if the robot is capable of detecting marker(s) and estimate its position relatively to it/them"
    echo "Arguments:"
    echo "  -file:     path to an image file that contains aruco marker(s)"
    echo ""
    echo "Outputs: for each detected marker"
    echo "  -id:       id of the detected marker"
    echo "  -d:        horizontal position (in pixels) between the center of the marker and the center of the image (x>0 -> right of center)"
    echo "  -y:        vertical position (in pixels) between the center of the marker and the center of the image (y>0 -> above center)"
    echo "  -dist:     estimate distance (in cm) between the center of the marker and the robot"
    echo "  -angle:    estimate angle (in degrees) between the center of the marker and the optical axis (trigonometric sense)"
    echo ""
    echo "Example:"
    echo "./test_integration.sh camera image.jpg"
    echo "6 120 -40 120 -10"
    echo ""
    echo "Notes: this test assume that the camera used is calibrated, and that its configuration is available in src/vision/camera_calibration.npz"
    echo ""
    echo ""
    echo "- SERVERS"
    echo "Command name: servers | -s"
    echo "Description: this part tests that the robot can start both a websocket and http server that serve the interface on the specified port"
    echo "Arguments:"
    echo "  -portWS:    websocket port to which the server must listen"
    echo "  -portHTTP:  HTTP port to which the server must listen"
    echo ""
    echo "Example:"
    echo "./test_integration.sh servers 8765 8080"
    echo ""
    echo ""
    echo "- TRACKING SERVER"
    echo "Command name: tracking | -t"
    echo "Description: this part tests that the robot can communicate its position and markers to the tracking server"
    echo "Arguments:"
    echo "  -serverURL:        URL to which the robot must send informations"
    echo "  -type=pos|marker   Specify wether we send a position or a marker"
    echo "  -posX:             x-coordinate (in cm) of the robot/marker"
    echo "  -posY:             y-coordinate (in cm) of the robot/marker"
    echo "  -markerID:         only if type=marker, id of the marker"
    echo ""
    echo "Examples:"
    echo "./test_integration.sh tracking http://proj103.r2.enst.fr pos 192 140"
    echo "./test_integration.sh tracking http://proj103.r2.enst.fr marker 192 140 6"
    echo ""
    echo "Notes: if a marker is sent, the posX and posY must be converted to case coordinate (e.g. A5, B9)"
    echo ""
    echo ""
    echo "- PRODUCTION"
    echo "Command name: prod | -p"
    echo "Description: this part starts the robot in its fully operationnal mode"
    echo "Arguments: none"
    echo ""
    echo "Example:"
    echo "./test_integration.sh prod"
    echo ""
    echo ""
}

run_config_env() {
    current_dir=$(basename "$(pwd)")

    echo "Running environment configurations..."

    if [ "$current_dir" != "team5" ]; then
        echo "ERROR: This script must be executed in root directory (team5)"
        echo "Current directory is: $current_dir"
        exit 1
    fi

    echo "Configurating tmux..."

    if ! command -v tmux &> /dev/null; then
        echo "tmux is not installed. Installing now..."
        if ! command -v apt &> /dev/null; then
            echo "ERROR: apt manager package manager not found. This script requires a Debian-based system."
            exit 1
        fi

        sudo apt update
        sudo apt install -y tmux

        if [ $? -ne 0 ]; then
            echo "ERROR: Failed to install tmux"
            exit 1
        fi
        echo "tmux installed successfully."
    else
        echo "tmux is already installed."
    fi

    echo "Configurating python virtual environment..."

    if [ ! -f "requirements.txt" ]; then
        echo "ERROR: requirements.txt not found"
        exit 1
    fi

    if [ ! -d ".venv" ]; then
        echo "Virtual environment not found. Creating .venv..."
        python3 -m venv .venv
        if [ $? -ne 0 ]; then
            echo "ERROR: Failed to create virtual environment"
            exit 1
        fi
    fi

    source .venv/bin/activate

    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to activate virtual environment"
        exit 1
    fi

    python -m pip install --upgrade pip

    pip install -r requirements.txt
    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to install requirements"
        exit 1
    fi

    pip install smbus # not in the req.txt file for dev purposes
    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to install smbus"
        exit 1
    fi

    echo "Updating python path..."
    export PYTHONPATH="${PYTHONPATH}:$(pwd)"

    echo "All environment requirements are met. Proceeding to integration tests now..."
}

run_production_test() {

    run_config_env

    echo "Running production tests..."

    tmux kill-session -t artefact 2>/dev/null

    echo "Creating new tmux session..."
    tmux new-session -d -s artefact

    echo "Starting first tmux window (server)..."
    tmux send-keys -t artefact:0 "cd $(pwd)" C-m
    tmux send-keys -t artefact:0 "source .venv/bin/activate" C-m
    tmux send-keys -t artefact:0 "python src/server/server.py" C-m
    tmux send-keys -t artefact:0 "export PYTHONPATH=\"${PYTHONPATH}:$(pwd)\"" C-m

    echo "Starting second tmux window (robot)..."
    tmux new-window -t artefact
    tmux send-keys -t artefact:1 "cd $(pwd)" C-m
    tmux send-keys -t artefact:1 "source .venv/bin/activate" C-m
    tmux send-keys -t artefact:1 "python src/main.py" C-m
    tmux send-keys -t artefact:1 "export PYTHONPATH=\"${PYTHONPATH}:$(pwd)\"" C-m

    echo "Attaching to the session..."
    tmux attach-session -t artefact
}

run_camera_tests() {
    run_config_env
    echo "Running camera test with image $1..."
    if [ ! -f "$1" ]; then
        echo "ERROR: the image provided was not found."
        exit 1
    fi
    python ./src/vision/aruco_detector.py "$1"
}

run_motors_tests() {
    run_config_env
    echo "Running movement tests..."
    python ./src/controllers/motor_controller.py "$1"
}

run_servers_tests() {
    run_config_env
    echo "Running the servers on the specified ports..."
    python ./src/server/server.py "$1" "$2"
}

run_tracking_server_tests() {
    run_config_env
    echo "Running tracking server tests..."
    python ./src/communication/tracking_server_manager.py "$1" "$2" "$3" "$4"
}

error_arguments() {
    echo "ERROR: missing or wrong arguments. See help for the right usage."
    echo "$ ./integration_tests.sh help"
}

if [ $# -eq 0 ]; then
    run_production_test
    exit 0
fi

mode=$1

case "$mode" in
config | -conf)
    run_config_env
    exit 0
    ;;

prod | -p)
    run_production_test
    exit 0
    ;;

help | -h)
    display_help
    exit 0
    ;;

camera | -c)
    if [ $# -ne 2 ]; then
        error_arguments
        exit 1
    fi
    run_camera_tests "$2"
    exit 0
    ;;

motors | -m)
    if [ $# -ne 2 ]; then
        error_arguments
        exit 1
    fi
    run_motors_tests "$2"
    exit 0
    ;;

servers | -s)
    if [ $# -ne 3 ]; then
        error_arguments
        exit 1
    fi
    run_servers_tests "$2" "$3"
    exit 0
    ;;

tracking | -t)
    if [ $# -eq 4 ]; then
        run_tracking_server_tests "$2" "$3" "$4" "none"
        exit 0
    elif [ $# -eq 5 ]; then
        run_tracking_server_tests "$2" "$3" "$4" "$5"
        exit 0
    else
        error_arguments
        exit 1
    fi
    ;;

*)
    error_arguments
    exit 1

esac
