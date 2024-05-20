#!/bin/bash

trap "echo 'Limpando... (apagando venv)'; rm -rf $(pwd)/src/venv" EXIT

python3 -m venv venv

source venv/bin/activate

pip install typer inquirer

TYPER_PATH=$(pip show typer | grep "Location:" | awk '{print $2}')

if [ -z "$TYPER_PATH" ]; then
    echo "Typer not found. Please make sure it is installed."
    exit 1
fi

export PYTHONPATH="$PYTHONPATH:$TYPER_PATH"

export ROS_DOMAIN_ID=86

cd workspace/

colcon build

source install/local_setup.zsh

ros2 run cli_interface cli_interface