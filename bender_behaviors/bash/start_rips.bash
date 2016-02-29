#!/bin/bash

gnome-terminal -x bash -c "source ~/fuerte_workspace/setup.bash;roslaunch bender_speech speech.launch"

sleep 1

gnome-terminal -x bash -c "source ~/fuerte_workspace/setup.bash;rosrun bender_behaviors stop_button.py"

sleep 1

gnome-terminal -x bash -c "source ~/fuerte_workspace/setup.bash;rosrun bender_face face.py"

sleep 1

gnome-terminal -x bash -c "source ~/fuerte_workspace/setup.bash;rosrun bender_behaviors entrance_rips.py"


