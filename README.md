# Tello OpenAI
DJI Tello drone running OpenAI Stable Baselines

## Env
    sudo apt-get install ros-melodic-joy
    rosrun joy joy_node 
    sudo chmod a+rw /dev/input/js0
    catkin_create_pkg tello_ctrl std_msgs rospy roscpp
    cd src
    git submodule init
    git submodule update
    cd ..
    catkin_make -DPYTHON_EXECUTABLE:FILEPATH=~/.virtualenvs/py3venv/bin/python

## Run
    source devel/setup.bash

## Reference
https://github.com/dji-sdk/Tello-Python
https://github.com/clydemcqueen/flock
https://github.com/dbaldwin/DroneBlocks-Tello-Python
https://github.com/geaxgx/tello-openpose
https://github.com/Ubotica/telloCV/blob/master/telloCV.py