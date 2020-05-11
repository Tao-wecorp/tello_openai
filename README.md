# Tello OpenAI
DJI Tello drone running OpenAI Stable Baselines

## Env
    catkin_create_pkg tello_ctrl std_msgs rospy roscpp
    cd src
    git submodule init
    git submodule update
    cd ..
    catkin_make -DPYTHON_EXECUTABLE:FILEPATH=~/.virtualenvs/py3venv/bin/python

## Run
    source devel/setup.bash
