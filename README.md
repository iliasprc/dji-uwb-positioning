# DJI-Matrice-100
Control of Quadcopter using Ultra Wide Band positioning system from Pozyx
The Pozyx Python lbrary (Pypozyx) is rquired to work with the pozyx indoor positioningsystem over USB . 
The library however supports only Python 3 while the ROS supports onlyPython 2.  We created a virtual environment preset to run Python 3 scripts so we can use oursensors with the ROS system.

# Appendix A.2.1. 
Virtual environment installation and activation

install virtualenv using pip3 `sudo pip3 install virtualenv`(Optional)

Create folder where we will create the virtual environment

`mkdir environments`

`cd environments`

Create a virtual environmentvirtualenv myenvActivate the virtual environmentsource myenv/bin/activateNow we are able to run our Python 3 program for the positioning.Appendix A.3.  Running the programAt first we launch the core node which tracks publishers and subscribers to topics as wellas services.  The role of the Master is to enable individual ROS nodes to locate one another.Once these nodes have located each other they communicate with each other peer-to-peer.Tolaunch the node we run the command :roslaunch djisdk sdk.launchThen in the virtual environment for Python3 we launch the node for the UWB system whichcalculates and publishes the position topic.rosrun djisdk uwbpub.py28
Finally we run the node that subscribes to the position topic and calculates the controlcommands.rosrun djisdk hover
