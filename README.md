# DJI-Matrice-100 autonomous navigation 
Implementation of Indoor Navigation of Quadrotors via Ultra-Wideband Wireless Technology

https://ieeexplore.ieee.org/abstract/document/8587889 

![alt text](https://github.com/IliasPap/DJI-Matrice-100/blob/master/images/drone1.jpg)
## Control of Quadcopter using Ultra Wide Band positioning system from Pozyx

The Pozyx Python library (Pypozyx) is required to work with the pozyx indoor positioning system over USB connection. 

https://github.com/pozyxLabs/Pozyx-Python-library

The library supports only Python3 while the ROS supports only Python2.  

To run with Python3 create a virtual environment to use UWB sensors with the ROS system.

## Virtual environment installation and activation

install virtualenv using pip3 `sudo pip3 install virtualenv`(Optional)

Create folder where we will create the virtual environment

`mkdir environments`

`cd environments`

Create a virtual environment `virtualenv myenv` 

Activate the virtual environment  `source myenv/bin/activate`

Now we are able to run our Python3 program for the positioning.
##  Running the program
At first we launch the core node which tracks publishers and subscribers to topics. 
The role of the Master is to enable individual ROS nodes to locate one another.
Once these nodes have located each other they communicate with each other peer-to-peer.

To launch the node we run the command : `roslaunch djisdk sdk.launch`

Then in the virtual environment for Python3 we launch the node for the UWB system which calculates and publishes the position topic
`rosrun djisdk uwbpub.py`

Finally we run the node that subscribes to the position topic and calculates the control commands `rosrun djisdk hover`



## Reference 

Please cite the paper below if you use this code in your research:

`@inproceedings{papastratis2018indoor,
  title={Indoor Navigation of Quadrotors via Ultra-Wideband Wireless Technology},
  author={Papastratis, Ilias and Charalambous, Themistoklis and Pappas, Nikolaos},
  booktitle={2018 Advances in Wireless and Optical Communications (RTUWO)},
  pages={106--111},
  year={2018},
  organization={IEEE}
}`


